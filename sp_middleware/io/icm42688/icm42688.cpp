// [解析] ICM42688 六轴 IMU 驱动实现：SPI 读写寄存器、上电配置、原始值换算与坐标系旋转。
// [解析] 是 icm42688.hpp 里 ICM42688 类的实现，上层周期任务调 update() 取 acc/gyro/temp。
// [解析] 与 bmi088 的三处差别：只有一片硅一根片选、读回只丢 1 个字节、数据是大端（高字节在前）。
// [解析] update() 用一次突发读取（0x1D 起 14 字节）同时取温度+加速度+角速度，保证来自同一采样时刻。
// [解析] 换算常量在文件头：16G 时 16/32768 g/LSB、2000dps 时 2000/32768 °/s、温度 = 原始值/132.48 + 25。

#include "icm42688.hpp"

#include <cstring>
#include "icm42688_defs.h"
#include "cmsis_os.h"
#include "tools/math_tools/math_tools.hpp"

// [解析] 三个编译期常量：16G 抗落地冲击、2000dps 够小陀螺高速自转、ODR 1kHz 决定 update() 的调用节奏。
// 构造时配置
constexpr uint8_t ICM42688_ACC_RANGE_SET = ICM42688_ACC_RANGE_16G;
constexpr uint8_t ICM42688_GYRO_RANGE_SET = ICM42688_GYRO_RANGE_2000DPS;
constexpr uint8_t ICM42688_ODR_SET = ICM42688_ODR_1KHZ; // 设定1000Hz刷新率

// 根据量程计算缩放因子 (16G & 2000DPS)
constexpr float G_TO_MPS2 = 9.8f;
// [解析] 加速度标度因数 16/32768：满量程 ±16g 对应 ±32768 个计数。
// [解析] 分子 16.0f 是手写的、不随 ICM42688_ACC_RANGE_SET 变化，改量程时必须同步改这里。
constexpr float ICM42688_ACC_INT_TO_G = 16.0f / 32768.0f;
constexpr float ICM42688_ACC_INT_TO_MPS2 = ICM42688_ACC_INT_TO_G * G_TO_MPS2;

// [解析] 陀螺仪标度因数 2000/32768 得 °/s，再除以 180 乘 π 就是 rad/s（gyro[] 对外单位）。
// [解析] 分子同样手写、改量程要一起改；这里除 32768、bmi088 那份除 32767，是两份手册的写法差异。
constexpr float ICM42688_GYRO_INT_TO_DPS = 2000.0f / 32768.0f;
constexpr float ICM42688_GYRO_INT_TO_RPS = ICM42688_GYRO_INT_TO_DPS / 180.0f * sp::SP_PI;

// 温度转换系数 (Datasheet提供公式: Temp = val / 132.48 + 25)
constexpr float ICM42688_TEMP_FACTOR = 1.0f / 132.48f;
constexpr float ICM42688_TEMP_OFFSET = 25.0f;

constexpr size_t ICM42688_INIT_TABLE_SIZE = 3;

// clang-format off

// [解析] 初始化表：[寄存器地址, 写入值, 回读不一致时返回的错误码]。
// [解析] PWR_MGMT0 必须最先写，否则后面的量程/ODR 配置不生效；量程与 ODR 的宏都已移好位，直接按位或。
// [解析] 本驱动不配中断引脚，靠任务主动轮询，所以 update() 必须按固定周期调用。
const uint8_t ICM42688_INIT_TABLE[ICM42688_INIT_TABLE_SIZE][3] = {
  {ICM42688_PWR_MGMT0,     ICM42688_PWR_LN_MODE,                              ICM42688_PWR_MGMT_ERROR   },
  {ICM42688_ACCEL_CONFIG0, ICM42688_ACC_RANGE_SET | ICM42688_ODR_SET,         ICM42688_ACCEL_CONF_ERROR },
  {ICM42688_GYRO_CONFIG0,  ICM42688_GYRO_RANGE_SET | ICM42688_ODR_SET,        ICM42688_GYRO_CONF_ERROR  }
};
// clang-format on

namespace sp
{
// [解析] 只保存配置、不碰硬件。r_ab_ 是 const 数组，只能用初始化列表按元素拷贝（存副本而非指针）。
ICM42688::ICM42688(
  SPI_HandleTypeDef * hspi, GPIO_TypeDef * cs_port, uint16_t cs_pin, 
  const float r_ab[3][3], const float gyro_multipliers[3])
: hspi_(hspi),
  cs_port_(cs_port),
  cs_pin_(cs_pin),
  r_ab_{
    {r_ab[0][0], r_ab[0][1], r_ab[0][2]},
    {r_ab[1][0], r_ab[1][1], r_ab[1][2]},
    {r_ab[2][0], r_ab[2][1], r_ab[2][2]}}
{
  if (gyro_multipliers != nullptr) {
    gyro_multipliers_[0] = gyro_multipliers[0];
    gyro_multipliers_[1] = gyro_multipliers[1];
    gyro_multipliers_[2] = gyro_multipliers[2];
  } else {
    gyro_multipliers_[0] = 1.0f;
    gyro_multipliers_[1] = 1.0f;
    gyro_multipliers_[2] = 1.0f;
  }
}

// [解析] 反复重试直到配置成功；比 bmi088 多了 osDelay(10)，失败时先让出 CPU，不会把同优先级任务饿死。
// [解析] osDelay 依赖调度器已启动，所以 init() 要在内核启动后的任务里调用一次。
void ICM42688::init()
{
  while (device_init() != ICM42688_NO_ERROR) {
      osDelay(10); // 避免死循环卡死系统，加入适当延时等待
  }
}

// [解析] 真正的初始化：软复位 -> 查 WHO_AM_I -> 按表写配置并回读校验，失败返回该行预设的错误码。
// [解析] 返回 0 成功、0xFF 表示 ID 不对（芯片不在线），其他值对应配置表里某一步失败。
uint8_t ICM42688::device_init()
{
  // 1. 软复位
  write_register(ICM42688_DEVICE_CONFIG, ICM42688_SOFT_RESET_VALUE);
  osDelay(10); // 等待复位完成

  // 2. 检查设备ID
  read_registers(ICM42688_WHO_AM_I, 1);
  osDelay(1);

  // rx_buff_[0]是虚拟字节，rx_buff_[1]是实际数据
  // [解析] 只跳 1 个无效字节（第 1 个是地址回环），数据从 rx_buff_[1] 起，不像 bmi088 加速度计要跳 2 个。
  uint8_t whoami = rx_buff_[1];
  if (whoami != ICM42688_CHIP_ID_VALUE) return ICM42688_NO_SENSOR;

  // 3. 按照Table配置寄存器
  uint8_t reg = 0, data = 0;
  for (uint8_t i = 0; i < ICM42688_INIT_TABLE_SIZE; i++) {
    reg = ICM42688_INIT_TABLE[i][0];

    // [解析] 写完等 2ms：PWR_MGMT0 之后电源域要稳定（手册要求 >200us），否则紧接着的写会被忽略。
    write_register(reg, ICM42688_INIT_TABLE[i][1]);
    osDelay(2); // ICM42688 需要在配置电源后有短暂延时

    read_registers(reg, 1);
    osDelay(1);

    data = rx_buff_[1];
    if (data != ICM42688_INIT_TABLE[i][1]) return ICM42688_INIT_TABLE[i][2];
  }

  return ICM42688_NO_ERROR;
}

// [解析] 一次 SPI 事务把温度、三轴加速度、三轴角速度全读回来（突发读取，芯片内部地址自动递增）。
// [解析] 地址从 0x1D 到 0x2A 共 14 字节：温度 2 字节，之后每轴 2 字节，依次加速度 XYZ、角速度 XYZ。
void ICM42688::update()
{
  // 突发读取从 0x1D (温度起始) 到 0x2A (陀螺仪Z轴结束)，共计 14 个字节
  read_registers(ICM42688_TEMP_DATA1, 14);

  // 第1个字节(rx_buff_[0]) 为寄存器地址占位，真实数据从rx_buff_[1]开始
  // ICM42688 传输是大端模式 (MSB First)
  // [解析] 大端(MSB First)：前一个字节是高 8 位；拼法与 bmi088 形式相同，但字节顺序正好相反。
  int16_t temp_int   = (rx_buff_[1] << 8)  | rx_buff_[2];

  int16_t acc_x_int  = (rx_buff_[3] << 8)  | rx_buff_[4];
  int16_t acc_y_int  = (rx_buff_[5] << 8)  | rx_buff_[6];
  int16_t acc_z_int  = (rx_buff_[7] << 8)  | rx_buff_[8];

  int16_t gyro_x_int = (rx_buff_[9] << 8)  | rx_buff_[10];
  int16_t gyro_y_int = (rx_buff_[11] << 8) | rx_buff_[12];
  int16_t gyro_z_int = (rx_buff_[13] << 8) | rx_buff_[14];

  // 计算原始物理量
  float acc_x = acc_x_int * ICM42688_ACC_INT_TO_MPS2;
  float acc_y = acc_y_int * ICM42688_ACC_INT_TO_MPS2;
  float acc_z = acc_z_int * ICM42688_ACC_INT_TO_MPS2;

  float gyro_x = gyro_x_int * ICM42688_GYRO_INT_TO_RPS * gyro_multipliers_[0];
  float gyro_y = gyro_y_int * ICM42688_GYRO_INT_TO_RPS * gyro_multipliers_[1];
  float gyro_z = gyro_z_int * ICM42688_GYRO_INT_TO_RPS * gyro_multipliers_[2];

  // 坐标系转换 (应用传入的旋转矩阵)
  // [解析] 左乘 r_ab_ 把传感器坐标系转到机器人坐标系；加速度与角速度同在芯片内，共用一个矩阵。
  this->acc[0] = r_ab_[0][0] * acc_x + r_ab_[0][1] * acc_y + r_ab_[0][2] * acc_z;
  this->acc[1] = r_ab_[1][0] * acc_x + r_ab_[1][1] * acc_y + r_ab_[1][2] * acc_z;
  this->acc[2] = r_ab_[2][0] * acc_x + r_ab_[2][1] * acc_y + r_ab_[2][2] * acc_z;

  this->gyro[0] = r_ab_[0][0] * gyro_x + r_ab_[0][1] * gyro_y + r_ab_[0][2] * gyro_z;
  this->gyro[1] = r_ab_[1][0] * gyro_x + r_ab_[1][1] * gyro_y + r_ab_[1][2] * gyro_z;
  this->gyro[2] = r_ab_[2][0] * gyro_x + r_ab_[2][1] * gyro_y + r_ab_[2][2] * gyro_z;

  this->temp = temp_int * ICM42688_TEMP_FACTOR + ICM42688_TEMP_OFFSET;
}

// [解析] 从 reg 起连读 len 字节到 rx_buff_，有效数据在 rx_buff_[1] ~ rx_buff_[len]。
// [解析] SPI 全双工：第 1 字节发"地址|0x80"（最高位 1 = 读）时 MISO 上还是无效回环，真实数据从第 2 个字节开始；
// [解析] 只丢 1 个字节正是这颗芯片与 bmi088 加速度计的时序差别。CS 拉高才结束事务、复位芯片内部地址指针。
void ICM42688::read_registers(uint8_t reg, uint8_t len)
{
  HAL_GPIO_WritePin(cs_port_, cs_pin_, GPIO_PIN_RESET);

  tx_buff_[0] = reg | 0x80; // 最高位设为1代表读取

  // 读取总长度是 寄存器地址字节(1) + 数据长度(len)
  HAL_SPI_TransmitReceive(hspi_, tx_buff_, rx_buff_, len + 1, 100);

  HAL_GPIO_WritePin(cs_port_, cs_pin_, GPIO_PIN_SET);
}

void ICM42688::write_register(uint8_t reg, uint8_t data)
{
  HAL_GPIO_WritePin(cs_port_, cs_pin_, GPIO_PIN_RESET);

  tx_buff_[0] = reg;       // 最高位设为0代表写入
  tx_buff_[1] = data;

  HAL_SPI_Transmit(hspi_, tx_buff_, 2, 100);

  HAL_GPIO_WritePin(cs_port_, cs_pin_, GPIO_PIN_SET);
}

}  // namespace sp