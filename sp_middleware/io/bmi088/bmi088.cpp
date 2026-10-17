// [解析] BMI088 六轴 IMU 驱动实现：SPI 读写寄存器、上电配置、原始值换算与坐标系旋转。
// [解析] 是 bmi088.hpp 里 BMI088 类的实现，INS/姿态任务周期性调 update() 取 acc/gyro/temp。
// [解析] 芯片其实分两颗独立器件：加速度计挂 CSB1、陀螺仪挂 CSB2，寄存器与时序都不通用。
// [解析] 两个最易踩的坑：加速度计上电默认 I2C，要靠一次片选上升沿切 SPI；它读回前 2 字节无效，陀螺仪只 1 字节。
// [解析] 换算常量集中在文件头：LSB -> g -> m/s^2、LSB -> °/s -> rad/s、温度 0.125℃/LSB + 23。

#include "bmi088.hpp"

#include <cstring>

#include "bmi088_defs.h"
#include "cmsis_os.h"
#include "tools/math_tools/math_tools.hpp"

// [解析] 量程是编译期常量，想改必须改代码重编译；6G / 2000dps 兼顾冲击不削顶与分辨率。
// TODO 构造时配置
constexpr uint8_t BMI088_ACC_RANGE_SET = BMI088_ACC_RANGE_6G;
constexpr uint8_t BMI088_GYRO_RANGE_SET = BMI088_GYRO_2000;

// [解析] 重力加速度取 9.8（标准值 9.80665，差 0.07%，工程上可接受）。
// ref: datasheet 5.3.4
constexpr float G_TO_MPS2 = 9.8f;
// [解析] 加速度标度因数：1 LSB = 满量程/32768 = 1.5*2^(n+1)/32768 g，本项目 n=1 时正好是 6/32768。
constexpr float BMI088_ACC_INT_TO_G = (1 << (BMI088_ACC_RANGE_SET + 1)) * 1.5f / 32768;
constexpr float BMI088_ACC_INT_TO_MPS2 = BMI088_ACC_INT_TO_G * G_TO_MPS2;

// [解析] 陀螺仪标度因数：手册满量程 = 2000/2^n，而这里写的是 /(n+1)，只在 n=0、n=1 时与手册吻合。
// [解析] 本项目固定 n=0（2000dps）所以正确；将来换成 500dps 等更小量程时必须一并修正。
// ref: datasheet 5.5.2
constexpr float BMI088_GYRO_INT_TO_DPS = 2000.0f / (BMI088_GYRO_RANGE_SET + 1) / 32767;
// [解析] °/s -> rad/s；gyro[] 对外承诺的单位就是 rad/s。
constexpr float BMI088_GYRO_INT_TO_RPS = BMI088_GYRO_INT_TO_DPS / 180 * sp::SP_PI;

// [解析] 温度 = 原始值 * 0.125 + 23：0.125℃/LSB 是分辨率，23 是零点偏移。
// ref: datasheet 5.3.7
constexpr float BMI088_TEMP_FACTOR = 0.125f;
constexpr float BMI088_TEMP_OFFSET = 23.0f;

// [解析] 表行数，数组定义与循环上限共用；改小了漏配置，改大了数组越界。
constexpr size_t BMI088_ACC_INIT_TABLE_SIZE = 6;
constexpr size_t BMI088_GYRO_INIT_TABLE_SIZE = 6;

// clang-format off

// [解析] 加速度计初始化表：[寄存器地址, 写入值, 回读不一致时返回的错误码]，顺序按手册推荐，不能随意调换。
// [解析] 第 3 行的 MUST_SET 是必须置 1 的保留位；改这里的 ODR 后，上层任务的调用频率也要跟着改。
const uint8_t BMI088_ACC_INIT_TABLE[BMI088_ACC_INIT_TABLE_SIZE][3] = {
  {BMI088_ACC_PWR_CTRL      , BMI088_ACC_ENABLE_ACC_ON                                                       , BMI088_ACC_PWR_CTRL_ERROR      },
  {BMI088_ACC_PWR_CONF      , BMI088_ACC_PWR_ACTIVE_MODE                                                     , BMI088_ACC_PWR_CONF_ERROR      },
  {BMI088_ACC_CONF          , BMI088_ACC_BWP_NORMAL | BMI088_ACC_ODR_800_HZ | BMI088_ACC_CONF_MUST_SET       , BMI088_ACC_CONF_ERROR          },
  {BMI088_ACC_RANGE         , BMI088_ACC_RANGE_SET                                                           , BMI088_ACC_RANGE_ERROR         },
  {BMI088_INT1_IO_CONF      , BMI088_ACC_INT1_OUT_ENABLE | BMI088_ACC_INT1_GPIO_PP | BMI088_ACC_INT1_GPIO_LOW, BMI088_INT1_IO_CONF_ERROR      },
  {BMI088_INT1_INT2_MAP_DATA, BMI088_DRDY_TO_INT1                                                            , BMI088_INT1_INT2_MAP_DATA_ERROR}
};

// [解析] 陀螺仪初始化表，结构同上。带宽编码与量程绑定，改了量程必须同时改带宽宏，否则滤波频率对不上。
const uint8_t BMI088_GYRO_INIT_TABLE[BMI088_GYRO_INIT_TABLE_SIZE][3] = {
  {BMI088_GYRO_RANGE       , BMI088_GYRO_RANGE_SET                                   , BMI088_GYRO_RANGE_ERROR       },
  {BMI088_GYRO_BANDWIDTH   , BMI088_GYRO_2000_230_HZ | BMI088_GYRO_BANDWIDTH_MUST_SET, BMI088_GYRO_BANDWIDTH_ERROR   },
  {BMI088_GYRO_LPM1        , BMI088_GYRO_NORMAL_MODE                                 , BMI088_GYRO_LPM1_ERROR        },
  {BMI088_GYRO_INT_CTRL    , BMI088_DRDY_ON                                          , BMI088_GYRO_CTRL_ERROR        },
  {BMI088_INT3_INT4_IO_CONF, BMI088_INT3_GPIO_PP | BMI088_INT3_GPIO_LOW              , BMI088_INT3_INT4_IO_CONF_ERROR},
  {BMI088_INT3_INT4_IO_MAP , BMI088_DRDY_IO_INT3                                     , BMI088_INT3_INT4_IO_MAP_ERROR }
};

// clang-format on

namespace sp
{
// [解析] 只保存配置、不碰硬件。r_ab_ 是 const 数组，只能用初始化列表按元素拷贝（存副本而非指针）。
BMI088::BMI088(
  SPI_HandleTypeDef * hspi, GPIO_TypeDef * csb1_port, uint16_t csb1_pin, GPIO_TypeDef * csb2_port,
  uint16_t csb2_pin, const float r_ab[3][3], const float gyro_multipliers[3])
: hspi_(hspi),
  csb1_port_(csb1_port),
  csb2_port_(csb2_port),
  csb1_pin_(csb1_pin),
  csb2_pin_(csb2_pin),
  r_ab_{
    {r_ab[0][0], r_ab[0][1], r_ab[0][2]},
    {r_ab[1][0], r_ab[1][1], r_ab[1][2]},
    {r_ab[2][0], r_ab[2][1], r_ab[2][2]}}
{
  // 初始化标度因数
  if (gyro_multipliers != nullptr) {
    gyro_multipliers_[0] = gyro_multipliers[0];
    gyro_multipliers_[1] = gyro_multipliers[1];
    gyro_multipliers_[2] = gyro_multipliers[2];
  }
  else {
    // 默认不校准
    gyro_multipliers_[0] = 1.0f;
    gyro_multipliers_[1] = 1.0f;
    gyro_multipliers_[2] = 1.0f;
  }
}

// [解析] 两颗芯片都配置成功才返回；while 空循环失败即无限重试，传感器不在线会永远卡在这里。
// [解析] 里面用了 osDelay，必须在 FreeRTOS 调度器启动后的任务里调用，不能放在 main() 里。
void BMI088::init()
{
  while (acc_init() != BMI088_NO_ERROR);
  while (gyro_init() != BMI088_NO_ERROR);
}

// [解析] 刷新全部对外数据；调用频率应与芯片 ODR 匹配，太快读到重复值、太慢丢帧。
void BMI088::update()
{
  acc_update();
  gyro_update();
}

// -------------------- 加速度计 --------------------

// [解析] 配置加速度计：复位 -> 查 ID -> 按表写寄存器并回读校验，返回 0 成功、0xFF 表示没检测到芯片。
// [解析] 难点：上电和每次软复位后芯片都回到 I2C 模式，必须靠一次 CSB1 上升沿切到 SPI，所以前两次读
// [解析] CHIP_ID 只为制造这个沿、数据丢弃；每步之后的 osDelay(1) 是在等芯片完成内部动作。
uint8_t BMI088::acc_init()
{
  // 上电后, 加速度计以I2C模式启动, 需要一次CSB1上升沿来切换到SPI模式, ref: datasheet 6.1
  acc_read(BMI088_ACC_CHIP_ID, 1);
  osDelay(1);

  // 软复位
  acc_write(BMI088_ACC_SOFTRESET, BMI088_ACC_SOFTRESET_VALUE);
  osDelay(1);

  // 软复位后, 加速度计以I2C模式启动, 需要一次CSB1上升沿来切换到SPI模式
  acc_read(BMI088_ACC_CHIP_ID, 1);
  osDelay(1);

  // 读取ID
  acc_read(BMI088_ACC_CHIP_ID, 1);
  osDelay(1);

  // 检查ID, 前两个字节为无效数据
  uint8_t whoami = rx_buff_[2];
  if (whoami != BMI088_ACC_CHIP_ID_VALUE) return BMI088_NO_SENSOR;

  // 配置寄存器
  uint8_t reg = 0, data = 0;
  for (uint8_t i = 0; i < BMI088_ACC_INIT_TABLE_SIZE; i++) {
    reg = BMI088_ACC_INIT_TABLE[i][0];

    acc_write(reg, BMI088_ACC_INIT_TABLE[i][1]);
    osDelay(1);

    acc_read(reg, 1);
    osDelay(1);

    // 检查是否写入成功, 前两个字节为无效数据
    data = rx_buff_[2];
    if (data != BMI088_ACC_INIT_TABLE[i][1]) return BMI088_ACC_INIT_TABLE[i][2];
  }

  return BMI088_NO_ERROR;
}

// [解析] 读一次加速度和温度：原始值 -> m/s^2 -> 机器人坐标系，结果写进 acc[] / temp。
// [解析] 有效数据从下标 2 起（前 2 字节无效）；小端拼接 (高 << 8) | 低 赋给 int16_t 即完成补码解释。
void BMI088::acc_update()
{
  // 接收加速度计数据
  acc_read(BMI088_ACC_DATA, 6);

  // 前两个字节为无效数据
  int16_t acc_x_int = (rx_buff_[2 + 1] << 8) | rx_buff_[2 + 0];
  int16_t acc_y_int = (rx_buff_[2 + 3] << 8) | rx_buff_[2 + 2];
  int16_t acc_z_int = (rx_buff_[2 + 5] << 8) | rx_buff_[2 + 4];
  float acc_x = acc_x_int * BMI088_ACC_INT_TO_MPS2;
  float acc_y = acc_y_int * BMI088_ACC_INT_TO_MPS2;
  float acc_z = acc_z_int * BMI088_ACC_INT_TO_MPS2;

  // 前两个字节为无效数据
  acc_read(BMI088_TEMP_DATA, 2);
  // [解析] 温度只有 11 位有效位，不是普通 16 位：高字节 << 3 | 第二字节 >> 5，超过 1023 再减 2048。
  int16_t temp_int = (rx_buff_[2 + 0] << 3) | (rx_buff_[2 + 1] >> 5);
  if (temp_int > 1023) temp_int -= 2048;  // ref: 5.3.7

  // 更新公开属性
  // [解析] 左乘 r_ab_ 把传感器坐标系转到机器人坐标系；r_ab_ 是单位矩阵时等于原样拷贝。
  this->acc[0] = r_ab_[0][0] * acc_x + r_ab_[0][1] * acc_y + r_ab_[0][2] * acc_z;
  this->acc[1] = r_ab_[1][0] * acc_x + r_ab_[1][1] * acc_y + r_ab_[1][2] * acc_z;
  this->acc[2] = r_ab_[2][0] * acc_x + r_ab_[2][1] * acc_y + r_ab_[2][2] * acc_z;
  this->temp = temp_int * BMI088_TEMP_FACTOR + BMI088_TEMP_OFFSET;
}

// [解析] 读 len 字节到 rx_buff_，有效数据从 rx_buff_[2] 开始。
// [解析] SPI 全双工：第 1 字节发"地址|0x80"（最高位 1 = 读）时收到的是回环数据，第 2 字节是 dummy，
// [解析] 所以发送长度是 len+2。退出前必须拉高 CSB1：结束事务并复位芯片内部地址指针。
void BMI088::acc_read(uint8_t reg, uint8_t len)
{
  // 拉低CSB1
  HAL_GPIO_WritePin(csb1_port_, csb1_pin_, GPIO_PIN_RESET);

  // ref: datasheet 6.1.2
  tx_buff_[0] = reg | 0x80;

  // 前两个字节为无效数据
  HAL_SPI_TransmitReceive(hspi_, tx_buff_, rx_buff_, len + 2, 100);

  // 拉高CSB1
  HAL_GPIO_WritePin(csb1_port_, csb1_pin_, GPIO_PIN_SET);
}

// [解析] 写 1 个寄存器：地址不加 0x80（最高位 0 = 写），2 个字节只发不收。
void BMI088::acc_write(uint8_t reg, uint8_t data)
{
  // 拉低CSB1
  HAL_GPIO_WritePin(csb1_port_, csb1_pin_, GPIO_PIN_RESET);

  // ref: datasheet 6.1.2
  tx_buff_[0] = reg;
  tx_buff_[1] = data;

  // 只发不收
  HAL_SPI_Transmit(hspi_, tx_buff_, 2, 100);

  // 拉高CSB1
  HAL_GPIO_WritePin(csb1_port_, csb1_pin_, GPIO_PIN_SET);
}

// -------------------- 陀螺仪 --------------------

// [解析] 配置陀螺仪，返回码含义同 acc_init()。
// [解析] 与加速度计不同：不用制造上升沿切 SPI 模式，且有效数据从 rx_buff_[1] 开始。
uint8_t BMI088::gyro_init()
{
  // 软复位
  gyro_write(BMI088_GYRO_SOFTRESET, BMI088_GYRO_SOFTRESET_VALUE);
  osDelay(1);

  // 读取ID
  gyro_read(BMI088_GYRO_CHIP_ID, 1);
  osDelay(1);

  // 检查ID, 第一个字节为无效数据
  uint8_t whoami = rx_buff_[1];
  if (whoami != BMI088_GYRO_CHIP_ID_VALUE) return BMI088_NO_SENSOR;

  // 配置寄存器
  uint8_t reg = 0, data = 0;
  for (uint8_t i = 0; i < BMI088_GYRO_INIT_TABLE_SIZE; i++) {
    reg = BMI088_GYRO_INIT_TABLE[i][0];

    gyro_write(reg, BMI088_GYRO_INIT_TABLE[i][1]);
    osDelay(1);

    gyro_read(reg, 1);
    osDelay(1);

    // 检查是否写入成功, 第一个字节为无效数据
    data = rx_buff_[1];
    if (data != BMI088_GYRO_INIT_TABLE[i][1]) return BMI088_GYRO_INIT_TABLE[i][2];
  }

  return BMI088_NO_ERROR;
}

// [解析] 读三轴角速度：换算成 rad/s 后乘 gyro_multipliers_ 做灵敏度微调，最后转到机器人坐标系。
void BMI088::gyro_update()
{
  // 接收陀螺仪数据
  gyro_read(BMI088_GYRO_DATA, 6);

  // 第一个字节为无效数据
  int16_t gyro_x_int = (rx_buff_[1 + 1] << 8) | rx_buff_[1 + 0];
  int16_t gyro_y_int = (rx_buff_[1 + 3] << 8) | rx_buff_[1 + 2];
  int16_t gyro_z_int = (rx_buff_[1 + 5] << 8) | rx_buff_[1 + 4];

  // 应用传入的标度因数
  float gyro_x = gyro_x_int * BMI088_GYRO_INT_TO_RPS * gyro_multipliers_[0];
  float gyro_y = gyro_y_int * BMI088_GYRO_INT_TO_RPS * gyro_multipliers_[1];
  float gyro_z = gyro_z_int * BMI088_GYRO_INT_TO_RPS * gyro_multipliers_[2];

  // 更新公开属性
  this->gyro[0] = r_ab_[0][0] * gyro_x + r_ab_[0][1] * gyro_y + r_ab_[0][2] * gyro_z;
  this->gyro[1] = r_ab_[1][0] * gyro_x + r_ab_[1][1] * gyro_y + r_ab_[1][2] * gyro_z;
  this->gyro[2] = r_ab_[2][0] * gyro_x + r_ab_[2][1] * gyro_y + r_ab_[2][2] * gyro_z;
}

// [解析] 读 len 字节到 rx_buff_，有效数据从 rx_buff_[1] 开始。
// [解析] 与 acc_read 的唯一区别：陀螺仪只多 1 个 dummy 字节，所以发送长度是 len+1 而不是 len+2。
void BMI088::gyro_read(uint8_t reg, uint8_t len)
{
  // 拉低CSB2
  HAL_GPIO_WritePin(csb2_port_, csb2_pin_, GPIO_PIN_RESET);

  // ref: datasheet 6.1.1
  tx_buff_[0] = reg | 0x80;

  // 第一个字节为无效数据
  HAL_SPI_TransmitReceive(hspi_, tx_buff_, rx_buff_, len + 1, 100);

  // 拉高CSB2
  HAL_GPIO_WritePin(csb2_port_, csb2_pin_, GPIO_PIN_SET);
}

void BMI088::gyro_write(uint8_t reg, uint8_t data)
{
  // 拉低CSB2
  HAL_GPIO_WritePin(csb2_port_, csb2_pin_, GPIO_PIN_RESET);

  // ref: datasheet 6.1.1
  tx_buff_[0] = reg;
  tx_buff_[1] = data;

  // 只发不收
  HAL_SPI_Transmit(hspi_, tx_buff_, 2, 100);

  // 拉高CSB2
  HAL_GPIO_WritePin(csb2_port_, csb2_pin_, GPIO_PIN_SET);
}

}  // namespace sp
