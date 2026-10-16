// [解析] ==========================================================================
// [解析] 【文件】sp_middleware/io/icm42688/icm42688.cpp
// [解析] 【作用】ICM42688 六轴 IMU 的驱动实现：SPI 读写寄存器、上电配置表、
// [解析]         原始值→物理量（m/s^2、rad/s、℃）换算、坐标系旋转。
// [解析] 【被谁用】icm42688.hpp 声明的 ICM42688 类；最终由 applications 里的周期任务调用 update()。
// [解析] 【看代码的重点】四个难点，下面都逐行标注：
// [解析]   1) 标度因数为什么是 16/32768 和 2000/32768（量程与 LSB 的关系）；
// [解析]   2) 配置寄存器里"量程 << 5 | ODR"的位拼接是怎么回事；
// [解析]   3) 为什么读一次就能拿到温度+加速度+角速度（突发读取 / burst read）；
// [解析]   4) 大端(MSB first) vs BMI088 小端的差别（字节拼接顺序相反）。
// [解析] 【阅读顺序】read_registers/write_register -> device_init -> update。
// [解析] ==========================================================================

#include "icm42688.hpp"

#include <cstring>
#include "icm42688_defs.h"
#include "cmsis_os.h"
#include "tools/math_tools/math_tools.hpp"
// [解析] 头文件作用：icm42688.hpp 是本类声明；<cstring> 是字符串/内存函数（本文件未直接使用）；
// [解析]   icm42688_defs.h 提供寄存器地址与配置宏；cmsis_os.h 提供 osDelay()；
// [解析]   math_tools.hpp 提供圆周率常量 sp::SP_PI。

// 构造时配置
// [解析] 三个编译期常量，决定芯片工作模式。注意它们必须和下面算标度因数用的值"手写一致"：
// [解析]   ICM42688_ACC_RANGE_SET = 16G  —— 机器人落地/碰撞冲击较大，16G 不容易饱和；
// [解析]   ICM42688_GYRO_RANGE_SET = 2000DPS —— 小陀螺高速自转时角速度可能上千 °/s；
// [解析]   ICM42688_ODR_SET = 1kHz —— 输出数据率，决定 update() 应该以多快频率调用。
constexpr uint8_t ICM42688_ACC_RANGE_SET = ICM42688_ACC_RANGE_16G;
constexpr uint8_t ICM42688_GYRO_RANGE_SET = ICM42688_GYRO_RANGE_2000DPS;
constexpr uint8_t ICM42688_ODR_SET = ICM42688_ODR_1KHZ; // 设定1000Hz刷新率

// 根据量程计算缩放因子 (16G & 2000DPS)
// [解析] 重力加速度，标准值 9.80665，这里取 9.8（误差 0.07%，工程上可接受）。
constexpr float G_TO_MPS2 = 9.8f;
// [解析] ★核心换算 1★ 加速度标度因数：每 1 个 LSB 代表多少 g。
// [解析] ICM42688 的加速度是 16 位有符号数，满量程 ±16g 对应 ±32768 个计数，
// [解析] 所以 1 LSB = 16 / 32768 = 4.883e-4 g = 4.787e-3 m/s^2。
// [解析] 注意：分子 16.0f 是"手写"的，没有从 ICM42688_ACC_RANGE_SET 推导出来。
// [解析]   如果以后把量程改成 8G，必须同时把这里改成 8.0f，否则读出来的加速度会小一倍！
// [解析]   （BMI088 那份驱动是用宏算出来的，改量程时不会漏，两种写法要注意区分。）
constexpr float ICM42688_ACC_INT_TO_G = 16.0f / 32768.0f;
constexpr float ICM42688_ACC_INT_TO_MPS2 = ICM42688_ACC_INT_TO_G * G_TO_MPS2;

// [解析] ★核心换算 2★ 陀螺仪标度因数：每 1 个 LSB 代表多少 °/s。
// [解析] 满量程 ±2000°/s 对应 ±32768 个计数，所以 1 LSB = 2000/32768 ≈ 0.061 °/s。
// [解析] 同样地，分子 2000.0f 也是手写的，改量程时必须一起改。
// [解析] 这里除以 32768（BMI088 那份用的是 32767），两者差的 0.003% 可以忽略，
// [解析]   但对比阅读时不要以为是笔误。
constexpr float ICM42688_GYRO_INT_TO_DPS = 2000.0f / 32768.0f;
// [解析] °/s -> rad/s：除以 180 再乘 π。因为 gyro[] 对外承诺的单位是 rad/s。
constexpr float ICM42688_GYRO_INT_TO_RPS = ICM42688_GYRO_INT_TO_DPS / 180.0f * sp::SP_PI;

// 温度转换系数 (Datasheet提供公式: Temp = val / 132.48 + 25)
// [解析] 温度换算：温度(℃) = 原始值 / 132.48 + 25，这是数据手册直接给出的公式，照抄即可。
// [解析]   132.48 是手册的综合换算系数（把 16 位原始值线性映射成摄氏度），不需要自己推导；
// [解析]   25 是零点偏移：原始值为 0 时对应 25℃。
// [解析] 代码里写成 ICM42688_TEMP_FACTOR = 1.0f/132.48f，是为了让"乘系数"的写法
// [解析]   和手册里的"除以 132.48"一眼能对上；constexpr 会在编译期就把这个数算好，
// [解析]   运行时不会真的多一次除法。
constexpr float ICM42688_TEMP_FACTOR = 1.0f / 132.48f;
constexpr float ICM42688_TEMP_OFFSET = 25.0f;

// [解析] 初始化表行数。数组定义和循环上限共用它，增删表项时只改这一处。
// [解析] 小心：改小会漏配置，改大会数组越界读到不确定内存。
constexpr size_t ICM42688_INIT_TABLE_SIZE = 3;

// [解析] clang-format off/on：告诉代码格式化工具"中间这段不要自动重排"，
// [解析] 因为下面表格的列是用空格手工对齐的，重排后就看不出对应关系了。
// clang-format off

// [解析] ★ICM42688 初始化表★ 每行 3 个元素，含义：
// [解析]   [0] 寄存器地址   [1] 要写入的值   [2] 回读校验失败时返回的错误码
// [解析] 逐行说明：
// [解析]   1) PWR_MGMT0 = 0x0F：给加速度计和陀螺仪同时上电，并都设为低噪声(Low Noise)模式。
// [解析]      注意这颗芯片必须先把电源模式配好，后面的量程寄存器才生效；
// [解析]      而且手册要求上电后要等一小会儿（device_init 里用 osDelay(2) 处理）；
// [解析]   2) ACCEL_CONFIG0 = 量程 | ODR：量程在 bit[7:5]、ODR 在 bit[3:0]，
// [解析]      两个宏本身已经"移好位"，所以直接按位或就能拼成一个字节（见 icm42688_defs.h）；
// [解析]   3) GYRO_CONFIG0 = 量程 | ODR：同样的位布局，陀螺仪的量程和 ODR 也各占一段。
// [解析] 注意：这里没有单独配置中断引脚（INT1/INT2），因为本驱动采用"主动轮询读数据"的方式，
// [解析]   不像 BMI088 那样靠 DRDY 中断触发。所以 update() 必须由任务按固定周期调用。
const uint8_t ICM42688_INIT_TABLE[ICM42688_INIT_TABLE_SIZE][3] = {
  {ICM42688_PWR_MGMT0,     ICM42688_PWR_LN_MODE,                              ICM42688_PWR_MGMT_ERROR   },
  {ICM42688_ACCEL_CONFIG0, ICM42688_ACC_RANGE_SET | ICM42688_ODR_SET,         ICM42688_ACCEL_CONF_ERROR },
  {ICM42688_GYRO_CONFIG0,  ICM42688_GYRO_RANGE_SET | ICM42688_ODR_SET,        ICM42688_GYRO_CONF_ERROR  }
};
// [解析] 恢复自动格式化。
// clang-format on

namespace sp
{
// [解析] 构造函数：用"初始化列表"把参数存进成员变量。
// [解析] 为什么必须写在冒号后面？因为成员 r_ab_ 是 const float[3][3]，const 成员只能初始化、
// [解析] 不能在函数体里赋值。其他成员为统一起见也一起放在这里。
// [解析] r_ab_{ {...}, {...}, {...} } 是把调用者传进来的矩阵"按元素拷贝"一份，而不是存指针，
// [解析] 所以即使传入的是局部数组也安全。注意这一行和 bmi088.cpp 里的写法完全一样。
// [解析] 构造函数不碰硬件、不发 SPI，只是为了保存配置。
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
  // [解析] gyro_multipliers 是可选的（默认 nullptr）。判空决定用"标定系数"还是"不变倍率 1.0"。
  if (gyro_multipliers != nullptr) {
    gyro_multipliers_[0] = gyro_multipliers[0];
    gyro_multipliers_[1] = gyro_multipliers[1];
    gyro_multipliers_[2] = gyro_multipliers[2];
  } else {
    // [解析] 三轴都取 1.0 表示不做灵敏度修正（乘 1 等于不变），这是默认行为。
    gyro_multipliers_[0] = 1.0f;
    gyro_multipliers_[1] = 1.0f;
    gyro_multipliers_[2] = 1.0f;
  }
}

// [解析] init()：反复尝试配置芯片，直到成功为止。
// [解析] 与 bmi088.cpp 的 init() 相比，这里多了一个 osDelay(10)：
// [解析]   失败时先让出 CPU 10ms 再重试，避免"空转死循环"把同优先级任务饿死、
// [解析]   也避免疯狂刷 SPI 影响其他传感器。这是更稳妥的写法。
// [解析] 注意：如果传感器一直不在线，这里会无限重试下去（任务永远不往下走），
// [解析]   调试时最好先单独调用 device_init() 看看返回的错误码。
// [解析] 注意：osDelay 依赖 FreeRTOS 调度器已启动，所以 init() 要在内核启动后的任务里调用。
void ICM42688::init()
{
  while (device_init() != ICM42688_NO_ERROR) {
      osDelay(10); // 避免死循环卡死系统，加入适当延时等待
  }
}

// [解析] 真正干活的初始化函数：软复位 -> 查 WHO_AM_I -> 按表配置寄存器并回读校验。
// [解析] 返回值：ICM42688_NO_ERROR(0x00) 成功；ICM42688_NO_SENSOR(0xFF) 表示 ID 不对、
// [解析]   芯片不在线；其他值为配置表中某一步失败对应的错误码。
// [解析] 调用者（init()）会根据返回值决定是否重试。
uint8_t ICM42688::device_init()
{
  // 1. 软复位
  // [解析] 往 DEVICE_CONFIG(0x11) 写 0x01 触发软复位，让芯片回到出厂默认状态。
  // [解析] 0x01 是手册规定的复位命令位。
  write_register(ICM42688_DEVICE_CONFIG, ICM42688_SOFT_RESET_VALUE);
  osDelay(10); // 等待复位完成
  // [解析] 复位需要几个毫秒，手册建议至少等 1ms；这里给 10ms 留足余量（复位期间任何读写都无效）。

  // 2. 检查设备ID
  // [解析] WHO_AM_I(0x75) 是只读"身份证"寄存器，ICM42688 固定返回 0x47。
  read_registers(ICM42688_WHO_AM_I, 1);
  osDelay(1);

  // rx_buff_[0]是虚拟字节，rx_buff_[1]是实际数据
  // [解析] 与 BMI088 的加速度计不同，这里只跳 1 个无效字节：
  // [解析]   第 1 个字节是"发出去的地址 + 读标志"回环，第 2 个字节起就是真实数据。
  // [解析]   详见 read_registers() 的注释。
  uint8_t whoami = rx_buff_[1];
  // [解析] ID 不对就直接返回"没找到传感器"：可能是没焊、接线错、SPI 模式/速率配置不对。
  if (whoami != ICM42688_CHIP_ID_VALUE) return ICM42688_NO_SENSOR;

  // 3. 按照Table配置寄存器
  // [解析] 和 BMI088 相同的套路：写 -> 延时 -> 回读 -> 比对，能立刻发现"写了但没生效"的情况。
  // [解析] 比较时用 uint8_t 直接 !=，前提是表里这几个寄存器都是"可读可写、读回原值"的类型。
  uint8_t reg = 0, data = 0;
  for (uint8_t i = 0; i < ICM42688_INIT_TABLE_SIZE; i++) {
    reg = ICM42688_INIT_TABLE[i][0];

    write_register(reg, ICM42688_INIT_TABLE[i][1]);
    osDelay(2); // ICM42688 需要在配置电源后有短暂延时
    // [解析] 2ms 延时的原因：写 PWR_MGMT0 之后芯片内部电源域需要时间稳定，
    // [解析]   手册明确要求等 200us 以上；如果紧接着写量程寄存器会被忽略（回读校验就会失败）。

    read_registers(reg, 1);
    osDelay(1);

    data = rx_buff_[1];
    // [解析] 不一致就返回该行第 3 个元素里预设的错误码，方便定位是哪一步配错了。
    if (data != ICM42688_INIT_TABLE[i][1]) return ICM42688_INIT_TABLE[i][2];
  }

  return ICM42688_NO_ERROR;
}

void ICM42688::update()
{
  // [解析] update()：一次 SPI 事务就把温度、三轴加速度、三轴角速度全部读回来，效率最高。
  // [解析] ★难点三：突发读取(burst read)★ 芯片内部的这些数据寄存器地址是连续的，
  // [解析]   所以只要给出起始地址，芯片会自动递增地址、连续吐出后续字节，不用逐个寄存器去读。
  // [解析] 地址布局（从 0x1D 开始，共 14 字节，每 2 字节一个大端有符号数）：
  // [解析]   0x1D~0x1E 温度；0x1F~0x20 加速度X；0x21~0x22 加速度Y；0x23~0x24 加速度Z；
  // [解析]   0x25~0x26 角速度X；0x27~0x28 角速度Y；0x29~0x2A 角速度Z。
  // [解析] 注意每个量的"高字节在前"，所以下面拼接时要"高字节在左"。
  // 突发读取从 0x1D (温度起始) 到 0x2A (陀螺仪Z轴结束)，共计 14 个字节
  read_registers(ICM42688_TEMP_DATA1, 14);
  // [解析] 读完后 rx_buff_[0] 是地址占位（无效），rx_buff_[1] ~ rx_buff_[14] 才是那 14 个数据字节。

  // 第1个字节(rx_buff_[0]) 为寄存器地址占位，真实数据从rx_buff_[1]开始
  // ICM42688 传输是大端模式 (MSB First)
  // [解析] ★难点四：大端★ 与 BMI088 正好相反，ICM42688 是"高字节在前(MSB First)"：
  // [解析]   BMI088 读回来的第 1 个数据字节是低 8 位，而 ICM42688 的第 1 个是高 8 位。
  // [解析]   两边的代码都写成 (前一个字节 << 8) | 后一个字节，形式一样但含义相反，
  // [解析]   对照阅读时务必看清哪个下标才是高字节，写反了数值会完全乱掉。
  // [解析] 类型说明：rx_buff_ 是 uint8_t，参与 << 8 时先提升为 int，
  // [解析]   得到一个 0~65535 的无符号结果，再赋给 int16_t 时按 16 位截断，
  // [解析]   恰好等价于补码解释（例如 0xFF00 -> -256），所以负数不用手写符号处理。
  int16_t temp_int   = (rx_buff_[1] << 8)  | rx_buff_[2];
  
  // [解析] 温度原始值 -> 摄氏度：温度 = 原始值 / 132.48 + 25，原始值范围约 ±32767。
  // [解析] 加速度三个轴：下标 3~8 对应 0x1F~0x24，每轴"高字节在前"，拼成 int16 原始计数。
  int16_t acc_x_int  = (rx_buff_[3] << 8)  | rx_buff_[4];
  int16_t acc_y_int  = (rx_buff_[5] << 8)  | rx_buff_[6];
  int16_t acc_z_int  = (rx_buff_[7] << 8)  | rx_buff_[8];

  // [解析] 角速度三个轴：下标 9~14 对应 0x25~0x2A，同样高字节在前。
  int16_t gyro_x_int = (rx_buff_[9] << 8)  | rx_buff_[10];
  int16_t gyro_y_int = (rx_buff_[11] << 8) | rx_buff_[12];
  int16_t gyro_z_int = (rx_buff_[13] << 8) | rx_buff_[14];

  // 计算原始物理量
  // [解析] 加速度换算：原始计数 × (16/32768 g/LSB) × 9.8 = m/s^2。
  // [解析] 16G 量程下 1 LSB ≈ 4.787e-3 m/s^2，静止平放时重力方向那一轴读数约 ±9.8。
  float acc_x = acc_x_int * ICM42688_ACC_INT_TO_MPS2;
  float acc_y = acc_y_int * ICM42688_ACC_INT_TO_MPS2;
  float acc_z = acc_z_int * ICM42688_ACC_INT_TO_MPS2;

  // [解析] 角速度换算：原始计数 × (2000/32768) 得到 °/s，再 × (π/180) 得到 rad/s，
  // [解析]   最后乘 gyro_multipliers_ 做三轴灵敏度微调（标定系数，默认 1.0 等于不修正）。
  // [解析] 注意这里没有对加速度乘 gyro_multipliers_，标定系数只作用于陀螺仪。
  float gyro_x = gyro_x_int * ICM42688_GYRO_INT_TO_RPS * gyro_multipliers_[0];
  float gyro_y = gyro_y_int * ICM42688_GYRO_INT_TO_RPS * gyro_multipliers_[1];
  float gyro_z = gyro_z_int * ICM42688_GYRO_INT_TO_RPS * gyro_multipliers_[2];

  // 坐标系转换 (应用传入的旋转矩阵)
  // [解析] 把传感器坐标系 {b} 下的向量左乘旋转矩阵 r_ab_，得到机器人坐标系 {a} 下的向量。
  // [解析] 展开就是矩阵乘列向量：out[i] = Σ_j r_ab_[i][j] * in[j]。
  // [解析] 加速度和陀螺仪用同一个矩阵，因为它们在同一颗芯片内部，三轴朝向完全一致。
  // [解析] 若 IMU 安装方向和机器人坐标系一致，r_ab_ 就是单位矩阵，这几行等于原样赋值。
  this->acc[0] = r_ab_[0][0] * acc_x + r_ab_[0][1] * acc_y + r_ab_[0][2] * acc_z;
  this->acc[1] = r_ab_[1][0] * acc_x + r_ab_[1][1] * acc_y + r_ab_[1][2] * acc_z;
  this->acc[2] = r_ab_[2][0] * acc_x + r_ab_[2][1] * acc_y + r_ab_[2][2] * acc_z;

  this->gyro[0] = r_ab_[0][0] * gyro_x + r_ab_[0][1] * gyro_y + r_ab_[0][2] * gyro_z;
  this->gyro[1] = r_ab_[1][0] * gyro_x + r_ab_[1][1] * gyro_y + r_ab_[1][2] * gyro_z;
  this->gyro[2] = r_ab_[2][0] * gyro_x + r_ab_[2][1] * gyro_y + r_ab_[2][2] * gyro_z;

  // [解析] 温度是标量，不做坐标系变换，直接套公式：温度 = 原始值 / 132.48 + 25（单位 ℃）。
  this->temp = temp_int * ICM42688_TEMP_FACTOR + ICM42688_TEMP_OFFSET;
}

// [解析] 从寄存器 reg 开始，连续读 len 个字节，结果放进成员 rx_buff_。
// [解析] 参数：reg = 起始寄存器地址；len = 要读的字节数（调用者负责别超缓冲区，最多 19）。
// [解析] 返回值：无。有效数据位于 rx_buff_[1] ~ rx_buff_[len]，rx_buff_[0] 是无效的占位字节。
// [解析] 副作用：操作 CS 引脚、占用 SPI 总线；HAL 的返回值和超时被有意忽略（没写判断）。
// [解析] ★为什么只有 1 个无效字节★
// [解析]   SPI 是全双工：主机把地址发出去的同时就在收数据。发第 1 个字节时芯片还在解析命令，
// [解析]   它送出的内容是无效的（地址回环或随机值），所以真实数据从第 2 个字节开始。
// [解析]   这比 BMI088 的加速度计少一个 dummy 字节，正是两颗芯片时序规定的差异。
void ICM42688::read_registers(uint8_t reg, uint8_t len)
{
  // [解析] 片选拉低 = 选中这颗芯片，开始一次 SPI 事务（写寄存器时同样要拉低）。
  HAL_GPIO_WritePin(cs_port_, cs_pin_, GPIO_PIN_RESET);

  // [解析] 0x80 = 1000_0000，既是地址最高位，也是 ICM42688 的"读/写"标志位：
  // [解析]   最高位 = 1 表示读，= 0 表示写，所以读的时候要把地址和 0x80 按位或。
  // [解析] 这种"用最高位当读写标志"的约定在 SPI 传感器里很常见（BMI088 也一样），
  // [解析]   但也有器件是反过来的，看新芯片手册时务必确认，别想当然。
  tx_buff_[0] = reg | 0x80; // 最高位设为1代表读取

  // 读取总长度是 寄存器地址字节(1) + 数据长度(len)
  // [解析] TransmitReceive 一次完成收发：MOSI 送出地址字节 + len 个空字节，
  // [解析]   MISO 同步收回 len+1 个字节（第 1 个无效，后面 len 个是数据）。
  // [解析] 最后的 100 是 HAL 超时时间，单位毫秒；超时会有错误返回码，但这里不检查。
  // [解析] 注意：tx_buff_[1..len] 没有被赋值，发出去的是上次遗留的数据；
  // [解析]   对读操作没影响（芯片只解析第一个字节的地址），所以作者没有清零。
  HAL_SPI_TransmitReceive(hspi_, tx_buff_, rx_buff_, len + 1, 100);

  HAL_GPIO_WritePin(cs_port_, cs_pin_, GPIO_PIN_SET);
}

// [解析] 向寄存器 reg 写入 1 个字节 data。
// [解析] 返回值：无（不检查写入是否成功；写校验由 device_init() 里的"回读比对"完成）。
// [解析] 与读的区别：地址不加 0x80（最高位 0 = 写），只发不收，长度固定 2 字节。
void ICM42688::write_register(uint8_t reg, uint8_t data)
{
  HAL_GPIO_WritePin(cs_port_, cs_pin_, GPIO_PIN_RESET);

  // [解析] 原样写地址，最高位保持 0，芯片就理解为"写操作"。
  tx_buff_[0] = reg;       // 最高位设为0代表写入
  tx_buff_[1] = data;

  HAL_SPI_Transmit(hspi_, tx_buff_, 2, 100);
  // [解析] 只发不收：用 Transmit 发 2 个字节（1 个地址 + 1 个数据），超时同样 100ms。
  // [解析] 因为不需要 MISO 数据，所以比读操作少一次无效字节的烦恼。

  HAL_GPIO_WritePin(cs_port_, cs_pin_, GPIO_PIN_SET);
}

// [解析] 文件结束。ICM42688 的操作口诀：
// [解析]   片选拉低 -> 发地址（读则 |0x80）-> 收发数据 -> 片选拉高；
// [解析]   读回的第 1 个字节丢弃，之后按"高字节在前（大端）"拼成 int16，
// [解析]   再乘标度因数得到物理量，最后左乘 r_ab_ 转到机器人坐标系。
// [解析] 提醒：上面两个函数最后的 HAL_GPIO_WritePin(..., GPIO_PIN_SET) 都是"片选拉高、结束事务"，
// [解析]   千万不能漏；否则芯片内部地址指针不会复位，下一次读写会从错误的位置开始。
}  // namespace sp