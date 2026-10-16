// [解析] ==========================================================================
// [解析] 【文件】sp_middleware/io/bmi088/bmi088.cpp
// [解析] 【作用】BMI088 驱动的具体实现：SPI 读写寄存器、初始化配置表、原始值→物理量换算、
// [解析]         传感器坐标系→机器人坐标系的旋转。所有"看不懂的数字"都在这里解释。
// [解析] 【被谁用】由 bmi088.hpp 里的 BMI088 类调用；最终被 applications 里的
// [解析]         INS/姿态解算任务周期性调用 update()。
// [解析] 【看代码的重点】四个最容易卡住的地方，下面都逐行标注了：
// [解析]   1) 上电时加速度计默认是 I2C 模式，必须先"拉一次片选"骗它切到 SPI 模式；
// [解析]   2) 读寄存器时那个 0x80（读标志）以及"前 1/2 个字节是无效数据"是怎么回事；
// [解析]   3) 标度因数怎么算出来的（为什么 6G 对应 6/32768），这是全文件最核心的换算；
// [解析]   4) 16 位小端字节拼接 + 符号位，以及温度只有 11 位有效位怎么处理。
// [解析] 【阅读顺序】建议先看 acc_read/acc_write 理解"怎么和芯片说话"，
// [解析]         再看 acc_init 理解"要配置哪些寄存器"，最后看 acc_update 理解"怎么换算"。
// [解析] ==========================================================================

#include "bmi088.hpp"

#include <cstring>

#include "bmi088_defs.h"
#include "cmsis_os.h"
#include "tools/math_tools/math_tools.hpp"
// [解析] 上面几个头文件的作用：
// [解析]   bmi088.hpp      —— 本类的声明；
// [解析]   <cstring>       —— memcpy/memset 之类（注意：本文件中其实没有直接用到）；
// [解析]   bmi088_defs.h   —— 寄存器地址和配置值宏；
// [解析]   cmsis_os.h      —— 提供 osDelay()（FreeRTOS 的毫秒级延时），复位后要等芯片；
// [解析]   math_tools.hpp —— 提供常量 sp::SP_PI（圆周率）。

// TODO 构造时配置
// [解析] 本项目固定用 6G 量程 + 2000dps 量程，是"编译期常量"（constexpr），
// [解析] 也就是说想改量程必须改代码重新编译，不能运行时切换。
// [解析] 6G：机器人落地冲击、弹丸后坐力都不会削顶，同时保留足够分辨率；
// [解析] 2000dps：小陀螺高速旋转时角速度可能上千 °/s，2000dps 才够用。
constexpr uint8_t BMI088_ACC_RANGE_SET = BMI088_ACC_RANGE_6G;
constexpr uint8_t BMI088_GYRO_RANGE_SET = BMI088_GYRO_2000;

// ref: datasheet 5.3.4
// [解析] 重力加速度。注意：标准值是 9.80665，这里用了 9.8（精度差约 0.07%，工程上可接受）。
constexpr float G_TO_MPS2 = 9.8f;
// [解析] ★全文件最重要的换算★ 加速度"原始值 -> g"的标度因数（每 1 个 LSB 代表多少 g）。
// [解析] 推导：BMI088 的加速度输出是 16 位有符号数，满量程对应 ±32768。
// [解析]   而寄存器里写的量程编码 n（0=3g,1=6G,2=12G,3=24G）满足：满量程 = 1.5 * 2^(n+1) g。
// [解析]   验证：n=0 -> 1.5*2=3g；n=1 -> 1.5*4=6g；n=2 -> 12g；n=3 -> 24g。完全对得上。
// [解析]   所以"1 个 LSB = 满量程/32768 = 1.5*2^(n+1)/32768" g。
// [解析] 本项目 n=BMI088_ACC_RANGE_6G=1，代入得 (1<<2)*1.5/32768 = 6/32768 ≈ 1.831e-4 g/LSB。
// [解析] 从寄存器读到的原始整数只要乘以这个数，就得到以 g 为单位的加速度。
constexpr float BMI088_ACC_INT_TO_G = (1 << (BMI088_ACC_RANGE_SET + 1)) * 1.5f / 32768;
// [解析] 再乘 9.8，就把 g 换成国际单位 m/s^2（acc[] 对外承诺的单位）。
constexpr float BMI088_ACC_INT_TO_MPS2 = BMI088_ACC_INT_TO_G * G_TO_MPS2;

// ref: datasheet 5.5.2
// [解析] 陀螺仪"原始值 -> °/s(度每秒)"的标度因数。
// [解析] 数据手册的规律是：满量程 = 2000 / 2^n，其中 n 就是头文件里的量程编码
// [解析]   （0=2000dps、1=1000dps、2=500dps、3=250dps、4=125dps），所以严格写法应是
// [解析]   2000.0f / (1 << BMI088_GYRO_RANGE_SET) / 32767。
// [解析] 现有写法 2000.0f / (BMI088_GYRO_RANGE_SET + 1) 只在 n=0、n=1 时与手册一致：
// [解析]   n=2 会算出 666.7 而不是 500，n=4 会算出 400 而不是 125。
// [解析] 本项目固定用 n=0（2000dps），此时 (n+1)=1，结果 2000/32767 完全正确，
// [解析] 所以当前不会有问题；但若将来改成 500dps 等更小量程，这里的标度因数会偏大，必须一起改。
// [解析] 除以 32767 是因为数据手册给出的灵敏度按 ±32767 标定。加速度那边除的是 32768，
// [解析] 两者差 1 个 LSB 的 0.003%，属手册自身写法，不影响使用。
constexpr float BMI088_GYRO_INT_TO_DPS = 2000.0f / (BMI088_GYRO_RANGE_SET + 1) / 32767;
// [解析] °/s -> rad/s：先除以 180 变成"圈/秒"的分数，再乘 π 得到弧度。
// [解析] 例如 180°/s 对应 π rad/s ≈ 3.1416 rad/s。gyro[] 对外承诺的单位就是 rad/s。
constexpr float BMI088_GYRO_INT_TO_RPS = BMI088_GYRO_INT_TO_DPS / 180 * sp::SP_PI;

// ref: datasheet 5.3.7
// [解析] 温度换算公式：温度(℃) = 原始值 * 0.125 + 23。
// [解析] 0.125 = 1/8，即温度的分辨率是 0.125℃/LSB；23 是零点偏移（原始值 0 对应 23℃）。
constexpr float BMI088_TEMP_FACTOR = 0.125f;
constexpr float BMI088_TEMP_OFFSET = 23.0f;

// [解析] 初始化表的行数。用 constexpr 而不是硬编码 6，是为了数组定义和循环上限保持同步：
// [解析] 以后往表里增删一行，只改这一个数字即可（改小了会漏配置，改大了会数组越界！）。
constexpr size_t BMI088_ACC_INIT_TABLE_SIZE = 6;
constexpr size_t BMI088_GYRO_INIT_TABLE_SIZE = 6;

// [解析] clang-format off/on 是给代码格式化工具看的："这两行之间的内容不要自动排版"。
// [解析] 因为下面两张表的列是靠空格手动对齐的，让格式化工具重排反而看不清对应关系。
// clang-format off

// [解析] ★加速度计初始化表★ 每行 3 个元素，含义固定为：
// [解析]   [0] 寄存器地址（写哪个寄存器）
// [解析]   [1] 要写入的值（写什么）
// [解析]   [2] 如果"写进去再读回来"对不上，返回哪个错误码
// [解析] acc_init() 会按顺序执行这 6 步，任何一步失败就立刻返回该行的错误码。
// [解析] 逐行含义（这也是标准手册推荐的上电配置顺序，顺序不能随意调换）：
// [解析]   1) ACC_PWR_CTRL=0x04 ：先给加速度计上电（不开电源后面所有寄存器都写不进去）；
// [解析]   2) ACC_PWR_CONF=0x00 ：从挂起模式切到正常工作模式；
// [解析]   3) ACC_CONF=0x80|BWP_NORMAL|ODR_800 ：0x80 是必须置 1 的保留位，
// [解析]      BWP_NORMAL 选常规滤波带宽，ODR_800 表示 800Hz 输出数据率；
// [解析]   4) ACC_RANGE=1      ：量程 ±6g（要和上面算标度因数用的 BMI088_ACC_RANGE_SET 一致！）；
// [解析]   5) INT1_IO_CONF    ：把 INT1 配成"输出使能 + 推挽 + 低电平有效"，
// [解析]      这样数据准备好时 INT1 会拉低，单片机用它触发 SPI 读数据（或接外部中断）；
// [解析]   6) INT1_INT2_MAP_DATA ：把 DRDY(数据就绪)事件映射到 INT1 引脚上。
// [解析] 注意：如果以后修改了这里的 ODR，上层任务的调用频率也要跟着改，
// [解析]       否则读到的数据会重复或丢帧。
const uint8_t BMI088_ACC_INIT_TABLE[BMI088_ACC_INIT_TABLE_SIZE][3] = {
  {BMI088_ACC_PWR_CTRL      , BMI088_ACC_ENABLE_ACC_ON                                                       , BMI088_ACC_PWR_CTRL_ERROR      },
  {BMI088_ACC_PWR_CONF      , BMI088_ACC_PWR_ACTIVE_MODE                                                     , BMI088_ACC_PWR_CONF_ERROR      },
  {BMI088_ACC_CONF          , BMI088_ACC_BWP_NORMAL | BMI088_ACC_ODR_800_HZ | BMI088_ACC_CONF_MUST_SET       , BMI088_ACC_CONF_ERROR          },
  {BMI088_ACC_RANGE         , BMI088_ACC_RANGE_SET                                                           , BMI088_ACC_RANGE_ERROR         },
  {BMI088_INT1_IO_CONF      , BMI088_ACC_INT1_OUT_ENABLE | BMI088_ACC_INT1_GPIO_PP | BMI088_ACC_INT1_GPIO_LOW, BMI088_INT1_IO_CONF_ERROR      },
  {BMI088_INT1_INT2_MAP_DATA, BMI088_DRDY_TO_INT1                                                            , BMI088_INT1_INT2_MAP_DATA_ERROR}
};

// [解析] ★陀螺仪初始化表★ 结构同上：[寄存器, 写入值, 失败错误码]，按顺序执行 6 步：
// [解析]   1) GYRO_RANGE=0x00    ：量程 ±2000°/s（编码越小量程越大）；
// [解析]   2) GYRO_BANDWIDTH    ：0x80 是必须置 1 的保留位，
// [解析]      低 4 位 0x01 = BMI088_GYRO_2000_230_HZ，表示 2000dps 量程下把低通滤波
// [解析]      截止频率设为 230Hz —— 用来滤掉车架高频振动，否则角速度噪声会毁掉姿态解算；
// [解析]   3) GYRO_LPM1=0x00     ：正常模式（不是挂起/深度挂起）；
// [解析]   4) GYRO_INT_CTRL=0x80 ：打开 DRDY 数据就绪中断；
// [解析]   5) INT3_INT4_IO_CONF  ：把 INT3 配成推挽输出 + 低电平有效；
// [解析]   6) INT3_INT4_IO_MAP   ：把 DRDY 事件映射到 INT3 引脚。
// [解析] 注意：第 2 步的带宽编码和量程是绑定的，如果改了量程却忘了改带宽编码，
// [解析]       滤波频率就和你以为的不一样了。
const uint8_t BMI088_GYRO_INIT_TABLE[BMI088_GYRO_INIT_TABLE_SIZE][3] = {
  {BMI088_GYRO_RANGE       , BMI088_GYRO_RANGE_SET                                   , BMI088_GYRO_RANGE_ERROR       },
  {BMI088_GYRO_BANDWIDTH   , BMI088_GYRO_2000_230_HZ | BMI088_GYRO_BANDWIDTH_MUST_SET, BMI088_GYRO_BANDWIDTH_ERROR   },
  {BMI088_GYRO_LPM1        , BMI088_GYRO_NORMAL_MODE                                 , BMI088_GYRO_LPM1_ERROR        },
  {BMI088_GYRO_INT_CTRL    , BMI088_DRDY_ON                                          , BMI088_GYRO_CTRL_ERROR        },
  {BMI088_INT3_INT4_IO_CONF, BMI088_INT3_GPIO_PP | BMI088_INT3_GPIO_LOW              , BMI088_INT3_INT4_IO_CONF_ERROR},
  {BMI088_INT3_INT4_IO_MAP , BMI088_DRDY_IO_INT3                                     , BMI088_INT3_INT4_IO_MAP_ERROR }
};

// [解析] 恢复自动格式化。
// clang-format on

namespace sp
{
// [解析] 构造函数：初始化列表（冒号后面那一串）就是"给成员变量赋值"的写法。
// [解析] 为什么必须用初始化列表而不能在函数体里写 hspi_ = hspi？
// [解析] 因为成员 r_ab_ 是 const float[3][3]（常量数组），C++ 规定 const 成员只能初始化、
// [解析] 不能赋值，所以只能写在冒号后面。为了风格统一，其他成员也都放这里。
// [解析] r_ab_{ {...}, {...}, {...} } 是把外面的二维数组"逐个元素拷贝"进来，
// [解析] 而不是保存指针 —— 这样即使调用者传的是局部数组（函数返回后就失效），本对象依然有效。
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
  // [解析] gyro_multipliers 是"可选的"第 7 个参数（默认 nullptr）。用指针判空区分两种情况。
  // [解析] 注意：与 r_ab_ 不同，gyro_multipliers_ 是普通 float 数组，所以可以在函数体里赋值。
  if (gyro_multipliers != nullptr) {
    gyro_multipliers_[0] = gyro_multipliers[0];
    gyro_multipliers_[1] = gyro_multipliers[1];
    gyro_multipliers_[2] = gyro_multipliers[2];
  }
  else {
    // 默认不校准
    // [解析] 1.0 表示"原样输出、不做任何缩放"。如果以后做了三轴标定，
    // [解析] 把测出来的修正系数传进来，就能补偿各轴灵敏度的微小差异。
    gyro_multipliers_[0] = 1.0f;
    gyro_multipliers_[1] = 1.0f;
    gyro_multipliers_[2] = 1.0f;
  }
}

// [解析] init()：把两颗芯片都配置好才返回。
// [解析] while (xxx() != BMI088_NO_ERROR); 是"空循环体 + 失败就无限重试"的写法：
// [解析]   分号表示循环体什么都不做，靠 acc_init()/gyro_init() 的副作用（写寄存器）推进。
// [解析] 注意：如果传感器没插好或 SPI 配置错误，这里会永远卡住，而且没有任何延时，
// [解析]   会以最快的速度反复刷 SPI（其他任务还能跑，但日志会被刷爆）。
// [解析]   调试建议：先单独调用 acc_init() 打印返回值，确认错误码后再放开这个循环。
// [解析] 注意：acc_init()/gyro_init() 内部用了 osDelay()，而 osDelay 依赖 FreeRTOS 调度器
// [解析]   已经启动。也就是说 init() 必须在 osKernelStart() 之后的某个任务里调用，
// [解析]   在 main() 里、调度器启动前调用会出问题。
void BMI088::init()
{
  while (acc_init() != BMI088_NO_ERROR);
  while (gyro_init() != BMI088_NO_ERROR);
}

// [解析] update()：一次刷新全部对外数据。没有返回值，结果直接写进 acc/gyro/temp。
// [解析] 顺序上先读加速度（顺带读温度）再读陀螺仪，两者互相独立，先读谁都可以。
// [解析] 调用频率受限于芯片配置的 ODR（加速度计 800Hz、陀螺仪约 1kHz），
// [解析] 调用太快只会读到重复数据，调用太慢则会丢数据。
void BMI088::update()
{
  acc_update();
  gyro_update();
}

// -------------------- 加速度计 --------------------

// [解析] 配置加速度计芯片。返回值：BMI088_NO_ERROR(0) 表示全部成功，
// [解析] 否则是具体某一步的错误码（BMI088_NO_SENSOR=0xFF 表示根本没读到正确的 ID）。
// [解析] ★难点一：上电默认是 I2C 模式★
// [解析]   加速度计芯片上电后默认工作在 I2C 模式，而本项目的硬件接的是 SPI。
// [解析]   数据手册规定：只要检测到片选 CSB1 出现一次"上升沿"，芯片就永久切到 SPI 模式。
// [解析]   所以代码里要"随便读一次"，目的不是取数据，而是制造这次上升沿
// [解析]   （acc_read 内部会先拉低再拉高 CSB1）。这也解释了为什么同一个寄存器要读两三次。
// [解析] 每一步后面都有 osDelay(1)：给芯片 1ms（FreeRTOS 的 1 个 tick）时间完成内部动作，
// [解析] 不给延时的话芯片可能还没醒过来，紧接着的读写就会失败。
uint8_t BMI088::acc_init()
{
  // 上电后, 加速度计以I2C模式启动, 需要一次CSB1上升沿来切换到SPI模式, ref: datasheet 6.1
  acc_read(BMI088_ACC_CHIP_ID, 1);
  osDelay(1);

  // 软复位
  // [解析] 写 0xB6 到 0x7E 让芯片回到出厂默认状态，避免上一次运行（比如复位前）留下的怪配置。
  acc_write(BMI088_ACC_SOFTRESET, BMI088_ACC_SOFTRESET_VALUE);
  osDelay(1);

  // 软复位后, 加速度计以I2C模式启动, 需要一次CSB1上升沿来切换到SPI模式
  // [解析] 复位会把"I2C/SPI 模式"也一起重置回 I2C，所以必须再制造一次上升沿切回 SPI。
  acc_read(BMI088_ACC_CHIP_ID, 1);
  osDelay(1);

  // 读取ID
  // [解析] 这次才是真正为了取 ID 而读。注意前两次读的结果都被丢掉了。
  acc_read(BMI088_ACC_CHIP_ID, 1);
  osDelay(1);

  // 检查ID, 前两个字节为无效数据
  // [解析] 为什么是 rx_buff_[2] 而不是 [0]？因为加速度计一次读会收到两个"无效字节"：
  // [解析]   第 1 个是你发出去的地址字节（回环回来），第 2 个是芯片要求的 dummy 字节，
  // [解析]   真正的数据从第 3 个字节（下标 2）才开始。详见 acc_read()。
  uint8_t whoami = rx_buff_[2];
  // [解析] 正确值应为 0x1E。不等于就说明芯片没焊好/接线错/SPI 模式没切过来，直接上报"无传感器"。
  if (whoami != BMI088_ACC_CHIP_ID_VALUE) return BMI088_NO_SENSOR;

  // 配置寄存器
  // [解析] 按初始化表逐行执行：写 -> 等 1ms -> 回读 -> 比对。
  // [解析] "回读比对"是很重要的自检手段：SPI 是全双工，写的时候其实也在收数据，
  // [解析] 如果写根本没生效（比如片选接错、芯片没上电），回读的值就对不上，能立刻发现。
  uint8_t reg = 0, data = 0;
  for (uint8_t i = 0; i < BMI088_ACC_INIT_TABLE_SIZE; i++) {
    reg = BMI088_ACC_INIT_TABLE[i][0];

    acc_write(reg, BMI088_ACC_INIT_TABLE[i][1]);
    osDelay(1);

    acc_read(reg, 1);
    osDelay(1);

    // 检查是否写入成功, 前两个字节为无效数据
    // [解析] 同样是跳过 2 个无效字节后取数据。
    // [解析] 注意：有些寄存器读回来可能夹带状态位（不是纯粹的写入值），
    // [解析]   本表选用的寄存器都能原样读回，所以可以直接用 != 判断。
    data = rx_buff_[2];
    // [解析] 不匹配就把这一行预先写好的错误码返回给上层，便于定位是哪一步失败的。
    if (data != BMI088_ACC_INIT_TABLE[i][1]) return BMI088_ACC_INIT_TABLE[i][2];
  }

  return BMI088_NO_ERROR;
}

// [解析] 读一次加速度计（并顺便读温度），完成"原始值 -> 物理量 -> 机器人坐标系"的全过程。
// [解析] 输出写到成员 acc[3] 和 temp；没有返回值。
// [解析] 三个关键知识点：
// [解析]   1) 小端拼接：芯片先发低字节、后发高字节，所以要"高字节左移 8 位再或上低字节"；
// [解析]   2) 补码：拼出来的 16 位数赋给 int16_t，负数会自然变成负的浮点数，不用手写符号处理；
// [解析]   3) 坐标系变换：acc[] 是"机器人坐标系"下的值，靠乘 r_ab_ 矩阵实现。
void BMI088::acc_update()
{
  // 接收加速度计数据
  // [解析] 从 0x12 开始连续读 6 个字节 = X/Y/Z 三轴，每轴 2 字节。
  acc_read(BMI088_ACC_DATA, 6);

  // 前两个字节为无效数据
  // [解析] 下标偏移 +2 的原因：前两个字节无效（地址回环 + dummy）。
  // [解析] (高字节 << 8) | 低字节 —— 这就是"小端"（Little Endian）拼接：
  // [解析]   低地址存低字节。BMI088 加速度计就是小端输出，和陀螺仪一样。
  // [解析] 注意移位与赋值时的类型变化：rx_buff_ 是 uint8_t，参与运算时先被提升为 int，
  // [解析]   左移 8 位后按位或，得到一个 0~65535 的 int；再赋给 int16_t 时按 16 位截断，
  // [解析]   恰好等价于"把补码解释成有符号数"。例如 0xFFFF 会变成 -1。
  int16_t acc_x_int = (rx_buff_[2 + 1] << 8) | rx_buff_[2 + 0];
  int16_t acc_y_int = (rx_buff_[2 + 3] << 8) | rx_buff_[2 + 2];
  int16_t acc_z_int = (rx_buff_[2 + 5] << 8) | rx_buff_[2 + 4];
  // [解析] 乘标度因数完成"LSB -> m/s^2"换算。6G 量程下每 1 LSB ≈ 1.831e-4 g ≈ 1.794e-3 m/s^2。
  // [解析] 静止平放时，z 轴读数约 ±9.8（具体正负取决于 r_ab 定义的机器人 z 轴朝向）。
  float acc_x = acc_x_int * BMI088_ACC_INT_TO_MPS2;
  float acc_y = acc_y_int * BMI088_ACC_INT_TO_MPS2;
  float acc_z = acc_z_int * BMI088_ACC_INT_TO_MPS2;

  // 前两个字节为无效数据
  // [解析] 温度是 2 字节寄存器（0x22、0x23），所以 len 传 2。
  // [解析] 注意：这次读会覆盖 rx_buff_ 的前 4 个字节，但因为 acc_x/y/z 已经算完了，所以没影响。
  acc_read(BMI088_TEMP_DATA, 2);
  // [解析] 温度的坑：它只有 11 位有效位，不是完整的 16 位。
  // [解析]   寄存器 0x22 是完整的 8 位（高 8 位），0x23 只有"高 3 位"是有效低位，
  // [解析]   所以拼法不是普通的 << 8，而是：高字节 << 3 | （第二字节 >> 5）。
  // [解析]   结果范围 0~2047，对应温度 -40℃ ~ +85℃ 左右，分辨率 0.125℃。
  int16_t temp_int = (rx_buff_[2 + 0] << 3) | (rx_buff_[2 + 1] >> 5);
  // [解析] 11 位补码的"符号扩展"：值 ≥1024（最高位为 1）说明是负数，
  // [解析]   减去 2^11=2048 就得到正确的负值（例如 2047 -> -1）。
  // [解析] 用 > 1023 判断等价于 >= 1024，边界正确。
  if (temp_int > 1023) temp_int -= 2048;  // ref: 5.3.7

  // 更新公开属性
  // [解析] 坐标系变换：把传感器坐标系 {b} 下的三轴向量，左乘旋转矩阵 r_ab_ 得到机器人坐标系 {a} 下的向量。
  // [解析] 展开就是标准的"矩阵 × 列向量"：acc_a[i] = Σ_j r_ab_[i][j] * acc_b[j]。
  // [解析] 如果 IMU 的 x/y/z 正好和机器人一致，r_ab_ 就是单位矩阵，这三行等于原样拷贝。
  this->acc[0] = r_ab_[0][0] * acc_x + r_ab_[0][1] * acc_y + r_ab_[0][2] * acc_z;
  this->acc[1] = r_ab_[1][0] * acc_x + r_ab_[1][1] * acc_y + r_ab_[1][2] * acc_z;
  this->acc[2] = r_ab_[2][0] * acc_x + r_ab_[2][1] * acc_y + r_ab_[2][2] * acc_z;
  // [解析] 温度是标量，不需要做坐标系变换，直接按 温度 = 原始值*0.125 + 23 换算即可。
  this->temp = temp_int * BMI088_TEMP_FACTOR + BMI088_TEMP_OFFSET;
}

// [解析] 从加速度计读取 len 个字节，结果放进成员缓冲区 rx_buff_。
// [解析] 参数：reg = 起始寄存器地址（8 位，0x00~0x7F）；len = 要读多少字节（调用者保证 ≤ 14）。
// [解析] 返回值：无。结果通过 rx_buff_ 传出，注意有效数据从 rx_buff_[2] 才开始！
// [解析] 副作用：操作 CSB1 片选电平、发起一次 SPI 收发。只在任务上下文调用。
// [解析] ★难点二：为什么读回来前两个字节是无效的★
// [解析]   SPI 是全双工：主机发 1 个字节的同时也在收 1 个字节。一次"读寄存器"的时序是：
// [解析]     第 1 字节：主机把"地址 + 读标志"发出去，此时 rx_buff_[0] 收到的是回环/垃圾数据；
// [解析]     第 2 字节：芯片需要额外一个时钟周期准备数据，所以是 dummy（空转）字节，也无效；
// [解析]     第 3 字节起：才是真正的寄存器内容。
// [解析]   所以总长度要发 len + 2，数据从下标 2 开始取。
void BMI088::acc_read(uint8_t reg, uint8_t len)
{
  // 拉低CSB1
  // [解析] 片选低电平有效：拉低表示"我在跟加速度计说话"，陀螺仪因为 CSB2 是高电平会忽略总线。
  // [解析] 这也是为什么两颗芯片能共用同一条 SPI：靠片选区分对象。
  HAL_GPIO_WritePin(csb1_port_, csb1_pin_, GPIO_PIN_RESET);

  // ref: datasheet 6.1.2
  // [解析] 0x80 = 二进制 1000_0000，即地址的最高位。加速度计规定：最高位=1 表示"读"，
  // [解析]   最高位=0 表示"写"。所以读操作要把地址和 0x80 按位或。
  // [解析] 注意：reg 本身不能超过 0x7F，否则会和这个标志位打架。
  tx_buff_[0] = reg | 0x80;

  // 前两个字节为无效数据
  // [解析] 收发一次性完成：tx_buff_ 里的内容从 MOSI 出去，同时 MISO 上的数据被写进 rx_buff_。
  // [解析] len+2 = 1 个地址字节 + 1 个 dummy 字节 + len 个数据字节。
  // [解析] 最后一个参数 100 是 HAL 的超时时间，单位毫秒；超时了 HAL 会返回错误，
  // [解析]   但这里故意不检查返回值（出问题时表现为数据是旧值，而不是报错）。
  // [解析] 注意：tx_buff_[1]（dummy 字节）没有被赋值，内容是上一次的残留。
  // [解析]   对 BMI088 来说 dummy 字节发什么都无所谓，所以不影响功能。
  HAL_SPI_TransmitReceive(hspi_, tx_buff_, rx_buff_, len + 2, 100);

  // 拉高CSB1
  // [解析] 必须拉高！片选拉高表示一次通信结束，芯片内部地址计数器复位；
  // [解析] 同时这个上升沿也有"切换 I2C/SPI 模式"的作用（见 acc_init 的说明）。
  // [解析] 如果忘了拉高，下一次读会从上次的地址继续，数据全错。
  HAL_GPIO_WritePin(csb1_port_, csb1_pin_, GPIO_PIN_SET);
}

// [解析] 向加速度计的一个寄存器写入 1 个字节。
// [解析] 参数：reg = 寄存器地址（0x00~0x7F）；data = 要写入的值。
// [解析] 返回值：无。不会回读校验（校验是 acc_init 里额外做的）。
// [解析] 与 acc_read 的区别：只发不收，用 HAL_SPI_Transmit；地址不加 0x80（最高位 0 = 写）。
void BMI088::acc_write(uint8_t reg, uint8_t data)
{
  // 拉低CSB1
  HAL_GPIO_WritePin(csb1_port_, csb1_pin_, GPIO_PIN_RESET);

  // ref: datasheet 6.1.2
  // [解析] 写时序只有 2 个字节：第 1 个是地址（最高位 0，不加 0x80），第 2 个是数据。
  tx_buff_[0] = reg;
  tx_buff_[1] = data;

  // 只发不收
  // [解析] Transmit 只往 MOSI 上发，不关心 MISO；长度固定 2 字节，超时同样 100ms。
  // [解析] 注意：有些芯片写寄存器后需要一段时间才生效，所以 acc_init 里每次写完都 osDelay(1)。
  HAL_SPI_Transmit(hspi_, tx_buff_, 2, 100);

  // 拉高CSB1
  HAL_GPIO_WritePin(csb1_port_, csb1_pin_, GPIO_PIN_SET);
}

// -------------------- 陀螺仪 --------------------

// [解析] 配置陀螺仪芯片。返回值含义同 acc_init()。
// [解析] 与加速度计的两点重要区别（新手最容易搞混）：
// [解析]   1) 陀螺仪不需要"制造上升沿切 SPI 模式"。只有加速度计上电默认 I2C；
// [解析]      所以这里没有 acc_init 里那两次"为了切换模式而读一次"的操作。
// [解析]   2) 读回来的有效数据从 rx_buff_[1] 开始（只有 1 个无效字节），不是 [2]。
// [解析]      原因见 gyro_read()：陀螺仪读时序只多一个 dummy 字节。
uint8_t BMI088::gyro_init()
{
  // 软复位
  // [解析] 写 0xB6 到 0x14，让陀螺仪回到出厂默认状态。延时 1ms 等它复位完成。
  gyro_write(BMI088_GYRO_SOFTRESET, BMI088_GYRO_SOFTRESET_VALUE);
  osDelay(1);

  // 读取ID
  // [解析] 读 0x00 寄存器拿"身份证"，正确值是 0x0F。
  gyro_read(BMI088_GYRO_CHIP_ID, 1);
  osDelay(1);

  // 检查ID, 第一个字节为无效数据
  // [解析] 注意是 rx_buff_[1]，不是加速度计那样的 [2]。这个差异源自两种读时序的 dummy 字节数不同，
  // [解析]   写代码时如果抄错，就会把"回环的地址字节"当成 ID，导致永远初始化失败。
  uint8_t whoami = rx_buff_[1];
  if (whoami != BMI088_GYRO_CHIP_ID_VALUE) return BMI088_NO_SENSOR;

  // 配置寄存器
  // [解析] 流程和加速度计完全一样：写 -> 延时 -> 回读 -> 比对，失败就返回该行预设的错误码。
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

// [解析] 读一次三轴角速度，换算成 rad/s、乘标度因数、再转到机器人坐标系，写入成员 gyro[3]。
// [解析] 与 acc_update 的三个区别：
// [解析]   1) 数据从下标 1 开始（陀螺仪只有 1 个无效字节）；
// [解析]   2) 多乘一个 gyro_multipliers_（三轴标定系数，默认 1.0）；
// [解析]   3) 不读温度（陀螺仪没有温度寄存器，温度统一由加速度计提供）。
void BMI088::gyro_update()
{
  // 接收陀螺仪数据
  // [解析] 从 0x02 连续读 6 字节 = X/Y/Z 三轴角速度，每轴 2 字节，同样是低字节在前（小端）。
  gyro_read(BMI088_GYRO_DATA, 6);

  // 第一个字节为无效数据
  // [解析] 拼接方式与加速度计一致：(高字节 << 8) | 低字节，赋给 int16_t 自动完成补码解释。
  // [解析] 原始值范围 -32768 ~ +32767，2000dps 量程下每 LSB ≈ 0.061 °/s。
  int16_t gyro_x_int = (rx_buff_[1 + 1] << 8) | rx_buff_[1 + 0];
  int16_t gyro_y_int = (rx_buff_[1 + 3] << 8) | rx_buff_[1 + 2];
  int16_t gyro_z_int = (rx_buff_[1 + 5] << 8) | rx_buff_[1 + 4];

  // 应用传入的标度因数
  // [解析] 两次换算合在一起：先乘 BMI088_GYRO_INT_TO_RPS 把 LSB 变成 rad/s，
  // [解析]   再乘 gyro_multipliers_ 做每轴灵敏度微调（标定系数，理想为 1.0）。
  // [解析] 注意乘法顺序不影响结果（浮点连乘结合律基本成立），但不要漏掉任何一个，
  // [解析]   漏掉标度因数得到的数会大 57 倍左右（因为单位是 °/s 而不是 rad/s）。
  float gyro_x = gyro_x_int * BMI088_GYRO_INT_TO_RPS * gyro_multipliers_[0];
  float gyro_y = gyro_y_int * BMI088_GYRO_INT_TO_RPS * gyro_multipliers_[1];
  float gyro_z = gyro_z_int * BMI088_GYRO_INT_TO_RPS * gyro_multipliers_[2];

  // 更新公开属性
  // [解析] 同样是"矩阵 × 列向量"的坐标系变换。注意：这里用的 r_ab_ 和加速度计是同一个矩阵，
  // [解析]   因为两颗芯片在物理上装在同一块板子上、朝向一致。
  this->gyro[0] = r_ab_[0][0] * gyro_x + r_ab_[0][1] * gyro_y + r_ab_[0][2] * gyro_z;
  this->gyro[1] = r_ab_[1][0] * gyro_x + r_ab_[1][1] * gyro_y + r_ab_[1][2] * gyro_z;
  this->gyro[2] = r_ab_[2][0] * gyro_x + r_ab_[2][1] * gyro_y + r_ab_[2][2] * gyro_z;
}

// [解析] 从陀螺仪读 len 个字节到 rx_buff_。
// [解析] 参数：reg = 起始寄存器地址；len = 字节数。返回值：无。
// [解析] ★与 acc_read 的唯一区别：这里只多 1 个 dummy 字节（总长 len + 1），
// [解析]   所以有效数据从 rx_buff_[1] 开始，而不是 [2]。这是两颗芯片时序规定不同造成的。
void BMI088::gyro_read(uint8_t reg, uint8_t len)
{
  // 拉低CSB2
  // [解析] 拉低 CSB2 选中陀螺仪；此时 CSB1 为高，加速度计不响应这条总线。
  HAL_GPIO_WritePin(csb2_port_, csb2_pin_, GPIO_PIN_RESET);

  // ref: datasheet 6.1.1
  // [解析] 陀螺仪同样是"地址最高位 1 = 读"，所以照样 | 0x80。
  // [解析] 这一点和 ICM42688 不同（那颗也是 0x80，但很多其它 SPI 传感器是 0x00/0x80 相反的）。
  tx_buff_[0] = reg | 0x80;

  // 第一个字节为无效数据
  // [解析] len+1 = 1 个地址字节 + len 个数据字节。陀螺仪不需要额外的 dummy 字节，
  // [解析]   从第 2 个字节（下标 1）开始就是有效数据。
  // [解析] 超时参数 100 同样单位是毫秒，且返回值被忽略。
  HAL_SPI_TransmitReceive(hspi_, tx_buff_, rx_buff_, len + 1, 100);

  // 拉高CSB2
  HAL_GPIO_WritePin(csb2_port_, csb2_pin_, GPIO_PIN_SET);
}

// [解析] 向陀螺仪一个寄存器写 1 字节。参数与 acc_write 相同：reg 地址、data 数据。
// [解析] 地址不加 0x80（最高位 0 = 写），发送 2 个字节（地址 + 数据），只发不收。
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
// [解析] 文件结束。复习一下核心套路：
// [解析]   片选拉低 -> 发地址(读则 |0x80) -> 收发数据 -> 片选拉高；
// [解析]   原始 int16 -> 乘标度因数得到物理量 -> 左乘 r_ab_ 转到机器人坐标系 -> 写入公开成员。
