// [解析] ==========================================================================
// [解析] 【文件】sp_middleware/io/bmi088/bmi088.hpp
// [解析] 【作用】STM32 通过 SPI 读取 BMI088 六轴 IMU（3 轴加速度 + 3 轴角速度 + 温度），
// [解析]         并把"原始寄存器值"换算成物理单位（m/s^2、rad/s、摄氏度）后放到公共成员里。
// [解析] 【被谁用】applications/（例如 INS/姿态解算任务）里创建对象：
// [解析]           sp::BMI088 bmi088(&hspi1, CS1_GPIO_Port, CS1_Pin, CS2_GPIO_Port, CS2_Pin, r_ab);
// [解析]           然后循环调用 bmi088.update()，再读 bmi088.acc / bmi088.gyro / bmi088.temp。
// [解析] 【关键点】1) BMI088 内部是两颗独立芯片，占两个片选 CSB1(加速度计)、CSB2(陀螺仪)，
// [解析]             必须由构造函数把两个片选引脚都告诉它，它才能分别拉低选中。
// [解析]           2) 构造函数传进来的 r_ab 是"安装方向旋转矩阵"，因为 IMU 焊在板子上的方向
// [解析]             和机器人坐标系通常不一致，读出来的三轴要乘这个矩阵转到机器人坐标系。
// [解析]           3) init() 里有 while 死等，传感器没接好就会卡在那里，调试时要注意。
// [解析] 【阅读顺序】先看本文件了解接口，再看 bmi088.cpp 看寄存器怎么读写、单位怎么换算。
// [解析] ==========================================================================

#ifndef SP__BMI088_HPP
#define SP__BMI088_HPP

#include "gpio.h"
#include "spi.h"

namespace sp
{
// [解析] BMI088 类：一个对象管一颗 IMU 芯片（SPI 总线 + 2 根片选线）。
// [解析] 典型使用流程：
// [解析]   1) BMI088 bmi088(&hspi1, csb1_port, csb1_pin, csb2_port, csb2_pin, r_ab);  // 只存参数
// [解析]   2) bmi088.init();    // 配置芯片，卡住直到配置成功（建议在任务开始处调用一次）
// [解析]   3) bmi088.update();  // 周期性调用（例如 1kHz 任务里），刷新 acc/gyro/temp
// [解析]   4) 读 bmi088.acc[0..2] / bmi088.gyro[0..2] / bmi088.temp
class BMI088
{
public:
  // CSB1: SPI片选信号, 低电平时选中加速度计
  // CSB2: SPI片选信号, 低电平时选中陀螺仪
  // R_ab: bmi088原始坐标系{b}在机器人坐标系{a}下的坐标矩阵, 详见io/bmi088/readme.md
  // [解析] 构造函数：只把参数（SPI 句柄、两个片选引脚、旋转矩阵、三轴标度因数）记到成员变量里，
  // [解析] 不碰硬件、不产生 SPI 通信。所以这里可以先创建对象，稍后再 init()。
  // [解析] 参数逐个说明：
  // [解析]   hspi        —— CubeMX 生成的 SPI 句柄，如 &hspi1，指明这颗 IMU 挂在哪条 SPI 上；
  // [解析]   csb1_port   —— 加速度计片选引脚所在端口（GPIOA/GPIOB...）；
  // [解析]   csb1_pin    —— 加速度计片选引脚号，如 GPIO_PIN_4；
  // [解析]   csb2_port   —— 陀螺仪片选引脚所在端口；
  // [解析]   csb2_pin    —— 陀螺仪片选引脚号；
  // [解析]   r_ab        —— 3x3 旋转矩阵（单位：无量纲），把传感器坐标系下的向量转到机器人坐标系，
  // [解析]                    通常就是几个 0/±1 构成的交换矩阵，具体取值看 readme.md 的安装说明；
  // [解析]   gyro_multipliers —— 陀螺仪三轴标度因数（无量纲，理想值 1.0），
  // [解析]                    用于补偿每个轴的灵敏度偏差（标定得到的修正系数）；
  // [解析]                    默认 nullptr 表示不校准，三轴系数都取 1.0。
  BMI088(
    SPI_HandleTypeDef * hspi, GPIO_TypeDef * csb1_port, uint16_t csb1_pin, GPIO_TypeDef * csb2_port,
    uint16_t csb2_pin, const float r_ab[3][3], const float gyro_multipliers[3] = nullptr);

  // [解析] 下面 3 个是"输出型"公共成员：update() 每调用一次就把最新数据写进去。
  // [解析] 调用者不需要接收返回值，直接读这几个数组即可。
  float acc[3];   // 只读! 单位: m/s^2
  // [解析] ↑ 机器人坐标系下的三轴加速度，下标 0/1/2 对应 x/y/z。
  // [解析]   静止水平放置时 z 轴大约是 +9.8 m/s^2（因为重力被计为向上的比力）。
  float gyro[3];  // 只读! 单位: rad/s
  // [解析] ↑ 机器人坐标系下的三轴角速度，单位是弧度/秒（不是度/秒！要换算成 °/s 需 ×57.3）。
  float temp;     // 只读! 单位: celsius
  // [解析] ↑ 芯片内部温度，单位摄氏度。注意它测的是芯片自身温度，会比环境温度略高。

  // [解析] 初始化：复位芯片、检查 ID、按初始化表写好量程/带宽/中断等寄存器。
  // [解析] 内部是 while 循环重试，直到全部配置成功才返回；传感器不在线时会死循环。
  // [解析] 调用时机：上电后、启动姿态解算任务之前，调用一次即可（不要在循环里反复调用）。
  void init();
  // [解析] 更新一次数据：读三轴加速度、三轴角速度、温度并完成单位换算 + 坐标系变换。
  // [解析] 调用时机：放在周期性任务里（本项目加速度计 800Hz、陀螺仪 230Hz 带宽，
  // [解析]   一般用 1kHz 任务调用它，靠 INT1/INT3 的 DRDY 信号同步）。
  // [解析] 副作用：会发起 SPI 通信（拉低/拉高片选），不是纯计算函数。
  void update();

private:
  // [解析] 下面是"记住构造函数传进来的配置"，一次存好，后面每次读写寄存器都用它。
  SPI_HandleTypeDef * hspi_;  // [解析] 本对象使用的 SPI 外设句柄
  GPIO_TypeDef * csb1_port_;  // [解析] 加速度计片选端口
  GPIO_TypeDef * csb2_port_;  // [解析] 陀螺仪片选端口
  uint16_t csb1_pin_;         // [解析] 加速度计片选引脚号
  uint16_t csb2_pin_;         // [解析] 陀螺仪片选引脚号
  const float r_ab_[3][3];    // [解析] 传感器坐标系 -> 机器人坐标系的旋转矩阵拷贝

  // [解析] SPI 收发共用缓冲区：rx_buff_ 存读回来的字节（含无效字节），tx_buff_ 存要发的字节。
  // [解析] 大小 16 字节足够：一次最多读 8 字节（1 字节地址 + 1 字节 dummy + 6 字节数据），写只要 2 字节。
  // [解析] 注意：这些缓冲区是成员变量，所以同一个对象不能被两个任务同时使用（非线程安全）。
  uint8_t rx_buff_[16];
  uint8_t tx_buff_[16];

  float gyro_multipliers_[3]; // 存储三轴标度因数
  // [解析] ↑ 三轴灵敏度修正系数，update() 里乘到角速度上；不传参时默认 {1.0, 1.0, 1.0}。

  // [解析] 以下私有函数是"内部零件"，外部不该直接调用，看 bmi088.cpp 里的实现即可。
  // [解析] 返回值：BMI088_NO_ERROR(0x00) 表示成功，其他值为错误码（见 bmi088_defs.h）。
  uint8_t acc_init();   // [解析] 配置加速度计，失败返回错误码
  void acc_update();    // [解析] 读加速度 + 温度，换算并写进 acc[] / temp
  void acc_read(uint8_t reg, uint8_t len);      // [解析] 从加速度计读 len 字节到 rx_buff_
  void acc_write(uint8_t reg, uint8_t data);    // [解析] 向加速度计某寄存器写 1 字节

  uint8_t gyro_init();  // [解析] 配置陀螺仪，失败返回错误码
  void gyro_update();   // [解析] 读三轴角速度，换算并写进 gyro[]
  void gyro_read(uint8_t reg, uint8_t len);     // [解析] 从陀螺仪读 len 字节到 rx_buff_
  void gyro_write(uint8_t reg, uint8_t data);   // [解析] 向陀螺仪某寄存器写 1 字节
};

}  // namespace sp

#endif  // SP__BMI088_HPP