// [解析] BMI088 六轴 IMU 驱动接口：SPI 读加速度/角速度/温度，换算后写进公共成员。
// [解析] 上层在 INS/姿态任务里建对象 -> init() -> 循环 update()，再读 acc/gyro/temp。
// [解析] 芯片是两颗器件两颗片选，构造函数要把两组片选引脚都传进来；init() 内是 while 死等。
// [解析] 单位约定：acc 为 m/s^2、gyro 为 rad/s（不是 °/s）、temp 为 ℃，且都已转到机器人坐标系。

#ifndef SP__BMI088_HPP
#define SP__BMI088_HPP

#include "gpio.h"
#include "spi.h"

namespace sp
{
// [解析] 一个对象管一颗 IMU。构造只存参数（r_ab 为安装方向矩阵，gyro_multipliers 可选，默认不校准）。
class BMI088
{
public:
  // CSB1: SPI片选信号, 低电平时选中加速度计
  // CSB2: SPI片选信号, 低电平时选中陀螺仪
  // R_ab: bmi088原始坐标系{b}在机器人坐标系{a}下的坐标矩阵, 详见io/bmi088/readme.md
  BMI088(
    SPI_HandleTypeDef * hspi, GPIO_TypeDef * csb1_port, uint16_t csb1_pin, GPIO_TypeDef * csb2_port,
    uint16_t csb2_pin, const float r_ab[3][3], const float gyro_multipliers[3] = nullptr);

  float acc[3];   // 只读! 单位: m/s^2
  float gyro[3];  // 只读! 单位: rad/s
  float temp;     // 只读! 单位: celsius

  // [解析] 复位 + 查 ID + 按表写寄存器，内部重试直到成功，传感器不在线会卡住。
  void init();
  // [解析] 读一次数据并换算/转坐标系；会发起 SPI，需按 ODR 周期调用，且非线程安全（缓冲区是成员）。
  void update();

private:
  SPI_HandleTypeDef * hspi_;
  GPIO_TypeDef * csb1_port_;
  GPIO_TypeDef * csb2_port_;
  uint16_t csb1_pin_;
  uint16_t csb2_pin_;
  const float r_ab_[3][3];

  uint8_t rx_buff_[16];
  uint8_t tx_buff_[16];

  float gyro_multipliers_[3]; // 存储三轴标度因数

  uint8_t acc_init();
  void acc_update();
  void acc_read(uint8_t reg, uint8_t len);
  void acc_write(uint8_t reg, uint8_t data);

  uint8_t gyro_init();
  void gyro_update();
  void gyro_read(uint8_t reg, uint8_t len);
  void gyro_write(uint8_t reg, uint8_t data);
};

}  // namespace sp

#endif  // SP__BMI088_HPP