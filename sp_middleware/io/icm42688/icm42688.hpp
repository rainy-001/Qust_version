// [解析] ICM42688 六轴 IMU 驱动接口：SPI 读加速度/角速度/温度，换算后写进公共成员。
// [解析] 接口与 bmi088 完全一致（acc/gyro/temp + init/update），两者可互换，只是片选少一根。
// [解析] 上层建对象 -> init() -> 循环 update()；init() 失败会 10ms 一次地重试，不会彻底卡死。
// [解析] 单位约定：acc 为 m/s^2、gyro 为 rad/s（不是 °/s）、temp 为 ℃，且都已转到机器人坐标系。

#ifndef SP__ICM42688_HPP
#define SP__ICM42688_HPP

#include "gpio.h"
#include "spi.h"

namespace sp
{
// [解析] 一个对象管一颗 IMU（1 条 SPI + 1 根片选）。构造只存参数（r_ab 为安装方向矩阵）。
class ICM42688
{
public:
  // CS: SPI片选信号, 低电平时选中ICM42688
  // R_ab: icm42688原始坐标系{b}在机器人坐标系{a}下的坐标矩阵
  ICM42688(
    SPI_HandleTypeDef * hspi, GPIO_TypeDef * cs_port, uint16_t cs_pin, 
    const float r_ab[3][3], const float gyro_multipliers[3] = nullptr);

  float acc[3];   // 只读! 单位: m/s^2
  float gyro[3];  // 只读! 单位: rad/s
  float temp;     // 只读! 单位: celsius

  // [解析] 软复位 + 查 WHO_AM_I + 按配置表写电源模式/量程/ODR，失败则 10ms 后重试。
  void init();
  // [解析] 一次突发读 14 字节（温度 + 加速度 + 角速度）后换算；会占用 SPI，需按 ODR 周期调用。
  void update();

private:
  SPI_HandleTypeDef * hspi_;
  GPIO_TypeDef * cs_port_;
  uint16_t cs_pin_;
  const float r_ab_[3][3];

  uint8_t rx_buff_[20];
  uint8_t tx_buff_[20];

  float gyro_multipliers_[3]; // 存储三轴标度因数

  uint8_t device_init();
  void read_registers(uint8_t reg, uint8_t len);
  void write_register(uint8_t reg, uint8_t data);
};

}  // namespace sp

#endif  // SP__ICM42688_HPP