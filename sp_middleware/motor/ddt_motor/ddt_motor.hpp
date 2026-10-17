// [解析] 达妙(Damiao) DDT 系列电机的类封装，用法与 RM_Motor 一致：read() / cmd() / write()。
// [解析] 本仓库暂无任务实例化它，属于备用驱动；一帧同样只装 4 台，1~4 号用 0x32、5~8 号用 0x33。
// [解析] 关键是 torque_const（填错会让读数与指令一起按比例偏）和 angle（折回 (-PI, PI] 的单圈角，不是累计角）。

#ifndef SP__DDT_MOTOR_HPP
#define SP__DDT_MOTOR_HPP

#include <cstdint>

namespace sp
{

// [解析] 一个对象 = 总线上的一台达妙电机；没有减速比、也没有多圈累计，所以比 RM_Motor 简单。
class DDT_Motor
{
public:
  DDT_Motor(uint8_t motor_id, float torque_const);

  const uint16_t rx_id;  // 电机反馈帧ID
  const uint16_t tx_id;  // 电机控制帧ID

  uint8_t error;  // 只读! 0x00: 无故障, 略
  uint8_t mode;  // 只读! 0x00: 电压开环, 0x01: 电流环, 0x02: 速度环, 0x09: 失能模式, OxOA: 使能模式

  float angle = 0;   // 只读! 单位: rad
  float speed = 0;   // 只读! 单位: rad/s
  float torque = 0;  // 只读! 单位: N·m

  // [解析] 大端拼帧，结果直接写进上面的公开成员；不记时间戳，所以没有 is_alive()，要掉线检测得自己记时间。
  void read(const uint8_t * data);
  void write(uint8_t * data) const;

  // 单位: N·m
  // [解析] 只更新 cmd_raw_，不发帧，必须再调 write() + can1.send()。
  void cmd(float value);

private:
  const uint8_t motor_id_;
  const float torque_const_;
  int16_t cmd_raw_ = 0;
};

}  // namespace sp

#endif  // SP__DDT_MOTOR_HPP