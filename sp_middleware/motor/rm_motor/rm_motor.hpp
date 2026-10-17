// [解析] 大疆 RoboMaster 官方电机（M2006 / M3508 / GM6020）的类封装。
// [解析] 上层 read() 取反馈、cmd() 下指令、write() 打包；can_task.cpp 负责收发，joint_controller 拿它当底层驱动。
// [解析] 三个易错点：编码器 0~8191 的弧度与多圈换算、各型号换算系数不同、一帧 8 字节由 4 台电机共用。

#ifndef SP__RM_MOTOR_HPP
#define SP__RM_MOTOR_HPP

#include <cstdint>

namespace sp
{
// [解析] 减速比：电机转 ratio 圈，输出轴才转 1 圈。读的时候除、写的时候乘。
constexpr float M2006_P36 = 36.0f;             // 原装减速箱减速比
constexpr float M3508_P19 = 3591.0f / 187.0f;  // 原装减速箱减速比

// [解析] 最大力矩乘了 *10 / *20 / *3，手册里没有这三个倍数；cpp 的换算系数带着同样倍数，改动必须两处一起改。
constexpr float M2006_P36_MAX_TORQUE = 0.18f * 10;  // N·m
constexpr float M3508_P19_MAX_TORQUE = 0.3f * 20;   // N·m
constexpr float GM6020_MAX_TORQUE = 0.741f * 3;     // N·m

enum class RM_Motors
{
  M2006,    // 无减速箱
  M3508,    // 无减速箱
  GM6020,   // 电流控制
  GM6020_V  // 电压控制
};

// [解析] 一个对象 = 总线上的一台电机；read() 在 CAN 接收中断里调、cmd()/write() 在主任务里调，无锁但只交换单个变量。
class RM_Motor
{
public:
  // motor_id: 电机ID, 取值: 1, 2, ..., 8
  // motor_type: 电机型号， 取值见`RM_Motors`
  // ratio: 减速比
  // [解析] ratio 默认 1.0f 即直驱；multi_circle 为 false 时 angle 不累计，只在 (-PI, PI] 里跳。
  RM_Motor(uint8_t motor_id, RM_Motors motor_type, float ratio = 1.0f, bool multi_circle = true);

  const uint16_t rx_id;  // 电机反馈帧ID
  const uint16_t tx_id;  // 电机控制帧ID

  float angle;          // 只读! 单位: rad
  float speed;          // 只读! 单位: rad/s
  float torque;         // 只读! 单位: N·m
  uint8_t temperature;  // 只读! 单位: 摄氏度

  bool is_open() const;
  // [解析] 距最后一次反馈不到 100ms 算在线，用减法比较以兼容定时器回绕；is_open() 一旦为 true 就不再变回 false。
  bool is_alive(uint32_t now_ms) const;

  // [解析] 解析一帧 8 字节反馈，调用前须先比对 rx_id；设计成在 CAN 接收中断里运行。
  void read(uint8_t * data, uint32_t stamp_ms);
  // [解析] 把 cmd_raw_ 写进共用的 8 字节帧，4 台都 write() 之后再统一 send() 一次。
  void write(uint8_t * data) const;

  // 如果非`RM_Motors::GM6020_V`, 单位: N·m
  // 如果是`RM_Motors::GM6020_V`, 单位: V
  // [解析] 只更新 cmd_raw_，不发帧，必须再调 write() + send() 才真正发出去。
  void cmd(float value);

private:
  const uint8_t motor_id_;
  const RM_Motors motor_type_;
  const float ratio_;
  const bool multi_circle_;

  bool has_read_;
  uint32_t last_read_ms_;
  int32_t circle_;
  uint16_t last_ecd_;
  int16_t cmd_raw_;
};

}  // namespace sp

#endif  // SP__RM_MOTOR_HPP