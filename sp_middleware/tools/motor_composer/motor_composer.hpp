// [解析] MotorComposer：把机械上并排/对装的两台电机等效成"一台电机"来闭环的双电机合成器。
// [解析] 上层机构类持有成员，每周期 calc_* 合成反馈 -> PID -> calc_speeds 拆两路；用前必须 init_angles() 记零位，且两台电机的方向符号要先统一。

#ifndef SP__MOTOR_COMPOSER_HPP
#define SP__MOTOR_COMPOSER_HPP

namespace sp
{
// [解析] 一个对象对应一对电机：只存两个方向符号和两个零位角度，不碰硬件、不加锁。
// [解析] 数据不走返回值：calc_xxx() 直接算进下面的公开成员，带"只读!"的是输出，别从外部改。
class MotorComposer
{
public:
  // [解析] 构造只把 reverse_l/reverse_r 翻译成 ±1.0f，默认"左正装、右倒装"；用乘符号而不是 if 取反。
  // 将两个电机等效为一个电机
  // reverse_l: 左侧电机反向旋转
  // reverse_r: 右侧电机反向旋转
  MotorComposer(bool reverse_l = false, bool reverse_r = true);

  float angle;    // 只读! calc_angle()计算结果, 单位: rad
  float speed;    // 只读! calc_speed()计算结果, 单位: rad/s
  float torque;   // 只读! calc_torque()计算结果, 单位: N·m
  float speed_l;  // 只读! calc_speeds()计算结果, 单位: rad/s
  float speed_r;  // 只读! calc_speeds()计算结果, 单位: rad/s

  // [解析] 记零位：使能完成、机械停在期望零点时调用一次；忘了调会让上电瞬间 PID 猛冲一下。
  // 初始化各电机转角
  // angle_l: 左侧电机转角, 单位: rad
  // angle_r: 右侧电机转角, 单位: rad
  void init_angles(float angle_l, float angle_r);

  // [解析] 等效角度 = (sign_l*(angle_l-零位_l) + sign_r*(angle_r-零位_r)) / 2，单位 rad、相对零位。
  // 各电机转角 -> 等效转角
  // angle_l: 左侧电机转角, 单位: rad
  // angle_r: 右侧电机转角, 单位: rad
  void calc_angle(float angle_l, float angle_r);

  // 各电机转速 -> 等效转速
  // speed_l: 左侧电机转速, 单位: rad
  // speed_r: 右侧电机转速, 单位: rad
  void calc_speed(float speed_l, float speed_r);

  // [解析] 力矩取"和"不是平均：sign_l*torque_l + sign_r*torque_r，做力矩闭环时阈值按总力矩定。
  // 各电机转矩 -> 等效转矩
  // torque_l: 左侧电机转矩, 单位: N·m
  // torque_r: 右侧电机转矩, 单位: N·m
  void calc_torque(float torque_l, float torque_r);

  // [解析] 反向拆分 speed_l = sign_l*speed、speed_r = sign_r*speed，与 calc_speed() 自洽；拆完还要自己 cmd() 下发。
  // 等效转速 -> 各电机转速
  // speed: 等效转速, 单位: rad/s
  void calc_speeds(float speed);

private:
  const float sign_l_;
  const float sign_r_;

  float angle_l0_;
  float angle_r0_;
};

}  // namespace sp

#endif  // SP__MOTOR_COMPOSER_HPP