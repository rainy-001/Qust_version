// [解析] 差速齿轮（两自由度关节）的解算器：两个输入电机 L/R 的转角经锥齿轮叠加成末端 pitch(共模)/roll(差模)。
// [解析] 上层云台/关节任务按 init_gear_angle -> calc_end_* 使用；calc_gear_speed() 是反解，用来下发两个电机。
// [解析] ratio_ 在角度/速度里是"除以"、在力矩里是"乘以"（功率共轭，不是笔误）；忘调 init_gear_angle() 会读到未初始化的零位。

#include "diff_gear.hpp"

namespace sp
{
DiffGear::DiffGear(float ratio, bool reverse_l, bool reverse_r)
: ratio_(ratio), sign_l_((reverse_l) ? -1.0f : 1.0f), sign_r_((reverse_r) ? -1.0f : 1.0f)
{
}

// [解析] 记录开机零位：差速机构只能测出齿轮转了多少、不知道末端绝对姿态，之后的 pitch/roll 都是相对这次读数的增量。
void DiffGear::init_gear_angle(float angle_l, float angle_r)
{
  angle_l0_ = angle_l;
  angle_r0_ = angle_r;
}

// [解析] 共模/差模：pitch 取两轮平均；roll 取差值的一半再 /ratio_ —— 锥齿轮副使齿轮转 ratio_ 圈、末端才转 1 圈。
void DiffGear::calc_end_angle(float angle_l, float angle_r)
{
  this->pitch = (sign_l_ * (angle_l - angle_l0_) + sign_r_ * (angle_r - angle_r0_)) / 2;
  this->roll = (sign_l_ * (angle_l - angle_l0_) - sign_r_ * (angle_r - angle_r0_)) / 2 / ratio_;
}

void DiffGear::calc_end_speed(float speed_l, float speed_r)
{
  this->v_pitch = (sign_l_ * speed_l + sign_r_ * speed_r) / 2;
  this->v_roll = (sign_l_ * speed_l - sign_r_ * speed_r) / 2 / ratio_;
}

// [解析] 力矩是速度的功率共轭量，所以既没有 /2、ratio_ 也从"除"变"乘"：t_pitch = τl+τr，t_roll = (τl-τr)*ratio_。
void DiffGear::calc_end_torque(float torque_l, float torque_r)
{
  this->t_pitch = (sign_l_ * torque_l + sign_r_ * torque_r);
  this->t_roll = (sign_l_ * torque_l - sign_r_ * torque_r) * ratio_;
}

// [解析] 反解 calc_end_speed()：v_roll*ratio_ 正是那边 /ratio_ 的逆运算，最后乘 sign_ 把装配方向补回去。
void DiffGear::calc_gear_speed(float v_pitch, float v_roll)
{
  this->v_left = sign_l_ * (v_pitch + v_roll * ratio_);
  this->v_right = sign_r_ * (v_pitch - v_roll * ratio_);
}

}  // namespace sp
