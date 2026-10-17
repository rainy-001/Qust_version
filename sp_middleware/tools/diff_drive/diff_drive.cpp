// [解析] 两轮差速底盘的正/反运动学：calc() 把 (v, w) 拆给左右轮（左 +w*w_、右 -w*w_），update() 反推回整车速度。
// [解析] 上层底盘任务每周期调 calc()，结果写 speed_l/speed_r 给两个电机 PID；update() 用于里程计。
// [解析] 构造参数顺序是 (半轮距, 轮半径)；half_width 是"左右轮中心距的一半"，量错会让自转快慢不对。

#include "diff_drive.hpp"

namespace sp
{
DiffDrive::DiffDrive(float half_width, float wheel_radius, bool reverse_l, bool reverse_r)
: w_(half_width), r_(wheel_radius), sign_l_(reverse_l ? -1 : 1), sign_r_(reverse_r ? -1 : 1)
{
}

void DiffDrive::calc(float v_set, float w_set)
{
  this->speed_l = sign_l_ * (v_set + w_set * w_) / r_;
  this->speed_r = sign_r_ * (v_set - w_set * w_) / r_;
}

void DiffDrive::update(float motor_speed_l, float motor_speed_r)
{
  this->v = (sign_l_ * motor_speed_l + sign_r_ * motor_speed_r) * r_ / 2;
  // [解析] 疑似笔误：应为 (sign_l_*motor_speed_l - sign_r_*motor_speed_r)，现写法把左右反馈写反，原地自转的 w 符号相反（v 不受影响，详见 docs 第 8 节）。
  this->w = (sign_l_ * motor_speed_r - sign_r_ * motor_speed_l) * r_ / (2 * w_);
}

}  // namespace sp
