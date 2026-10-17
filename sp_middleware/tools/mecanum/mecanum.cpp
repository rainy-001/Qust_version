// [解析] 麦克纳姆轮底盘的正/反运动学：一行公式一个轮子，把整车 (vx, vy, wz) 拆给四个轮子，也能反推回整车速度。
// [解析] 上层底盘任务每周期调 calc()，把四个 speed_xx 喂给电机 PID；update() 拿轮速反馈做里程计。
// [解析] 口诀：vx 全正、vy 反对角同号、wz 左负右正；最后都 /r_ 把 m/s 换成 rad/s，sign_xx_ 只在最外层翻一次装配方向。

#include "mecanum.hpp"

namespace sp
{
// [解析] 参数登记，不碰硬件：本类成员全是 const，只能走冒号后面的初始化列表；函数体里顺便把四个输出清零。
Mecanum::Mecanum(
  float wheel_radius, float half_length, float half_width, bool reverse_lf, bool reverse_lr,
  bool reverse_rf, bool reverse_rr)
: r_(wheel_radius),
  l_(half_length),
  w_(half_width),
  sign_lf_((reverse_lf) ? -1.0f : 1.0f),
  sign_lr_((reverse_lr) ? -1.0f : 1.0f),
  sign_rf_((reverse_rf) ? -1.0f : 1.0f),
  sign_rr_((reverse_rr) ? -1.0f : 1.0f)
{
  this->speed_lf = 0.0f;
  this->speed_lr = 0.0f;
  this->speed_rf = 0.0f;
  this->speed_rr = 0.0f;
}

// [解析] 正解：(l_+w_) 是"轮子接地点绕底盘中心的等效旋转半径"，所以 wz 乘它；三项线性叠加后 /r_ 得角速度。
void Mecanum::calc(float vx, float vy, float wz)
{
  // ref: https://modernrobotics.northwestern.edu/nu-gm-book-resource/13-2-omnidirectional-wheeled-mobile-robots-part-1-of-2
  this->speed_lf = sign_lf_ * (vx - vy - (l_ + w_) * wz) / r_;
  this->speed_lr = sign_lr_ * (vx + vy - (l_ + w_) * wz) / r_;
  this->speed_rf = sign_rf_ * (vx + vy + (l_ + w_) * wz) / r_;
  this->speed_rr = sign_rr_ * (vx - vy + (l_ + w_) * wz) / r_;
}

// [解析] 先 /sign_xx_ 抵消装配方向（±1 除等价于乘）、再 ×r_ 把 rad/s 换成 m/s，之后才能按标准约定反解。
// [解析] 能"精确"反解而不是近似，是因为系数矩阵三列两两正交，最小二乘退化成各自除以列模长平方，/4 与 /(l_+w_) 由此而来。
void Mecanum::update(float speed_lf, float speed_lr, float speed_rf, float speed_rr)
{
  float sign_speed_lf = speed_lf / sign_lf_;
  float sign_speed_lr = speed_lr / sign_lr_;
  float sign_speed_rf = speed_rf / sign_rf_;
  float sign_speed_rr = speed_rr / sign_rr_;

  this->vx = (sign_speed_lf + sign_speed_lr + sign_speed_rf + sign_speed_rr) / 4 * r_;
  this->vy = (-sign_speed_lf + sign_speed_lr + sign_speed_rf - sign_speed_rr) / 4 * r_;
  this->wz = (-sign_speed_lf - sign_speed_lr + sign_speed_rf + sign_speed_rr) / 4 * r_ / (l_ + w_);
}

}  // namespace sp
