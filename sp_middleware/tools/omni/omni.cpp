// [解析] 四全向轮底盘（四个角各一个全向轮）的正/反运动学：轮子只能沿自己的滚动方向 u 出力，故轮速 = u·(v + wz×p) / r。
// [解析] 上层底盘任务每周期调 calc() 下发四个轮速，或用 update() 把轮速反馈换成整车速度做里程计。
// [解析] 关键量 d_ 是半对角线 sqrt(l²+w²)：正解里既要除以它（投影），又要乘以它（自转半径）。

#include "omni.hpp"

#include <cmath>

namespace sp
{
Omni::Omni(
  float wheel_radius, float half_length, float half_width, bool reverse_lf, bool reverse_lr,
  bool reverse_rf, bool reverse_rr)
: r_(wheel_radius),
  l_(half_length),
  w_(half_width),
  d_(std::sqrt(half_length * half_length + half_width * half_width)),
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

// [解析] 括号里是"整车速度投影到该轮滚动方向 u 上"，把各轮位置 p=(±l_, ±w_) 代进去就得到四个系数：
// [解析] vx 项左右反号（-y/d_），vy 项前后反号（+x/d_）；后面的 + d_*wz 是自转分量，对四个轮子同号。
void Omni::calc(float vx, float vy, float wz)
{
  // 各轮滚动方向上的线速度: u · (v + wz × p) = (-y * vx + x * vy) / D + D * wz, 单位: m/s
  // 注意 x = l_ 处为车头方向, y = w_ 处为车身左侧
  const float v_lf = (-w_ * vx + l_ * vy) / d_ + d_ * wz;
  const float v_lr = (-w_ * vx - l_ * vy) / d_ + d_ * wz;
  const float v_rf = (+w_ * vx + l_ * vy) / d_ + d_ * wz;
  const float v_rr = (+w_ * vx - l_ * vy) / d_ + d_ * wz;

  // 线速度 / 半径 = 转速, 再按电机装配方向取反
  // [解析] ×sign_xx_ 只在这里翻一次装配方向，和几何推导解耦；本函数不限幅，vx 过大时限幅要调用方自己做。
  this->speed_lf = sign_lf_ * v_lf / r_;
  this->speed_lr = sign_lr_ * v_lr / r_;
  this->speed_rf = sign_rf_ * v_rf / r_;
  this->speed_rr = sign_rr_ * v_rr / r_;
}

// [解析] 先 /sign_xx_ 抵消装配方向（±1 除等价于乘）、再 ×r_ 把 rad/s 换成 m/s，之后才能用标准约定反解。
void Omni::update(float speed_lf, float speed_lr, float speed_rf, float speed_rr)
{
  // 电机转速 -> 各轮滚动方向上的线速度, 单位: m/s
  const float v_lf = speed_lf / sign_lf_ * r_;
  const float v_lr = speed_lr / sign_lr_ * r_;
  const float v_rf = speed_rf / sign_rf_ * r_;
  const float v_rr = speed_rr / sign_rr_ * r_;

  // 最小二乘反解 (H^T H 为对角阵, 故为精确解)
  // [解析] 叫"最小二乘"却是精确解：H 的三列两两正交，H^T·H 退化成对角阵，求逆只剩"除以各列模长平方"，这个 /4 就是它。
  // [解析] vx 多乘 d_/w_、vy 多乘 d_/l_ 是列模长化简的结果；因 d_ ≥ w_ 会放大轮速噪声，实测值一般要先滤波。
  this->vx = (-v_lf - v_lr + v_rf + v_rr) / 4 * d_ / w_;
  this->vy = (+v_lf - v_lr + v_rf - v_rr) / 4 * d_ / l_;
  this->wz = (+v_lf + v_lr + v_rf + v_rr) / 4 / d_;
}

}  // namespace sp
