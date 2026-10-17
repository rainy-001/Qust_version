// [解析] yaw 轴力矩前馈的实现：T = J·α + 阻尼项，α 用相邻两帧目标角速度差分近似。
// [解析] 调用者每个控制周期调一次 calc()，再把 out 叠加到速度环 PID 的输出上。
// [解析] 顺序很关键：先算 α（读滤波器的上一帧输出），最后才 update 滤波器，否则 α 恒为 0。

#include "yaw_feedward.hpp"

namespace sp
{
YawFeedward::YawFeedward(float dt, float inertia, float k_ff, float damping, float damping_out_max)
: dt_(dt), inertia_(inertia), k_ff_(k_ff), damping_(damping), damping_out_max_(damping_out_max)
{
}

// [解析] dt_ 做分母（不能为 0）；damping_out_max_ 默认 0，limit_max 会把阻尼项恒压成 0。
void YawFeedward::calc(float speed_set)
{
  this->alpha_set_ = (speed_set - speed_filter.out) / dt_;

  this->intertia_out = k_ff_ * inertia_ * this->alpha_set_;

  this->damping_out = limit_max(speed_set * damping_, damping_out_max_);

  this->out = intertia_out + damping_out;

  speed_filter.update(speed_set);
}

}  // namespace sp