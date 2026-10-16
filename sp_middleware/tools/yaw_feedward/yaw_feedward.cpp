// [解析] ==========================================================================
// [解析] 【文件】sp_middleware/tools/yaw_feedward/yaw_feedward.cpp
// [解析] 【作用】实现 yaw 轴力矩前馈：把"目标角速度的变化率"当成角加速度，用 T = J·α 算出补偿力矩。
// [解析] 【被谁用】想给云台 yaw 加前馈的代码（见同目录 readme.md 的三行调用示例）。
// [解析] 【看代码的重点】1) 角加速度为什么要用"两帧目标速度做差 ÷ dt"来近似；
// [解析]   2) 为什么要先过一个低通滤波器（差分会把噪声放大）；
// [解析]   3) 阻尼项默认被 damping_out_max_ = 0 压成了 0（limit_max 的特性）。
// [解析] ==========================================================================

#include "yaw_feedward.hpp"

namespace sp
{
// [解析] 构造函数：只是把 5 个参数存进 const 成员，不做任何计算，也不碰硬件。
// [解析] const 成员必须在"初始化列表"（冒号后面那串）里赋值，不能写在花括号里 —— 这是 C++ 的硬性规定。
// [解析] 花括号里是空的，因为真的没有什么要做的。
// [解析] 注意：成员初始化顺序由"成员在类里的声明顺序"决定，跟这里书写的顺序无关（这里恰好一致）。
YawFeedward::YawFeedward(float dt, float inertia, float k_ff, float damping, float damping_out_max)
: dt_(dt), inertia_(inertia), k_ff_(k_ff), damping_(damping), damping_out_max_(damping_out_max)
{
}

// [解析] 每个控制周期调用一次，输入是"目标角速度"（rad/s），通常在 1kHz 的任务里调用。
// [解析] 整个函数就是 readme.md 里那条物理公式的代码版：T = J·α + 阻尼项。
void YawFeedward::calc(float speed_set)
{
  // [解析] 用"差分"近似求导得到目标角加速度 α（单位 rad/s^2）：
  // [解析]   α ≈ (本帧目标速度 - 上一帧速度) / 控制周期。
  // [解析] 为什么这里的"上一帧速度"不直接存一个成员变量，而是用 speed_filter.out？
  // [解析]   因为 speed_filter 的输出正是"上一帧输入经过低通后的值"，
  // [解析]   相当于顺手把差分前的信号滤了一遍，抑制了传感器/指令噪声被 dt 放大。
  // [解析] 代价：滤波器会滞后，所以这个 α 估计比真实值慢几毫秒；dt_ 做分母，传 0 会得到 inf/NaN。
  this->alpha_set_ = (speed_set - speed_filter.out) / dt_;

  // [解析] 惯性补偿：T = k_ff × J × α，单位 N·m。
  // [解析]   物理含义：要让质量为 J 的负载获得 α 的角加速度，电机必须多出这么多力矩，
  // [解析]   否则 PID 要等"速度没跟上"才反应过来，表现为起步迟钝、跟随有滞后。
  // [解析] 注意：这里直接用 alpha_set_，没有做任何限幅，α 突变时这一项可能瞬间很大，
  // [解析]   实际使用时通常还要靠电机的力矩上限或后面的限幅兜底。
  this->intertia_out = k_ff_ * inertia_ * this->alpha_set_;
  
  // [解析] 阻尼补偿：力矩与目标速度成正比，方向与运动方向相同（加速时助力、减速时反拖），
  // [解析]   效果类似给云台加了"电子阻尼"，能压住振荡、让运动更"跟手"。
  // [解析] 用 limit_max 做正负对称限幅，上限就是构造时传的 damping_out_max_。
  // [解析] ★ 注意：damping_out_max_ 默认值是 0.0，而 limit_max(x, 0) 会把任何数都夹成 0，
  // [解析]   所以不传这个参数时，阻尼补偿这一项恒为 0（等于没开），别以为是代码没生效。
  this->damping_out = limit_max(speed_set * damping_, damping_out_max_);

  // [解析] 总输出 = 惯性补偿 + 阻尼补偿，单位 N·m。
  // [解析] 调用者拿到 out 后，直接叠加到速度环 PID 的输出上（见 readme.md 的 give_torque 那行），
  // [解析] 注意是"叠加"而不是"替换"：PID 仍然在闭环纠错，前馈只是让它少出力、出得更早。
  this->out = intertia_out + damping_out;

  // [解析] 最后一步才更新滤波器（顺序很关键！）：
  // [解析]   因为前面算 α 时用的就是"上一帧的 speed_filter.out"，
  // [解析]   如果把这行提到最前面，就会变成"自己减自己"，α 恒等于 0，前馈直接失效。
  // [解析] 一阶低通公式：out = 0.1×新值 + 0.9×上次输出（系数 0.1 来自 hpp 里的构造参数）。
  speed_filter.update(speed_set);
}

}  // namespace sp