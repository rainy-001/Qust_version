// [解析] ==========================================================================
// [解析] 【文件】sp_middleware/tools/yaw_feedward/yaw_feedward.hpp
// [解析] 【作用】云台 yaw 轴的"力矩前馈补偿"计算器：根据目标角速度算出"要额外补多少力矩"，
// [解析]         让云台起步更快、刹车更干脆（弥补 PID 只能"事后纠错"的缺点）。
// [解析] 【被谁用】目前主干代码里没有默认调用它（是作者提供的可选增强模块）。
// [解析]         用法见同目录 readme.md：
// [解析]           sp::YawFeedward yaw_feedward(dt, inertia, k_ff, damping, damping_out_max);
// [解析]           yaw_feedward.calc(yaw_pos_pid.out);              // 每个周期调一次
// [解析]           give_torque = yaw_feedward.out + yaw_speed_pid.out;  // 前馈 + PID 一起给电机
// [解析] 【关键点】1) 用到的物理公式是 T = J·α（力矩 = 转动惯量 × 角加速度），
// [解析]   角加速度用"相邻两帧目标角速度之差 ÷ dt"近似；
// [解析]   2) damping_out_max_ 默认 0，意味着默认情况下阻尼补偿恒为 0（等于没开）；
// [解析]   3) 名字里的 feedward 是 feedforward（前馈）的拼写笔误，代码里到处都这么写，别改。
// [解析] ==========================================================================

#ifndef SP__YAW_FEEDWARD_HPP
#define SP__YAW_FEEDWARD_HPP

#include "tools/low_pass_filter/low_pass_filter.hpp"
#include "tools/math_tools/math_tools.hpp"

namespace sp
{
// [解析] 【前馈是什么】PID 是"看到误差才动作"，天生慢半拍；前馈是"提前按物理规律算好该出多大力"，
// [解析]   不等误差出现就先把力矩加上去。本类就是给 yaw 轴做这件事。
// [解析] 【三个输出分量】
// [解析]   intertia_out（惯性补偿）：T = k_ff · J · α，负责"加速/减速时克服转动惯量"。
// [解析]   damping_out （阻尼补偿）：与目标速度成正比，相当于给系统加"电子阻尼"，抑制振荡。
// [解析]   out           （总输出）  ：两者相加，直接叠加到速度环 PID 的输出上。
class YawFeedward
{
private:
  // [解析] 下面 5 个 const 成员都在构造函数里赋值，之后不能再改（const 保证不会被误写）。
  const float dt_;       // 控制周期，单位s
  // [解析] ↑ 多久调用一次 calc()，单位秒。1kHz 控制就是 0.001s。它做分母，千万不能传 0。
  const float inertia_;  // 转动惯量，单位kg·m^2
  // [解析] ↑ yaw 轴（含云台、枪管等）等效转动惯量 J。越大说明越"沉"，越难加速。
  // [解析]   实际值很难精确测量，一般按经验试出来或者做参数辨识。
  const float k_ff_;     // 前馈增益，单位N·m/(rad/s^2)
  // [解析] ↑ 前馈总增益，默认 1.0。它相当于"我对模型有多信任"：
  // [解析]   设 1.0 表示完全按理论公式出力；实际由于摩擦、模型误差，往往要调到 1 附近再看效果。
  const float damping_;  // 阻尼系数，单位N·m/(rad/s)
  // [解析] ↑ 目标速度 × 这个系数 = 阻尼补偿力矩。给得越大"刹车"越强、越不容易振荡，
  // [解析]   但太大会让云台变得迟钝、发烫。默认 0.0（不补偿）。
  const float damping_out_max_;// 阻尼补偿最大值，单位N·m
  // [解析] ↑ 阻尼补偿的限幅上限（正负对称，即 [-max, +max]）。
  // [解析]   注意：默认值也是 0.0，而 limit_max(x, 0) 永远返回 0，
  // [解析]   所以不显式传这个参数的话，阻尼补偿这一项等于关闭。
  float alpha_set_;      // 目标角加速度，单位rad/s^2
  // [解析] ↑ 由"本帧目标速度 - 上一帧滤波后的速度"除以 dt_ 得到，是内部中间量，
  // [解析]   外部读不到（私有），想观察加速度请自己在 calc() 附近打印。

  sp::LowPassFilter speed_filter{0.1f};  // 速度低通滤波器
  // [解析] ↑ alpha = 0.1 的一阶低通：out = 0.1×新值 + 0.9×上次输出，滤得很狠。
  // [解析]   为什么要滤：角加速度是"做差再除以很小的 dt"，噪声会被放大几十上百倍，
  // [解析]   不滤的话前馈力矩会变成抖动源。代价是这一项有相位滞后（响应变慢）。
  // [解析]   滤波器的 update() 在 calc() 的最后才调用，所以 calc() 里读到的 out 是"上一帧的滤波结果"。

public:
  // [解析] 三个公开输出成员，注释里写的"只读"是约定：请读它们、不要去赋值，
  // [解析] 否则算出来的前馈量会被你覆盖掉。它们在类内部每个周期都会被刷新。
  float out = 0;  // 输出力矩，单位N·m（只读）
  // [解析] ↑ 最终结果 = intertia_out + damping_out，直接加到电机的力矩指令上。
  float damping_out = 0;  // 阻尼补偿，单位N·m（只读）
  // [解析] ↑ 阻尼分量 = limit_max(speed_set × damping_, damping_out_max_)。
  float intertia_out = 0; // 惯性补偿，单位N·m（只读）
  // [解析] ↑ 惯性分量 = k_ff_ × inertia_ × alpha_set_。
  // [解析] 注意：成员名拼成了 intertia（少了一个 n），正确的英文是 inertia；
  // [解析]   但改名会破坏已有调用代码，所以保留现状，写代码时照着抄就行。
  YawFeedward(float dt, float inertia, float k_ff = 1.0f, float damping = 0.0f, float damping_out_max = 0.0f);
  // [解析] 构造函数，参数含义见上面的成员说明，顺序不能传错（全是 float，编译器不会帮你挡）：
  // [解析]   dt              控制周期，单位 s（如 0.001f），必须 > 0
  // [解析]   inertia         转动惯量 J，单位 kg·m^2（如 0.02f）
  // [解析]   k_ff            前馈增益，默认 1.0，一般不用改
  // [解析]   damping         阻尼系数，默认 0.0（即不启用阻尼补偿）
  // [解析]   damping_out_max 阻尼限幅，默认 0.0（默认情况下把阻尼项完全压成 0）
  void calc(float speed_set);
  // [解析] 每个控制周期调用一次，喂进"目标角速度"（rad/s），内部刷新 out 等三个输出。
  // [解析] 参数 speed_set：目标角速度，单位 rad/s —— 通常直接传位置环 PID 的输出。
  // [解析] 返回值：无（结果放在公开成员里读）。
  // [解析] 副作用：会更新内部的低通滤波器状态，所以必须"周期性、不跳帧"地调用，
  // [解析]   否则算出来的角加速度会失真。
};

}  // namespace sp

#endif  // YAW_FEEDWARD_HPP