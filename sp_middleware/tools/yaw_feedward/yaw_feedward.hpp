// [解析] yaw 轴力矩前馈：按 T = J·α 提前算出该补多少力矩，弥补 PID 只能事后纠错的滞后。
// [解析] 主干代码默认不调用它，属可选增强模块；用法见同目录 readme.md 的三行示例。
// [解析] 三个输出 intertia_out / damping_out / out 只读；damping_out_max_ 默认 0，阻尼项等于关闭。

#ifndef SP__YAW_FEEDWARD_HPP
#define SP__YAW_FEEDWARD_HPP

#include "tools/low_pass_filter/low_pass_filter.hpp"
#include "tools/math_tools/math_tools.hpp"

namespace sp
{
class YawFeedward
{
private:
  const float dt_;       // 控制周期，单位s
  const float inertia_;  // 转动惯量，单位kg·m^2
  const float k_ff_;     // 前馈增益，单位N·m/(rad/s^2)
  const float damping_;  // 阻尼系数，单位N·m/(rad/s)
  const float damping_out_max_;// 阻尼补偿最大值，单位N·m
  float alpha_set_;      // 目标角加速度，单位rad/s^2

  sp::LowPassFilter speed_filter{0.1f};  // 速度低通滤波器

public:
  // [解析] 成员名 intertia 少了一个字母 n，是历史笔误；改名会破坏调用方，照抄即可。
  float out = 0;  // 输出力矩，单位N·m（只读）
  float damping_out = 0;  // 阻尼补偿，单位N·m（只读）
  float intertia_out = 0; // 惯性补偿，单位N·m（只读）
  YawFeedward(float dt, float inertia, float k_ff = 1.0f, float damping = 0.0f, float damping_out_max = 0.0f);
  void calc(float speed_set);
};

}  // namespace sp

#endif  // YAW_FEEDWARD_HPP