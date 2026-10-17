// [解析] 模糊 PID：在普通 PID 上叠一层 7×7 模糊规则，按误差大小和误差变化率实时微调 kp/ki/kd。
// [解析] 实际增益 = kp0+Δkp、ki0+Δki、kd0+Δkd，Δ 由模糊推理算出，所以参数是"活"的。
// [解析] error_scale / error_rate_scale 是把真实误差归一化到 [-3,3] 论域的尺度，必须大于 0。
// [解析] 用法与 sp::PID 类似（构造 -> 每周期 calc），但只有 calc(set, fdb) 一个版本，也没有 clear()。

#ifndef SP__FUZZY_PID_HPP
#define SP__FUZZY_PID_HPP

#include <array>
#include "tools/math_tools/math_tools.hpp"

namespace sp
{
// [解析] 内部状态 + 各项分解输出；重点是 fuzzy_kp/ki/kd —— 它们是套上模糊修正后真正生效的增益。
struct FuzzyData
{
  float set = 0;  // 设定值
  float fdb = 0;  // 反馈值(feedback)

  float pout = 0;  // P项输出值
  float iout = 0;  // I项输出值
  float dout = 0;  // D项输出值

  float err[3] = {0};   // 误差缓冲区
  float err_rate = 0;   // 误差变化率
  float dbuf[3] = {0};  // D项滤波器缓冲区

  float trapezoid = 0;  // 梯形积分
  float dynamic_ki = 0; // 动态积分

  float fuzzy_kp = 0;  // P项系数（kp0+Δkp）
  float fuzzy_ki = 0;  // I项系数（ki0+Δki）
  float fuzzy_kd = 0;  // D项系数（kd0+Δkd）
};

// [解析] 接口刻意做得和 sp::PID 很像，方便互换着试；calc() 里有两层 7×7 循环（49 条规则），比普通 PID 重不少。
class FuzzyPID
{
public:
  // [解析] 7×7 的 int 矩阵：行列都是 NB/NM/NS/ZO/PS/PM/PB 七档，元素取值 -3~+3，就是一张模糊规则表。
  using RuleTable = std::array<std::array<int, 7>, 7>;

  FuzzyPID(
    float dt, float kp0, float ki0, float kd0, float max_out, float max_iout, float error_scale, float error_rate_scale,
    float alpha = 1, bool angular = false, bool dynamic = false);

  float out = 0;   // PID输出值
  FuzzyData data;  // PID数据

  // [解析] 一次完整流程：微分先行 + 滤波 -> 误差窗口移位 -> 算 err_rate -> 查三张表得 Δkp/Δki/Δkd
  // [解析]   -> 用调整后的增益算 P/I/D -> 限幅写 out。注意 data.iout 有记忆，重新起步只能自己归零。
  void calc(float set, float fdb);

private:

  const float dt_;
  const float kp_, ki_, kd_;
  const float max_out_, max_iout_;
  const float error_scale_, error_rate_scale_;
  const float alpha_;
  const bool angular_;  // 是否为角度控制
  const bool dynamic_;  // 是否为变速积分

  float delta_kp_ = 0;  // P项系数模糊量
  float delta_ki_ = 0;  // I项系数模糊量
  float delta_kd_ = 0;  // D项系数模糊量

  // 模糊推理+解模糊
  // [解析] 把 error/error_rate 归一化到 [-3,3] 后按 rules 表加权平均出结论，返回 [-1,1] 的修正系数。
  float fuzzy_inference(float error, float error_rate, const RuleTable & rules) const;
  // 隶属度计算
  // [解析] 三角形隶属度：返回 x 属于第 set 档的程度（0~1）；set 直接当数组下标用，传 -1 或 7 会越界。
  float membership_calc(float x, int set) const;
};

}  // namespace sp

#endif  // SP__FUZZY_PID_HPP