// [解析] 模糊 PID 的实现：三张规则表 + 三角形隶属度 + 加权平均解模糊 + 带模糊增益的 PID 计算。
// [解析] 数据流是 membership_calc() -> fuzzy_inference() -> calc()，自下往上读最顺。
// [解析] 一句话理解：把"误差大/中/小"的调参经验写成表，用隶属度（0~1）描述"有多像某一档"，
// [解析]   按表查出该往哪调参数，再加权平均回一个具体数字 —— 这就是解模糊。
// [解析] 关键约束：membership_params_ 的行顺序必须和 NB_~PB_ 的编号一一对应，错行则整张规则表作废。

#include "fuzzy_pid.hpp"

#include <algorithm>
#include <cmath>

namespace sp
{

// 模糊论域分级常量值，对应语言变量 NB, NM, NS, ZO, PS, PM, PB
// [解析] 七档语言变量 NB/NM/NS/ZO/PS/PM/PB 取值 -3/-2/-1/0/+1/+2/+3，归一化后的误差就落在这个论域上。
// [解析] 它们只在本 .cpp 内可见；用 constexpr int 是为了规则表能把它们直接当数组元素和下标用。
inline static constexpr int NB_ = -3;
inline static constexpr int NM_ = -2;
inline static constexpr int NS_ = -1;
inline static constexpr int ZO_ = 0;
inline static constexpr int PS_ = 1;
inline static constexpr int PM_ = 2;
inline static constexpr int PB_ = 3;

// KP模糊规则表
// [解析] 读法：行是误差 e 的档位（上 NB 下 PB），列是误差变化率 ec（左 NB 右 PB），格子里的值就是 Δkp 档位。
// [解析] 趋势：e 为 NB 时无论 ec 都给 PB/PM 的正修正 —— 对应"偏差大就加大 kp 赶紧拽回来"的经验。
inline static constexpr FuzzyPID::RuleTable delta_kp_rule_ = {{
  //NB   NM   NS   ZO   PS   PM   PB   (de →)
  {{PB_, PB_, PM_, PM_, PS_, ZO_, ZO_}},  // NB (e ↓)
  {{PB_, PB_, PM_, PS_, PS_, ZO_, NS_}},  // NM
  {{PM_, PM_, PM_, PS_, ZO_, NS_, NS_}},  // NS
  {{PM_, PM_, PS_, ZO_, NS_, NM_, NM_}},  // ZO
  {{PS_, PS_, ZO_, NS_, NS_, NM_, NM_}},  // PS
  {{PS_, ZO_, NS_, NM_, NM_, NM_, NB_}},  // PM
  {{ZO_, ZO_, NM_, NM_, NM_, NB_, NB_}}   // PB
}};
// [解析] Δki 表的方向和 Δkp 相反：误差大时压小 ki、接近目标才允许加大，等价于积分分离。
inline static constexpr FuzzyPID::RuleTable delta_ki_rule_ = {{
  //NB   NM   NS   ZO   PS   PM   PB   (de →)
  {{NB_, NB_, NM_, NM_, NS_, ZO_, ZO_}},  // NB (e ↓)
  {{NB_, NB_, NM_, NS_, NS_, ZO_, ZO_}},  // NM
  {{NB_, NM_, NS_, NS_, ZO_, PS_, PS_}},  // NS
  {{NM_, NM_, NS_, ZO_, PS_, PM_, PM_}},  // ZO
  {{NM_, NS_, ZO_, PS_, PS_, PM_, PB_}},  // PS
  {{ZO_, ZO_, PS_, PS_, PM_, PB_, PB_}},  // PM
  {{ZO_, ZO_, PS_, PM_, PM_, PB_, PB_}}   // PB
}};
// [解析] Δkd 表中间大片为 0：D 项是阻尼、最怕噪声，只在"误差大且变化剧烈"时才给大的正修正。
inline static constexpr FuzzyPID::RuleTable delta_kd_rule_ = {{
  //NB   NM   NS   ZO   PS   PM   PB   (de →)
  {{PS_, NS_, NB_, NB_, NB_, NM_, PS_}},  // NB (e ↓)
  {{PS_, NS_, NB_, NM_, NM_, NS_, ZO_}},  // NM
  {{ZO_, NS_, NM_, NM_, NS_, NS_, ZO_}},  // NS
  {{ZO_, NS_, NS_, NS_, NS_, NS_, ZO_}},  // ZO
  {{ZO_, ZO_, ZO_, ZO_, ZO_, ZO_, ZO_}},  // PS
  {{PB_, NS_, PS_, PS_, PS_, PS_, PB_}},  // PM
  {{PB_, PM_, PM_, PM_, PS_, PS_, PB_}}   // PB
}};

// clang-format off
// 模糊隶属度函数参数
// [解析] 7 个三角形隶属度函数，每行 {左底点, 顶点, 右底点}，横轴是归一化论域 [-3, +3]。
// [解析] 相邻三角形在底点处搭肩，任一输入的各档隶属度之和恒为 1，所以输出对输入连续、不会跳变。
// [解析] 行顺序必须和 NB_~PB_ 严格对应；边缘行顶点与底点重合，靠 membership_calc 的边界判断躲开除零。
inline static constexpr float membership_params_[7][3] = {
  {-3,-3,-2},    // NB
  {-2,-2,-1},    // NM
  {-1,-1, 0},    // NS
  {-1, 0, 1},    // ZO
  { 0, 1, 1},    // PS
  { 1, 2, 2},    // PM
  { 2, 3, 3}     // PB
};
// clang-format on

FuzzyPID::FuzzyPID(
  float dt, float kp0, float ki0, float kd0, float max_out, float max_iout, float error_scale,float error_rate_scale,
  float alpha, bool angular, bool dynamic) 
: dt_(dt),
  kp_(kp0),
  ki_(ki0),
  kd_(kd0),
  max_out_(max_out),
  max_iout_(max_iout),
  error_scale_(error_scale),
  error_rate_scale_(error_rate_scale),
  alpha_(alpha),
  angular_(angular),
  dynamic_(dynamic)
{
}

// [解析] 三角形隶属度：x 在 [a,b) 走上升段 (x-a)/(b-a)，在 (b,c] 走下降段 (c-x)/(c-b)，两侧之外为 0。
// [解析] NB 行 {-3,-3,-2} 的上升段分母为 0，靠前面 "x <= a 直接返回" 拦住；自己改表时要重新验算。
float FuzzyPID::membership_calc(float x, int set) const
{
  const auto & params = membership_params_[static_cast<int>(set)];

  // 三角形隶属度函数
  if (x <= params[0] || x >= params[2]) return 0.0f;
  if (x <= params[1]) 
    return (x - params[0]) / (params[1] - params[0]);
  return (params[2] - x) / (params[2] - params[1]);
}

// [解析] 把清晰输入变成模糊结论再变回清晰值：归一化 -> 求隶属度 -> 按规则表加权平均 -> 除以 3 归一。
// [解析] 返回 [-1,1]，与具体增益无关，所以 Δkp/Δki/Δkd 三张表可以共用这一个函数。
float FuzzyPID::fuzzy_inference(float error, float error_rate, const RuleTable & rules) const
{
  // 输入归一化到[-3, 3]
  // [解析] error/error_scale*3 是"量化因子"换算；error_scale_ 为 0 会直接在这里除零得到 inf。
  float e = error / error_scale_ * 3.f;
  float ec = error_rate / error_rate_scale_ * 3.f;

  // 限制输入范围
  e = fabs(e) > 3.0f ? (e > 0.0f ? 3.0f : -3.0f) : e;
  ec = fabs(ec) > 3.0f ? (ec > 0.0f ? 3.0f : -3.0f) : ec;

  // 计算隶属度
  std::array<float, 7> e_degrees, ec_degrees;
  for (int i = 0; i < 7; ++i) {
    e_degrees[i] = membership_calc(e, static_cast<int>(i));
    ec_degrees[i] = membership_calc(ec, static_cast<int>(i));
  }

  // 规则推理
  float fuzzy_rule = 0.f, weight_sum = 0.f;
  for (int i = 0; i < 7; ++i) {
    for (int j = 0; j < 7; ++j) {
      // 计算规则激活强度
      const float w = e_degrees[i] * ec_degrees[j];
      if (w <= 0.0f) continue;
      // 更新输出隶属度
      fuzzy_rule += w * rules[i][j];
      weight_sum += w;
    }
  }

  // 防止除零
  // [解析] 理论上不会为 0，兜底成 1e-6；真为 0 时 fuzzy_rule 也是 0，等价于"不做任何模糊修正"。
  if (weight_sum == 0.0f) {
    weight_sum = 1e-6f;
  }
  // 计算模糊推理结果的加权平均值
  fuzzy_rule /= weight_sum;

  return fuzzy_rule / 3.f;
}

// [解析] 每周期调一次：微分先行 + 滤波 -> 移位算 err / err_rate -> 查三张表得 Δkp/Δki/Δkd
// [解析]   -> 用 fuzzy_kp/ki/kd 算 P/I/D -> 限幅写 out。本类没有 clear()，重新起步只能自己把 iout 和 out 归零。
// [解析] 注意 err_rate 是纯数值差分，噪声会被放大；反馈抖动大时先做低通再送进来。
void FuzzyPID::calc(float set, float fdb)
{
  // 微分先行
  // [解析] 这里没有 dout_ready_ 那种"第一帧保护"：首拍 data.fdb 还是 0，会算出一个 (0 - fdb) 的假微分。
  this->data.dbuf[2] = this->data.dbuf[1];
  this->data.dbuf[1] = this->data.dbuf[0];
  this->data.dbuf[0] = angular_ ? limit_angle((this->data.fdb - fdb)) : (this->data.fdb - fdb);

  // 滤波
  this->data.dbuf[0] = alpha_ * this->data.dbuf[0] + (1 - alpha_) * this->data.dbuf[1];

  this->data.err[2] = this->data.err[1];
  this->data.err[1] = this->data.err[0];
  this->data.err[0] = angular_ ? limit_angle(set - fdb) : (set - fdb);

  this->data.err_rate = (this->data.err[0] - this->data.err[1]) / dt_;

  this->data.set = set;
  this->data.fdb = fdb;

  this->data.dynamic_ki = ki_ / (1 + std::abs(this->data.err[0])); 

  // 假设fuzzy_out的范围是PID参数的1/2
  // [解析] fuzzy_inference() 返回 [-1,1]，乘"基础增益/2"得到 Δ；所以增益只在 [0.5k, 1.5k] 内浮动。
  // [解析] Δki 的基准是 (dynamic_ ? dynamic_ki : ki_)，变速积分打开时两个机制的效果会叠加。
  delta_kp_ = fuzzy_inference(this->data.err[0], this->data.err_rate, delta_kp_rule_) * kp_ / 2.f;
  delta_ki_ = fuzzy_inference(this->data.err[0], this->data.err_rate, delta_ki_rule_) * (dynamic_ ? this->data.dynamic_ki : ki_) / 2.f;
  delta_kd_ = fuzzy_inference(this->data.err[0], this->data.err_rate, delta_kd_rule_) * kd_ / 2.f;

  this->data.fuzzy_kp = kp_ + delta_kp_;
  this->data.fuzzy_ki = (dynamic_ ? this->data.dynamic_ki : ki_) + delta_ki_;
  this->data.fuzzy_kd = kd_ + delta_kd_;

  this->data.pout = this->data.fuzzy_kp * this->data.err[0];

  this->data.trapezoid = (this->data.err[0] + this->data.err[1]) / 2 * dt_;  // 梯形积分
  this->data.iout =
    limit_max(this->data.iout + this->data.fuzzy_ki * this->data.trapezoid, max_iout_);

  // [解析] 这里没有除以 dt_：dbuf[0] 是"两拍差值"而不是每秒量纲，kd 比 tools/pid 多一个秒，移植要换算。
  this->data.dout = this->data.fuzzy_kd * this->data.dbuf[0];

  this->out = limit_max(this->data.pout + this->data.iout + this->data.dout, max_out_);
}
}  // namespace sp