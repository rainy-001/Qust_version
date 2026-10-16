// [解析] ==========================================================================
// [解析] 【文件】sp_middleware/tools/fuzzy_pid/fuzzy_pid.cpp
// [解析] 【作用】模糊 PID 的具体实现：包含三张模糊规则表、隶属度函数、模糊推理，
// [解析]         以及"每周期微调增益再算 PID"的完整流程。
// [解析] 【先看什么】建议按这个顺序读：
// [解析]   1) 开头的 NB_~PB_ 七个常量 —— 七档语言变量的编号；
// [解析]   2) 三张 RuleTable —— 规则表，行列都是那七档；
// [解析]   3) membership_params_ —— 三角形隶属度函数的顶点坐标；
// [解析]   4) membership_calc() -> fuzzy_inference() -> calc()，正好是数据流方向。
// [解析] 【一句话理解模糊控制】把"误差大/中/小"这种人类经验写成表格，
// [解析]   用数学上的"隶属度"（0~1）去描述"有多像大/有多像小"，
// [解析]   再按规则表查出该往哪个方向调参数，最后加权平均成一个具体数字（解模糊）。
// [解析] ==========================================================================

#include "fuzzy_pid.hpp"

#include <algorithm>
#include <cmath>

namespace sp
{

// 模糊论域分级常量值，对应语言变量 NB, NM, NS, ZO, PS, PM, PB
// [解析] 这是模糊控制里的标准命名，背下来以后看任何模糊论文都不慌：
// [解析]   NB = Negative Big   负大      值 -3
// [解析]   NM = Negative Medium 负中     值 -2
// [解析]   NS = Negative Small  负小     值 -1
// [解析]   ZO = Zero            零       值  0
// [解析]   PS = Positive Small  正小     值 +1
// [解析]   PM = Positive Medium 正中     值 +2
// [解析]   PB = Positive Big    正大     值 +3
// [解析] 【为什么用 -3~+3】这是"归一化论域"的常见取法：模糊推理前会先把真实误差
// [解析]   除以 scale 再乘 3，强行映射到这七个整数档上，这样规则表就与具体物理量解耦了。
// [解析] 【为什么用 constexpr int 而不是宏】有类型、有作用域，编译器还能直接内联，
// [解析]   调试时也能看到变量名。inline static 是为了让它们只有一份实体（C++17 写法）。
// [解析] 【注意】这些常量只在本编译单元（.cpp）内可见，头文件里并没有，
// [解析]   所以外面拿不到 NB_ 这类名字。
inline static constexpr int NB_ = -3;
inline static constexpr int NM_ = -2;
inline static constexpr int NS_ = -1;
inline static constexpr int ZO_ = 0;
inline static constexpr int PS_ = 1;
inline static constexpr int PM_ = 2;
inline static constexpr int PB_ = 3;

// KP模糊规则表
// [解析] 【规则表怎么读】行是误差 e 的档位（NB 在最上面，PB 在最下面），
// [解析]   列是误差变化率 ec 的档位（NB 在最左，PB 在最右），表里的值就是 Δkp 的档位。
// [解析] 【这张表的物理含义（以第一行为例）】e 为 NB（误差是很负的大值），
// [解析]   此时无论 ec 是什么，Δkp 都取 PB/PM（正的大修正）—— 意思是"偏差这么大，
// [解析]   必须把 P 增益加大，赶紧往回拽"。这正对应人工调参时"误差大就加大 kp"的经验。
// [解析] 【为什么中间一行（ZO）是 PM,PM,PS,ZO,NS,NM,NM】误差接近 0 时不再需要猛拉，
// [解析]   于是 Δkp 随 ec 的正负而变号：ec 正在变正说明要冲过头了，就减一点 kp。
// [解析] 【表是自己调的】这三张表来自经验（经典模糊 PID 论文里常见），
// [解析]   如果实车效果不好，是可以按同样思路手工改数字的 —— 改完直接看 fuzzy_kp 的变化。
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
// [解析] 【Δki 规则表】积分增益的修正。整体趋势是"误差大时把 ki 压小（NB/NM），
// [解析]   误差小时允许 ki 变大（PS/PM/PB）"。
// [解析]   道理和积分分离一样：大误差阶段积分容易饱和，先别积；接近目标了再让积分干活。
// [解析]   注意它和 Δkp 表方向是反的 —— 这正是模糊控制的价值：两项不再同步调整。
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
// [解析] 【Δkd 规则表】微分增益的修正，形状和前两张完全不同，这是有意的：
// [解析]   微分项是"阻尼"，它最怕噪声。所以规则设计上，只有"误差大且变化剧烈"那种
// [解析]   真正需要刹车的情况才给大的正修正（PB）；平稳区间（中间几行几列）大量填 0，
// [解析]   就是不希望 D 项在常态下被放大而把噪声引进系统。
// [解析] 【注意】表中有些格子（如 PM 行、NB 列是 NS_，PM 行、PS 列是 PS_）跳变较大，
// [解析]   属于经验凑出来的结果，不是连续函数，实车如果抖可以重点试改这几格。
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
// [解析] 【这是什么】7 个三角形隶属度函数，每个用 3 个数描述：
// [解析]   {左底点, 顶点, 右底点}，单位是归一化论域 [-3, +3]。
// [解析]   三角形内部按线性插值算隶属度，两个底点之外隶属度直接为 0。
// [解析] 【逐行解读】以 NB 行 {-3,-3,-2} 为例：三角形左底点和顶点都压在 -3 上，
// [解析]   说明它在 -3 处达到最高（隶属度 1），往右到 -2 处降到 0，是个"半三角形"
// [解析]   （被论域边界截断了）。PB 行 {2,3,3} 是它的镜像。
// [解析] 【ZO 行 {-1,0,1}】左右各展开 1 格的完整对称三角形，是最标准的一个。
// [解析] 【为什么相邻行要重叠】相邻两个三角形并非"尖对尖"，而是会搭上半格的肩。
// [解析]   以 NM={-2,-2,-1} 和 NS={-1,-1,0} 为例：在 x=-0.5 处，
// [解析]     NM 落在下降段，隶属度 = (-1-(-0.5))/(-1-(-2)) = 0.5；
// [解析]     NS 落在上升段，隶属度 = (-0.5-(-1))/(-1-(-1)) = 0.5。
// [解析]   两档各 0.5，加起来正好是 1 —— 这就是模糊控制要的效果：
// [解析]   "这个值有一半像负中、一半像负小"，于是同一时刻通常有两档被激活，
// [解析]   输出对输入是连续的，不会出现跳变。全程算下来所有档的隶属度之和恒为 1。
// [解析] 【clang-format off/on】告诉代码格式化工具别动这块，免得把表格排版打乱。
// [解析] 【注意】这张表必须和上面 NB_~PB_ 的编号顺序一一对应，
// [解析]   插入或调换任何一行都会让"档位 -> 隶属函数"错位，规则表就全乱了。
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

// [解析] 构造函数：初始化列表把 11 个参数存进对应的 const 成员。
// [解析] 注意形参名与成员名不同（kp0->kp_、error_scale->error_scale_），
// [解析]   这样写是为了区分别人的调用习惯和内部存储，也能避免同名遮蔽。
// [解析] 函数体为空：不在这里做任何计算，也不清零 data/out（好在它们都有 =0 默认值）。
// [解析] 【注意】这里没有对参数做任何合法性检查，error_scale 传 0 会在
// [解析]   fuzzy_inference() 里产生除零（得到 inf/nan），alpha 越界也不会报错。
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

// [解析] 三角形隶属度函数：回答"输入 x 有多像第 set 档"。
// [解析] 参数 x：已经归一化到 [-3, 3] 的输入（误差或误差变化率）。
// [解析] 参数 set：档位下标 0~6，对应 NB_~PB_。它直接被当作数组下标使用。
// [解析] 返回：隶属度，范围 [0, 1]。0 = 完全不像，1 = 完全就是这一档（三角形顶点）。
// [解析] 副作用：无（const 成员函数，只看不改）。
// [解析] 【分段公式怎么来的】三个参数 {a, b, c} 构成一个三角形：
// [解析]   x 在 [a,b) 之间走上升段：(x-a)/(b-a)，从 0 线性升到 1；
// [解析]   x 在 (b,c] 之间走下降段：(c-x)/(c-b)，从 1 线性降回 0。
// [解析] 【注意分母】NB_ 行是 {-3,-3,-2}，左底点和顶点重合（params[0]==params[1]），
// [解析]   下降段的分母 (c-b) = -2-(-3) = 1，没问题；上升段的分母 (b-a) = 0，
// [解析]   逻辑上确实是"除以 0"。之所以没事，是因为前一行的 x<=a 判定已经把
// [解析]   x<=-3 全部拦掉了，而 x>-3 时又必然 x>params[1]=-3，走不到上升段。
// [解析]   右边对称的 PB_ 行 {2,3,3} 同理（顶点与右底点重合，下降段用不到）。
// [解析]   这属于"靠数据凑巧躲开除零"，自己改这张表时一定要重新验算一遍。
float FuzzyPID::membership_calc(float x, int set) const
{
  // [解析] 取出这一档的三个坐标。用 const auto& 引用，避免每次拷贝整个数组。
  const auto & params = membership_params_[static_cast<int>(set)];

  // 三角形隶属度函数
  // [解析] 在两个底点之外（含端点），隶属度为 0。端点用 <= / >= 判断，
  // [解析]   所以恰好落在 -3 或 +3 这类边界值时也返回 0。
  if (x <= params[0] || x >= params[2]) return 0.0f;
  // [解析] 左半边（上升段）。顶点与左底点重合时分母会变成 0，
  // [解析]   但如上所述，那种情况下 x <= params[0] 已经提前返回了。
  if (x <= params[1]) 
    return (x - params[0]) / (params[1] - params[0]);
  // [解析] 右半边（下降段），落在 (params[1], params[2]) 开区间内。
  return (params[2] - x) / (params[2] - params[1]);
}

// [解析] 模糊推理 + 解模糊（把"清晰数值"变成"模糊结论"再变回"清晰数值"）。
// [解析] 参数 error：当前误差（真实物理量，比如 rad）。
// [解析] 参数 error_rate：误差变化率（单位/秒）。
// [解析] 参数 rules：要查哪张规则表（Δkp / Δki / Δkd 三张之一）。
// [解析] 返回：修正量系数，范围 [-1, 1]（末尾除以 3 归一化的结果）。
// [解析] 副作用：无。只读成员变量 error_scale_ / error_rate_scale_。
float FuzzyPID::fuzzy_inference(float error, float error_rate, const RuleTable & rules) const
{
  // 输入归一化到[-3, 3]
  // [解析] 这一步叫"量化因子"换算：把真实物理误差映射到模糊论域 [-3, 3]。
  // [解析] 公式是 (error / error_scale) * 3，也就是"error 达到 error_scale 时对应论域的 3"。
  // [解析] 【举例】误差控制里 error_scale=3.0f、误差=1.5rad，则 e=1.5，落在论域中部；
  // [解析]   误差=6rad 时 e=6，会被下面的限幅砍到 3。
  // [解析] 【踩坑】error_scale_ 必须是正数且不能为 0，否则这里除零得到 inf。
  float e = error / error_scale_ * 3.f;
  float ec = error_rate / error_rate_scale_ * 3.f;

  // 限制输入范围
  // [解析] 把超出论域的值夹到 ±3（饱和处理），避免"极端输入映射到不存在的档位"。
  // [解析] 注意这里用的是 fabs 而不是三目全写，NaN 输入时两个比较都为假，结果还是 NaN，
  // [解析]   会一路传下去（不会崩，但输出就没意义了），所以上游务必别喂 NaN。
  e = fabs(e) > 3.0f ? (e > 0.0f ? 3.0f : -3.0f) : e;
  ec = fabs(ec) > 3.0f ? (ec > 0.0f ? 3.0f : -3.0f) : ec;
  
  // 计算隶属度
  // [解析] 分别算出 e 和 ec 属于七档中每一档的程度，得到两个长度 7 的数组。
  // [解析] 由于论域设计得正好覆盖，每个数组的元素之和约等于 1。
  std::array<float, 7> e_degrees, ec_degrees;
  for (int i = 0; i < 7; ++i) {
    e_degrees[i] = membership_calc(e, static_cast<int>(i));
    ec_degrees[i] = membership_calc(ec, static_cast<int>(i));
  }

  // 规则推理
  // [解析] fuzzy_rule 累加"加权后的规则结论"，weight_sum 累加"总权重"，
  // [解析]   最后相除就是加权平均 —— 这就是最简单的"重心法/加权平均解模糊"。
  // [解析] 【数值含义】rules[i][j] 是 -3~+3 的整数档位，乘上权重 0~1 后加起来。
  float fuzzy_rule = 0.f, weight_sum = 0.f;
  for (int i = 0; i < 7; ++i) {
    for (int j = 0; j < 7; ++j) {
      // 计算规则激活强度
      // [解析] 第 (i,j) 条规则的"激活强度" = e 属于第 i 档的程度 × ec 属于第 j 档的程度。
      // [解析]   这叫"取小/乘积"型推理，乘积法更平滑，工程上最常用。
      // [解析]   因为两个隶属度数组各自只有一个或两个非零项，所以 49 条规则里
      // [解析]   其实只有 1~4 条真正被激活，其余都为 0。
      const float w = e_degrees[i] * ec_degrees[j];
      // [解析] 权重为 0 说明这条规则根本不被触发，直接跳过，省一次乘加。
      if (w <= 0.0f) continue;
      // 更新输出隶属度
      // [解析] 累加"结论 × 权重"。注意 rules[i][j] 的正负决定是"增大增益"还是"减小增益"。
      fuzzy_rule += w * rules[i][j];
      weight_sum += w;
    }
  }

  // 防止除零
  // [解析] 理论上 weight_sum 不会为 0（隶属度之和恒为 1），但浮点运算可能有极端情况，
  // [解析]   所以留一个兜底：换成 1e-6（极小正数），保证除法不会算出 inf。
  // [解析] 【注意】如果 weight_sum 真的是 0，那 fuzzy_rule 也必然是 0，
  // [解析]   所以最终返回的是 0（等价于"不做任何模糊修正"），是安全的降级行为。
  if (weight_sum == 0.0f) {
    weight_sum = 1e-6f;
  }
  // 计算模糊推理结果的加权平均值
  // [解析] 除完得到的是 -3~+3 之间的一个实数（对应某个"档位之间的中间值"）。
  fuzzy_rule /= weight_sum;

  // [解析] 再除以 3，把结果压到 [-1, 1]，方便调用方用"基础增益 × 比例"的方式缩放。
  // [解析] 【为什么要这一步】这样 fuzzy_inference() 的返回值就与具体增益无关了，
  // [解析]   三张规则表可以共用同一个函数，返回的都是"归一化修正系数"。
  return fuzzy_rule / 3.f;
}

// [解析] 核心入口：每个控制周期调用一次，喂进目标值 set 和反馈值 fdb。
// [解析] set/fdb 的单位由调用者决定（角度控制就是 rad），两者单位必须一致。
// [解析] 结果写在公开成员 out 里（已按 max_out 限幅），同时 data 里记录了全过程。
// [解析] 副作用：会更新 data 的全部字段，尤其是"有记忆"的 data.iout。
// [解析] 【注意】本类没有 clear()，无法一键复位；若要重新起步，需要自己把
// [解析]   data.iout 和 out 归零（直接改公开成员是可行的，但请确保清楚后果）。
void FuzzyPID::calc(float set, float fdb)
{
  // 微分先行
  // [解析] 与 tools/pid 的 calc(set, fdb) 完全同一套思路：先算微分、再更新误差。
  // [解析] 此时 data.fdb 里还是"上一帧反馈值"，所以能用它减本帧 fdb 得到反馈变化量。
  // [解析] 窗口后移一格（dbuf[2]←dbuf[1]←dbuf[0]），新的变化量写进 dbuf[0]。
  // [解析] 【和普通 PID 的一个差别】这里没有 dout_ready_ 这种"第一帧保护"标志：
  // [解析]   第一帧调用时 data.fdb 初值是 0，如果真实的 fdb 不是 0，
  // [解析]   就会算出一个 (0 - fdb) 的假微分。所以第一次调用前最好先自己喂一帧，
  // [解析]   或者接受起步瞬间 D 项有一次小扰动。
  // [解析] angular_ 为真时两帧角度差会先折到 (-π, π]，避免跨越 ±π 时出现巨大跳变。
  this->data.dbuf[2] = this->data.dbuf[1];
  this->data.dbuf[1] = this->data.dbuf[0];
  this->data.dbuf[0] = angular_ ? limit_angle((this->data.fdb - fdb)) : (this->data.fdb - fdb);

  // 滤波
  // [解析] 一阶低通滤波，抑制微分对噪声的放大：
  // [解析]   新值权重 alpha_，历史值权重 (1-alpha_)。alpha_=1 时等于不滤波。
  // [解析]   结果直接写回 dbuf[0]，后面 Kd 那行用的就是滤波后的值。
  this->data.dbuf[0] = alpha_ * this->data.dbuf[0] + (1 - alpha_) * this->data.dbuf[1];

  // [解析] 误差窗口后移一拍：err[0]=本周期误差，err[1]=上一周期误差。
  // [解析]   为什么只需要 err[1]？因为 err_rate 要算差分、梯形积分要算平均，都用上一拍。
  this->data.err[2] = this->data.err[1];
  this->data.err[1] = this->data.err[0];
  // [解析] 本周期误差 = 目标 - 反馈。角度控制时先折叠到 (-π, π] 取最短转向，
  // [解析]   单位仍是 rad；不折叠的话 359°→1° 会被当成"反向转 358°"。
  this->data.err[0] = angular_ ? limit_angle(set - fdb) : (set - fdb);

  // [解析] 误差变化率 ec：用"本拍误差 - 上拍误差"除以 dt_，得到"误差每秒变化多少"。
  // [解析] 【量纲】err 的单位 / 秒。它是模糊推理的第二个输入，
  // [解析]   代表"系统正在远离目标（同号放大）还是在靠近目标（异号）"。
  // [解析] 【注意】这是纯数值差分，对噪声很敏感。如果 err 本身抖动大，
  // [解析]   err_rate 会抖得更厉害，建议对反馈信号先做低通滤波再送进来。
  // [解析] 【注意】dt_ 不能为 0，否则这里会算出 inf。
  this->data.err_rate = (this->data.err[0] - this->data.err[1]) / dt_;

  // [解析] 记录本帧的 set/fdb（同样必须在算完微分、误差之后）。
  this->data.set = set;
  this->data.fdb = fdb;

  // [解析] 变速积分系数：ki / (1 + |err|)。误差越大，这个"临时 ki"越小，
  // [解析]   起到"大误差阶段少积分"的作用，和积分分离是同一个目的、两种实现。
  // [解析] 它会在下面既作为 Δki 的缩放基准，又（在 dynamic_ 打开时）作为 I 项基础增益。
  this->data.dynamic_ki = ki_ / (1 + std::abs(this->data.err[0])); 

  // 假设fuzzy_out的范围是PID参数的1/2
  // [解析] ↑ 这句是原作者的关键假设，也是本文件最需要理解的一行：
  // [解析]   fuzzy_inference() 返回的是 [-1, 1] 的归一化系数，
  // [解析]   乘上"基础增益 / 2"就得到了 Δ 的物理量纲。
  // [解析] 【由此推出的调整幅度】
  // [解析]   fuzzy 返回 +1 时：Δkp = +kp_/2，即 fuzzy_kp = 1.5 × kp_（增益放大 50%）；
  // [解析]   fuzzy 返回 -1 时：Δkp = -kp_/2，即 fuzzy_kp = 0.5 × kp_（增益减半）。
  // [解析]   所以 kp 实际的调节范围是 [0.5×kp_, 1.5×kp_]，绝不会变成负数或 0。
  // [解析]   如果觉得模糊调节"太温柔"或"太激进"，改这个 /2.f 的除数即可。
  // [解析] 【三行的差别】Δkp 和 Δkd 都用固定的 kp_/kd_ 缩放；
  // [解析]   而 Δki 用的基准是 (dynamic_ ? dynamic_ki : ki_)，也就是当变速积分打开时，
  // [解析]   模糊修正是在"已经缩小过的 ki"基础上再叠加，两个机制的效果会叠加在一起，
  // [解析]   调参时要留意别把 ki 压得过于小。
  delta_kp_ = fuzzy_inference(this->data.err[0], this->data.err_rate, delta_kp_rule_) * kp_ / 2.f;
  delta_ki_ = fuzzy_inference(this->data.err[0], this->data.err_rate, delta_ki_rule_) * (dynamic_ ? this->data.dynamic_ki : ki_) / 2.f;
  delta_kd_ = fuzzy_inference(this->data.err[0], this->data.err_rate, delta_kd_rule_) * kd_ / 2.f;

  // [解析] 把基础增益和模糊修正量相加，得到"本周期真正使用"的三个增益。
  // [解析]   fuzzy_kp = kp0 + Δkp，这正是"模糊 PID"这个名字的由来。
  // [解析]   I 项基础值同样受 dynamic_ 开关影响（打开时起点是 dynamic_ki）。
  // [解析] 【调试建议】在调试器里把 fuzzy_kp 和构造时的 kp_ 一起看，
  // [解析]   就能直观知道模糊规则此刻在"帮忙"还是"帮倒忙"。
  this->data.fuzzy_kp = kp_ + delta_kp_;
  this->data.fuzzy_ki = (dynamic_ ? this->data.dynamic_ki : ki_) + delta_ki_;
  this->data.fuzzy_kd = kd_ + delta_kd_;

  // [解析] P 项：用"模糊调整后"的 fuzzy_kp 乘误差，而不是固定 kp_。
  // [解析]   这正是模糊 PID 与普通 PID 在数学上的唯一区别（其余流程基本一致）。
  this->data.pout = this->data.fuzzy_kp * this->data.err[0];

  // [解析] 梯形积分：用 (本拍误差 + 上拍误差)/2 作为这一小段时间的平均误差，再乘 dt_，
  // [解析]   近似出误差曲线下的一小块面积（积分增量）。比矩形法更准。
  this->data.trapezoid = (this->data.err[0] + this->data.err[1]) / 2 * dt_;  // 梯形积分
  // [解析] 积分累加 + 抗饱和限幅：
  // [解析]   iout(新) = limit_max(iout(旧) + fuzzy_ki × 本拍增量, ±max_iout)。
  // [解析]   这里乘的是"模糊调整后的 ki"，所以模糊规则对积分的影响比普通 PID 更强。
  this->data.iout =
    limit_max(this->data.iout + this->data.fuzzy_ki * this->data.trapezoid, max_iout_);

  // [解析] D 项：fuzzy_kd × dbuf[0]。
  // [解析] 【注意！和 tools/pid 不一致的地方】普通 PID 这里是 kd_ * dbuf[0] / dt_，
  // [解析]   本行没有除以 dt_。也就是说 dbuf[0] 是"两拍之间的差值"，不是"每秒变化量"，
  // [解析]   所以这里 kd 的实际量纲比普通 PID 多了一个"秒"。
  // [解析]   后果：从这里抄 kd 到 tools/pid（或反过来）会差 dt_ 倍（1kHz 下就是 1000 倍），
  // [解析]   而且改变控制频率时本项的强度会跟着变。移植参数时请务必注意。
  this->data.dout = this->data.fuzzy_kd * this->data.dbuf[0];

  // [解析] 三项求和后用 max_out_ 对称限幅，写入 out。
  // [解析]   限幅是保护电机的最后一道关：无论前面的运算多离谱，下发的指令都不会超限。
  this->out = limit_max(this->data.pout + this->data.iout + this->data.dout, max_out_);
}
}  // namespace sp