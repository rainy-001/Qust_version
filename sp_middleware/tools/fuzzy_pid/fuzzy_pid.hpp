// [解析] ==========================================================================
// [解析] 【文件】sp_middleware/tools/fuzzy_pid/fuzzy_pid.hpp
// [解析] 【作用】"模糊 PID"控制器：在普通 PID 的基础上，用一套"模糊规则表"根据
// [解析]         当前误差大小和误差变化快慢，实时地微调 kp/ki/kd 三个参数。
// [解析] 【被谁用】和 sp::PID 用法几乎一样（构造 -> 每周期 calc），
// [解析]         主要用在"负载会变、工况会变"的场合，目前项目里使用较少，属于进阶工具。
// [解析] 【和普通 PID 的区别】普通 PID 的 kp/ki/kd 是固定的；
// [解析]         模糊 PID 把它变成 kp0+Δkp、ki0+Δki、kd0+Δkd，Δ 由模糊推理算出来。
// [解析] 【关键点】1) 多了 error_scale 和 error_rate_scale 两个"归一化尺度"，先用不到的可以先理解成
// [解析]           "误差多大算大"；2) 规则表是 7×7 的整数矩阵，行列都是
// [解析]           NB/NM/NS/ZO/PS/PM/PB 七档；3) 相关实现全部在 fuzzy_pid.cpp。
// [解析] 【阅读顺序】先看 fuzzy_pid.cpp 开头的规则表和隶属度函数，再回头看本文件。
// [解析] ==========================================================================

#ifndef SP__FUZZY_PID_HPP
#define SP__FUZZY_PID_HPP

#include <array>
#include "tools/math_tools/math_tools.hpp"

namespace sp
{
// [解析] FuzzyData 和 tools/pid 里的 PIDData 几乎是同一套字段，只是多了三个"模糊调整后"的
// [解析] 实际增益（fuzzy_kp / fuzzy_ki / fuzzy_kd）。调试时最该看的就是这三个：
// [解析]   把它们和构造时给的 kp0/ki0/kd0 对比，就能知道模糊规则到底把参数调偏了多少。
struct FuzzyData
{
  float set = 0;  // 设定值
  // [解析] ↑ 上一次 calc() 的目标值。单位由调用者决定（常见是 rad 或 rad/s）。
  float fdb = 0;  // 反馈值(feedback)
  // [解析] ↑ 上一次 calc() 的反馈值。

  float pout = 0;  // P项输出值
  // [解析] ↑ = fuzzy_kp × err[0]，注意用的是"模糊调整后"的 kp，不是构造时的 kp0。
  float iout = 0;  // I项输出值
  // [解析] ↑ 有记忆的积分累加器，被 max_iout 夹在 ±max_iout 内。
  float dout = 0;  // D项输出值
  // [解析] ↑ = fuzzy_kd × dbuf[0]。注意本类没有除以 dt_，见 .cpp 里的说明。

  float err[3] = {0};   // 误差缓冲区
  // [解析] ↑ 滑动窗口：err[0] 最新，err[1] 上一拍，err[2] 上上拍。
  float err_rate = 0;   // 误差变化率
  // [解析] ↑ 本周期的误差变化率 = (err[0] - err[1]) / dt_，单位是"误差单位/秒"。
  // [解析]   它是模糊推理的第二个输入（常写作 ec / de），代表"误差是在缩小还是在扩大"。
  float dbuf[3] = {0};  // D项滤波器缓冲区
  // [解析] ↑ D 项的滑动窗口，装的是"反馈值的变化量"（微分先行）。

  float trapezoid = 0;  // 梯形积分
  // [解析] ↑ 本周期的积分增量（未乘 ki）。
  float dynamic_ki = 0; // 动态积分
  // [解析] ↑ 变速积分用的临时 ki = ki0 / (1 + |err|)。
  
  float fuzzy_kp = 0;  // P项系数（kp0+Δkp）
  // [解析] ↑ 本周期真正生效的 P 增益，等于基础值 kp0 加上模糊推理给出的修正量 Δkp。
  float fuzzy_ki = 0;  // I项系数（ki0+Δki）
  // [解析] ↑ 本周期真正生效的 I 增益。注意基础值是 (dynamic_ ? dynamic_ki : ki0)，
  // [解析]   也就是变速积分开关打开时，起点就已经不是 ki0 了。
  float fuzzy_kd = 0;  // D项系数（kd0+Δkd）
  // [解析] ↑ 本周期真正生效的 D 增益。
};

// [解析] FuzzyPID 类：接口刻意做得和 sp::PID 很像，方便互相替换着试。
// [解析] 典型用法：
// [解析]   sp::FuzzyPID pid(1e-3f, 10.f, 0.5f, 0.05f, 20.f, 10.f, 1.0f, 10.0f);
// [解析]   pid.calc(target, now);  motor.set_current(pid.out);
// [解析] 【重要】这个类只有 calc(set, fdb) 一个版本（没有 set_dot 前馈版、
// [解析]   也没有积分分离版），需要那些功能请用 tools/pid。
// [解析] 【性能提醒】calc() 里有两层 7×7 循环（共 49 次规则遍历）+ 三角函数无关的浮点运算，
// [解析]   比普通 PID 重不少，在 1kHz 控制周期里要留意 CPU 占用。
class FuzzyPID
{
public:
  // [解析] RuleTable = 7 行 7 列的 int 矩阵，就是"模糊规则表"的类型别名。
  // [解析]   行列下标都对应七档语言变量（0=NB 到 6=PB），元素取值是 -3~+3（NB_~PB_）。
  // [解析]   用 std::array 而不是裸数组，好处是能被赋值/拷贝、也能安全地传引用。
  // [解析]   为什么是 7 档：档位越多控制越细腻，但规则表会按平方增长（7×7=49 条规则），
  // [解析]   对手写调参来说 7 档已经是能承受的上限。
  using RuleTable = std::array<std::array<int, 7>, 7>;

  // [解析] 【构造参数说明】前 5 个 (dt, kp0, ki0, kd0, max_out, max_iout) 含义与 sp::PID 完全相同：
  // [解析]   dt      控制周期(s)，用于算 err_rate 和梯形积分，必须和真实调用周期一致。
  // [解析]   kp0/ki0/kd0 三个"基础增益"，模糊推理只在这个基础上做小幅修正（Δ）。
  // [解析]   max_out 总输出对称限幅；max_iout 积分项单独限幅（防积分饱和）。
  // [解析] 【本类特有的两个参数】
  // [解析]   error_scale：误差的"归一化尺度"。模糊推理要先把误差除以它、再乘 3，
  // [解析]     映射到 [-3, 3] 的模糊论域上。所以它约等于"你认为多大的误差算最大"。
  // [解析]     举例：位置控制里误差可能到 3 rad，就填 3.0f 左右。
  // [解析]     填太小 → 一点点误差就被当成极端情况，修正量常年顶格，反而失去调节作用。
  // [解析]   error_rate_scale：误差变化率的归一化尺度，含义同理，只是量纲是"误差/秒"。
  // [解析]     它和 dt 有关，不同控制频率要重新调。
  // [解析] 【注意】这两个 scale 都必须大于 0（代码里要拿它们做除数），填 0 会算出 inf/nan。
  // [解析]   alpha    ：D 项低通滤波系数 [0,1]，1=不滤波。
  // [解析]   angular  ：true = 角度控制，误差先折到 (-π, π] 取最短转向。
  // [解析]   dynamic  ：true = 启用变速积分（用 dynamic_ki 代替 ki0）。
  FuzzyPID(
    float dt, float kp0, float ki0, float kd0, float max_out, float max_iout, float error_scale, float error_rate_scale,
    float alpha = 1, bool angular = false, bool dynamic = false);

  float out = 0;   // PID输出值
  // [解析] ↑ calc() 的结果，已按 max_out 限幅。只读。
  FuzzyData data;  // PID数据
  // [解析] ↑ 全部中间状态和各项分解输出，调试用。

  // [解析] 计算一次（喂目标值 set 和反馈值 fdb），内部流程是：
  // [解析]   1) 用上一帧反馈算微分并滤波（和普通 PID 的微分先行一样）；
  // [解析]   2) 算误差 err[0]、误差窗口移位；
  // [解析]   3) 算误差变化率 err_rate；
  // [解析]   4) 查三张模糊规则表，得出 Δkp/Δki/Δkd；
  // [解析]   5) 用调整后的增益算 P/I/D 三项，限幅后写进 out。
  // [解析] 副作用：更新 data 的所有字段，包括有记忆的 data.iout。没有 clear() 可用来复位，
  // [解析]   需要"重新起步"时只能自己把 data 和 out 归零（注意这一步代码里没提供接口）。
  void calc(float set, float fdb);

private:

  // [解析] 以下全部是 const，构造后不可改，想换参数只能重新构造对象。
  const float dt_;
  const float kp_, ki_, kd_;
  // [解析] ↑ 三个基础增益（对应构造函数的 kp0/ki0/kd0）。
  const float max_out_, max_iout_;
  // [解析] ↑ 输出限幅、积分限幅。
  const float error_scale_, error_rate_scale_;
  // [解析] ↑ 两个归一化尺度，模糊推理的除法分母，必须非 0。
  const float alpha_;
  // [解析] ↑ D 项滤波系数 [0,1]。
  const bool angular_;  // 是否为角度控制
  // [解析] ↑ 打开后误差会走 limit_angle()，单位必须是弧度。
  const bool dynamic_;  // 是否为变速积分
  // [解析] ↑ 打开后 I 项基础值用 ki/(1+|err|)，而不是固定 ki。

  float delta_kp_ = 0;  // P项系数模糊量
  // [解析] ↑ 模糊推理给出的 Δkp（已经乘过 kp0/2 缩放，见 .cpp）。
  float delta_ki_ = 0;  // I项系数模糊量
  // [解析] ↑ 模糊推理给出的 Δki。
  float delta_kd_ = 0;  // D项系数模糊量
  // [解析] ↑ 模糊推理给出的 Δkd。
  // [解析] 【为什么用成员变量而不是局部变量】这三个值每次 calc() 都会被重算，
  // [解析]   保存在成员里纯粹是为了调试时能在外面看到它们（本身没有记忆作用）。

  // 模糊推理+解模糊
  // [解析] 两个"从清晰到模糊、再回到清晰"的输入，是模糊控制的核心：
  // [解析]   error：当前误差（会先归一化到 [-3,3]）；error_rate：误差变化率（同样归一化）。
  // [解析]   rules：要查哪一张规则表（Δkp / Δki / Δkd 三张表之一）。
  // [解析] 返回：归一化后的修正量，范围约 [-1, 1]（内部最后除以了 3）。
  // [解析] 【const 成员函数】说明它不会修改本对象的任何成员，只看不改。
  float fuzzy_inference(float error, float error_rate, const RuleTable & rules) const;
  // 隶属度计算
  // [解析] 三角形隶属度函数：给定归一化输入 x 和某一档 set（0~6，对应 NB~PB），
  // [解析] 返回 x 属于这一档的程度，范围 [0, 1]。0 表示完全不属于，1 表示完全属于
  // [解析]   （三角形顶点处取 1）。
  // [解析] 参数 x：已经归一化到 [-3, 3] 的量；set：档位下标。
  // [解析] 【注意】set 是被直接当成数组下标用的（membership_params_[set]），
  // [解析]   传 -1 或 7 就会数组越界，所以内部调用时都做了 static_cast 且范围受控。
  float membership_calc(float x, int set) const;
};

}  // namespace sp

#endif  // SP__FUZZY_PID_HPP