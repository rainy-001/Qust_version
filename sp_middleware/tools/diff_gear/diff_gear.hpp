// [解析] ==========================================================================
// [解析] 【文件】sp_middleware/tools/diff_gear/diff_gear.hpp
// [解析] 【作用】"差速器式两自由度云台/关节"（differential gear，差速齿轮）的解算器。
// [解析]         它把两个输入电机（左齿轮、右齿轮）的转角/转速/力矩，
// [解析]         换算成末端机构的 pitch（俯仰）和 roll（横滚）两个自由度，反过来也能算。
// [解析] 【典型场景】RM 里常见于"两轴差速云台"：两个电机不是分别直连俯仰轴和横滚轴，
// [解析]         而是通过一套锥齿轮把两个电机的运动"叠加"，一个自由度由两电机同向转得到，
// [解析]         另一个自由度由两电机反向转得到。好处是结构紧凑、力矩大。
// [解析] 【被谁用】本仓库里暂时没有调用者，属于中间件工具库。
// [解析]         上层 applications/ 的云台任务自己建对象，例如：
// [解析]           sp::DiffGear gear(2.0f);            // 锥齿轮减速比 2:1
// [解析]           gear.init_gear_angle(a_l, a_r);     // ★上电后必须先调用一次！
// [解析]           gear.calc_end_angle(a_l, a_r);
// [解析] 【关键点】1) 本类使用前必须先调 init_gear_angle()，否则角度是未定义的；
// [解析]           2) ratio（减速比）在"角度/速度"和"力矩"两种换算里出现的位置不同
// [解析]              （一个除、一个乘），这是因为力矩是速度的"功率共轭量"；
// [解析]           3) 单位：角度 rad，角速度 rad/s，力矩 N·m。
// [解析] 【阅读顺序】先看本文件了解 5 个函数各自干什么，再看 diff_gear.cpp 的公式。
// [解析] ==========================================================================

#ifndef SP__DIFF_GEAR_HPP
#define SP__DIFF_GEAR_HPP

namespace sp
{
// [解析] DiffGear 类：差速齿轮（两自由度关节）的运动学/静力学换算工具。
// [解析] 本类纯粹是"算数"，不操作硬件、不发数据，因此可以在任意任务或中断里调用；
// [解析]   但要注意：它内部保存了开机零位 angle_l0_ / angle_r0_，是有状态的，
// [解析]   不要在多处同时用同一个对象（每个关节应该各建一个对象）。
class DiffGear
{
public:
  // [解析] ratio：锥齿轮减速比（末端转 1 圈 = 齿轮转 ratio 圈，或者反过来，
  // [解析]   具体方向以实车标定为准；常见取值 1.0 ~ 3.0）。必须大于 0，不能传 0。
  // [解析] reverse_l / reverse_r：左/右电机装配方向，true 表示该电机反装。
  // [解析] 默认 reverse_l = false、reverse_r = true，也就是默认"右电机反装"——
  // [解析]   这和 mecanum 的默认值思路一致：结构上两侧对称时，一侧自然要取反。
  // [解析] 没有针对 ratio 的合法性检查：传 0 会导致后面出现除零。
  // ratio: 锥齿轮减速比
  // reverse_l: left反向旋转
  // reverse_r: right反向旋转
  DiffGear(float ratio, bool reverse_l = false, bool reverse_r = true);

  // [解析] ---- 输出区 1：calc_end_angle() 的结果 ----
  // [解析] pitch / roll 都是"相对于开机零位"的累计角度，单位 rad，不是 [-π, π]。
  // [解析]   pitch：俯仰角（绕"车体左右方向"的轴转），+ 方向由 v_pitch 的约定决定；
  // [解析]   roll ：横滚角（绕"车体前后方向"的轴转）。
  // [解析] 为什么是累计值而不是限幅到 ±π？因为关节可能转好几圈，累计值才能反映真实位置。
  float pitch;    // 只读! calc_end_angle()计算结果, 单位: rad
  float roll;     // 只读! calc_end_angle()计算结果,  单位: rad
  // [解析] ---- 输出区 2：calc_end_speed() 的结果 ----
  // [解析] v_pitch / v_roll：末端两个自由度的角速度，单位 rad/s。
  float v_pitch;  // 只读! calc_end_speed()计算结果, 单位: rad/s
  float v_roll;   // 只读! calc_end_speed()计算结果, 单位: rad/s
  // [解析] ---- 输出区 3：calc_end_torque() 的结果 ----
  // [解析] t_pitch / t_roll：末端两个自由度上的等效力矩，单位 N·m。
  // [解析] 注意这两个量与输入的关系是"转速关系的功率共轭"（详见 diff_gear.cpp 的说明）：
  // [解析]   ratio 在速度里是"除以"，在力矩里就变成"乘以"，不要觉得是笔误。
  float t_pitch;  // 只读! calc_end_torque()计算结果, 单位: N·m
  float t_roll;   // 只读! calc_end_torque()计算结果, 单位: N·m
  // [解析] ---- 输出区 4：calc_gear_speed() 的结果 ----
  // [解析] v_left / v_right：为了让末端达到 (v_pitch, v_roll)，左右两个齿轮该转多快，
  // [解析]   单位 rad/s，直接作为两个电机的 PID 目标值。
  // [解析] 注意它已经包含了 reverse 的符号（末尾乘了 sign_l_/sign_r_），可以直接下发。
  float v_left;   // 只读! calc_gear_speed()计算结果, left转速, 单位: rad/s
  float v_right;  // 只读! calc_gear_speed()计算结果, right转速, 单位: rad/s

  // [解析] init_gear_angle()：记录"开机零位"。★必须在其它 calc_xxx 之前调用一次。
  // [解析] 为什么需要它？因为差速机构是相对量：只能测出齿轮转了多少，不知道末端的绝对姿态。
  // [解析]   所以约定"上电时末端处于机械中位"，把当时的齿轮转角存下来当基准，
  // [解析]   之后算出来的 pitch/roll 都是相对这个基准的偏移。
  // [解析] 调用时机：初始化阶段（电机编码器读数稳定之后），或者需要重新标定零位时。
  // [解析] 副作用：写 angle_l0_ / angle_r0_，会影响后续所有 calc_end_angle() 的结果。
  // [解析] 参数单位 rad，取值范围不限（可以是多圈的累计角度，也可以被归一化过，
  // [解析]   只要两次传入的口径一致即可）。
  // 初始化各齿轮转角
  // angle_l: 左侧齿轮转角, 单位: rad
  // angle_r: 右侧齿轮转角, 单位: rad
  void init_gear_angle(float angle_l, float angle_r);

  // [解析] calc_end_angle()：齿轮累计转角 -> 末端姿态角。
  // [解析] 调用时机：每个控制周期调一次，用来做"末端姿态反馈"（给 PID 当反馈量）。
  // [解析] 副作用：写 pitch / roll。内部会先减去 init_gear_angle() 记下的零位。
  // [解析] 注意：如果忘了先调 init_gear_angle()，angle_l0_ / angle_r0_ 是未初始化成员，
  // [解析]   结果会是随机的（这一点在 diff_gear.cpp 里有详细标注）。
  // 各齿轮转角 -> 末端转角
  // angle_l: 左侧齿轮转角, 单位: rad
  // angle_r: 右侧齿轮转角, 单位: rad
  void calc_end_angle(float angle_l, float angle_r);

  // [解析] calc_end_speed()：齿轮转速 -> 末端角速度。
  // [解析] 用途：一是做速度前馈，二是把电机反馈转速换成末端速度用于状态估计。
  // [解析] 副作用：写 v_pitch / v_roll。这里不需要零位，因为速度是差值，基准会被消掉。
  // 各齿轮转速 -> 末端转速
  // speed_l: 左侧齿轮转速, 单位: rad/s
  // speed_r: 右侧齿轮转速, 单位: rad/s
  void calc_end_speed(float speed_l, float speed_r);

  // [解析] calc_end_torque()：齿轮输出力矩 -> 末端等效力矩。
  // [解析] 用途：估算末端能输出多大力矩，或用于力控/摩擦补偿。
  // [解析] 副作用：写 t_pitch / t_roll。注意 ratio 这里是"乘"，和速度那边相反，
  // [解析]   原因见 diff_gear.cpp 里"功率守恒"的推导。
  // 各齿轮转矩 -> 末端转矩
  // torque_l: 左侧齿轮转矩, 单位: N·m
  // torque_r: 右侧齿轮转矩, 单位: N·m
  void calc_end_torque(float torque_l, float torque_r);

  // [解析] calc_gear_speed()：反解，末端期望角速度 -> 两个齿轮各自的目标转速。
  // [解析] 这是实际控制链路里最常用的一个：控制器算好末端要转多快，
  // [解析]   用它翻译成两个电机的转速指令（结果已在 v_left / v_right 里）。
  // [解析] 调用时机：控制周期内，算出末端指令之后、下发电机 PID 之前。
  // [解析] 副作用：写 v_left / v_right。
  // [解析] 没有对应的 calc_gear_angle()：因为差速机构没有"绝对位置"反解，
  // [解析]   要控位置得自己在末端做闭环（或对 v_left/v_right 积分）。
  // 末端转速 -> 各齿轮转速
  // v_pitch: 大拇指朝左，右手螺旋方向转速, 单位: rad/s
  // v_roll: 大拇指朝前，右手螺旋方向转速, 单位: rad/s
  void calc_gear_speed(float v_pitch, float v_roll);

private:
  // [解析] ratio_：锥齿轮减速比，构造时定下，之后不能改。
  // [解析] sign_l_ / sign_r_：装配方向符号，只取 +1.0f 或 -1.0f。
  const float ratio_;
  const float sign_l_;
  const float sign_r_;

  // [解析] angle_l0_ / angle_r0_：init_gear_angle() 记下的"开机零位"，
  // [解析]   单位 rad，仅用于 calc_end_angle() 里做差值。
  // [解析] ★特别注意：这两个成员在构造函数里没有被初始化！
  // [解析]   如果没调 init_gear_angle() 就直接调 calc_end_angle()，
  // [解析]   读到的是未定义的值（通常是内存里的随机数），结果毫无意义。
  float angle_l0_;
  float angle_r0_;
};

}  // namespace sp

#endif  // SP__DIFF_GEAR_HPP