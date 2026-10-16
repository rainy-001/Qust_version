// [解析] ==========================================================================
// [解析] 【文件】sp_middleware/tools/diff_gear/diff_gear.cpp
// [解析] 【作用】diff_gear.hpp 里声明的函数的具体实现：差速齿轮（两自由度关节）的换算。
// [解析] 【一句话记住全部公式】"同向相加 / 2 = 俯仰，反向相减 / 2 = 横滚"：
// [解析]   pitch 方向  <- 左齿轮 + 右齿轮（共模）
// [解析]   roll  方向  <- 左齿轮 - 右齿轮（差模），再按减速比 ratio_ 缩放
// [解析] 【被谁用】上层 applications/ 里控制云台/关节的任务，创建 sp::DiffGear 对象后调用。
// [解析] 【看代码的重点】1) 为什么角度/速度里 ratio_ 是"除以"，而力矩里是"乘以"；
// [解析]                 2) 为什么都带一个 /2（因为做了平均）；
// [解析]                 3) angle_l0_ / angle_r0_ 必须在 init_gear_angle() 里先赋值。
// [解析] ==========================================================================

#include "diff_gear.hpp"

namespace sp
{
// [解析] 构造函数：只登记减速比和装配方向，函数体为空。
// [解析] 注意这里没有给 angle_l0_ / angle_r0_ 赋值，也就是说：
// [解析]   在调用 init_gear_angle() 之前，这两个成员是"未初始化"状态，
// [解析]   此时若调用 calc_end_angle()，算出来的 pitch/roll 会是随机值。
// [解析]   正确用法是初始化阶段先 init_gear_angle(当前左角, 当前右角)。
DiffGear::DiffGear(float ratio, bool reverse_l, bool reverse_r)
: ratio_(ratio), sign_l_((reverse_l) ? -1.0f : 1.0f), sign_r_((reverse_r) ? -1.0f : 1.0f)
{
}

// [解析] init_gear_angle()：把当前左右齿轮的转角记成"零位基准"。
// [解析] 为什么差速机构必须要有零位？因为两个电机只能告诉我们"齿轮转了多少"，
// [解析]   而末端真实的机械姿态是它们的组合；没有绝对编码器时，只能约定一个起点，
// [解析]   之后的 pitch/roll 都是相对起点的变化量（增量式）。
// [解析] 副作用：写 angle_l0_ / angle_r0_，会让之后所有 calc_end_angle() 的结果整体平移。
// [解析] 调用时机：只能在初始化阶段（或重新标定时）调用，运行中每周期调用没问题，
// [解析]   但会不停把"当前姿态"当作零位，等于永远输出 0，那就失去意义了。
// [解析] 参数单位 rad，可以是多圈累计值（比如 3.5 圈 = 21.99 rad），本函数不做归一化。
void DiffGear::init_gear_angle(float angle_l, float angle_r)
{
  angle_l0_ = angle_l;
  angle_r0_ = angle_r;
}

// [解析] calc_end_angle()：齿轮累计转角 -> 末端 pitch / roll 角度。
// [解析] 【公式推导】设 θ_l、θ_r 为左右齿轮相对零位的转角（已抵消装配方向）：
// [解析]   俯仰（共模）：两个齿轮同向转时，末端只做 pitch。取平均得 pitch = (θ_l + θ_r)/2；
// [解析]   横滚（差模）：两个齿轮反向转时，末端只做 roll。差值再折算到末端就是
// [解析]     roll = (θ_l - θ_r) / (2 × ratio_)。
// [解析] 【为什么 roll 要多除一个 ratio_】因为这两个齿轮与横滚轴之间隔了锥齿轮副，
// [解析]   齿轮转 ratio_ 圈，末端横滚轴才转 1 圈，所以要把"齿轮侧的角度"换算成"末端角度"。
// [解析]   pitch 走的是另一条传动路径（共模路径比值恰好为 1），所以不用除。
// [解析] 【为什么有 /2】因为做了"取平均 / 取差的一半"，见上面的推导。
// [解析] 【副作用】写 this->pitch 和 this->roll。两者都是累计角度，不会被限幅到 ±π。
// [解析] 【陷阱】若此前没调用 init_gear_angle()，angle_l0_ / angle_r0_ 未初始化，
// [解析]   结果随机。这是本文件最容易踩的坑。
void DiffGear::calc_end_angle(float angle_l, float angle_r)
{
  // [解析] 先 (angle_l - angle_l0_) 得到"相对零位转过的角度"，再乘 sign 把装配方向抵消。
  this->pitch = (sign_l_ * (angle_l - angle_l0_) + sign_r_ * (angle_r - angle_r0_)) / 2;
  // [解析] 除法从左往右结合：先 /2 再 /ratio_，等价于 /(2 * ratio_)。
  this->roll = (sign_l_ * (angle_l - angle_l0_) - sign_r_ * (angle_r - angle_r0_)) / 2 / ratio_;
}

// [解析] calc_end_speed()：齿轮转速 -> 末端角速度。
// [解析] 结构和 calc_end_angle() 完全一样，只是把"角度差"换成"转速"，
// [解析]   并且不用再减零位（速度是变化率，基准值求导后是 0，自动消掉）。
// [解析] 单位：输入 rad/s，输出也是 rad/s（除以 ratio_ 之后仍是角速度）。
// [解析] 副作用：写 this->v_pitch / this->v_roll。
// [解析] 用途：① 做速度前馈，让末端更快跟上指令；② 把电机反馈换算成末端速度做观测。
void DiffGear::calc_end_speed(float speed_l, float speed_r)
{
  this->v_pitch = (sign_l_ * speed_l + sign_r_ * speed_r) / 2;
  this->v_roll = (sign_l_ * speed_l - sign_r_ * speed_r) / 2 / ratio_;
}

// [解析] calc_end_torque()：齿轮输出力矩 -> 末端等效力矩。
// [解析] ★这是本文件最"反直觉"的地方：这里既没有 /2，ratio_ 还从"除"变成了"乘"。
// [解析]   它并不是笔误，而是由"功率守恒"决定的（力矩是速度的功率共轭量）：
// [解析]   同一套机构，功率必须两边相等：P_末端 = v_pitch·t_pitch + v_roll·t_roll
// [解析]                                 P_电机 = speed_l·torque_l + speed_r·torque_r
// [解析]   把上面的速度关系代进去可以验证（设 sign 均为 1）：
// [解析]     v_pitch·t_pitch + v_roll·t_roll
// [解析]       = (s_l+s_r)/2 · (τ_l+τ_r) + (s_l-s_r)/(2ratio) · (τ_l-τ_r)·ratio
// [解析]       = [(s_l+s_r)(τ_l+τ_r) + (s_l-s_r)(τ_l-τ_r)] / 2
// [解析]       = s_l·τ_l + s_r·τ_r  ✓ 恰好等于电机侧功率。
// [解析]   也就是说：t_pitch = τ_l + τ_r 正好是"俯仰轴上的等效输出力矩"，
// [解析]   t_roll = (τ_l - τ_r)·ratio_ 正好是"横滚轴上的等效输出力矩"
// [解析]     （减速比在力矩侧起放大作用：转速降 ratio_ 倍，力矩就升 ratio_ 倍）。
// [解析]   ratio_ 在速度里是"除以"（减速，转速变慢），在力矩里就必然是"乘以"（增矩），
// [解析]   这是所有减速机构共有的规律，记住"速度除以减速比、力矩乘以减速比"就不会混。
// [解析] 另一个理解角度：把角度/速度写成矩阵 [pitch;roll] = J·[θ_l;θ_r]，
// [解析]   那么力矩就是 [τ_l;τ_r] = J^T·[t_pitch;t_roll]，即"转置关系"，
// [解析]   所以少了那个 /2、多了那个 ratio_，正是矩阵转置后的结果。
// [解析] 副作用：写 this->t_pitch / this->t_roll。单位：输入 N·m，输出 N·m。
// [解析] 注意：这里没有考虑齿轮传动效率（真实机构有摩擦损耗，效率 η < 1），
// [解析]   所以算出来的末端力矩是"理想上限"，实际会小一些。
void DiffGear::calc_end_torque(float torque_l, float torque_r)
{
  this->t_pitch = (sign_l_ * torque_l + sign_r_ * torque_r);
  this->t_roll = (sign_l_ * torque_l - sign_r_ * torque_r) * ratio_;
}

// [解析] calc_gear_speed()：反解，末端期望角速度 -> 左右齿轮各自的目标转速。
// [解析] 【推导】把 calc_end_speed() 的两式当方程组，已知 v_pitch、v_roll 求解 s_l、s_r：
// [解析]   v_pitch     = (s_l + s_r) / 2
// [解析]   v_roll·ratio = (s_l - s_r) / 2
// [解析]   两式相加得 s_l = v_pitch + v_roll·ratio_，两式相减得 s_r = v_pitch - v_roll·ratio_。
// [解析] 这就是下面两行的来历：ratio_ 又回到"乘"在 v_roll 上的位置（和正解互为逆运算）。
// [解析] 最后乘 sign_l_ / sign_r_，把装配方向补偿回去，结果可以直接下发给电机。
// [解析] 副作用：写 this->v_left / this->v_right，单位 rad/s。
// [解析] 这是实际控制链路里最常用的函数：上层算好末端指令，用它翻译成两个电机指令。
// [解析] 注意：本函数是全速域可用的（没有奇异性），因为差速机构没有"奇异位形"，
// [解析]   这一点比二连杆、云台直连那种结构更友好。
// [解析] 另外注意：正解符号是"左 + 右"对应 pitch，所以反解里 v_roll 对左右轮是同加同减。
void DiffGear::calc_gear_speed(float v_pitch, float v_roll)
{
  // [解析] v_roll * ratio_：把"末端横滚角速度"换算回"齿轮侧的角速度"，
  // [解析]   正好是 calc_end_speed() 里那个 "/ratio_" 的逆运算。
  this->v_left = sign_l_ * (v_pitch + v_roll * ratio_);
  this->v_right = sign_r_ * (v_pitch - v_roll * ratio_);
}

}  // namespace sp
