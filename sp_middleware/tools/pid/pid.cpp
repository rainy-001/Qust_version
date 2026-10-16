// [解析] ==========================================================================
// [解析] 【文件】sp_middleware/tools/pid/pid.cpp
// [解析] 【作用】pid.hpp 里 4 个 calc() 重载的具体实现，也就是 PID 公式真正算的地方。
// [解析] 【看代码的重点】本文件有四个"长得很像"的 calc()，建议这样看：
// [解析]   calc(set, fdb)                      -> 微分先行 + alpha 滤波 + 可选变速积分（最常用）
// [解析]   calc(set, fdb, threshold)           -> 上面那个 + 积分分离
// [解析]   calc(set, fdb, set_dot, fdb_dot)    -> 用外部给定的速度做 D 项（前馈微分）
// [解析]   calc(set, fdb, set_dot, fdb_dot, threshold) -> 上面那个 + 积分分离
// [解析] 【最容易看晕的 3 个点】
// [解析]   1) "微分先行"为什么要用 data.fdb（上一帧的反馈）去减本帧 fdb；
// [解析]   2) err/dbuf 每次都要"整体后移一格"，这是为了拿到上一周期的值；
// [解析]   3) 积分那两行：trapezoid 是"面积"，iout 是"累加器"，limit_max 是"防饱和"。
// [解析] ==========================================================================

#include "pid.hpp"

namespace sp
{
// [解析] 构造函数：冒号后面那一串叫"初始化列表"，把参数依次存进 const 成员变量。
// [解析] const 成员只能在初始化列表里赋值，函数体里再赋值就会编译报错。
// [解析] 函数体是空的 {}，因为这里没有任何计算要做（纯存参数，不碰硬件）。
PID::PID(
  float dt, float kp, float ki, float kd, float max_out, float max_iout, float alpha, bool angular,
  bool dynamic)
: dt_(dt),
  kp_(kp),
  ki_(ki),
  kd_(kd),
  max_out_(max_out),
  max_iout_(max_iout),
  alpha_(alpha),
  angular_(angular),
  dynamic_(dynamic)
{
}

void PID::calc(float set, float fdb)
{
  // [解析] 进入本函数时 data.fdb 里存的还是"上一个周期的反馈值"，本周期的新 fdb 还没有写进去。
  // [解析] 这两句正是在利用这个时间差：先用旧 fdb 求微分，再把新 fdb 存起来。
  if (dout_ready_ == true) {
    // 微分先行
    // [解析] "微分先行"= 不去微分成"误差"，而是去微分成"反馈值"。
    // [解析] 做的是三件事：把窗口整体后移一格，然后把 (上一帧 fdb - 本帧 fdb) 放进 dbuf[0]。
    // [解析] 【为什么用 旧-新 而不是 新-旧】因为最终 D 项要起"刹车"作用：
    // [解析]   反馈值正在变大（比如角度在往正方向转）时，上一帧 < 本帧，差为负，
    // [解析]   乘上正系数 kd 后 D 项为负，正好抵消一点正向输出 —— 这就是阻尼。
    // [解析] 【微分先行的好处】如果用"误差的变化率"做微分，那么目标值一突变，
    // [解析]   D 项会出现一个巨大的瞬时尖峰（微分冲击），电机就会"咯噔"一下；
    // [解析]   只微分成反馈值，目标值突变不会进 D 项，输出平滑得多。
    this->data.dbuf[2] = this->data.dbuf[1];
    this->data.dbuf[1] = this->data.dbuf[0];
    // [解析] angular_ 为真时，两帧角度差可能跨越 ±π（例如从 +179° 到 -179°），
    // [解析]   直接相减得到 -358° 这种"绕远路"的值，所以先折到 (-π, π] 取最短变化量。
    this->data.dbuf[0] = angular_ ? limit_angle((this->data.fdb - fdb)) : (this->data.fdb - fdb);
  }
  else {
    // [解析] 第一帧：还没有"上一帧 fdb"可用（data.fdb 初值 0 是假的），
    // [解析]   所以这一帧不算微分，只打上"下次可以算了"的标记。
    dout_ready_ = true;
  }

  // 滤波
  // [解析] 一阶低通滤波（加权平均）：新值占 alpha_，旧值占 (1-alpha_)。
  // [解析]   alpha_=1 时结果就是 dbuf[0] 本身，等于不滤波（最灵敏、也最抖）；
  // [解析]   alpha_=0.2 时 80% 沿用旧值，输出很平滑但会滞后。
  // [解析] 【注意】第一帧走 else 分支时 dbuf[0] 还是 0，滤波后 dbuf[0] 被压成 0（或更小），
  // [解析]   所以刚启动的头几个周期 D 项偏小，这是有意为之，不是 bug。
  // [解析] 另外这里把结果写回 dbuf[0]，也就是"滤波后的微分"覆盖掉了原始微分，
  // [解析]   后面 Kd 那行直接用 dbuf[0] 就行。
  this->data.dbuf[0] = alpha_ * this->data.dbuf[0] + (1 - alpha_) * this->data.dbuf[1];

  // [解析] 误差窗口整体后移一格：这样下一次调用时 err[1] 就是"本周期误差"，
  // [解析]   梯形积分要用的"上一周期误差"就现成了。
  this->data.err[2] = this->data.err[1];
  this->data.err[1] = this->data.err[0];
  // [解析] 本周期误差 = 目标 - 反馈。角度控制时先折叠到 (-π, π]。
  // [解析] 【符号约定】err 为正表示"还没到目标，需要继续往正方向出力"，
  // [解析]   所以 P/I/D 三项都是直接相加（不需要取负号）。如果发现电机反着跑，
  // [解析]   说明反馈方向反了（或 kp 符号写反了），改接线/改符号，别在这里加负号。
  this->data.err[0] = angular_ ? limit_angle(set - fdb) : (set - fdb);

  // [解析] 把本帧的 set/fdb 记下来。注意 data.fdb 必须在算完微分之后才更新，
  // [解析]   否则下一帧的"上一帧 fdb"就变成"本帧 fdb"，微分恒为 0 了。
  this->data.set = set;
  this->data.fdb = fdb;

  // Kp
  // [解析] P 项 = kp * 当前误差。误差越大出力越大，是响应速度的主要来源。
  // [解析] 注意这里没有单独给 P 项限幅，它可能远大于 max_out，最终靠 out 那一步统一夹住。
  this->data.pout = kp_ * this->data.err[0];

  // Ki
  // [解析] 梯形积分：用 (本周期误差 + 上周期误差)/2 当作这一小段时间里的"平均误差"，
  // [解析]   再乘 dt_ 就近似出"误差-时间曲线下的一小块面积"，也就是本周期的积分增量。
  // [解析]   为什么不直接用 err[0]*dt_？那样是矩形法，会系统性地偏大/偏小；
  // [解析]   梯形法是它和 err[1] 的平均，精度更高（这也是要留 err[1] 的原因）。
  // [解析] 单位：err 的单位 × 秒。注意它只是"增量"，真正的累加在下面 iout 那行。
  this->data.trapezoid = (this->data.err[0] + this->data.err[1]) / 2 * dt_;  // 梯形积分
  // [解析] 变速积分：动态 ki = ki / (1 + |误差|)。
  // [解析]   误差为 0 时 ki 原样；误差为 10 时 ki 只剩 1/11，积分几乎停下。
  // [解析]   目的是"大误差阶段少积分、小误差阶段多积分"，兼顾"快速接近"和"消稳态误差"。
  // [解析] 【注意】这一行无论 dynamic_ 开关是真是假都会计算，只是不用而已（白算一次，无害）。
  // [解析]   std::abs 对 float 的重载来自 <cmath>，由 math_tools.hpp 间接引入。
  this->data.dynamic_ki = ki_ / (1 + std::abs(this->data.err[0]));           // 变速积分
  // [解析] 积分累加器：iout(新) = iout(旧) + ki × 本周期增量。
  // [解析]   dynamic_ 为真就乘"变速 ki"，否则乘固定的 ki_。
  // [解析]   limit_max(..., max_iout_) 把它夹在 ±max_iout_ 内 —— 这一步就是"抗积分饱和"。
  // [解析] 【没有它会怎样】电机被机械结构卡住时，误差一直存在，iout 会一路涨到天上；
  // [解析]   等卡住解除，这个巨大的 iout 会让电机猛地冲出去，很危险。
  // [解析] 【注意】这里依赖"iout 有记忆"，所以同一个 PID 被调用前请先 clear()。
  this->data.iout = limit_max(
    this->data.iout + (dynamic_ ? this->data.dynamic_ki : ki_) * this->data.trapezoid, max_iout_);

  // Kd
  // [解析] D 项 = kd × 滤波后的微分 ÷ dt_。
  // [解析] 【为什么要除以 dt_】dbuf[0] 是"两帧之间的差值"（不是变化率），
  // [解析]   除以时间间隔才得到"每秒变化多少"，也就是真正的速度量纲。
  // [解析]   这样调整控制频率时 D 项的量级才不会跟着变。
  this->data.dout = kd_ * this->data.dbuf[0] / dt_;

  // [解析] 三项相加得到最终输出，再用 max_out_ 对称限幅后写入 out。
  // [解析] 限幅对应物理含义：电机能给的电流/力矩是有上限的，不下发超限值保护硬件。
  this->out = limit_max(this->data.pout + this->data.iout + this->data.dout, max_out_);
}

// [解析] 【重载说明】set_dot / fdb_dot 分别是"目标值的变化率"和"反馈值的变化率"，
// [解析]   单位是"set 的单位 / 秒"（例如 set 是角度 rad，那 set_dot 就是 rad/s）。
// [解析]   这个版本让调用者自己把速度喂进来，D 项就不用再靠两帧差分去估了。
void PID::calc(float set, float fdb, float set_dot, float fdb_dot)
{
  // [解析] 误差窗口照旧后移一格，留出 err[1] 给梯形积分用。
  // [解析] 本重载同样支持 angular_：角度控制时误差会先折到 (-π, π]，取最短转向。
  this->data.err[2] = this->data.err[1];
  this->data.err[1] = this->data.err[0];
  this->data.err[0] = angular_ ? limit_angle(set - fdb) : (set - fdb);

  // Kp
  // [解析] P 项算法和上面完全一样。
  this->data.pout = kp_ * this->data.err[0];

  // Ki
  // [解析] 积分增量还是梯形法，但写法与上面略有不同（上面是 push 到已有 iout 上，
  // [解析]   这里是 ki×增量 再加旧 iout），数学上等价。
  this->data.trapezoid = (this->data.err[0] + this->data.err[1]) / 2 * dt_;  // 梯形积分
  // [解析] 【注意】本重载没有算 data.dynamic_ki，也没有判断 dynamic_ 开关，
  // [解析]   所以即使构造时写了 dynamic=true，这个版本也不会启用变速积分。
  // [解析]   如果这一路需要变速积分，请改用第一个 calc(set, fdb)。
  // [解析] 同样用 max_iout_ 夹住，防止积分饱和。
  this->data.iout = limit_max(ki_ * this->data.trapezoid + this->data.iout, max_iout_);

  // Kd
  // [解析] D 项 = kd × (目标速度 - 反馈速度)。
  // [解析]   意义："反馈速度还不够快，就额外加一点输出追上去；反馈冲得太快，就压一压"。
  // [解析]   这就是标准的"误差微分"形式，因为 d(误差)/dt = d(set)/dt - d(fdb)/dt。
  // [解析] 【和重载 1 的区别】重载 1 只用 fdb 的微分（微分先行，目标突变时更平滑）；
  // [解析]   这里 set_dot 参与了运算。若调用者把 set_dot 写成 0，就退化成"纯反馈微分"。
  // [解析] 【注意】这里没有除以 dt_，因为 set_dot/fdb_dot 本身已经是"每秒变化量"，
  // [解析]   再除一次就错了。也正因如此，这个重载的 kd 量级和重载 1 完全不同，不能照抄参数。
  this->data.dout = kd_ * (set_dot - fdb_dot);

  // [解析] 三项求和 + 输出限幅。
  // [解析] 【注意】本重载不会更新 data.set / data.fdb，它们仍然是上一次调用的值，
  // [解析]   调试看波形时别被这两个字段误导。另外它也不使用 alpha_ 滤波和 dbuf。
  this->out = limit_max(this->data.pout + this->data.iout + this->data.dout, max_out_);
}

// [解析] 【重载说明】= 上一个重载 + 积分分离。参数含义：
// [解析]   integral_pause_threshold：积分"暂停阈值"。当 |P 项输出| 大于它时，本周期不加积分。
// [解析]   为什么拿 P 项输出当判据而不是直接拿误差？因为 pout = kp×err，
// [解析]   用输出判断相当于"考虑了这个环的增益大小"，换一个 kp 也不用重新标定阈值。
void PID::calc(float set, float fdb, float set_dot, float fdb_dot, float integral_pause_threshold)
{
  // [解析] 误差窗口后移，与前面几个重载一致。
  this->data.err[2] = this->data.err[1];
  this->data.err[1] = this->data.err[0];
  this->data.err[0] = angular_ ? limit_angle(set - fdb) : (set - fdb);

  // Kp
  // [解析] P 项照常计算。注意下面判断积分是否暂停时用的就是它。
  this->data.pout = kp_ * this->data.err[0];

  // 积分分离
  // 当P项输出的绝对值已经很大时，暂停积分累积
  // [解析] 积分分离的直观理解："离目标还远的时候别忙着积分"。
  // [解析]   因为起步阶段误差大，如果这时也积分，iout 会迅速堆起来（积分饱和），
  // [解析]   结果就是冲过目标一大截再慢慢退回来（超调）。先只靠 P 项快速冲过去，
  // [解析]   等近了再让 I 项去磨掉最后那点稳态误差，响应会更漂亮。
  if (std::abs(this->data.pout) > integral_pause_threshold) {
    // 将本次的积分增量设为0
    // [解析] 注意：是把"本周期的增量"设成 0，而不是把已有的 iout 清零。
    // [解析]   所以只是"这一拍不涨了"，已经积累的积分仍然保留并继续发挥作用。
    this->data.trapezoid = 0.0f;
  }
  else {
    // 只有当P项输出在可接受范围内时，才正常计算积分增量
    // [解析] P 项已经足够小（说明接近目标了），才开始正常做梯形积分。
    this->data.trapezoid = (this->data.err[0] + this->data.err[1]) / 2 * dt_;  // 梯形积分
  }

  // Ki
  // 此处完全同步了你第二个函数中动态 Ki (dynamic_ki) 的逻辑
  // [解析] 这句话是原作者写的。实际含义：变速积分的公式和第一个 calc() 里那一行一致。
  // [解析] 之所以在这里单独写一遍（而不是调用别的函数），是因为要保证这一版也能吃到
  // [解析]   dynamic_ 开关的效果 —— 这一点比"重载 3"做得更完整（重载 3 漏了 dynamic_）。
  this->data.dynamic_ki = ki_ / (1 + std::abs(this->data.err[0]));
  // 如果触发了暂停条件，trapezoid为0，iout的值将保持不变。
  // [解析] ↑ 原作者注释。补充：不是"精确不变"，而是"增量乘以 0 后等于原值"，
  // [解析]   如果原来的 iout 已经贴在 ±max_iout_ 边界上，limit_max 也会让它保持不动。
  this->data.iout = limit_max(
    this->data.iout + (dynamic_ ? this->data.dynamic_ki : ki_) * this->data.trapezoid, max_iout_);

  // Kd
  // 保持第一个函数原有的前馈微分计算方式
  // [解析] ↑ 原作者注释里的"第一个函数"指本文件最上面那个 calc(set, fdb)；
  // [解析]   但实际这行用的是 set_dot - fdb_dot，属于"重载 3"的前馈微分写法。
  // [解析]   说法略有出入，看代码本身为准：这里不除 dt_、也不做 alpha 滤波。
  this->data.dout = kd_ * (set_dot - fdb_dot);

  // [解析] 求和 + 限幅收尾，和前几个重载一致。
  this->out = limit_max(this->data.pout + this->data.iout + this->data.dout, max_out_);
}

// [解析] 【重载说明】= 最上面那个 calc(set, fdb)（微分先行 + 滤波 + 变速积分）
// [解析]   + 积分分离。参数 integral_pause_threshold 的含义见上面那个重载的说明。
// [解析] 想"既要微分先行平滑、又要积分分离防超调"，就用这一个。
void PID::calc(float set, float fdb, float integral_pause_threshold)
{
  // [解析] 和第一个 calc() 完全同一套微分先行逻辑，注释不再重复，只强调两点：
  // [解析]   1) 用的是 data.fdb（上一帧反馈）减本帧 fdb，所以必须在更新 data.fdb 之前做；
  // [解析]   2) 第一帧没有上一帧可减，只置位 dout_ready_ 就跳过。
  if (dout_ready_ == true) {
    // 微分先行
    this->data.dbuf[2] = this->data.dbuf[1];
    this->data.dbuf[1] = this->data.dbuf[0];
    this->data.dbuf[0] = angular_ ? limit_angle((this->data.fdb - fdb)) : (this->data.fdb - fdb);
  }
  else {
    dout_ready_ = true;
  }

  // 滤波
  // [解析] 一阶低通：新微分占 alpha_、旧微分占 (1-alpha_)，结果写回 dbuf[0]。
  this->data.dbuf[0] = alpha_ * this->data.dbuf[0] + (1 - alpha_) * this->data.dbuf[1];

  // [解析] 误差窗口后移一拍，为梯形积分准备好"上一周期误差"。
  this->data.err[2] = this->data.err[1];
  this->data.err[1] = this->data.err[0];
  this->data.err[0] = angular_ ? limit_angle(set - fdb) : (set - fdb);

  // [解析] 记录本帧 set/fdb（同样要在微分算完之后）。
  this->data.set = set;
  this->data.fdb = fdb;

  // Kp
  // [解析] P 项，无单独限幅。
  this->data.pout = kp_ * this->data.err[0];

  // 积分分离
  // 当P项输出的绝对值已经很大时，暂停积分累积
  // [解析] 判据是 |P 项输出| 与阈值的比较，与上一个重载完全相同。
  if (std::abs(this->data.pout) > integral_pause_threshold) {
    // 将本次的积分增量设为0
    this->data.trapezoid = 0.0f;
  }
  else {
    // 只有当P项输出在可接受范围内时，才正常计算积分增量
    this->data.trapezoid = (this->data.err[0] + this->data.err[1]) / 2 * dt_;
  }

  // Ki
  // [解析] 变速积分系数：ki / (1 + |err|)，只有 dynamic_=true 时才真正参与运算。
  this->data.dynamic_ki = ki_ / (1 + std::abs(this->data.err[0]));
  // 如果触发了暂停条件，trapezoid为0，iout的值将保持不变。
  // [解析] 积分累加 + 积分限幅，抗饱和。
  this->data.iout = limit_max(
    this->data.iout + (dynamic_ ? this->data.dynamic_ki : ki_) * this->data.trapezoid, max_iout_);

  // Kd
  // [解析] 这里用的是 dbuf[0]（滤波后的"两帧反馈差"），所以要除以 dt_ 变成速度量纲。
  // [解析] 这也是它和上面两个 set_dot/fdb_dot 重载最大的区别。
  this->data.dout = kd_ * this->data.dbuf[0] / dt_;

  // [解析] 求和 + 输出限幅。
  this->out = limit_max(this->data.pout + this->data.iout + this->data.dout, max_out_);
}

// [解析] 清零函数：把内部状态恢复到"刚构造完"的样子。
// [解析] 什么时候必须调用：
// [解析]   1) 电机首次使能前 —— 避免残留 iout 让车"一上电就窜"；
// [解析]   2) 遥控器从"手动"切回"自动"时 —— 否则积分器还记着手动期间的误差；
// [解析]   3) 重新标定零点/坐标系之后 —— 否则 err 的历史值是错的。
// [解析] 注意它只清自己的状态，不影响 kp/ki/kd 等 const 参数（那些本来也改不了）。
void PID::clear()
{
  this->data = {};   // 清空所有的中间状态和缓存数组
  // [解析] ↑ `= {}` 是 C++11 的"值初始化"写法，把 PIDData 的每个成员都恢复成声明时的默认值
  // [解析]   （set/fdb/pout/iout/dout/err/dbuf/trapezoid/dynamic_ki 全部归 0）。
  // [解析]   比手写一大堆 = 0 更不容易漏。
  this->out = 0.0f;  // 清空输出
  // [解析] out 不属于 data，所以要单独清。忘了这一句，电机仍会收到上一次的输出值。
  this->dout_ready_ = false;
  // [解析] 复位"微分已就绪"标志。下一次 calc() 会像第一帧那样跳过一次微分计算，
  // [解析]   避免拿"清零后的 0"和"真实的反馈值"相减，算出一个巨大的假微分尖峰。
}

}  // namespace sp