// [解析] 4 个 calc() 重载的实现。四者只差两件事：D 项从哪里来、要不要积分分离。
// [解析]   calc(set, fdb[, thr])                   微分先行 + alpha 滤波，最常用
// [解析]   calc(set, fdb, set_dot, fdb_dot[, thr]) 外部直接给速度，去掉差分和滤波
// [解析] 三处最容易看晕：微分必须用"上一帧 data.fdb"算，所以要在更新它之前完成；
// [解析]   err/dbuf 每拍整体后移一格；重载 3/4 不除 dt_，kd 量纲和重载 1/2 差一个 dt。

#include "pid.hpp"

namespace sp
{
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

// [解析] 重载 1：D 项走"微分先行"，微分成 fdb 而不是 err，目标值突变时不会产生微分冲击。
// [解析] 代价是目标速度不进 D 项；噪声先由 alpha_ 做一阶低通压一下。
void PID::calc(float set, float fdb)
{
  // [解析] 旧减新：反馈在变大时差为负，乘正 kd 后 D 项为负，正好抵消一点正向输出 —— 这就是阻尼。
  // [解析] 必须在更新 data.fdb 之前算，否则"上一帧 fdb"就成了"本帧 fdb"，微分恒为 0。
  if (dout_ready_ == true) {
    // 微分先行
    this->data.dbuf[2] = this->data.dbuf[1];
    this->data.dbuf[1] = this->data.dbuf[0];
    // [解析] angular_ 打开时先把角度差折到 (-π, π]，避免 179°→-179° 被当成绕远路 358°。
    this->data.dbuf[0] = angular_ ? limit_angle((this->data.fdb - fdb)) : (this->data.fdb - fdb);
  }
  else {
    // [解析] 第一帧没有"上一帧 fdb"可用（data.fdb 的 0 是假的），只置标志、跳过微分。
    dout_ready_ = true;
  }

  // 滤波
  this->data.dbuf[0] = alpha_ * this->data.dbuf[0] + (1 - alpha_) * this->data.dbuf[1];

  this->data.err[2] = this->data.err[1];
  this->data.err[1] = this->data.err[0];
  this->data.err[0] = angular_ ? limit_angle(set - fdb) : (set - fdb);

  this->data.set = set;
  this->data.fdb = fdb;

  // Kp
  this->data.pout = kp_ * this->data.err[0];

  // Ki
  this->data.trapezoid = (this->data.err[0] + this->data.err[1]) / 2 * dt_;  // 梯形积分
  // [解析] 变速积分：dynamic_ 为真时用 ki/(1+|err|) —— 大误差少积分、小误差多积分，兼顾快与稳。
  this->data.dynamic_ki = ki_ / (1 + std::abs(this->data.err[0]));           // 变速积分
  // [解析] trapezoid 只是本拍积分增量（未乘 ki），iout 才是有记忆的累加器；外层 limit_max 就是抗积分饱和。
  this->data.iout = limit_max(
    this->data.iout + (dynamic_ ? this->data.dynamic_ki : ki_) * this->data.trapezoid, max_iout_);

  // Kd
  this->data.dout = kd_ * this->data.dbuf[0] / dt_;

  this->out = limit_max(this->data.pout + this->data.iout + this->data.dout, max_out_);
}

// [解析] 前馈版：D 项直接取 kd × (set_dot - fdb_dot)，适合速度已由传感器测出的场合，比数值微分干净。
// [解析] 这版不做 alpha 滤波、不动 dbuf，也没有变速积分；而且不更新 data.set / data.fdb。
void PID::calc(float set, float fdb, float set_dot, float fdb_dot)
{
  this->data.err[2] = this->data.err[1];
  this->data.err[1] = this->data.err[0];
  this->data.err[0] = angular_ ? limit_angle(set - fdb) : (set - fdb);

  // Kp
  this->data.pout = kp_ * this->data.err[0];

  // Ki
  this->data.trapezoid = (this->data.err[0] + this->data.err[1]) / 2 * dt_;  // 梯形积分
  this->data.iout = limit_max(ki_ * this->data.trapezoid + this->data.iout, max_iout_);

  // Kd
  // [解析] 这里没有除 dt_：set_dot/fdb_dot 本身已是"每秒变化量"，再除一次就错了；kd 量级与重载 1 不同。
  this->data.dout = kd_ * (set_dot - fdb_dot);

  this->out = limit_max(this->data.pout + this->data.iout + this->data.dout, max_out_);
}

// [解析] = 上一个重载 + 积分分离，是"全功能"版本。
// [解析] 阈值比较的是 |P 项输出|（= kp×err）而不是 err，所以换了 kp 也不必重新标定阈值。
void PID::calc(float set, float fdb, float set_dot, float fdb_dot, float integral_pause_threshold)
{
  this->data.err[2] = this->data.err[1];
  this->data.err[1] = this->data.err[0];
  this->data.err[0] = angular_ ? limit_angle(set - fdb) : (set - fdb);

  // Kp
  this->data.pout = kp_ * this->data.err[0];

  // 积分分离
  // 当P项输出的绝对值已经很大时，暂停积分累积
  if (std::abs(this->data.pout) > integral_pause_threshold) {
    // 将本次的积分增量设为0
    // [解析] 只是把本拍增量置 0，已经积累的 iout 仍然保留并继续发挥作用。
    this->data.trapezoid = 0.0f;
  }
  else {
    // 只有当P项输出在可接受范围内时，才正常计算积分增量
    this->data.trapezoid = (this->data.err[0] + this->data.err[1]) / 2 * dt_;  // 梯形积分
  }

  // Ki
  // 此处完全同步了你第二个函数中动态 Ki (dynamic_ki) 的逻辑
  this->data.dynamic_ki = ki_ / (1 + std::abs(this->data.err[0]));
  // 如果触发了暂停条件，trapezoid为0，iout的值将保持不变。
  this->data.iout = limit_max(
    this->data.iout + (dynamic_ ? this->data.dynamic_ki : ki_) * this->data.trapezoid, max_iout_);

  // Kd
  // 保持第一个函数原有的前馈微分计算方式
  // [解析] ↑ 原作者这条注释说法不准：本行用的是 set_dot - fdb_dot，属重载 3 的写法，不除 dt_、不滤波。
  this->data.dout = kd_ * (set_dot - fdb_dot);

  this->out = limit_max(this->data.pout + this->data.iout + this->data.dout, max_out_);
}

// [解析] = 重载 1 + 积分分离：既要微分先行的平滑，又要防止起步阶段积分堆太高。
// [解析] 内部逻辑同重载 1，只多了"P 项够小才积分"这个开关。
void PID::calc(float set, float fdb, float integral_pause_threshold)
{
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
  this->data.dbuf[0] = alpha_ * this->data.dbuf[0] + (1 - alpha_) * this->data.dbuf[1];

  this->data.err[2] = this->data.err[1];
  this->data.err[1] = this->data.err[0];
  this->data.err[0] = angular_ ? limit_angle(set - fdb) : (set - fdb);

  this->data.set = set;
  this->data.fdb = fdb;

  // Kp
  this->data.pout = kp_ * this->data.err[0];

  // 积分分离
  // 当P项输出的绝对值已经很大时，暂停积分累积
  if (std::abs(this->data.pout) > integral_pause_threshold) {
    // 将本次的积分增量设为0
    this->data.trapezoid = 0.0f;
  }
  else {
    // 只有当P项输出在可接受范围内时，才正常计算积分增量
    this->data.trapezoid = (this->data.err[0] + this->data.err[1]) / 2 * dt_;
  }

  // Ki
  this->data.dynamic_ki = ki_ / (1 + std::abs(this->data.err[0]));
  // 如果触发了暂停条件，trapezoid为0，iout的值将保持不变。
  this->data.iout = limit_max(
    this->data.iout + (dynamic_ ? this->data.dynamic_ki : ki_) * this->data.trapezoid, max_iout_);

  // Kd
  this->data.dout = kd_ * this->data.dbuf[0] / dt_;

  this->out = limit_max(this->data.pout + this->data.iout + this->data.dout, max_out_);
}

// [解析] 把 data 和 out 全部归零并复位 dout_ready_；不复位的话下一拍会拿 0 与真实反馈相减，算出假微分尖峰。
// [解析] 电机使能前、手动/自动切换后、重新标定零点后都要调用，残留的 iout 会让车一上电就冲一下。
void PID::clear()
{
  this->data = {};   // 清空所有的中间状态和缓存数组
  this->out = 0.0f;  // 清空输出
  this->dout_ready_ = false;
}

}  // namespace sp