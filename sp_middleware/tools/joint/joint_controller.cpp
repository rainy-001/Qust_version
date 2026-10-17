// [解析] JointMotorController<MotorType> 的全部实现，末尾两行是显式实例化（模板实现写在这里，不写就链接不过）。
// [解析] cmd_xxx() 都是同一套路"设模式 + 设目标 + 洗掉其它目标"；sign_ 只在关节<->电机的边界上乘，乘错方向会反。

#include "joint_controller.hpp"

#include "tools/math_tools/math_tools.hpp"

// [解析] 参数全部交给 const 成员，函数体为空；实际初始化次序由 hpp 里的声明顺序决定，不是这里书写的顺序。
template <typename MotorType>
JointMotorController<MotorType>::JointMotorController(
  float mid, float min, float max, float min_m, float max_m, float max_v, bool reverse,
  MotorType & motor, sp::PID & pid, sp::PID & motor_speed_pid, bool feedforward, bool limited,
  float pos_filter, float vel_filter)
: mid_(mid),
  min_(min),
  max_(max),
  min_m_(min_m),
  max_m_(max_m),
  max_v_(max_v),
  sign_(reverse ? -1.0f : 1.0f),
  motor_(motor),
  pid_(pid),
  motor_speed_pid_(motor_speed_pid),
  feedforward_(feedforward),
  limited_(limited),
  pos_filter_(pos_filter),
  vel_filter_(vel_filter)
{
}

// [解析] 松手：切 DISABLE、set_ 同步成当前位置、速度/力矩目标清零；真正不出力发生在下一次 control()。
template <typename MotorType>
void JointMotorController<MotorType>::disable()
{
  mode_ = ControlMode::DISABLE;
  set_ = pos;
  v_set_ = 0;
  t_set_ = 0;
}

// [解析] 记零点 init_angle_（未乘符号的原始角度），control() 里的 pos 都要减掉它；上电后调用一次。
template <typename MotorType>
void JointMotorController<MotorType>::init()
{
  init_angle_ = motor_.angle;
}

// [解析] 在"当前目标 set_"上叠加（不是实测 pos），所以连续调用会累加；内部走 cmd()，模式变 POSITION。
template <typename MotorType>
void JointMotorController<MotorType>::add(float value)
{
  cmd(set_ + value);
}

// [解析] 位置指令：切 POSITION、清速度目标、目标限幅到 [min_, max_] 后存进 set_，防顶机械限位。
template <typename MotorType>
void JointMotorController<MotorType>::cmd(float value)
{
  mode_ = ControlMode::POSITION;
  v_set_ = 0;
  set_ = sp::limit_min_max(value, min_, max_);
}

template <typename MotorType>
void JointMotorController<MotorType>::cmd_max()
{
  cmd(max_);
}

template <typename MotorType>
void JointMotorController<MotorType>::cmd_min()
{
  cmd(min_);
}

// [解析] 速度指令：切 VELOCITY，目标限幅 ±max_v_，并把 set_ 同步成当前位置，免得切回位置模式时往回猛拉。
template <typename MotorType>
void JointMotorController<MotorType>::cmd_v(float value)
{
  mode_ = ControlMode::VELOCITY;
  v_set_ = sp::limit_max(value, max_v_);
  set_ = this->pos;
}

// [解析] 按速度 value 转，|torque_fdb| 超 t_threshold 就切 TORQUE 卸力(0 力矩)并返回 true；读的是上一拍的力矩值。
template <typename MotorType>
bool JointMotorController<MotorType>::cmd_v_until_t(float value, float t_threshold)
{
  if (fabs(this->torque_fdb) > t_threshold) {
    mode_ = ControlMode::TORQUE;
    t_set_ = 0;
    v_set_ = 0;
    set_ = this->pos;
    return true;
  }
  else {
    mode_ = ControlMode::VELOCITY;
    v_set_ = sp::limit_max(value, max_v_);
    set_ = this->pos;
    return false;
  }
}

// [解析] 加强版：还要求 |vel| < v_threshold 才算卡住，只看力矩会把起步加速误判成顶住而提前卸力。
template <typename MotorType>
bool JointMotorController<MotorType>::cmd_v_until_stuck(
  float value, float t_threshold, float v_threshold)
{
  if (fabs(this->torque_fdb) > t_threshold && fabs(this->vel) < v_threshold) {
    mode_ = ControlMode::TORQUE;
    t_set_ = 0;
    v_set_ = 0;
    set_ = this->pos;
    return true;
  }
  else {
    mode_ = ControlMode::VELOCITY;
    v_set_ = sp::limit_max(value, max_v_);
    set_ = this->pos;
    return false;
  }
}

// [解析] 位置版的到顶自动停：以 value 做位置闭环，力矩超阈值就切 TORQUE 卸力并返回 true（夹爪夹住东西的场景）。
template <typename MotorType>
bool JointMotorController<MotorType>::cmd_pos_until_t(float value, float t_threshold)
{
  if (fabs(this->torque_fdb) > t_threshold) {
    mode_ = ControlMode::TORQUE;
    t_set_ = 0;
    v_set_ = 0;
    set_ = this->pos;
    return true;
  }
  else {
    mode_ = ControlMode::POSITION;
    v_set_ = 0;
    set_ = sp::limit_min_max(value, min_, max_);
    return false;
  }
}

// [解析] 力矩指令：不做闭环，直接 t_set_ = sign_ * value（关节方向 -> 电机方向），本函数不限幅。
template <typename MotorType>
void JointMotorController<MotorType>::cmd_t(float value)
{
  mode_ = ControlMode::TORQUE;
  t_set_ = sign_ * value;
  set_ = this->pos;
  v_set_ = this->vel;
}

// [解析] 前馈力矩：加在 control() 里"乘 sign_ 之前"，所以要用关节方向；feedforward_ 为 false 时一律记 0。
template <typename MotorType>
void JointMotorController<MotorType>::set_feedforward(float value)
{
  this->feedforward_t_ = feedforward_ ? value : 0;
}

// [解析] 每周期调一次：读反馈 -> 按 mode_ 分派 -> 下发力矩（内部会真的调 motor_.cmd()）。
template <typename MotorType>
void JointMotorController<MotorType>::control()
{
  // [解析] 关节角度 = sign_*motor_.angle - mid_ - init_angle_；init_angle_ 是未乘符号的原始角度，reverse 时零点不对称。
  this->pos = sign_ * motor_.angle - mid_ - init_angle_;
  // [解析] 单圈展开：电机 angle 可能是多圈累计值，超出 [min_m_, max_m_] 就减/加一圈(2π)；只折一次，不是 while。
  if (limited_) {
    if (this->pos > max_m_) this->pos -= 2 * sp::SP_PI;
    if (this->pos < min_m_) this->pos += 2 * sp::SP_PI;
  }
  // [解析] 位置低通：PID 用滤波值压噪声，对外仍公布原始 pos，判断到位时不被滞后骗到。
  motor_pos_filter_.update(this->pos);
  this->pos_filtered = motor_pos_filter_.out;

  this->vel = sign_ * motor_.speed;
  motor_vel_filter_.update(this->vel);
  this->vel_filtered = motor_vel_filter_.out;
  // [解析] 力矩反馈是电机的"实际出力估计值"，不是下发指令；两个 until 函数的阈值判据用的就是它。
  this->torque_fdb = sign_ * motor_.torque;

  // [解析] 失能必须真的下发 motor_.cmd(0)：只 return 的话有些电机（尤其 DM 系列）会保持上次指令继续出力。
  if (mode_ == ControlMode::DISABLE) {
    torque_cmd = 0.0f;
    motor_.cmd(0);
    return;
  }

  // [解析] 力矩模式直接发 t_set_；这里不能再乘 sign_，cmd_t() 里已经乘过，重复乘会反。
  if (mode_ == ControlMode::TORQUE) {
    torque_cmd = t_set_;
    motor_.cmd(torque_cmd);
  }
  else {
    // [解析] 位置模式是串级双环：外环 pid_ 输出目标角速度，内环 motor_speed_pid_ 输出目标力矩；两个 PID 的 dt 必须等于本函数调用周期。
    if (mode_ == ControlMode::POSITION) {
      pid_.calc(set_, this->pos_filtered);
      motor_speed_pid_.calc(pid_.out, this->vel_filtered);
    }
    // [解析] 速度模式只跑内环，位置环 pid_ 不被调用，它的积分会一直保留上一次的值。
    else if (mode_ == ControlMode::VELOCITY) {
      motor_speed_pid_.calc(v_set_, this->vel_filtered);
    }

    // [解析] 输出力矩 = sign_ * (速度环输出 + 可选前馈)：PID 输出是关节方向，在这里统一换成电机方向。
    torque_cmd =
      sign_ * (feedforward_ ? motor_speed_pid_.out + feedforward_t_ : motor_speed_pid_.out);
    // [解析] 这一步只把指令记进电机对象，真正打包 CAN 帧发送由上层任务（如 can_task）负责。
    motor_.cmd(torque_cmd);
  }
}

// [解析] 显式实例化：不写这两行编译器不会为模板生成代码，链接时就是 undefined reference。
// [解析] 目前只有 DM_Motor / RM_Motor 能用；新增电机要在这里补一行 template class ...，否则编译过、链接挂。
template class JointMotorController<sp::DM_Motor>;
template class JointMotorController<sp::RM_Motor>;