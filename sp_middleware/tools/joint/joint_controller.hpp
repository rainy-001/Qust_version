// [解析] 关节控制器：一台电机 + 两个 PID + 限幅 + 低通，打包成一个能直接用的关节（cmd 下目标，control 每周期闭环一次）。
// [解析] 模板实现写在 .cpp 里靠显式实例化，目前只支持 DM_Motor / RM_Motor；motor_/pid_ 是引用，必须比本对象活得久。

#ifndef JOINT_CONTROLLER_HPP
#define JOINT_CONTROLLER_HPP

#include "motor/dm_motor/dm_motor.hpp"
#include "motor/rm_motor/rm_motor.hpp"
#include "tools/joint/control_mode.hpp"
#include "tools/low_pass_filter/low_pass_filter.hpp"
#include "tools/pid/pid.hpp"

// [解析] JointControllerBase：抽象基类，让上层只持基类指针、不关心底下是 DM 还是 RM 电机（成员都有默认初值 0）。
class JointControllerBase
{
public:
  virtual ~JointControllerBase() = default;

  // [解析] 反馈量由 control() 每周期刷新，外面只读；pos/vel 单位 rad、rad/s，torque N·m；带 _filtered 的是过完低通、专门喂 PID 的版本。
  float pos = 0;
  float vel = 0;
  float pos_filtered = 0;
  float vel_filtered = 0;
  float torque_fdb = 0;
  float torque_cmd = 0;

  // [解析] 纯虚函数（= 0），派生类必须实现；行尾的 pos/vel/torque 标出它属于哪种控制模式。
  virtual void cmd(float value) = 0;
  virtual void disable() = 0;
  virtual void add(float value) = 0;
  virtual void init() = 0;
  virtual void cmd_max() = 0;                     //pos
  virtual void cmd_min() = 0;                     //pos
  virtual void cmd_v(float value) = 0;            // vel
  virtual void set_feedforward(float value) = 0;  // feedforward_t
  virtual void cmd_t(float value) = 0;            // torque
  virtual void control() = 0;
};

// [解析] JointMotorController<MotorType>：接口的实现。POSITION 是串级双环：pid_ 输出目标角速度，motor_speed_pid_ 输出目标力矩，乘 sign_ 后 cmd()。
// [解析] 用法：构造 -> init() 记零点 -> cmd(x) -> 每周期 control()；两个 PID 的 dt 必须和控制周期一致。
template <typename MotorType>
class JointMotorController : public JointControllerBase
{
public:
  // [解析] 参数（角度 rad、角速度 rad/s、力矩 N·m）：mid 机械中位、min/max 关节软限位、min_m/max_m 单圈展开区间、max_v 速度限幅、reverse 是否反装。
  // [解析] motor/pid/motor_speed_pid 是引用（两个关节绝不能共用同一个 PID）；feedforward 是否允许前馈，limited 是否单圈展开，pos_filter/vel_filter 是低通系数 [0,1]。
  JointMotorController(
    float mid, float min, float max, float min_m, float max_m, float max_v, bool reverse,
    MotorType & motor, sp::PID & pid, sp::PID & motor_speed_pid, bool feedforward = false,
    bool limited = true, float pos_filter = 1.0f, float vel_filter = 1.0f);

  void disable() override;
  void add(float value) override;
  void init() override;

  void cmd(float value) override;                      // pos
  void cmd_max() override;                             //pos
  void cmd_min() override;                             //pos
  void cmd_v(float value) override;                    // vel
  // [解析] 力矩到顶自动停：|torque_fdb| > t_threshold 就切 TORQUE 卸力并返回 true（顶限位/夹住东西的场景）。
  bool cmd_v_until_t(float value, float t_threshold);  // vel with torque threshold
  // [解析] 加强版：还要求 |vel| < v_threshold 才算卡住，避免起步加速的大力矩被误判成顶住。
  bool cmd_v_until_stuck(float value, float t_threshold, float v_threshold);
  // [解析] 位置版的到顶自动停：以 value 做位置闭环，力矩超阈值就切 TORQUE 卸力并返回 true。
  bool cmd_pos_until_t(float value, float t_threshold);  // pos with torque threshold
  void set_feedforward(float value) override;            // feedforward_t
  void cmd_t(float value) override;                      // torque
  void control() override;

  const float mid_;
  const float min_;
  const float max_;
  const float min_m_;
  const float max_m_;
  const float max_v_;
  const float sign_;

  // [解析] motor_/pid_/motor_speed_pid_ 是引用：本对象不拥有它们，必须比本对象活得久；类内部不加锁，多任务共用要自己保证互斥。
  MotorType & motor_;

  sp::PID & pid_;
  sp::PID & motor_speed_pid_;

  ControlMode mode_ = ControlMode::DISABLE;
  float set_ = 0;
  float v_set_ = 0;
  float t_set_ = 0;
  float feedforward_t_ = 0;
  float init_angle_ = 0.0f;

private:
  const bool feedforward_;
  bool limited_;
  const float pos_filter_;
  const float vel_filter_;
  // [解析] 两个滤波器用 {pos_filter_} 构造：C++ 按声明顺序初始化，所以这两个系数必须声明在滤波器之前，挪到后面会拿未初始化的值去构造。
  sp::LowPassFilter motor_pos_filter_{pos_filter_};
  sp::LowPassFilter motor_vel_filter_{vel_filter_};
};

#endif  // JOINT_CONTROLLER_HPP