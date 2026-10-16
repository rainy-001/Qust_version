// [解析] ==========================================================================
// [解析] 【文件】sp_middleware/tools/joint/joint_controller.hpp
// [解析] 【作用】"关节控制器"：把【一台电机 + 两个 PID + 限幅 + 低通滤波】打包成一个能直接用的关节。
// [解析]         上层只管用 cmd()/cmd_v()/cmd_t() 下目标，然后每个控制周期调一次 control()，
// [解析]         它自己负责读电机反馈、按当前模式做闭环、把力矩指令写回电机。
// [解析] 【被谁用】上层机构类（机械臂、升降、夹爪、拨盘……）里放一个
// [解析]         JointMotorController<sp::DM_Motor> 成员；
// [解析]         本文件的 .cpp 末尾显式实例化了 <sp::DM_Motor> 和 <sp::RM_Motor> 两种，
// [解析]         所以目前只能直接用这两种电机（换成别的电机要自己去 .cpp 里补一行实例化）。
// [解析] 【关键点】1) 这是模板类，实现写在 .cpp 里靠"显式实例化"让编译器看见，所以类型是受限的；
// [解析]   2) motor_ / pid_ / motor_speed_pid_ 都是"引用成员"，构造时传进来的对象必须比它活得久；
// [解析]   3) 所有单位统一：角度 rad、角速度 rad/s、力矩 N·m；
// [解析]   4) control() 是"一个周期走一遍"的函数，读它之前先看懂下面每个成员的含义。
// [解析] 【阅读顺序】先看本文件了解成员和接口，再看 joint_controller.cpp 看 control() 的分支逻辑。
// [解析] ==========================================================================

#ifndef JOINT_CONTROLLER_HPP
#define JOINT_CONTROLLER_HPP

// [解析] ↓ 头文件按需引入：注意这里一个都没多引，因为每个都真的用到了。
// [解析]   dm_motor.hpp / rm_motor.hpp：两种电机的定义。★ 必须在 .cpp 里能看见它们，
// [解析]     因为末尾要写 template class JointMotorController<sp::DM_Motor>; 来显式实例化。
// [解析]   control_mode.hpp：ControlMode 枚举（DISABLE/VELOCITY/POSITION/TORQUE）。
// [解析]   low_pass_filter.hpp：sp::LowPassFilter，用来滤掉位置/速度反馈里的噪声。
// [解析]   pid.hpp：sp::PID，位置环和速度环各需要一个对象，由外部传引用进来。
#include "motor/dm_motor/dm_motor.hpp"
#include "motor/rm_motor/rm_motor.hpp"
#include "tools/joint/control_mode.hpp"
#include "tools/low_pass_filter/low_pass_filter.hpp"
#include "tools/pid/pid.hpp"

// [解析] JointControllerBase：抽象基类（相当于一份"接口说明书"），不能直接创建对象。
// [解析] 为什么要有它：上层机构类可以只持有一个 JointControllerBase*，
// [解析]   从而不必关心底下到底是 DM 电机还是 RM 电机 —— 这就是 C++ 的多态。
// [解析]   代价是每次调用都要走一次虚函数表（对本项目的控制频率来说开销可以忽略）。
// [解析] 注意：它没有任何构造函数，成员都有默认初值，所以子类构造时会自动初始化为 0。
class JointControllerBase
{
public:
  // [解析] 虚析构函数：保证用基类指针 delete 派生类对象时，派生类的析构也会被执行（不漏资源）。
  // [解析] = default 表示"用编译器生成的默认版本就行"，这里没有需要手动释放的东西。
  // [解析] 为什么必须写成 virtual：否则通过基类指针删除派生对象是未定义行为。
  virtual ~JointControllerBase() = default;

  // [解析] ↓ 六个公开成员：所有反馈量都由 control() 每个周期刷新，外面只读、不要自己改。
  // [解析]   单位统一：pos 是角度(rad)、vel 是角速度(rad/s)、torque 是力矩(N·m)。
  // [解析]   带 _filtered 后缀的是"过了一遍低通滤波"的版本，专门喂给 PID；
  // [解析]     滤波能压住编码器/电流采样的噪声，代价是带来一点相位滞后。
  float pos = 0;
  // [解析] ↑ 关节当前角度(rad)，已经过"减中位、减零位、乘方向符号"的换算。
  float vel = 0;
  // [解析] ↑ 关节当前角速度(rad/s)。
  float pos_filtered = 0;
  // [解析] ↑ pos 经过低通滤波的结果(rad)，位置环 PID 用的是它而不是 pos。
  float vel_filtered = 0;
  // [解析] ↑ vel 经过低通滤波的结果(rad/s)，速度环 PID 用的是它而不是 vel。
  float torque_fdb = 0;
  // [解析] ↑ 电机反馈的实际力矩估计值(N·m)，注意不是我们下发的指令值。
  float torque_cmd = 0;
  // [解析] ↑ 本周期算出来、并已经发给电机的力矩指令(N·m)，调试时看它就知道"这一拍出了多大力"。

  // [解析] ↓ 下面这一组是"纯虚函数"（= 0）：只有声明没有实现，派生类必须全部实现。
  // [解析]   因此不能直接 new JointControllerBase，也不能创建它的对象。
  // [解析]   每个函数后面注释里的 pos/vel/torque 表示它属于哪种控制模式。
  virtual void cmd(float value) = 0;
  // [解析] ↑ 位置模式：把关节目标角度设成 value(rad)，内部会限幅到 [min_, max_]。
  virtual void disable() = 0;
  // [解析] ↑ 失能：模式切回 DISABLE、力矩输出 0，并把位置目标同步成当前位置（下次使能不会猛冲）。
  virtual void add(float value) = 0;
  // [解析] ↑ 在"当前的位置目标"上再叠加 value(rad)，常用于遥控器微调/点动。
  virtual void init() = 0;
  // [解析] ↑ 记录此刻的电机角度作为零点，之后 pos 都相对这个零点计算。上电时调一次。
  virtual void cmd_max() = 0;                     //pos
  // [解析] ↑ 一键转到最大角度 max_（内部就是调 cmd(max_)）。
  virtual void cmd_min() = 0;                     //pos
  // [解析] ↑ 一键转到最小角度 min_（内部就是调 cmd(min_)）。
  virtual void cmd_v(float value) = 0;            // vel
  // [解析] ↑ 速度模式：目标角速度 value(rad/s)，会被限幅到 ±max_v_。
  virtual void set_feedforward(float value) = 0;  // feedforward_t
  // [解析] ↑ 设置前馈力矩 value(N·m)。只有构造时 feedforward = true 才真正生效，否则一律当 0。
  virtual void cmd_t(float value) = 0;            // torque
  // [解析] ↑ 力矩模式：直接给力矩 value(N·m)，内部会乘方向符号 sign_。
  virtual void control() = 0;
  // [解析] ↑ ★ 每个控制周期必须调用一次！它负责"读反馈 -> 按模式算 -> 写电机指令"。
  // [解析]   注意它内部只是调用 motor_.cmd()（把指令记进电机对象），
  // [解析]   真正把 CAN 帧发出去的仍然是上层任务（例如 applications/can_task.cpp）。
};

// [解析] JointMotorController<MotorType>：上面那份接口的具体实现，模板参数就是电机类型。
// [解析]   MotorType 需要提供这几个成员/函数（DM_Motor、RM_Motor、LK_Motor 都有）：
// [解析]     float angle(rad) / float speed(rad/s) / float torque(N·m) —— 反馈；
// [解析]     void cmd(float torque)                                    —— 下发力矩指令。
// [解析] 【控制链路（POSITION 模式，最常用的串级双环）】
// [解析]   目标角度 set_
// [解析]     └─ 外环 pid_.calc(set_, pos_filtered)          -> 算出"目标角速度"
// [解析]        └─ 内环 motor_speed_pid_.calc(目标角速度, vel_filtered) -> 算出"目标力矩"
// [解析]           └─ 乘方向符号 sign_ -> motor_.cmd()      -> 电机执行
// [解析]   为什么用双环：外环管"到没到位"，内环管"转得多快"，比单个位置 PID 更稳、抗扰更好。
// [解析] 【典型用法】
// [解析]   sp::PID pos_pid(1e-3f, ...);  sp::PID spd_pid(1e-3f, ...);
// [解析]   JointMotorController<sp::DM_Motor> joint(mid, min, max, min_m, max_m, max_v,
// [解析]                                                  false, motor, pos_pid, spd_pid);
// [解析]   joint.init();          // 上电后记零点
// [解析]   joint.cmd(0.5f);       // 目标角度 0.5 rad
// [解析]   每个周期 joint.control();  // 必须周期性调用，PID 的 dt 要和这个周期一致
template <typename MotorType>
class JointMotorController : public JointControllerBase
{
public:
  // [解析] 构造函数参数很多，按顺序逐个说清（单位：角度 rad、角速度 rad/s、力矩 N·m）：
  // [解析]   mid      —— 机械中位角(rad)。换算关系见 control()：
  // [解析]               pos = sign_*motor_.angle - mid_ - init_angle_，用来把电机读数变成关节角度。
  // [解析]   min/max  —— 关节角度的软限位(rad)，cmd() 会把目标夹在 [min, max] 内，防止撞机械限位。
  // [解析]   min_m/max_m —— 电机侧的单圈展开区间(rad)，只在 limited = true 时用（见 control() 的第②步）。
  // [解析]   max_v    —— 速度限幅(rad/s)，cmd_v() 把目标夹在 [-max_v, +max_v] 内。
  // [解析]   reverse  —— true 表示"电机正转 = 关节负方向"，内部换算成 sign_ = -1。
  // [解析]   motor    —— 底层电机对象的【引用】，必须比本对象活得久（通常是上层类的成员）。
  // [解析]   pid      —— 位置环 PID 的【引用】（外环），只在 POSITION 模式用。
  // [解析]   motor_speed_pid —— 速度环 PID 的【引用】（内环），POSITION/VELOCITY 模式都用。
  // [解析]   feedforward —— 是否允许 set_feedforward() 生效，默认 false（不启用前馈）。
  // [解析]   limited  —— 是否在 control() 里把角度折到 [min_m_, max_m_] 一圈内，默认 true。
  // [解析]   pos_filter / vel_filter —— 位置/速度低通滤波系数，范围 [0,1]，1 = 不滤波，默认 1.0f。
  // [解析] ★ 注意：PID 是引用传进来的，所以两个关节绝对不能共用同一个 PID 对象，
  // [解析]   否则它们的目标值和积分状态会互相污染（表现是"动一个关节另一个也跟着抖"）。
  JointMotorController(
    float mid, float min, float max, float min_m, float max_m, float max_v, bool reverse,
    MotorType & motor, sp::PID & pid, sp::PID & motor_speed_pid, bool feedforward = false,
    bool limited = true, float pos_filter = 1.0f, float vel_filter = 1.0f);

  // [解析] ↓ 这 10 个是重写基类的虚函数，功能与基类注释一一对应，不再重复。
  // [解析]   override 关键字是"我确实在重写基类的虚函数"的声明：写错名字/参数时编译器会报错，
  // [解析]   避免"以为重写了、其实写了个新函数"这种极难查的 bug。
  void disable() override;
  // [解析] ↑ 失能：模式切 DISABLE，并把位置目标同步到当前位置。
  void add(float value) override;
  // [解析] ↑ 在当前位置目标上叠加 value(rad)。
  void init() override;
  // [解析] ↑ 记录当前电机角度为零点 init_angle_。上电、使能后调用一次。

  void cmd(float value) override;                      // pos
  // [解析] ↑ 位置目标 value(rad)，限幅到 [min_, max_]，同时把速度目标清零。
  void cmd_max() override;                             //pos
  // [解析] ↑ 等价于 cmd(max_)。
  void cmd_min() override;                             //pos
  // [解析] ↑ 等价于 cmd(min_)。
  void cmd_v(float value) override;                    // vel
  // [解析] ↑ 速度目标 value(rad/s)，限幅到 ±max_v_。
  bool cmd_v_until_t(float value, float t_threshold);  // vel with torque threshold
  // [解析] 带"力矩到顶自动停"的速度指令（返回值是它比普通 cmd_v 多出来的信息）。
  // [解析]   value —— 目标速度(rad/s)；t_threshold —— 力矩阈值(N·m)。
  // [解析]   逻辑：|torque_fdb| > 阈值 就切成 TORQUE 模式并卸力(0 力矩)，否则继续按速度转。
  // [解析]   返回：true = 本周期已触发切换（表示"已经顶住了"）；false = 还在正常转。
  // [解析]   典型场景：让机构朝一个方向顶过去，顶到限位/夹住东西后自动停下，不会硬顶烧电机。
  bool cmd_v_until_stuck(float value, float t_threshold, float v_threshold);
  // [解析] 比上面更严格的版本：必须"力矩超阈值"且"速度已经掉下来"才认输。
  // [解析]   v_threshold —— 速度阈值(rad/s)，要求 |vel| < v_threshold 才算停住。
  // [解析]   为什么加这个条件：启动加速的瞬间力矩也很大，只看力矩会误判成"顶住了"提前卸力。
  // [解析]   返回：true = 已切换到力矩模式(0 力矩)；false = 还在转。
  bool cmd_pos_until_t(float value, float t_threshold);  // pos with torque threshold
  // [解析] 位置版的"到顶自动停"：以 value(rad) 为目标做位置闭环，
  // [解析]   一旦 |torque_fdb| > t_threshold 就切 TORQUE 模式卸力并返回 true。
  // [解析]   典型场景：夹爪慢慢合拢去夹物体，夹住后力矩升高就停手，不会把东西夹坏。
  void set_feedforward(float value) override;            // feedforward_t
  // [解析] ↑ 设置前馈力矩 value(N·m)：它是"已知需要出的力"（比如重力补偿），
  // [解析]   直接加在 PID 输出上，让 PID 只负责修误差，响应更快、超调更小。
  void cmd_t(float value) override;                      // torque
  // [解析] ↑ 力矩目标 value(N·m)，内部乘 sign_ 存进 t_set_，不做任何闭环。
  void control() override;
  // [解析] ↑ ★ 每个控制周期调用一次：刷反馈 -> 按 mode_ 分派 -> 下发力矩指令。
  // [解析]   注意它内部会真的调用 motor_.cmd()，所以不要在多处重复调用它。

  // [解析] ↓ 下面 7 个是"构造时就定死"的常量参数，全部 const（想改只能重新构造对象）。
  // [解析]   它们和构造函数的参数一一对应，含义见上面构造函数的说明。
  const float mid_;
  const float min_;
  const float max_;
  const float min_m_;
  const float max_m_;
  const float max_v_;
  const float sign_;
  // [解析] ↑ sign_ = reverse ? -1.0f : +1.0f，作用是把"电机方向"和"关节方向"统一起来，
  // [解析]   读反馈时乘它，下发指令时也乘它（除了 cmd_t()，那里在存 t_set_ 时就乘过了）。

  // [解析] ★ 下面三个都是【引用】（&）：本对象不拥有它们，只是"借用"。
  // [解析]   所以它们必须比本对象活得久；在 FreeRTOS 多任务环境里，如果另一个任务也会访问
  // [解析]   同一个电机/PID 对象，需要你自己保证互斥（本类内部没有任何加锁）。
  MotorType & motor_;
  // [解析] ↑ 底层电机（DM_Motor / RM_Motor）。
  // [解析]   control() 读它的 angle/speed/torque，写它的 cmd()；write()/发帧由上层任务负责。

  sp::PID & pid_;
  // [解析] ↑ 位置环 PID（外环），只在 POSITION 模式使用，输出被当作"目标角速度"。
  sp::PID & motor_speed_pid_;
  // [解析] ↑ 速度环 PID（内环），POSITION 和 VELOCITY 两种模式都要用，输出被当作"目标力矩"。

  // [解析] ↓ 运行过程中会变的状态：当前模式 + 三个目标值 + 前馈 + 零位。
  // [解析]   它们都是 public 的（方便调试观察），但请只读，不要从外部乱改。
  ControlMode mode_ = ControlMode::DISABLE;
  // [解析] ↑ 当前控制模式，初值 DISABLE 很关键：保证对象刚构造、还没下任何指令时不会输出力矩。
  float set_ = 0;
  // [解析] ↑ 位置目标(rad)，由 cmd()/cmd_max()/cmd_min()/add() 改写。
  float v_set_ = 0;
  // [解析] ↑ 速度目标(rad/s)，由 cmd_v() 系列改写。
  float t_set_ = 0;
  // [解析] ↑ 力矩目标(N·m)，由 cmd_t() 改写（注意存进去时已经乘过 sign_ 了）。
  float feedforward_t_ = 0;
  // [解析] ↑ 前馈力矩(N·m)，由 set_feedforward() 写入；feedforward_ 为 false 时会被强制成 0。
  float init_angle_ = 0.0f;
  // [解析] ↑ init() 时记录的电机原始角度(rad)，用来把 pos 的零点挪到"上电时的姿态"。

private:
  // [解析] ↓ 这四个是行为开关/滤波参数，同样是构造时定死的。
  const bool feedforward_;
  // [解析] ↑ 是否允许前馈力矩生效。false 时 set_feedforward() 传什么进来都等于 0（写了也白写）。
  bool limited_;
  // [解析] ↑ 是否做"单圈展开"（把 pos 折回 [min_m_, max_m_] 内）。
  // [解析]   ★ 注意它是这里唯一【没有】const 的成员，所以运行中可以被改（虽然本项目里没有地方改它）。
  const float pos_filter_;
  // [解析] ↑ 位置低通滤波系数，范围 [0,1]，1 = 不滤波。只用于在初始化列表里构造下面的滤波器。
  const float vel_filter_;
  // [解析] ↑ 速度低通滤波系数，范围 [0,1]，1 = 不滤波。
  // [解析] ★ 注意下面两行的写法：{pos_filter_} 括号里用的是"另一个成员变量"pos_filter_。
  // [解析]   C++ 的成员初始化顺序是按【声明顺序】，而不是按初始化列表的书写顺序。
  // [解析]   这里 pos_filter_ / vel_filter_ 恰好声明在上面，所以能取到正确的值；
  // [解析]   如果哪天把这两个滤波器挪到 pos_filter_ 声明之前，就会用未初始化的值去构造滤波器 ——
  // [解析]   编译器不一定报错，运行时却是随机的，属于很隐蔽的坑。
  sp::LowPassFilter motor_pos_filter_{pos_filter_};
  // [解析] ↑ 位置低通滤波器。control() 每个周期喂一次，读它的 out 得到 pos_filtered。
  sp::LowPassFilter motor_vel_filter_{vel_filter_};
  // [解析] ↑ 速度低通滤波器，同上，输出 vel_filtered。
  // [解析]   ★ 小提醒：这两个滤波器是有状态的（内部记住上一帧的值），
  // [解析]     所以"重新使能/清零"时它们不会被自动复位，需要时请自己重建或另加复位逻辑。
};

#endif  // JOINT_CONTROLLER_HPP