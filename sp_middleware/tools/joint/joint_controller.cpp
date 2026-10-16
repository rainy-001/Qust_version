// [解析] ==========================================================================
// [解析] 【文件】sp_middleware/tools/joint/joint_controller.cpp
// [解析] 【作用】JointMotorController<MotorType> 的全部实现，外加上最后两行"显式实例化"。
// [解析] 【看代码的重点】
// [解析]   1) control() 是唯一的"大函数"，读它之前先看 hpp 里每个成员的含义和单位；
// [解析]   2) 所有的 cmd_xxx() 干的都是同一套动作：【设模式 + 设目标 + 把其它目标洗干净】，
// [解析]      理解了这个套路，8 个 cmd 函数一眼就能扫完；
// [解析]   3) 符号 sign_ 出现在哪些地方、为什么有的地方乘有的地方不乘（这是最容易写错的地方）；
// [解析]   4) 文件末尾两行"显式实例化"是模板类能链接通过的关键，缺了就会 undefined reference。
// [解析] 【阅读顺序】先看 hpp，再看本文件的 control()，最后回头看各个 cmd_xxx()。
// [解析] ==========================================================================

#include "joint_controller.hpp"

// [解析] 用到 math_tools 里的两个限幅函数和圆周率常量：
// [解析]   sp::limit_min_max(x, min, max) —— 把 x 夹在 [min, max] 区间（非对称，用于关节角度）；
// [解析]   sp::limit_max(x, max)          —— 把 x 夹在 [-max, max]（对称，用于速度、力矩）；
// [解析]   sp::SP_PI                      —— π ≈ 3.14159，下面"折一圈"要用到 2π。
// [解析] 另外 fabs() 来自 <cmath>，由这个头文件间接引入，用来取力矩的绝对值。
#include "tools/math_tools/math_tools.hpp"

// [解析] 构造函数：参数全部通过"初始化列表"交给 const 成员，函数体是空的。
// [解析] 参数含义、单位、取值范围见 joint_controller.hpp 里那一大段说明，这里只说写法上的坑：
// [解析]   ① 冒号后面的每一项都是"成员名(初值)"，用逗号分隔，顺序建议和 hpp 里的声明顺序保持一致；
// [解析]   ② sign_(reverse ? -1.0f : 1.0f) 把 bool 翻译成 ±1，避免后面到处写 if；
// [解析]   ③ motor_/pid_/motor_speed_pid_ 只是把"引用"绑定上去，不拷贝对象、也不检查有效性；
// [解析]   ④ 实际初始化次序由 hpp 里的【声明顺序】决定，不是这里从上到下的书写顺序。
// [解析] 副作用：完全没有硬件操作、也没有给电机下任何指令（真正出力发生在 control() 里）。
template <typename MotorType>
JointMotorController<MotorType>::JointMotorController(
  float mid, float min, float max, float min_m, float max_m, float max_v, bool reverse,
  MotorType & motor, sp::PID & pid, sp::PID & motor_speed_pid, bool feedforward, bool limited,
  float pos_filter, float vel_filter)
: mid_(mid),                                    // [解析] 机械中位角(rad)，参与 pos 的换算
  min_(min),                                    // [解析] 关节角度下限(rad)，cmd() 的软限位
  max_(max),                                    // [解析] 关节角度上限(rad)
  min_m_(min_m),                                // [解析] 电机侧单圈展开下限(rad)，limited_ 为 true 时生效
  max_m_(max_m),                                // [解析] 电机侧单圈展开上限(rad)
  max_v_(max_v),                                // [解析] 速度限幅(rad/s)，cmd_v() 用 ±max_v_
  sign_(reverse ? -1.0f : 1.0f),                // [解析] 方向符号：reverse 为真取 -1，之后所有换算都乘它
  motor_(motor),                                // [解析] 绑定底层电机的引用（务必活得比本对象久）
  pid_(pid),                                    // [解析] 绑定位置环 PID 的引用（外环）
  motor_speed_pid_(motor_speed_pid),            // [解析] 绑定速度环 PID 的引用（内环）
  feedforward_(feedforward),                    // [解析] 是否允许前馈力矩生效
  limited_(limited),                            // [解析] 是否做单圈展开；注意它是唯一非 const 的成员
  pos_filter_(pos_filter),                      // [解析] 位置滤波系数 [0,1]，1 = 不滤波
  vel_filter_(vel_filter)                       // [解析] 速度滤波系数 [0,1]，1 = 不滤波
{
}

// [解析] disable()：让关节"松手"，不再输出任何力矩。
// [解析] 三件事：① 模式切到 DISABLE；② 把位置目标 set_ 同步成当前位置 pos；
// [解析]   ③ 把速度目标 v_set_ 和力矩目标 t_set_ 清零。
// [解析] 为什么必须 set_ = pos：否则下次一使能（比如切回位置模式），目标还停在很久以前的某个角度，
// [解析]   电机就会"嗖"地一下窜过去，很容易撞坏机构。
// [解析] 副作用：只改这几个内部变量；真正的"不出力"发生在下一次 control() 的 DISABLE 分支里
// [解析]   （那里会调用 motor_.cmd(0)）。所以 disable() 之后必须继续周期性地调 control() 才安全。
template <typename MotorType>
void JointMotorController<MotorType>::disable()
{
  mode_ = ControlMode::DISABLE;
  set_ = pos;
  v_set_ = 0;
  t_set_ = 0;
}

// [解析] init()：把此刻电机的原始角度记为零点，存进 init_angle_。
// [解析]   之后 control() 里的 pos 都会减掉它，于是"上电时的姿态"就对应 pos 的一个固定值
// [解析]   （具体是 -mid_，见 control() 第①步的说明）。
// [解析] 调用时机：上电、电机使能完成、机械处于你认可的初始姿态时，调用一次。
// [解析] 副作用：不读硬件，只是读一下 motor_.angle 这个已经被中断刷好的缓存值。
// [解析] ★ 注意：这里存的是"未乘符号"的原始角度。当 reverse = true（sign_ = -1）时，
// [解析]   control() 里是 pos = -angle - mid_ - init_angle_，init() 之后 pos 并不等于 -mid_。
// [解析]   如果你的关节用了 reverse，请实车确认零点是否就是你期望的那个。
template <typename MotorType>
void JointMotorController<MotorType>::init()
{
  init_angle_ = motor_.angle;
}

// [解析] add()：相对移动 —— 在"当前的位置目标 set_"上再叠加 value(rad)，然后走正常的位置指令流程。
// [解析] ★ 注意加的是 set_（目标值）而不是 pos（实测位置）：
// [解析]   所以连续调用会不断叠加目标，不会因为电机还没走到位而"追不上"。
// [解析]   典型用途：遥控器拨杆微调、按键点动（每按一次加一点点）。
// [解析] 副作用：最终调用 cmd()，所以模式会变成 POSITION、目标会被限幅到 [min_, max_]。
template <typename MotorType>
void JointMotorController<MotorType>::add(float value)
{
  cmd(set_ + value);
}

// [解析] cmd()：位置指令（最常用）。
// [解析]   value —— 目标关节角度，单位 rad。
// [解析] 三步：① 模式切成 POSITION；② 速度目标清零；③ 目标角限幅后存进 set_。
// [解析]   限幅用的是 sp::limit_min_max(value, min_, max_)，也就是把目标夹在软限位区间内，
// [解析]     即使上层误传了一个很大的角度，也不会把机械顶到限位上去。
// [解析] 为什么要把 v_set_ 清零：避免上一次速度模式残留的目标速度被 control() 误用。
// [解析] 注意：这里没有调用 pid_.clear()。如果你从 VELOCITY 切回 POSITION 时发现"冲一下"，
// [解析]   可以在切换处自己加一次 clear()（本任务不允许改代码，仅在此提醒）。
// [解析] 副作用：只改内部状态；真正出力发生在下一次 control()。
template <typename MotorType>
void JointMotorController<MotorType>::cmd(float value)
{
  mode_ = ControlMode::POSITION;
  v_set_ = 0;
  set_ = sp::limit_min_max(value, min_, max_);
}

// [解析] cmd_max()：一键转到最大角度。
// [解析] 内部直接调 cmd(max_)，所以软限幅、模式切换这些流程都会照走一遍，不会绕过任何保护。
template <typename MotorType>
void JointMotorController<MotorType>::cmd_max()
{
  cmd(max_);
}

// [解析] cmd_min()：一键转到最小角度，内部就是 cmd(min_)。
// [解析] 典型用途：机构复位/回零（让关节自己慢慢靠到机械零位那一端）。
template <typename MotorType>
void JointMotorController<MotorType>::cmd_min()
{
  cmd(min_);
}

// [解析] cmd_v()：速度指令。
// [解析]   value —— 目标关节角速度，单位 rad/s，正负号决定转向。
// [解析]   用 sp::limit_max(value, max_v_) 做对称限幅，即把目标夹在 [-max_v_, +max_v_] 内。
// [解析]   为什么要限速：速度太快时位置环/电流环容易跟不上，而且撞到东西的冲击也更大。
// [解析] set_ = this->pos 这一句很重要：把"位置目标"同步到"当前实际位置"，
// [解析]   这样将来从速度模式切回位置模式时，目标是"从现在这里开始"，
// [解析]   不会因为 set_ 还停在上次的位置而突然往回猛拉。
// [解析] 副作用：只改内部状态；真正出力发生在下一次 control()。
template <typename MotorType>
void JointMotorController<MotorType>::cmd_v(float value)
{
  mode_ = ControlMode::VELOCITY;
  v_set_ = sp::limit_max(value, max_v_);
  set_ = this->pos;
}

// [解析] cmd_v_until_t()：以速度 value(rad/s) 转动，直到"顶住了"就自动卸力。
// [解析]   value       —— 目标关节角速度(rad/s)，同样会被限幅到 ±max_v_；
// [解析]   t_threshold —— 力矩阈值(N·m)，判据是 |torque_fdb| > t_threshold。
// [解析] 返回：true = 本周期已经判定"顶住了"并切成了力矩模式；false = 还在正常按速度转。
// [解析]   调用者可以拿返回值决定后续动作（比如"夹住了就开始下一步"）。
// [解析] 触发后做的事：模式切成 TORQUE、t_set_ 和 v_set_ 清零（卸力，不是抱死）、
// [解析]   并把位置目标同步到当前位置，避免将来切回位置模式时猛拉。
// [解析] 典型场景：让机构朝某个方向顶过去，顶到限位/夹住东西后力矩升高就自动停手。
// [解析] 注意：判据用了 fabs()，所以正反转方向都会触发；t_threshold 要用正值。
// [解析] ★ 注意：this-> 前缀不是必须的（基类并不依赖模板参数），但这里全部带上，
// [解析]   是模板代码里的好习惯：万一哪天基类也变成模板，不加 this-> 就会找不到成员。
// [解析] ★ 注意：它读的是 torque_fdb，那是"上一拍 control() 刷新"的值，
// [解析]   所以本次调用用的是上一周期的力矩，不是实时力矩（差一个控制周期，通常可以忽略）。
template <typename MotorType>
bool JointMotorController<MotorType>::cmd_v_until_t(float value, float t_threshold)
{
  if (fabs(this->torque_fdb) > t_threshold) {
    // [解析] 力矩超过阈值：认为已经顶到东西了 —— 切力矩模式，输出 0 力矩（松力，不硬顶）。
    mode_ = ControlMode::TORQUE;
    t_set_ = 0;
    v_set_ = 0;
    set_ = this->pos;
    return true;
  }
  else {
    // [解析] 力矩还没到：继续按速度走。set_ 同步成当前位置，防止之后切模式时跳变。
    mode_ = ControlMode::VELOCITY;
    v_set_ = sp::limit_max(value, max_v_);
    set_ = this->pos;
    return false;
  }
}

// [解析] cmd_v_until_stuck()：cmd_v_until_t() 的加强版，判据是"力矩大 且 转不动"。
// [解析]   v_threshold —— 速度阈值(rad/s)，要求 |vel| < v_threshold 才算"停住了"。
// [解析] 为什么要多这一个条件：刚启动加速那一下力矩也会很大，
// [解析]   只看力矩会把"正常起步"误判成"顶住了"，于是提前卸力、机构永远走不到位。
// [解析] 返回：true = 已判定卡住并切成力矩模式(0 力矩)；false = 还在正常转。
// [解析] 注意：这里的 fabs(this->vel) 用的是关节角速度（已经乘过 sign_），所以正反转都适用。
// [解析] 调参提示：v_threshold 要略大于"电机在堵转前的最低爬行速度"，太小会永远触发不了。
template <typename MotorType>
bool JointMotorController<MotorType>::cmd_v_until_stuck(
  float value, float t_threshold, float v_threshold)
{
  if (fabs(this->torque_fdb) > t_threshold && fabs(this->vel) < v_threshold) {
    // [解析] 两个条件同时满足：力矩已经很大，而且几乎不转了 —— 确实是卡住了，卸力。
    mode_ = ControlMode::TORQUE;
    t_set_ = 0;
    v_set_ = 0;
    set_ = this->pos;
    return true;
  }
  else {
    // [解析] 否则继续按速度转（注意 set_ 也要同步，理由同上）。
    mode_ = ControlMode::VELOCITY;
    v_set_ = sp::limit_max(value, max_v_);
    set_ = this->pos;
    return false;
  }
}

// [解析] cmd_pos_until_t()：位置指令版"到顶自动停"。
// [解析]   value       —— 目标关节角度(rad)，会被软限位到 [min_, max_]；
// [解析]   t_threshold —— 力矩阈值(N·m)，判据同样是 |torque_fdb| > t_threshold。
// [解析] 返回：true = 已触发（切 TORQUE 模式卸力）；false = 还在往目标位置走。
// [解析] 与 cmd_v_until_t() 的唯一区别：没触发时走的是 POSITION 模式（位置闭环），
// [解析]   而不是 VELOCITY 模式；所以它更适合"定位 + 末端力保护"的场合。
// [解析] 典型场景：夹爪慢慢合拢去夹物体，夹住后力矩上升就自动停手，不会把东西夹坏。
// [解析] 注意：触发后再调本函数，力矩依然超阈值，所以会一直保持在"卸力"状态，
// [解析]   想重新夹紧需要先 cmd_t() 或切回位置模式并让力矩降下来。
template <typename MotorType>
bool JointMotorController<MotorType>::cmd_pos_until_t(float value, float t_threshold)
{
  if (fabs(this->torque_fdb) > t_threshold) {
    // [解析] 力矩超阈值：判定顶住了，切成 0 力矩（注意和上面两个 until 函数处理方式完全一致）。
    mode_ = ControlMode::TORQUE;
    t_set_ = 0;
    v_set_ = 0;
    set_ = this->pos;
    return true;
  }
  else {
    // [解析] 没到阈值：正常做位置闭环。v_set_ 清零，表示"不走速度模式"。
    mode_ = ControlMode::POSITION;
    v_set_ = 0;
    set_ = sp::limit_min_max(value, min_, max_);
    return false;
  }
}

// [解析] cmd_t()：力矩指令（力控模式），不做任何闭环，直接把力矩发下去。
// [解析]   value —— 期望的"关节方向"力矩，单位 N·m，正负号决定用力方向。
// [解析] ★ t_set_ = sign_ * value：这里必须乘 sign_，因为 value 是关节方向的量，
// [解析]   而电机只认自己坐标系里的力矩。所以后面 control() 的 TORQUE 分支里【不再】重复乘 sign_，
// [解析]   否则符号会被乘两次，力量方向正好反掉。
// [解析] 同步 set_ = pos、v_set_ = vel 的目的：万一之后切回位置/速度模式，
// [解析]   目标是"从现在这个状态接着走"，不会突然跳变。
// [解析] 注意：这里没有对 value 做限幅！力矩上限需要调用者自己保证（或用电机自身的力矩限幅）。
template <typename MotorType>
void JointMotorController<MotorType>::cmd_t(float value)
{
  mode_ = ControlMode::TORQUE;
  t_set_ = sign_ * value;
  set_ = this->pos;
  v_set_ = this->vel;
}

// [解析] set_feedforward()：设置前馈力矩，单位 N·m。
// [解析]   写成 feedforward_ ? value : 0 的意思是：构造时没开前馈（feedforward = false）就一律记 0，
// [解析]   这样调用者即使写了 set_feedforward() 也不会有效果 —— 属于"配置开关优先"的写法。
// [解析] 前馈的意义：把"已知必须出的力"（典型是重力补偿）直接加到 PID 输出上，
// [解析]   PID 就只需要负责修正误差，不用靠积分慢慢把重力"顶"起来，响应更快、超调更小。
// [解析] ★ 符号/坐标：它加在 control() 里"乘 sign_ 之前"，所以应当用【关节方向】的力矩
// [解析]   （和 PID 输出同一坐标系），不要自己先乘反了 sign_。
// [解析] 副作用：只改 feedforward_t_ 一个成员；不碰电机。
template <typename MotorType>
void JointMotorController<MotorType>::set_feedforward(float value)
{
  this->feedforward_t_ = feedforward_ ? value : 0;
}

// [解析] control()：★ 每个控制周期调用一次，是整个类"干活"的地方。
// [解析] 流程分三段：① 读电机反馈，换算成关节坐标系并滤波（无论什么模式都要做）；
// [解析]   ② 如果是 DISABLE，直接下发 0 力矩并返回；
// [解析]   ③ 否则按模式（TORQUE / POSITION / VELOCITY）算出力矩并下发给电机。
template <typename MotorType>
void JointMotorController<MotorType>::control()
{
  // [解析] ①-1 角度换算：电机原始角度 -> 关节角度，单位 rad。
  // [解析]   sign_ * motor_.angle ：统一方向（reverse 时取反）；
  // [解析]   - mid_              ：减掉机械中位角，让 0 落在机械中位附近；
  // [解析]   - init_angle_       ：减掉 init() 记下的上电零点。
  // [解析] ★ 注意：init_angle_ 存的是"未乘符号"的原始角度，这里却是直接相减。
  // [解析]   当 reverse = true（sign_ = -1）时，init() 之后 pos = -angle0*2 - mid_，并不等于 -mid_。
  // [解析]   也就是说 reverse 情况下这个零点的位置和直连情况不对称，用之前请实车确认。
  this->pos = sign_ * motor_.angle - mid_ - init_angle_;
  // [解析] ①-2 单圈展开：把角度"折"回 [min_m_, max_m_] 区间内。
  // [解析]   电机反馈的 angle 可能是多圈累计值（转 3 圈就是约 18.8 rad），
  // [解析]   而关节的物理行程一般只在一圈之内，所以超出上限就减一圈、低于下限就加一圈。
  // [解析]   2 * sp::SP_PI 就是一整圈（2π ≈ 6.283 rad）。
  // [解析]   注意：只折"一次"，不是 while 循环；如果偏差超过一整圈就折不回来。
  // [解析]   limited_ 为 false 时整段跳过（对应构造参数 limited = false）。
  if (limited_) {
    if (this->pos > max_m_) this->pos -= 2 * sp::SP_PI;
    if (this->pos < min_m_) this->pos += 2 * sp::SP_PI;
  }
  // [解析] ①-3 位置低通滤波：把 pos 送进滤波器，输出存进 pos_filtered。
  // [解析]   为什么滤波值和原始值都保留：PID 用滤波值可以压住编码器噪声，
  // [解析]   而对外公布原始 pos 是为了"判断有没有到位"时不被滤波的滞后骗到。
  // [解析]   pos_filter_ = 1.0f 时等效于不滤波。
  motor_pos_filter_.update(this->pos);
  this->pos_filtered = motor_pos_filter_.out;
  
  // [解析] ①-4 速度换算并滤波：sign_ * motor_.speed 把电机角速度(rad/s)换成关节角速度，
  // [解析]   再过一遍低通，得到 vel_filtered 给速度环用。速度没有"零位"概念，不减 init_angle_。
  this->vel = sign_ * motor_.speed;
  motor_vel_filter_.update(this->vel);
  this->vel_filtered = motor_vel_filter_.out;
  // [解析] ①-5 力矩反馈：sign_ * motor_.torque 换成关节方向的力矩(N·m)。
  // [解析]   它是电机反馈的"实际出力估计值"，不是我们下发的指令值，
  // [解析]   两个 until 函数的阈值判断用的就是它。
  this->torque_fdb = sign_ * motor_.torque;

  // [解析] ② 失能模式：指令力矩记 0，并且主动调用 motor_.cmd(0) 告诉电机"别出力"。
  // [解析]   ★ 一定要真的下发 0，不能只 return：否则有些电机（尤其 DM 系列）会保持上一次的指令继续出力。
  // [解析]   return 之后下面的闭环一条都不会执行，所以失能时 PID 也不更新（下次使能要注意积分残留）。
  if (mode_ == ControlMode::DISABLE) {
    torque_cmd = 0.0f;
    motor_.cmd(0);
    return;
  }

  // [解析] ③-1 力矩模式：不做 PID，直接把 cmd_t() 存好的 t_set_ 当力矩指令发下去。
  // [解析]   注意这里【没有】再乘 sign_，因为 cmd_t() 里存 t_set_ 时已经乘过了，
  // [解析]   再乘一次符号就反了（这是本文件最容易踩的坑之一）。
  if (mode_ == ControlMode::TORQUE) {
    torque_cmd = t_set_;
    motor_.cmd(torque_cmd);
  }
  else {
    // [解析] ③-2 位置模式：串级双环。
    // [解析]   外环：pid_.calc(目标角度 set_, 滤波后角度 pos_filtered)，输出被当作"目标角速度"；
    // [解析]   内环：motor_speed_pid_.calc(外环输出 pid_.out, 滤波后角速度 vel_filtered)，
    // [解析]         输出被当作"目标力矩"。
    // [解析]   两个 PID 每周期各调一次，它们的 dt 必须和本函数的真实调用周期一致
    // [解析]     （dt 是构造 PID 时定死的，改控制频率就要重新构造 PID）。
    if (mode_ == ControlMode::POSITION) {
      pid_.calc(set_, this->pos_filtered);
      motor_speed_pid_.calc(pid_.out, this->vel_filtered);
    }
    // [解析] ③-3 速度模式：跳过位置环，只跑内环速度 PID，目标就是 v_set_。
    // [解析]   注意此时位置环 pid_ 完全没被调用，所以它的积分会保留上一次的值。
    else if (mode_ == ControlMode::VELOCITY) {
      motor_speed_pid_.calc(v_set_, this->vel_filtered);
    }

    // [解析] ③-4 输出力矩 = sign_ * (速度环输出 + 可选前馈)。
    // [解析]   为什么现在才乘 sign_：前面算出的 PID 输出都是"关节方向"的量，
    // [解析]     要在这里统一换成"电机方向"再交给 motor_.cmd()。
    // [解析]   前馈项 feedforward_t_ 加在乘 sign_ 之前，所以它也必须用关节方向的力矩。
    // [解析]   feedforward_ 为 false 时整项被跳过（效果等价于前馈为 0）。
    torque_cmd =
      sign_ * (feedforward_ ? motor_speed_pid_.out + feedforward_t_ : motor_speed_pid_.out);
    // [解析] 把力矩指令交给电机：这一步只是"把指令记进电机对象"，
    // [解析]   真正打包成 CAN 帧并发送，仍然由上层任务（比如 can_task）调用 write()/send() 完成。
    motor_.cmd(torque_cmd);
  }
}

// [解析] ★ 显式实例化（模板类能链接通过的关键，共两行，分别对应两种电机）。
// [解析] 为什么需要它：本文件的实现写在 .cpp 里，编译器编译这个 .cpp 时并不知道
// [解析]   外面会用哪些 MotorType，于是什么机器码都不会生成，链接时就会报 undefined reference。
// [解析] 这两行等价于告诉编译器："请把 MotorType = sp::DM_Motor / sp::RM_Motor 的整个类都生成出来"。
// [解析] 后果与用法：目前只有这两种电机能用这个模板；
// [解析]   以后新增电机（例如 lk_motor）想复用它，必须在这里再补一行
// [解析]   template class JointMotorController<新电机类型>; 否则编译能过、链接会挂。
template class JointMotorController<sp::DM_Motor>;
template class JointMotorController<sp::RM_Motor>;