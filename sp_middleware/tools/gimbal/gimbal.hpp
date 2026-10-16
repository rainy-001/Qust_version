// [解析] ==========================================================================
// [解析] 【文件】sp_middleware/tools/gimbal/gimbal.hpp
// [解析] 【作用】云台姿态解算与目标解算的"总控"：把 IMU 姿态和电机编码器角度揉在一起，
// [解析]         算清楚三件事 —— 底盘相对地面是什么姿态、云台相对底盘转了多少度、
// [解析]         以及为了让枪口指向目标，云台电机"应该"转到哪个角度/速度/加速度。
// [解析] 【被谁用】本快照的 applications/ 目录下还没有创建 Gimbal 对象的代码（尚未接入），
// [解析]           预期由云台控制任务创建，用法类似：
// [解析]             sp::Gimbal gimbal(yaw0, pitch0, false, false, 1e-3f, install_roll, gimbal_roll0, cfg);
// [解析]           然后每个控制周期调用 update_all_single()（或 update_all_dual()）刷反馈值，
// [解析]           再调用 calc_all_target() 算目标值，最后把 *_target_relative_* 送给电机 PID。
// [解析]           也会被 sp_middleware/tools/mahony/ 的姿态解算结果驱动（gimbal_imu 参数）。
// [解析] 【关键点】这是整套中间件里数学最重的文件，新手最容易卡在 3 个地方：
// [解析]   1) 四元数顺序是 {w, x, y, z}，不是数学书常见的 {x, y, z, w}；顺序搞反结果全错。
// [解析]   2) 四元数的含义是"坐标系变换"：v_world = q ⊗ v_body ⊗ q*（q* 为共轭）。
// [解析]      所以同名函数里的 conjugate 参数决定是"正着换"还是"反着换"，非常容易搞反。
// [解析]   3) 单 IMU / 双 IMU 是两套互斥的流程：单 IMU 用 update_all_single()，
// [解析]      双 IMU 用 update_all_dual()，同一次运行里只能选一套，混着调会互相覆盖状态。
// [解析] 【阅读顺序】先看本文件了解"有哪些输入输出"，再看 gimbal.cpp 看公式是怎么推的。
// [解析] ==========================================================================

#ifndef SP__GIMBAL_HPP
#define SP__GIMBAL_HPP

#include "tools/low_pass_filter/low_pass_filter.hpp"
#include "tools/mahony/mahony.hpp"
#include "tools/math_tools/math_tools.hpp"
namespace sp
{

// [解析] 这个结构体只是"一包参数"，本身不干活：把 10 个低通滤波器的 alpha 收在一起，
// [解析] 好处是构造函数只多一个参数，改滤波强度时只改这里，不用去改构造函数签名。
// [解析] 【怎么用】先写 sp::GimbalFilterConfig cfg; 再逐个赋值 cfg.yaw_angle = 0.3f; ...
// [解析] 最后把 cfg 当作 Gimbal 构造函数的最后一个参数传进去。
// 滤波器配置结构体，用于在构造时统一设置所有滤波器的 alpha 参数
// alpha 含义：为1时无滤波效果，为0时无更新效果，通常取 0~1 之间的值
struct GimbalFilterConfig
{
  // [解析] 下面每个字段都是一个低通滤波器的 alpha，计算公式固定为（见 low_pass_filter.cpp）：
  // [解析]   out = alpha * 本次输入 + (1 - alpha) * 上次输出
  // [解析] alpha 越接近 1 越"跟手"（滤波弱、噪声大、延迟小）；
  // [解析] alpha 越接近 0 越"迟钝"（滤波强、噪声小、延迟大，延迟太大 PID 会振荡）。
  // [解析] 调参习惯：先全部保持默认 1.0f（等于不滤波）把系统跑通，再逐个往下压到 0.1~0.5。
  // 电机相对角度低通滤波器 alpha（单IMU/双IMU均使用）
  float yaw_angle = 1.0f;    // yaw  相对角度滤波
  float pitch_angle = 1.0f;  // pitch 相对角度滤波
  float roll_angle = 1.0f;   // roll  相对角度滤波（双IMU使用）

  // [解析] 速度环的坑：反馈速度和目标速度的滤波强度必须一致（alpha 填同一个数）。
  // [解析] 因为滤波器会引入相位延迟，若反馈值比目标值"慢半拍"，PID 会看到虚假误差而抖动。
  // 电机反馈速度低通滤波器 alpha（需与目标速度滤波器保持一致，否则速度环会有延迟）
  float yaw_fdb_speed = 1.0f;    // yaw  反馈速度滤波
  float pitch_fdb_speed = 1.0f;  // pitch 反馈速度滤波
  float roll_fdb_speed = 1.0f;   // roll  反馈速度滤波

  // [解析] 目标速度滤波：作用是把上游（视觉/键鼠）送来的速度指令"抹平"，避免阶跃冲击。
  // 电机目标速度低通滤波器 alpha（建议与反馈速度滤波器设置相同）
  float yaw_target_speed = 1.0f;    // yaw  目标速度滤波
  float pitch_target_speed = 1.0f;  // pitch 目标速度滤波

  // [解析] 加速度只用于"前馈"（直接给电流环一个提前量），不参与反馈，所以噪声再大也不会振荡，
  // [解析] 但会被直接放大到电流上，因此这里通常取整套参数里最小的 alpha（例如 0.1f）。
  // 电机目标加速度低通滤波器 alpha
  float yaw_target_acc = 1.0f;    // yaw  目标加速度滤波
  float pitch_target_acc = 1.0f;  // pitch 目标加速度滤波
};

// [解析] Gimbal 类：一个对象对应"一台车的云台"。全工程一般只创建 1 个（云台只有一套）。
// [解析] 它的定位是"纯算法模块"：自己不碰 CAN、不碰电机、不碰 HAL，
// [解析]   只吃 IMU 姿态 + 编码器角度，吐出电机该走的角度/速度/加速度和一堆中间量。
// [解析] 【标准调用顺序】（每个控制周期执行一次，通常是 1ms）
// [解析]   1) gimbal.update_all_single(imu, yaw_angle, pitch_angle);  // 或 update_all_dual()
// [解析]        —— 刷新反馈：yaw_fdb_in_joint / *_relative_speed / q_chassis2world ...
// [解析]   2) gimbal.calc_all_target(imu, yaw_set, pitch_set, vyaw_set, vpitch_set, ...);
// [解析]        —— 刷新目标：yaw_target_relative_angle / _speed / _acc
// [解析]   3) 把 yaw_target_relative_angle 等送进电机 PID。
// [解析] 【重要提醒】对象内部"有记忆"（上次四元数、滤波器上一拍的值、角度展开器），
// [解析]   因此只能在同一个任务里按固定周期调用，不能多个任务并发调用（非线程安全）。
class Gimbal
{
public:
  // [解析] 构造函数：只把参数和滤波器存起来，不做任何硬件/数学计算（第一次计算在 update_* 里）。
  // [解析] 参数说明（角度全部是 rad，不是度！）：
  // [解析]   yaw0        —— 云台 yaw 轴的"机械零点"编码器读数，用来把绝对编码器值转成相对角度。
  // [解析]   pitch0      —— 同上，pitch 轴的机械零点编码器读数。
  // [解析]   reverse_yaw —— yaw 电机正转方向与云台右手系相反时填 true（内部变成 -1 的符号位）。
  // [解析]   reverse_pitch —— 同上，pitch 轴。
  // [解析]   dt          —— 控制周期，单位 s；默认 1e-3f 即 1000Hz。它被用来对角速度做微分，
  // [解析]                   填错会让算出来的角加速度整体缩放错，默认值必须与真实调用周期一致。
  // [解析]   install_roll —— 底盘 IMU 的安装横滚角，仅双 IMU 模式用来矫正底盘姿态，单 IMU 传 0。
  // [解析]   gimbal_roll0 —— 云台 IMU 的安装横滚角，直接读云台 IMU 的 roll 得到（相当于把云台
  // [解析]                   机械安装的歪斜算进"相对角度 roll"里）。
  // [解析]   filter_config —— 上面那个滤波器参数包，不传就用默认（全部 1.0f = 不滤波）。
  Gimbal(
    float yaw0 = 0.0f, float pitch0 = 0.0f, bool reverse_yaw = false, bool reverse_pitch = false,
    float dt = 1e-3f, float install_roll = 0.0f, float gimbal_roll0 = 0.0f,
    const GimbalFilterConfig & filter_config = GimbalFilterConfig{});

  // [解析] 【单 IMU 模式】主更新函数：只有云台装了 IMU 时用这套。
  // [解析] 它靠"云台姿态 + 编码器角度"反推出底盘姿态（因为底盘没装 IMU，只能这么算）。
  // [解析] 参数：
  // [解析]   gimbal_imu  —— 云台 IMU 的 Mahony 对象（只读，取它的 q / w / roll / pitch 等）。
  // [解析]   yaw_angle   —— yaw 电机编码器当前角度（rad，绝对读数，内部会减掉 yaw0_）。
  // [解析]   pitch_angle —— pitch 电机编码器当前角度（rad，绝对读数，内部会减掉 pitch0_）。
  // [解析] 副作用：会写一大堆公开成员（yaw_fdb_in_joint、*_relative_speed、q_chassis2world、
  // [解析]   chassis_euler_in_world、k_torque 等），并推进内部滤波器状态。
  // [解析] 注意：参数写成 const float & 只是形式，对 float 传引用并不省时间（见 .cpp 的说明）。
  //单imu更新函数，输入云台姿态和电机角度
  void update_all_single(
    const sp::Mahony & gimbal_imu, const float & yaw_angle, const float & pitch_angle);
  // [解析] 【双 IMU 模式】主更新函数：云台和底盘各装一个 IMU 时用这套，底盘姿态直接由底盘 IMU 给出，
  // [解析] 精度比单 IMU 高（特别是底盘颠簸时），代价是要多标定一个 install_roll。
  // [解析] 参数 chassis_imu 是"可写引用"（注意没有 const）：因为函数内部会调用
  // [解析] correct_chassis_imu() 直接修改底盘 IMU 的四元数和角速度，把它对齐到云台 IMU 的 roll 上。
  // [解析] 与 update_all_single 二选一，不要同时调用。
  //双imu更新函数，输入云台姿态、底盘姿态
  void update_all_dual(const sp::Mahony & gimbal_imu, sp::Mahony & chassis_imu);

  // [解析] 双 IMU 专用：把底盘 IMU 的姿态"绕底盘 X 轴转 install_roll 角"，让它的 roll 与云台 IMU 对齐。
  // [解析] 为什么需要：两个 IMU 装在同一台车上但安装角度不完全相同，不矫正的话两者算出的
  // [解析] 相对姿态里会混进一个固定的安装偏差，云台会一直朝一边偏。
  // [解析] 参数 chassis_imu 会被就地修改（四元数 q 和角速度 w 都被改写），调用前最好确认它可改。
  //双imu特有的函数，用于矫正底盘imu的姿态
  void correct_chassis_imu(sp::Mahony & chassis_imu);

  // [解析] 已经被 update_q_chassis2world() 取代的旧版更新函数（内部是用旋转矩阵而不是四元数算的）。
  // [解析] 现在保留只是为了兼容/对照，新代码不要调用它。
  void update(const sp::Mahony & gimbal_imu, const float & yaw_angle, const float & pitch_angle);

  // [解析] 单 IMU 模式的核心：由"云台 IMU 姿态 + 两个电机角度"推出底盘在世界系下的姿态。
  // [解析] 参数 roll0_ 是云台的安装横滚角（update_all_single 里传的是 gimbal_roll0_）。
  // [解析] 输出（副作用）：q_chassis2world[4]（底盘→世界的四元数）、dq[4]（底盘角速度）、
  // [解析]   yaw_rel / pitch_rel / roll_rel（云台相对底盘的欧拉角，也就是"等效电机角度"）。
  // 通过云台姿态和电机角度，算出底盘在世界坐标系下的姿态四元数和角速度
  void update_q_chassis2world(
    const sp::Mahony & gimbal_imu, const float & yaw_angle, const float & pitch_angle,
    const float & roll0_ = 0.0f);

  // [解析] 双 IMU 模式的核心：两个 IMU 的姿态都有了，直接做四元数除法得到"云台相对底盘"的旋转。
  // [解析] 内部实现是 q_gimbal2chassis = q_chassis* ⊗ q_gimbal（先对底盘四元数取共轭）。
  // [解析] 输出（副作用）：q_gimbal2chassis[4]、q_chassis2world[4]（直接抄底盘 IMU 的 q），
  // [解析]   以及 yaw_rel / pitch_rel / roll_rel（由相对四元数转成的欧拉角，等效电机角度，已滤波）。
  //通过底盘的姿态四元数和云台姿态四元数,算出底盘和云台之间的相对四元数,并将相对四元数转换成欧拉角(等效电机角度)
  void update_q_gimbal2chassis(const sp::Mahony & gimbal_imu, const sp::Mahony & chassis_imu);

  // [解析] 目标解算（自瞄 / 键鼠共用）：输入"枪口在世界系下应该指向哪"，输出电机目标量。
  // [解析] 参数（角度 rad，角速度 rad/s，角加速度 rad/s²；以世界系/地面系为参考，不是相对底盘）：
  // [解析]   yaw_set_in_world     —— 期望枪口指向的世界系 yaw 角（自瞄时来自视觉，键鼠时来自鼠标积分）。
  // [解析]   pitch_set_in_world   —— 期望枪口指向的世界系 pitch 角。
  // [解析]   vyaw_set_in_world    —— 期望的世界系 yaw 角速度（键鼠模式用；自瞄一般传 0）。
  // [解析]   vpitch_set_in_world  —— 期望的世界系 pitch 角速度。
  // [解析]   acc_yaw_set_in_world / acc_pitch_set_in_world —— 期望的世界系角加速度。
  // [解析]     只有自瞄才需要（用视觉给出的目标运动做前馈）；键鼠模式这两个留默认 0 就行。
  // [解析] 返回值：无。结果写在 yaw_target_relative_angle / yaw_target_relative_speed /
  // [解析]   yaw_target_relative_acc 等公开成员里（注意是"相对底盘"的量，可以直接喂给电机）。
  // [解析] 副作用：会推进 4 个目标滤波器，并刷新一堆中间量（A/B/C/D/F 等）。
  // [解析] 【重要】前半段算完目标角度后有一个除零保护：当 cos(pitch_rel) 接近 0（云台快竖直）时
  // [解析]   直接 return，此时只有目标角度是新的，目标速度和加速度还停留在上一帧的值。
  //自瞄:输入云台姿态和视觉发来的目标值;输出云台的目标角度、目标速度和目标加速度
  //键鼠:输入云台姿态和键鼠发来的目标角度和速度,无需输入目标加速度; 输出云台的目标角度和目标速度(只调用此两者就行)
  void calc_all_target(
    const sp::Mahony & gimbal_imu, float yaw_set_in_world = 0.0f, float pitch_set_in_world = 0.0f,
    float vyaw_set_in_world = 0.0f, float vpitch_set_in_world = 0.0f,
    float acc_yaw_set_in_world = 0.0f, float acc_pitch_set_in_world = 0.0f);

  // [解析] 旧版目标解算（基于旋转矩阵 R_base2world_，需要先调用 update() 把矩阵算出来）。
  // [解析] 它不支持速度/加速度前馈，只输出 yaw_set_in_joint / pitch_set_in_joint 两个角度。
  // [解析] 新代码请用 calc_all_target()，这个只在老工程/对照时才有意义。
  void calc(float yaw_set_in_world, float pitch_set_in_world);

  // [解析] 下面这一组 static 函数是"数学工具箱"，不属于某个具体对象，可以当成普通函数直接用：
  // [解析]   sp::Gimbal::quaternion_multiply(...)、sp::Gimbal::cross_product(...) 等。
  // [解析] 之所以挂在类里，是为了表明它们和云台解算属于同一套坐标约定。
  // [解析] 【统一约定，务必先记住再看公式】
  // [解析]   - 四元数一律写成 q[4] = {w, x, y, z}，w 是实部（第一个元素！）。
  // [解析]   - q 描述的是"坐标系变换"：把同一个物理向量从 body 系换到 world 系，即 v_world = q ⊗ v_body ⊗ q*。
  // [解析]     反过来想把 world 系的值换回 body 系，就要用共轭 q*（函数里那个 conjugate 参数就是干这个的）。
  // [解析]   - 欧拉角一律是 ZYX 内旋顺序：先绕世界 Z 转 yaw，再绕"转完之后的"新 Y 转 pitch，
  // [解析]     最后绕新 X 转 roll。顺序很关键，ZYX 和 XYZ 算出来的数完全不同。
  // [解析]   - 所有角度单位是 rad，角速度 rad/s，角加速度 rad/s²。

  // 欧拉角(ZYX顺序: yaw->pitch->roll)转四元数
  // 输入: yaw, pitch, roll 的含义为: 机体系从与世界系重合开始, 先绕世界Z轴旋转yaw, 再绕新Y轴旋转pitch, 最后绕新X轴旋转roll
  // 输出: q[4] = {w, x, y, z}, 表示将机体系下向量变换到世界系下的四元数, 即 v_world = q ⊗ v_body ⊗ q*
  // [解析] 这是"欧拉角 → 四元数"的唯一入口。参数 yaw/pitch/roll 单位 rad，输出 q 已做归一化。
  // [解析] 用半角公式实现（下面 .cpp 里有推导注释）：q = qyaw ⊗ qpitch ⊗ qroll，每个单轴四元数用半角。
  // [解析] 典型用途：构造函数里把 install_roll 变成四元数存进 install_roll_q_。
  static void euler_zyx_to_quaternion(float yaw, float pitch, float roll, float q[4]);

  // [解析] "四元数 → 欧拉角"。输出顺序是 {roll, pitch, yaw}，单位 rad。
  // [解析] 坑点：这里的数组下标和四元数不一样！q 是 {w,x,y,z}，euler 是 {roll,pitch,yaw}，
  // [解析]   很容易顺手把 euler[0] 当成 yaw 用，一用就错。
  // [解析] 内部用 2*(w*y - x*z) 求 sin(pitch)，并对它做 [-1,1] 限幅防止 asinf 收到非法值变 NaN。
  // [解析] 万向锁处理：pitch 接近 ±90° 时 roll 和 yaw 无法分开，函数会把 roll 强行置 0，
  // [解析]   把两个角度的和全部塞进 yaw。云台正常工作时 pitch 到不了 90°，一般不会触发。
  // 四元数转欧拉角：q[0]=w, q[1]=x, q[2]=y, q[3]=z
  // 返回数组顺序：[0]=roll, [1]=pitch, [2]=yaw，单位：rad
  // 四元数定义为：q = {w, x, y, z},能将body系下的向量换回到world系 例:v_world = q ⊗ v_body ⊗ q*
  // 欧拉角定义:body系相对于world系的欧拉角旋转，按照ZYX顺序，即先绕z轴旋转yaw，再绕y轴旋转pitch，最后绕x轴旋转roll
  static void quaternion_to_euler(const float q[4], float euler[3]);

  // [解析] "旋转矩阵 → 欧拉角"，输出顺序同样为 {roll, pitch, yaw}，单位 rad。
  // [解析] 只被旧版 update() 之后的调试流程使用；新版都用四元数转欧拉角了。
  // [解析] 取 sin(pitch) = -R[2][0]，这就是 ZYX 顺序下 pitch 的标准解法（对应 R 矩阵第三行第一列）。
  // 旋转矩阵转欧拉角：R[3][3]表示物体相对地面的旋转矩阵
  // 返回数组顺序：[0]=roll, [1]=pitch, [2]=yaw，单位：rad
  static void rotation_matrix_to_euler(const float R[3][3], float euler[3]);

  // [解析] 四元数乘法（Hamilton 积），公式见 .cpp。q_result 可以和输入是同一块内存吗？
  // [解析]   函数是逐项写出结果的，所以 q_result 允许与 q1 或 q2 是同一个数组（原地覆盖）也没问题。
  // [解析] 【conjugate_q1/q2 的物理意义】四元数取共轭 = 反向旋转 = "把坐标系反过来换"。
  // [解析]   所以 q1 ⊗ q2 表示"先做 q2 的变换，再做 q1 的变换"；
  // [解析]   而对某一个取共轭，就等于把那一级的变换方向倒过来。
  // [解析]   实践中最常见的调用是 quaternion_multiply(A, B, out, true, false)，
  // [解析]   它算的是 A* ⊗ B，也就是"从 B 坐标系看 A 坐标系的相对姿态"。
  // 四元数乘法：q_result = q1 ⊗ q2
  // 输入/输出格式：q[4] = {w, x, y, z}
  // 表示坐标系变换的复合：先 q2 再 q1
  // conjugate_q1: 是否对 q1 取共轭
  // conjugate_q2: 是否对 q2 取共轭
  static void quaternion_multiply(
    const float q1[4], const float q2[4], float q_result[4], bool conjugate_q1 = false,
    bool conjugate_q2 = false);

  // [解析] 用四元数把一个三维向量换到另一个坐标系，等价于 v_out = q ⊗ v_in ⊗ q*。
  // [解析] conjugate_q 是最容易搞反的参数：q 表示"由 A 到 B"时，
  // [解析]   不加共轭是 A→B，加了共轭（true）就变成 B→A。不确定时就用一组已知向量试一下。
  // [解析] 实现上不真的做两次四元数乘法，而是用等价的高效公式（见 .cpp），结果一样但快很多。
  // 四元数旋转向量：v_out = q ⊗ v_in ⊗ q*
  // 输入：q[4] = {w, x, y, z} 表示从坐标系A到坐标系B的旋转
  //      v_in[3] = {x, y, z} 在坐标系A中的向量
  // 输出：v_out[3] = {x, y, z} 在坐标系B中的向量
  // conjugate_q: 是否对 q 取共轭（即使用 q* 而不是 q）
  static void quaternion_frame_transform(
    const float q[4], const float v_in[3], float v_out[3], bool conjugate_q = false);

  // [解析] 普通的三维向量叉乘，就是数学课本那个行列式展开，没有坐标系相关的东西。
  // [解析] 结果 = v1 × v2，方向服从右手定则；若两向量平行则结果为 {0,0,0}。
  // [解析] 云台里用它算"角速度耦合项"（向心/科氏项），是 calc_all_target() 里的 C 项。
  // 向量叉乘：result = v1 × v2
  // 输入：v1[3], v2[3] 两个三维向量
  // 输出：result[3] 叉乘结果
  static void cross_product(const float v1[3], const float v2[3], float result[3]);

  // [解析] 已知"载体系角速度"求"欧拉角速率"，也就是 T⁻¹ 矩阵（欧拉运动学方程）。
  // [解析] 【为什么需要它】云台上的陀螺仪量到的是 w = {wx, wy, wz}（载体系，三个轴互相耦合），
  // [解析]   但 PID 控制的是 yaw/pitch/roll 各自的角速度，两者不是一回事，必须用这个矩阵换算。
  // [解析] 【唯一的大坑】公式里有 1/cos(pitch)。pitch 接近 ±90°（云台竖直朝天/朝地）时分母趋近 0，
  // [解析]   输出会瞬间爆成巨大的数，这就是"万向锁"。所以云台的机械限位必须避开 ±90°。
  // [解析] 输入：omiga_in_body[3] 载体系角速度（rad/s，直接用 IMU 的 w）；
  // [解析]       roll/pitch/yaw 当前欧拉角（rad）。
  // [解析] 输出：vroll/vpitch/vyaw 是"引用出参"（不是返回值！），单位 rad/s。
  // 载体系角速度转欧拉角速率  对应T逆
  // 输入：omiga_in_body[3] = {wx, wy, wz} 载体系下的角速度,即imu测量到的角速度原始值
  //      euler[3] = {roll, pitch, yaw} 当前欧拉角 相对于A系
  // 输出：euler_rates[3] = {roll_rate, pitch_rate, yaw_rate} 欧拉角微分 相对于A系 相对于什么系就能得到相对于同一个系的欧拉角的微分
  static void transform_omiga_in_body_2_euler_rates(
    const float omiga_in_body[3], const float roll, const float pitch, const float yaw,
    float & vroll, float & vpitch, float & vyaw);

  // [解析] 上一条的逆运算：由欧拉角速率求载体系角速度，也就是 T 矩阵（雅可比矩阵）。
  // [解析] 没有除法、没有奇点，所以非常安全，随便什么 pitch 都能算。
  // [解析] 参数里的 yaw 其实用不到（函数内部 (void)yaw 明确忽略），保留只是为了签名对称好看。
  // [解析] 输入：vroll/vpitch/vyaw 三个欧拉角速率（rad/s），roll/pitch/yaw 当前欧拉角（rad）。
  // [解析] 输出：omiga_in_body[3] 载体系角速度（rad/s），是数组出参。
  // 欧拉角速率转载体系角速度 对应T (or Jacobian)
  // 输入：vroll, vpitch, vyaw 欧拉角微分
  //      roll, pitch, yaw 当前欧拉角
  // 输出：omiga_in_body[3] = {wx, wy, wz} 载体系下的角速度
  static void transform_euler_rates_2_omiga_in_body(
    const float vroll, const float vpitch, const float vyaw, const float roll, const float pitch,
    const float yaw, float omiga_in_body[3]);

  // [解析] 这是把"速度相关的非线性项"整理出来的一项（数学上叫 T 矩阵对时间的导数 × 欧拉角速率）。
  // [解析] 【为什么要有它】云台角加速度 = 欧拉角加速度 + 这些耦合项。直接对目标角速度做差分得到的
  // [解析]   并不是纯角加速度，还混着"两个轴互相转动产生的耦合"，这项就是把耦合减掉。
  // [解析] 表达式里每一项都带着两个速率相乘（如 vpitch*vyaw），量纲因此是 rad/s²，属于"二阶小量"，
  // [解析]   系数都是 1 或 -1，没有需要额外记的常数。yaw 同样不参与（(void)yaw）。
  // [解析] 输入：roll/pitch/yaw（rad）+ 三个欧拉角速率（rad/s）；输出：result[3]（rad/s²）。
  // 变换矩阵的微分乘以欧拉角速率  这个函数存在的原因是微分的性质:要对所有变量对t求导数
  // 输入：roll, pitch, yaw 当前欧拉角
  //      vroll, vpitch, vyaw 欧拉角速率
  // 输出：result[3] 变换结果
  static void Transform_matrix_rates_multipy_Euler_rates(
    const float roll, const float pitch, const float yaw, const float vroll, const float vpitch,
    const float vyaw, float result[3]);

  // [解析] ===== 以下都是"对外输出"的公开成员：外部只读，不要手动赋值（写坏了整套解算就乱了）=====
  // [解析] 命名规律：fdb = feedback（反馈值）、set = setpoint（设定值）、
  // [解析]   in_joint = 关节坐标（相对码盘零点）、in_world = 世界系（相对地面）、rel = relative（相对底盘）。
  // [解析] 【两套角度别混淆】in_joint 系列是"电机轴自己的角度"（做过限位折叠，带 ±π 跳变）；
  // [解析]   *_rel 系列是"展开后的连续角度"（AngleUnwrapper 处理过，可以超过 ±π，用于速度/微分计算）。

  float yaw_fdb_in_joint;    //只读！ 云台yaw轴相对于码盘的反馈角度，单位：rad
  // [解析] ↑ 由 update() 计算：sign_yaw_ * limit_angle(编码器读数 - yaw0_)，再经弃用滤波器输出。
  // [解析]   注意它已经被 limit_angle 折到 (-π, π]，跨圈时有跳变，不能拿它做微分。
  float pitch_fdb_in_joint;  //只读！ 云台pitch轴相对于码盘的反馈角度，单位：rad
  // [解析] ↑ 同上，pitch 轴。
  float yaw_set_in_joint;    //只读！ 云台yaw轴相对于码盘的设定角度，单位：rad
  // [解析] ↑ 由旧版 calc() 算出的目标角度（关节坐标）。新版用下面的 *_target_relative_angle。
  float pitch_set_in_joint;  //只读！ 云台pitch轴相对于码盘的设定角度，单位：rad
  // [解析] ↑ 同上，pitch 轴。

  // [解析] 底盘系 → 地面系的 3×3 旋转矩阵（"已经考虑了上电瞬间世界系由云台 x 轴投影定义"，
  // [解析] 意思是世界系的原点朝向不是地理北，而是上电那一刻云台指向在水平面的投影，上车前要朝向标准方向）。
  // [解析] 只有旧版 update() 会填它；新版走四元数路线，这个矩阵可能一直是单位阵，用之前先确认。
  float R_base2world_[3]
                     [3];  //底盘系相对于地面系的旋转矩阵,已经考虑了上电瞬间世界系由云台x轴投影定义

  // [解析] dq[4]："底盘相对地面"的角速度，用四元数增量形式表示，在 update_q_chassis2world() 里算出来。
  // [解析] 【重要】它不是普通的四元数，第一个分量 dq[0] 也不是 0！
  // [解析]   原始算式是 dq = q_now ⊗ q_last*（一个极小量），随后整组乘了 2/dt（泰勒展开近似）。
  // [解析]   乘完之后：dq[1..3] 才是地面系下的角速度 wx/wy/wz（rad/s），dq[0] 只是残留的小量。
  // [解析]   所以 update_all_single() 里是 w_chassis_in_worldframe[i] = dq[i+1]，而不是从 dq[0] 开始取。
  float dq[4];
  //只读！底盘系相对于地面系的角速度 (在地面系下) 第一个分量是1,后三个分量才是角速度wx,wy,wz
  // [解析] ↑ 这句原注释的"第一个分量是1"只是描述四元数增量的常数项（理论上接近 1），
  // [解析]   实际代码乘过 2/dt 之后它是个接近 0 的残差，真正能用的只有后 3 个分量 —— 别被这句话误导。
  float q_gimbal2chassis
    [4];  //云台系相对于底盘系的四元数表示 能将云台系的向量转换到底盘系,用于计算电机相对角度
  // [解析] ↑ 双 IMU 模式下由 q_chassis* ⊗ q_gimbal 算出；单 IMU 模式下这个成员不会被更新（保持初值）。
  // [解析]   它转成欧拉角就是"云台相对底盘"的角度，也就是电机真正要转到的角度。
  float q_chassis2world[4];  //底盘系相对于地面系的四元数表示 能将底盘系的向量转换到地面系
  // [解析] ↑ 单 IMU 模式：由云台姿态和电机角度反推；双 IMU 模式：直接抄底盘 IMU 的四元数。
  // [解析]   顺序为 {w, x, y, z}。世界系的朝向就是上电瞬间的底盘朝向。

  // [解析] yaw_rel / pitch_rel / roll_rel：云台相对底盘的欧拉角（rad），这是"电机等效角度"的核心量，
  // [解析]   已经过低通滤波。step 计算时通常是先做坐标变换、再对这三个量求微分得到下面那三个速度。
  float yaw_rel;    //只读！ 云台yaw轴相对于设定零点的相对角度，单位：rad
  float pitch_rel;  //只读！ 云台pitch轴相对于设定零点的相对角度，单位：rad
  float roll_rel;   //只读！ 云台roll轴相对于设定零点的相对角度，往往是零,单位：rad

  // [解析] 三个轴的"云台相对底盘"角速度（rad/s）。单 IMU 模式下不是直接查表得来的：
  // [解析]   先用 IMU 角速度减去"底盘角速度在云台系下的分量"得到 w_relative，
  // [解析]   再用 T⁻¹ 矩阵换算成欧拉角速率，最后过一层低通滤波。可直接当电机的速度环反馈。
  float roll_relative_speed;   //只读！ 云台roll轴相对底盘的角速度，单位：rad/s
  float pitch_relative_speed;  //只读！ 云台pitch轴相对底盘的角速度，单位：rad/s
  float yaw_relative_speed;    //只读！ 云台yaw轴相对底盘的角速度，单位：rad/s
  // [解析] 重力补偿系数：把地面系的重力方向 {0,0,1} 用云台四元数转到云台系，得到的三维单位向量。
  // [解析]   用法是"力矩 = k_torque[2] * m * g"，补偿掉 pitch 轴因枪管重量而低头的力矩。
  // [解析]   它是"方向"而不是系数：k_torque[2] 随云台俯仰而变，正是重力在云台 pitch 轴上的投影。
  float k_torque[3] = {
    0.0f, 0.0f, 0.0f};  //pitch轴的力矩补偿系数 应将k_torque[2]乘mg得到pitch轴的力矩补偿值

  // [解析] ===== 以下 6 个是 calc_all_target() 的输出：云台电机"相对底盘"的目标量 =====
  // [解析] 【单位】角度 rad、速度 rad/s、加速度 rad/s²。
  // [解析] 【为什么是"相对底盘"】因为电机装在底盘上，它只能感知"自己和底盘之间的角度"，
  // [解析]   世界系的目标必须先减去底盘自身的运动，才能变成电机能执行的指令。
  float yaw_target_relative_angle = 0.0f;    //只读！ 云台yaw轴目标相对角度（相对底盘），单位：rad
  float pitch_target_relative_angle = 0.0f;  //只读！ 云台pitch轴目标相对角度（相对底盘），单位：rad
  float yaw_target_relative_speed = 0.0f;  //只读！ 云台yaw轴目标相对角速度（相对底盘），单位：rad/s
  float pitch_target_relative_speed =
    0.0f;  //只读！ 云台pitch轴目标相对角速度（相对底盘），单位：rad/s
  float yaw_target_relative_acc =
    0.0f;  //只读！ 云台yaw轴目标相对角加速度（相对底盘），单位：rad/s²
  float pitch_target_relative_acc =
    0.0f;  //只读！ 云台pitch轴目标相对角加速度（相对底盘），单位：rad/s²

  // [解析] 下面这几个是"底盘在世界系下的姿态"，主要给视觉/小陀螺补偿/发射机构判断车身朝向用。
  // [解析] 顺序是 {roll, pitch, yaw}，和四元数 {w,x,y,z} 不一样；单位 rad。
  float chassis_euler_in_world
    [3];                    //只读！底盘系相对于地面系的欧拉角表示，单位：rad，顺序为roll pitch yaw
  // [解析] ↑ 拆出来单列的 3 个标量，和上面那个数组是同一份数据（分别对应 [2]、[1]、[0]）。
  float base_yaw_in_world;  //只读！ 底盘系相对于地面系的yaw角，单位：rad
  float base_pitch_in_world;  //只读！ 底盘系相对于地面系的pitch角，单位：rad
  float base_roll_in_world;   //只读！ 底盘系相对于地面系的roll角，单位：rad

  // [解析] ================================ 以下是私有成员，外部访问不到 ================================
private:
  // [解析] 两个码盘零点：上电/标定那一刻记录的电机绝对角度（rad）。
  // [解析] 之后所有"相对角度"都是 编码器当前值 - 零点。改它等于重新定义云台的朝向基准。
  float yaw0_;          //云台yaw轴码盘零点位置，单位：rad
  float pitch0_;        //云台pitch轴码盘零点位置，单位：rad
  float gimbal_roll0_;  //云台imu安装roll,直接读云台imu.roll，单位：rad
  // [解析] ↑ 云台 IMU 安装的横滚角。云台在机械上不可能装得完全水平，这个量把安装误差算进 roll_rel，
  // [解析]   让"云台水平"对应 roll_rel = 0，否则解算出来的相对角度会一直带个固定偏置。
  // [解析] 两个符号位：reverse_xxx 参数为 true 时变成 -1.0f，用来翻转电机正方向和云台右手系不一致的问题。
  // [解析]   一旦发现"yaw 轴越转角度越小"或者云台往反方向跑，先检查这两个符号。
  float sign_yaw_;
  float sign_pitch_;
  float dt_;                                            //控制周期
  // [解析] ↑ 单位 s，来自构造参数。只在 update_q_chassis2world() 里把四元数增量换算成角速度时用到
  // [解析]   （除以 dt_ 再乘 2），填错会让底盘角速度整体缩放错。
  float euler_motor[3];                                 //电机相对于底盘的欧拉角表示
  // [解析] ↑ 双 IMU 模式下的中转变量：相对四元数转成的欧拉角 {roll,pitch,yaw}，随后赋给 roll_rel 等。
  float q_last_chassis2world[4];                        //底盘系相对于地面系的上次四元数表示
  // [解析] ↑ "上一拍"的四元数，用来做四元数微分求底盘角速度（dq = q_now ⊗ q_last*）。
  // [解析]   第一拍调用时它全是 0，所以最初几次算出的角速度不可信，属正常现象。
  float install_roll_ = 0.0f;                           //底盘安装roll角,用于双imu矫正底盘姿态
  float install_roll_q_[4] = {1.0f, 0.0f, 0.0f, 0.0f};  //底盘安装roll角对应的四元数表示
  // [解析] ↑ 初始值是 {1,0,0,0}，即"单位四元数 = 不旋转"。构造函数里会被 euler_zyx_to_quaternion
  // [解析]   用 (0, 0, install_roll_) 覆盖成"只绕 X 轴转 install_roll_"的四元数。

  // [解析] 底盘角速度的三种表示：世界系 / 云台系 / 底盘系，外加"云台相对底盘"的角速度。
  // [解析] 【为什么同一个量要存三份】这是云台解算的常见套路：先在某个系里算好，
  // [解析]   再统一转到需要的系，避免每次重复转换（也方便调试时逐项打印比对）。
  // [解析] 【单位】都是 rad/s。
  float w_chassis_in_worldframe[3] = {0.0f, 0.0f, 0.0f};
  // [解析] ↑ 底盘角速度在地面系下的表示。单 IMU 模式下取自 dq 的后三位，双 IMU 模式下由底盘 IMU 转来。
  float w_last_chassis_in_worldframe[3] = {0.0f, 0.0f, 0.0f};
  // [解析] ↑ 上一拍的 w_chassis_in_worldframe，专门用来做差分求角加速度。

  float w_chassis_in_gimbalframe[3] = {0.0f, 0.0f, 0.0f};
  // [解析] ↑ 底盘角速度在云台系下的表示。它是"云台相对角速度"的减数：w_relative = 云台IMU角速度 - 它。
  // [解析]   这就是"小陀螺"能一边自转一边稳住枪口的原理：把底盘转动的部分从 IMU 读数里减掉。
  float w_chassis_in_chassisframe[3] = {0.0f, 0.0f, 0.0f};
  // [解析] ↑ 底盘角速度在底盘系下的表示，注释说"可用作小陀螺补偿"，即告诉底盘自己转多快。
  float w_relative[3] = {0.0f, 0.0f, 0.0f};

  // [解析] ↑ 云台相对底盘的角速度（在云台系下），rad/s。它的三个分量是一堆耦合的载体系角速度，
  // [解析]   必须经 transform_omiga_in_body_2_euler_rates() 才能变成能控制的三轴角速度。
  // [解析] 底盘角加速度的两个表示：地面系下和云台系下（单位 rad/s²）。
  // [解析] 获取方式：对上面的角速度做差分（sp::diff_vec3，除以 dt_），再转到云台系。
  // [解析] 用途：前馈补偿。底盘在加减速时云台会受反作用力矩，提前把它算进目标加速度里抵消掉。
  // [解析] 【注意】差分对噪声极度敏感，底盘 IMU 的噪声会被 dt_=1ms 放大 1000 倍，
  // [解析]   所以实际使用时这两个量通常还要额外滤波，否则画面会抖。
  float acc_chassis_in_worldframe[3] = {0.0f, 0.0f, 0.0f};
  float acc_chassis_in_gimbalframe[3] = {0.0f, 0.0f, 0.0f};

  // [解析] 枪口目标方向向量：由 calc_all_target() 的 yaw/pitch 目标角构造出来的单位向量，
  // [解析]   地面系下是 {cos(pitch)cos(yaw), cos(pitch)sin(yaw), -sin(pitch)}（见 .cpp）。
  // [解析]   注意 z 分量是负的 sin(pitch)：因为本工程的 pitch 定义是"抬头为正"，
  // [解析]   而地面系 z 轴朝天，所以抬头对应 z 变小。
  float gun_target_vector[3] = {0.0f, 0.0f, 0.0f};
  float gun_target_vector_in_chassisframe[3] = {0.0f, 0.0f, 0.0f};
  // [解析] ↑ 同一个向量换到"底盘系"后的结果，紧接着用 atan2 / asin 反解出电机该转到的角度。

  // [解析] 两个角度展开器（AngleUnwrapper）：作用是把 (-π, π] 的单圈角度变成连续角度，消除跨圈跳变。
  // [解析] 【为什么必须有】若直接把带跳变的角度拿去微分，跳变那一拍会算出巨大的角速度，
  // [解析]   云台会"抽"一下，甚至会触发电流保护。
  // [解析] 【绝对不能共用实例】每个被展开的角度都要有自己的对象，否则两个角度的增量会串味。
  // [解析]   这里 yaw 反馈用一个、yaw 目标用另一个，正好各管一路。
  sp::AngleUnwrapper yaw_unwrapper_;  // yaw角度展开器,将输入的yaw电机角度展开
  sp::AngleUnwrapper
    yaw_target_relative_angle_unwrapper;  // yaw电机目标角度展开器,将输出的yaw电机目标角度展开

  // [解析] 下面一大片都是低通滤波器对象，构造时用 GimbalFilterConfig 里的 alpha 初始化。
  // [解析] 【用法】每个控制周期调用 f.update(x); 然后读 f.out，就是滤波后的值。
  // [解析]   update() 内部会记住上一次结果（有状态），所以同一个滤波器不要跨轴乱用。
  //电机反馈角度滤波器
  sp::LowPassFilter yaw_relative_angle_filter;
  sp::LowPassFilter pitch_relative_angle_filter;
  sp::LowPassFilter roll_relative_angle_filter;
  //从最开始要将电机目标速度滤波器与电机反馈速度滤波器设置成相同
  // 电机目标速度滤波器
  sp::LowPassFilter pitch_target_relative_speed_filter;
  sp::LowPassFilter yaw_target_relative_speed_filter;
  // 电机反馈速度滤波器
  sp::LowPassFilter roll_relative_speed_filter;
  sp::LowPassFilter pitch_relative_speed_filter;
  sp::LowPassFilter yaw_relative_speed_filter;
  // 电机目标加速度滤波器
  sp::LowPassFilter pitch_motor_target_acc_filter;
  sp::LowPassFilter yaw_motor_target_acc_filter;
  // [解析] 这两个是早期版本（"杨少代码"）遗留的滤波器，alpha 硬编码为 1.0f，
  // [解析]   只在旧的 update() 里用。新流程 update_q_chassis2world() 用的是上面那三个带配置的滤波器。
  //这两个滤波器是杨少代码用的滤波器,已经弃用
  sp::LowPassFilter yaw_relative_angle_filter0;
  sp::LowPassFilter pitch_relative_angle_filter0;
  // [解析] 下面 6 个数组是 calc_all_target() 内部推导角加速度时的"草稿纸"：把长公式拆成 A~F 几项，
  // [解析]   一项一项算完再拼起来，最后乘一个矩阵求逆得到电机轴上的目标角加速度。
  // [解析] 【为什么写成成员变量而不是局部变量】局部变量每帧都要重新分配栈空间，
  // [解析]   写成成员可以复用同一块内存（老代码在 STM32F4 上的常见优化习惯）。
  // [解析]   副作用是：这些中间量能被调试器实时看到，排查公式错误时非常方便。
  // [解析] 【单位】A、B、D、F、Final 都是 rad/s²；C 是叉乘结果，量纲是 rad²/s²。
  float A_Tdot_Edot[3] = {0.0f, 0.0f, 0.0f};
  // [解析] ↑ A = Ṫ·Ė，即"变换矩阵的时间导数 × 欧拉角速率"，描述云台自身转动带来的耦合加速度。
  float B_acc_chassis_in_gimbalframe[3] = {0.0f, 0.0f, 0.0f};
  // [解析] ↑ B = 底盘角加速度在云台系下的表示（从 acc_chassis_in_gimbalframe 拷过来）。
  float C_cross_product[3] = {0.0f, 0.0f, 0.0f};
  // [解析] ↑ C = 底盘角速度 × 云台相对角速度（叉乘），即两个转动互相耦合产生的加速度项。
  float D_Jdot_E_Motor_dot[3] = {0.0f, 0.0f, 0.0f};
  // [解析] ↑ D = J̇·θ̇，由云台相对角度和相对角速度算出的耦合项。
  float F[3] = {0.0f, 0.0f, 0.0f};
  // [解析] ↑ F = A - B - C - D：把上面各项合并，得到"载体系下还需要额外补偿的角加速度"。
  float Final[3] = {0.0f, 0.0f, 0.0f};
  // [解析] ↑ Final = T⁻¹ · F，也就是换算到欧拉角空间的最终目标角加速度 {roll,pitch,yaw}。
  // [解析]   代码里只取 Final[1]、Final[2]（pitch、yaw），再投影到电机轴上变成
  // [解析]   pitch_target_relative_acc / yaw_target_relative_acc —— roll 轴由机械结构固定，不需要控制。
};

}  // namespace sp

#endif