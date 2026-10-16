// [解析] ==========================================================================
// [解析] 【文件】sp_middleware/tools/gimbal/gimbal.cpp
// [解析] 【作用】gimbal.hpp 里所有函数的具体实现，本质是"一堆坐标系变换 + 四元数代数"。
// [解析] 【看代码的重点】本文件很长，建议按下面的顺序读，不要从头一行行啃：
// [解析]   1) 先看 4 个纯数学小工具（第 533 行起）：quaternion_to_euler / quaternion_multiply /
// [解析]      quaternion_frame_transform / cross_product —— 看懂它们，后面全是在调用它们。
// [解析]   2) 再看 update_q_chassis2world()：单 IMU 模式的核心，六个 Step 注释很清楚。
// [解析]   3) 然后 update_all_single()：把上一步的结果拿去算速度、加速度、重力补偿。
// [解析]   4) 最后 calc_all_target()：目标解算，公式最长、噪声最大，看不懂可以先跳过。
// [解析] 【统一约定，读之前先记住】
// [解析]   - 四元数一律 {w, x, y, z}；欧拉角一律 {roll, pitch, yaw}；两者下标不一样！
// [解析]   - 角度单位 rad、角速度 rad/s、角加速度 rad/s²、时间 s。
// [解析]   - "in_world" = 地面系、"in_gimbalframe" = 云台系、"in_chassisframe" = 底盘系、
// [解析]     "relative" = 云台相对底盘。看懂后缀就知道这个量在哪个坐标系里。
// [解析]   - conjugate_q = true 表示"反向旋转"，即把 A→B 变成 B→A，最容易写反的地方。
// [解析] ==========================================================================

#include "gimbal.hpp"

#include <cmath>

#include "tools/math_tools/math_tools.hpp"

namespace sp
{

// [解析] 构造函数用"初始化列表"（冒号后面那一串）给成员变量赋初值，比在函数体里赋值更规范，
// [解析] 顺序必须和 hpp 里成员的声明顺序一致，否则编译器会警告（实际按声明顺序执行）。
// [解析] 【参数怎么变成成员】yaw0/pitch0/dt/install_roll/gimbal_roll0 直接存；
// [解析]   reverse_yaw / reverse_pitch 是 bool，这里转成 +1.0f / -1.0f 存进 sign_yaw_ / sign_pitch_，
// [解析]   后面只需要做乘法就能翻转方向，比每次写 if 判断省事。
// [解析] 【滤波器构造】括号里传的就是 alpha，直接来自 GimbalFilterConfig 的各个字段。
// [解析]   注意最后两个 filter0 传的是硬编码 1.0f（等于不滤波），因为它们是弃用的旧滤波器。
Gimbal::Gimbal(
  float yaw0, float pitch0, bool reverse_yaw, bool reverse_pitch, float dt, float install_roll,
  float gimbal_roll0, const GimbalFilterConfig & fc)
: yaw0_(yaw0),
  pitch0_(pitch0),
  gimbal_roll0_(gimbal_roll0),
  sign_yaw_((reverse_yaw) ? -1.0f : 1.0f),
  sign_pitch_((reverse_pitch) ? -1.0f : 1.0f),
  dt_(dt),
  install_roll_(install_roll),
  yaw_relative_angle_filter(fc.yaw_angle),
  pitch_relative_angle_filter(fc.pitch_angle),
  roll_relative_angle_filter(fc.roll_angle),
  //电机目标速度滤波器
  pitch_target_relative_speed_filter(fc.pitch_target_speed),
  yaw_target_relative_speed_filter(fc.yaw_target_speed),
  // 电机反馈速度滤波器
  roll_relative_speed_filter(fc.roll_fdb_speed),
  pitch_relative_speed_filter(fc.pitch_fdb_speed),
  yaw_relative_speed_filter(fc.yaw_fdb_speed),
  //电机目标加速度滤波器
  pitch_motor_target_acc_filter(fc.pitch_target_acc),
  yaw_motor_target_acc_filter(fc.yaw_target_acc),
  //弃用滤波器
  yaw_relative_angle_filter0(1.0f),
  pitch_relative_angle_filter0(1.0f)
{
  // [解析] 手动把几个"输出成员"清零。为什么不直接在 hpp 里写 = 0.0f？
  // [解析]   因为这几个成员在 hpp 里没有默认初值（裸声明），不显式清零就是随机垃圾值，
  // [解析]   上电后如果上层还没调用 update 就读它们，会读到天文数字。
  // [解析] 注意：这里只清了 4 个 joint 角度和 3 个 base 姿态角，
  // [解析]   像 yaw_rel / pitch_rel 这些并没有在这里初始化：它们要等到第一次调用
  // [解析]   update_*() 才会被赋值，在那之前读到的是不确定的值。
  this->yaw_fdb_in_joint = 0.0f;
  this->pitch_fdb_in_joint = 0.0f;
  this->yaw_set_in_joint = 0.0f;
  this->pitch_set_in_joint = 0.0f;
  this->base_yaw_in_world = 0.0f;
  this->base_pitch_in_world = 0.0f;
  this->base_roll_in_world = 0.0f;

  // [解析] 把 R_base2world_ 初始化为"单位矩阵"：对角线上是 1，其他是 0。
  // [解析] 单位矩阵的物理含义是"不旋转"，即假设上电瞬间底盘系和地面系完全重合。
  // [解析] 写成双重循环 + 三元表达式 (i == j) ? 1 : 0，等价于手写 9 个数，但不容易抄错。
  // 初始化为单位矩阵
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      this->R_base2world_[i][j] = (i == j) ? 1.0f : 0.0f;
    }
  }
  // [解析] 把"底盘安装横滚角"转成四元数存起来（yaw、pitch 都传 0，只绕 X 轴转 install_roll_）。
  // [解析] 为什么要提前算好？因为 correct_chassis_imu() 每个周期都要用它做两次变换，
  // [解析]   每次都现算三角函数纯属浪费 CPU，存成四元数后只做乘法即可。
  euler_zyx_to_quaternion(0.0f, 0.0f, install_roll_, this->install_roll_q_);
}
// [解析] 【单 IMU 模式主流程】只有云台装了 IMU 时走这里。整个函数做 5 件事：
// [解析]   ① 用云台姿态 + 电机角度反推底盘姿态（update_q_chassis2world）；
// [解析]   ② 把底盘姿态四元数转成欧拉角存起来；
// [解析]   ③ 对底盘角速度做差分得到角加速度，再换到云台系；
// [解析]   ④ 算"云台相对底盘"的角速度，作为电机速度环反馈，并滤波；
// [解析]   ⑤ 算重力方向在云台系下的投影，供 pitch 轴重力补偿用。
// [解析] 【关于 const float & 参数】浮点数只有 4 字节，传引用并不比传值快（引用本质也是地址），
// [解析]   这里写成引用只是风格问题，不必纠结，也千万不要写出依赖"引用会被修改"的代码。
void Gimbal::update_all_single(
  const sp::Mahony & gimbal_imu, const float & yaw_angle, const float & pitch_angle)
{
  // [解析] 第一步：核心解算。第 4 个参数传 gimbal_roll0_（云台 IMU 安装横滚角），
  // [解析]   它会被当成云台的 roll_rel 使用 —— 也就是把安装误差折算成"云台相对底盘的横滚"。
  //更新电机角度和底盘姿态及角速度
  update_q_chassis2world(gimbal_imu, yaw_angle, pitch_angle, this->gimbal_roll0_);
  // [解析] 第二步：{w,x,y,z} → {roll,pitch,yaw}，方便上层直接读角度（发视觉、判断翻车等）。
  //转换出底盘姿态欧拉角
  quaternion_to_euler(this->q_chassis2world, this->chassis_euler_in_world);
  // [解析] 第三步：算底盘角加速度。先"备份"上一拍的角速度，再取本拍的新值。
  // [解析] 注意 dq 的下标：dq[0] 是不能用的残差，真正的角速度是 dq[1]、dq[2]、dq[3]，
  // [解析]   所以这里写成 [0]=dq[1]、[1]=dq[2]、[2]=dq[3]（差一位，极易写错，务必留意）。
  //更新底盘的角加速度,并换到云台系表示
  w_last_chassis_in_worldframe[0] = w_chassis_in_worldframe[0];
  w_last_chassis_in_worldframe[1] = w_chassis_in_worldframe[1];
  w_last_chassis_in_worldframe[2] = w_chassis_in_worldframe[2];
  w_chassis_in_worldframe[0] = this->dq[1];
  w_chassis_in_worldframe[1] = this->dq[2];
  w_chassis_in_worldframe[2] = this->dq[3];
  // [解析] diff_vec3(新值, 旧值, 结果, dt) 内部就是 (新-旧)/dt，即数值微分。
  // [解析] 差分对噪声很敏感（dt_=1ms 相当于把噪声放大 1000 倍），所以算出来的角加速度偏"毛刺"，
  // [解析] 它只用来做前馈，不参与反馈闭环，所以才敢这么用。
  sp::diff_vec3(
    w_chassis_in_worldframe, w_last_chassis_in_worldframe, acc_chassis_in_worldframe, dt_);
  // [解析] 用云台四元数的"共轭"（true）把地面系下的角加速度换到云台系。
  // [解析] 记忆窍门：gimbal_imu.q 表示"云台系 → 地面系"，所以要反过来换必须取共轭。
  quaternion_frame_transform(
    gimbal_imu.q, acc_chassis_in_worldframe, acc_chassis_in_gimbalframe, true);

  // [解析] 第四步：算云台相对底盘的角速度，这是速度环的反馈值。
  // [解析] 同样先把底盘角速度换到云台系（共轭 = 反向），
  // [解析] 然后做减法：IMU 量到的是"云台相对地面"的角速度，减去"底盘在云台系下的角速度"，
  // [解析] 剩下的就是"云台相对底盘"的角速度 —— 这正是电机需要对抗/跟踪的量。
  //底盘角速度换到云台系并更新云台相对于底盘的欧拉角变化率,可用作电机角速度反馈值
  quaternion_frame_transform(gimbal_imu.q, w_chassis_in_worldframe, w_chassis_in_gimbalframe, true);
  w_relative[0] = gimbal_imu.w[0] - w_chassis_in_gimbalframe[0];
  w_relative[1] = gimbal_imu.w[1] - w_chassis_in_gimbalframe[1];
  w_relative[2] = gimbal_imu.w[2] - w_chassis_in_gimbalframe[2];
  // [解析] 关键的一步换算：w_relative 是"载体系角速度"（三个轴互相耦合），
  // [解析]   而 PID 控制的是 yaw/pitch/roll 各自的角速度，必须用 T⁻¹ 矩阵换算过去。
  // [解析] 后三个参数是引用出参，函数会把结果直接写进 roll/pitch/yaw_relative_speed。
  // [解析] 注意这里传入的欧拉角是 roll_rel / pitch_rel / yaw_rel（云台相对底盘的角度），
  // [解析]   所以算出来的也是"相对底盘"的欧拉角速率，和电机反馈值量纲对得上。
  sp::Gimbal::transform_omiga_in_body_2_euler_rates(
    w_relative, roll_rel, pitch_rel, yaw_rel, roll_relative_speed, pitch_relative_speed,
    yaw_relative_speed);
  // [解析] 滤波：update() 只更新滤波器的内部状态和 out 成员，所以紧接着要手动把 out 抄回去。
  // [解析] 这三行是固定套路：update(原始值) → 用 .out 覆盖原变量。
  // 对电机反馈速度进行滤波
  roll_relative_speed_filter.update(roll_relative_speed);
  // [解析] ↓ 这三组"update + 取 out"就是上面说的固定套路，yaw/pitch 各来一遍。
  roll_relative_speed = roll_relative_speed_filter.out;
  pitch_relative_speed_filter.update(pitch_relative_speed);
  pitch_relative_speed = pitch_relative_speed_filter.out;
  yaw_relative_speed_filter.update(yaw_relative_speed);
  yaw_relative_speed = yaw_relative_speed_filter.out;

  // [解析] 第五步（附带输出）：把底盘角速度换到"底盘系"存一份。
  // [解析] 用途是小陀螺补偿：底盘自己在转，轮子受力会变化，底盘控制需要知道自己转多快。
  // [解析] 同样的 q_chassis2world 表示"底盘系 → 地面系"，所以换回去也要用共轭 true。
  //底盘角速度换到底盘系w_chassis_in_chassisframe可用作小陀螺补偿
  quaternion_frame_transform(
    q_chassis2world, w_chassis_in_worldframe, w_chassis_in_chassisframe, true);

  // [解析] 重力补偿：地面系的重力方向固定为 {0, 0, 1}（z 轴朝天，所以是 +1，不是 -1）。
  // [解析] 用云台四元数把它转到云台系，得到的三维向量就是"重力在云台各轴上的投影方向"。
  // [解析] 它的第三个分量 k_torque[2] 随 pitch 变化（抬头时变小、低头时变大），
  // [解析]   上层可以用"力矩 = k_torque[2] × 枪管质量 × g"给 pitch 轴加一个抵消重力的前馈力矩，
  // [解析]   这样 PID 只需要负责动态误差，静态下枪口不会因为自重而往下掉。
  // [解析] 注意这是用云台 IMU 的 q（不是底盘 q），因为要补偿的是云台自己的姿态。
  //重力补偿
  static float g[3] = {0.0f, 0.0f, 1.0f};  //地面系下重力方向
  // [解析] ↑ static 表示"只在第一次执行时初始化为 {0,0,1}"，之后一直存在。
  // [解析]   这里是个只读常量，用 static 可以省掉每帧重新初始化的开销；
  // [解析]   但也正因为是 static，它是"函数共享的"，千万不要在别处改它。
  sp::Gimbal::quaternion_frame_transform(
    gimbal_imu.q, g, this->k_torque,
    true);  //将重力方向转换到云台系下,k_torque[2]可以用来做重力补偿
}

// [解析] 【双 IMU 模式主流程】底盘也装了 IMU 时走这里。整体结构和 update_all_single() 几乎一样，
// [解析]   只有"底盘姿态和角速度的来源"不同，所以下面只标注与单 IMU 的差异点，相同部分不再重复。
// [解析] 【最大区别】单 IMU 是靠"云台姿态 + 电机角度"反推底盘姿态（有误差累积）；
// [解析]   双 IMU 是底盘 IMU 直接量出底盘姿态（更准，但多一个安装角要标定）。
void Gimbal::update_all_dual(const sp::Mahony & gimbal_imu, sp::Mahony & chassis_imu)
{
  // [解析] 差异点 1：先矫正底盘 IMU。因为它和云台 IMU 的安装角度不同，
  // [解析]   不矫正的话两个 IMU 的 roll 参考面不重合，算出来的相对姿态里会混进固定偏差。
  // [解析] 注意 chassis_imu 是引用（非 const），这个函数会就地改掉它的 q 和 w。
  //使用底盘安装roll去矫正chassis_imu的姿态,使其与gimbal_imu在roll轴上对齐,从而获得更准确的底盘姿态
  correct_chassis_imu(chassis_imu);
  // [解析] 差异点 2：用四元数乘法的逆运算直接求"云台相对底盘"的旋转，
  // [解析]   不需要像单 IMU 那样先反推底盘姿态，同时也顺带把 q_chassis2world 填好了。
  //更新等效电机角度
  update_q_gimbal2chassis(gimbal_imu, chassis_imu);
  // [解析] 差异点 3：底盘角速度直接来自底盘 IMU 的 w（载体系），
  // [解析]   这次用 conjugate = false 把它从"底盘系"换到"地面系"（因为 q 的定义就是底盘系→地面系）。
  // [解析]   对比单 IMU 版本：那边是反过来用 dq 算出来的，这里直接量出来，精度更高。
  //将底盘角速度换到地面系,并微分获得角加速度,再换到云台系表示底盘角加速度
  //转换出底盘姿态欧拉角
  quaternion_to_euler(this->q_chassis2world, this->chassis_euler_in_world);

  // [解析] 备份上一拍角速度 → 写入本拍新值 → 差分求角加速度，和单 IMU 版本完全同一套路。
  w_last_chassis_in_worldframe[0] = w_chassis_in_worldframe[0];
  w_last_chassis_in_worldframe[1] = w_chassis_in_worldframe[1];
  w_last_chassis_in_worldframe[2] = w_chassis_in_worldframe[2];
  quaternion_frame_transform(chassis_imu.q, chassis_imu.w, w_chassis_in_worldframe, false);
  sp::diff_vec3(
    w_chassis_in_worldframe, w_last_chassis_in_worldframe, acc_chassis_in_worldframe, dt_);
  quaternion_frame_transform(
    gimbal_imu.q, acc_chassis_in_worldframe, acc_chassis_in_gimbalframe, true);

  // [解析] 从这一行开始到本函数结束，代码与 update_all_single() 的后半段逐字相同，
  // [解析]   区别只在于 w_chassis_in_worldframe 的来源不同。含义请参考上面的注释。
  //底盘角速度换到云台系并更新云台相对于底盘的欧拉角变化率,可用作电机角速度反馈值
  quaternion_frame_transform(gimbal_imu.q, w_chassis_in_worldframe, w_chassis_in_gimbalframe, true);
  w_relative[0] = gimbal_imu.w[0] - w_chassis_in_gimbalframe[0];
  w_relative[1] = gimbal_imu.w[1] - w_chassis_in_gimbalframe[1];
  w_relative[2] = gimbal_imu.w[2] - w_chassis_in_gimbalframe[2];
  // [解析] T⁻¹ 换算 + 滤波，之后 roll/pitch/yaw_relative_speed 就是可用的电机速度反馈。
  sp::Gimbal::transform_omiga_in_body_2_euler_rates(
    w_relative, roll_rel, pitch_rel, yaw_rel, roll_relative_speed, pitch_relative_speed,
    yaw_relative_speed);
  // 对电机反馈速度进行滤波
  roll_relative_speed_filter.update(roll_relative_speed);
  roll_relative_speed = roll_relative_speed_filter.out;
  pitch_relative_speed_filter.update(pitch_relative_speed);
  pitch_relative_speed = pitch_relative_speed_filter.out;
  yaw_relative_speed_filter.update(yaw_relative_speed);
  yaw_relative_speed = yaw_relative_speed_filter.out;

  // [解析] 存一份"底盘系下的底盘角速度"，供小陀螺补偿使用。
  //底盘角速度换到底盘系w_chassis_in_chassisframe可用作小陀螺补偿
  quaternion_frame_transform(
    q_chassis2world, w_chassis_in_worldframe, w_chassis_in_chassisframe, true);

  // [解析] 重力补偿：把地面系重力的投影存进 k_torque，用法与单 IMU 版本相同。
  //重力补偿
  static float g[3] = {0.0f, 0.0f, 1.0f};  //地面系下重力方向
  sp::Gimbal::quaternion_frame_transform(
    gimbal_imu.q, g, this->k_torque,
    true);  //将重力方向转换到云台系下,k_torque[2]可以用来做重力补偿
}

// [解析] 【目标解算，全文件最难的一块】整体思路分三段，先理解"要干什么"再去看公式：
// [解析]   第一段（好懂）：把世界系下的目标 yaw/pitch 变成一个"枪口指向单位向量"，
// [解析]     再用 q_chassis2world 的共轭把它换到"底盘系"，最后用 atan2/asin 反解出
// [解析]     云台相对底盘的目标角度 —— 这个角度就是电机要转到的位置。
// [解析]   第二段（难）：算目标角速度。因为底盘自己在动，云台要保持世界系指向不动，
// [解析]     其相对角速度必须等于"世界系期望角速度 - 底盘运动带来的影响"，公式就是对这个关系求导。
// [解析]   第三段（最难）：算目标角加速度。先把角加速度表达式拆成 A、B、C、D 四项分别算，
// [解析]     合并成 F 后再乘 T⁻¹ 矩阵，最后投影到 pitch/yaw 两个电机轴上。
// [解析] 【给新手的建议】不要一行行推导公式。先接受"这里是在做坐标变换"，
// [解析]   重点记住每个变量的单位（角度 rad、速度 rad/s、加速度 rad/s²）和符号含义，
// [解析]   调试时优先怀疑输入值（目标角、底盘角速度）而不是公式本身。
void Gimbal::calc_all_target(
  const sp::Mahony & gimbal_imu, float yaw_set_in_world, float pitch_set_in_world,
  float vyaw_set_in_world, float vpitch_set_in_world, float acc_yaw_set_in_world,
  float acc_pitch_set_in_world)
{
  // [解析] 第一段第一步：由 yaw/pitch 构造"枪口指向单位向量"（地面系下）。
  // [解析]   标准球坐标公式应该是 {cos(p)cos(y), cos(p)sin(y), sin(p)}，
  // [解析]   但这里 z 分量写成 -sin(pitch)。这是本工程的符号约定：pitch 抬头为正，
  // [解析]   而地面系 z 轴朝天，抬头时枪口往下压 z 分量，所以取负号。
  // [解析]   也正因为这个负号，下面反解 pitch 时用了 asinf(-z)。
  gun_target_vector[0] = cosf(yaw_set_in_world) * cosf(pitch_set_in_world);
  gun_target_vector[1] = sinf(yaw_set_in_world) * cosf(pitch_set_in_world);
  gun_target_vector[2] = -sinf(pitch_set_in_world);
  // [解析] 第一段第二步：把目标向量从地面系换到"底盘系"。
  // [解析]   q_chassis2world 表示"底盘系 → 地面系"，所以要反过来换必须传 conjugate = true。
  // [解析]   换成底盘系的原因：电机只能感知"自己和底盘之间"的角度，目标必须先落到这个坐标系里。
  quaternion_frame_transform(
    q_chassis2world, gun_target_vector, gun_target_vector_in_chassisframe, true);

  // [解析] 第一段第三步：反解目标角度（这就是最终给电机的"相对角度"指令）。
  // [解析]   yaw = atan2(y, x)：单位向量在底盘系水平面的投影角。需要注意 x、y 的传参顺序，
  // [解析]     atan2f(第1个=分子=y, 第2个=分母=x)，写反了角度会整体转 90°。
  // [解析]   外面套的 yaw_target_relative_angle_unwrapper 用来消除 ±π 跳变，
  // [解析]     否则目标角从 +179° 变到 -179° 时，电机 PID 会指令云台"倒着转 358°"。
  //解算云台相对于底盘目标欧拉角
  yaw_target_relative_angle = this->yaw_target_relative_angle_unwrapper.update(
    atan2f(gun_target_vector_in_chassisframe[1], gun_target_vector_in_chassisframe[0]));

  // [解析]   pitch = asin(-z)：同样因为 pitch 的符号约定，这里带一个负号。
  // [解析]   asinf 的输入理论上必须在 [-1, 1]，向量是单位向量所以天然满足；
  // [解析]     但如果 q_chassis2world 没归一化，这里有可能越界返回 NaN。
  // [解析]     本函数没有像 quaternion_to_euler() 那样先限幅，是个可以改进的点。
  pitch_target_relative_angle = asinf(-gun_target_vector_in_chassisframe[2]);

  // [解析] 【除零保护】后面算速度/加速度的公式里有一堆除以 cos(pitch_rel) 的项。
  // [解析]   pitch_rel 接近 ±90°（云台枪口几乎竖直朝天/朝地）时 cos 趋近 0，结果会炸成 ±∞，
  // [解析]   电机收到这种指令会瞬间满电流狂转，非常危险，所以这里直接 return。
  // [解析] 【代价】提前返回意味着本周期只更新了目标角度，目标速度和目标加速度仍是上一帧的旧值。
  // [解析]   正常情况下云台机械限位到不了 ±90°，这个分支几乎不会触发。
  if (fabs(cosf(this->pitch_rel)) < 1e-5) {
    return;
    //防止除零爆炸,只更新目标角度,冻结其他目标值沿用上一帧
  }
  // [解析] ===== 第二段：解算"目标相对角速度" =====
  // [解析] 【公式在讲什么】要保证枪口在世界系里按 vpitch/vyaw 的期望速度转，
  // [解析]   那么"云台相对底盘"的角速度 = 期望的世界系角速度
  // [解析]     - 底盘转动给云台带来的牵连速度（就是 w_chassis_in_gimbalframe 那几项）。
  // [解析]   由于云台相对底盘的 pitch/roll 之间存在轴系耦合，减完之后还要做一次
  // [解析]   "roll_rel 与 IMU roll 之差"的旋转分解（cos/sin 那两项），这就是公式里两组长括号的来历。
  // [解析] 【为什么有一堆 0.0f * cos(...)】这是作者从矩阵乘法手工展开后留下的"占位项"，
  // [解析]   对应矩阵里本来就是 0 的元素。代码为了和推导稿一一对应而保留，可以直接当 0 看，
  // [解析]   不影响结果，只是式子写得更长、更难一眼看懂。
  //解算云台相对于底盘目标欧拉角变化率
  pitch_target_relative_speed =
    cosf(this->roll_rel - gimbal_imu.roll) *
      (0.0f - this->w_chassis_in_gimbalframe[1] * cosf(gimbal_imu.roll) +
       this->w_chassis_in_gimbalframe[2] * sinf(gimbal_imu.roll)) +
    sinf(this->roll_rel - gimbal_imu.roll) *
      (this->w_chassis_in_gimbalframe[2] * cosf(gimbal_imu.roll) - 0.0f * cosf(gimbal_imu.pitch) +
       this->w_chassis_in_gimbalframe[1] * sinf(gimbal_imu.roll));
  // [解析] 【顺序很重要】这里是"先滤波、再加视觉速度"，全程只对"由底盘运动引起的那部分"滤波。
  // [解析]   好处是视觉发来的期望速度（vpitch_set_in_world）不会被滤波拖慢，瞄准不会有延迟。
  // [解析]   下面被注释掉的一大段是另一种策略"先加视觉值再滤波"，作者做了对比后弃用了。
  // [解析]   ↑ 这段解析说的是紧随其后的 update + 加法两句。
  pitch_target_relative_speed_filter.update(pitch_target_relative_speed);
  pitch_target_relative_speed = pitch_target_relative_speed_filter.out;
  // [解析] 加号后面这一坨：把"期望的世界系 pitch 速度 vpitch_set_in_world"和
  // [解析]   "期望的世界系 yaw 速度 vyaw_set_in_world"分解投影到云台 pitch 轴上。
  // [解析]   注意 yaw 速度也会影响 pitch 轴（因为两个轴不正交于同一个平面），这就是交叉项
  // [解析]   -vyaw_set_in_world * cos(gimbal_imu.pitch) 的由来。键鼠模式下 vyaw 一般传 0，此项消失。
  pitch_target_relative_speed +=
    cosf(this->roll_rel - gimbal_imu.roll) *
      (vpitch_set_in_world - 0.0f * cosf(gimbal_imu.roll) + 0.0f * sinf(gimbal_imu.roll)) +
    sinf(this->roll_rel - gimbal_imu.roll) *
      (0.0f * cosf(gimbal_imu.roll) - vyaw_set_in_world * cosf(gimbal_imu.pitch) +
       0.0f * sinf(gimbal_imu.roll));

  // [解析] yaw 轴同理，只是分解方式不同：除了 cos/sin 之外还要除以 cos(pitch_rel)，
  // [解析]   因为 yaw 轴和世界竖直轴之间夹了一个 pitch 角，pitch 越大同样角速度对应的 yaw 变化越大。
  // [解析]   这就是上面除零保护存在的原因；同时也说明 pitch 越接近 90°，yaw 环的增益被"放大"得越厉害。
  yaw_target_relative_speed =
    (sinf(this->roll_rel - gimbal_imu.roll) *
     (0.0f - this->w_chassis_in_gimbalframe[1] * cosf(gimbal_imu.roll) +
      this->w_chassis_in_gimbalframe[2] * sinf(gimbal_imu.roll))) /
      cosf(this->pitch_rel) -
    (cosf(this->roll_rel - gimbal_imu.roll) *
     (this->w_chassis_in_gimbalframe[2] * cosf(gimbal_imu.roll) - 0.0f * cosf(gimbal_imu.pitch) +
      this->w_chassis_in_gimbalframe[1] * sinf(gimbal_imu.roll))) /
      cosf(this->pitch_rel);
  yaw_target_relative_speed_filter.update(yaw_target_relative_speed);
  yaw_target_relative_speed = yaw_target_relative_speed_filter.out;
  // [解析] yaw 轴同样加上视觉/键鼠的期望世界系速度，写法与 pitch 完全对称。
  yaw_target_relative_speed +=
    (sinf(this->roll_rel - gimbal_imu.roll) *
     (vpitch_set_in_world - 0.0f * cosf(gimbal_imu.roll) + 0.0f * sinf(gimbal_imu.roll))) /
      cosf(this->pitch_rel) -
    (cosf(this->roll_rel - gimbal_imu.roll) *
     (0.0f * cosf(gimbal_imu.roll) - vyaw_set_in_world * cosf(gimbal_imu.pitch) +
      0.0f * sinf(gimbal_imu.roll))) /
      cosf(this->pitch_rel);
  // [解析] ↓ 下面这一整段是被注释掉的"另一种策略"：把底盘运动和视觉期望速度合并之后再滤波。
  // [解析]   既然已经弃用，读代码时可以直接跳过（不要以为是自己看漏了）。
  // [解析]   唯一的用途是：如果实车发现"先滤波再加视觉值"导致瞄准滞后太厉害，
  // [解析]   可以把当前实现注释掉、把这一段放开，做 A/B 对比。
  //优先滤波再加视觉值
  // pitch_target_relative_speed =
  //   cosf(this->roll_rel - gimbal_imu.roll) *
  //     (vpitch_set_in_world - this->w_chassis_in_gimbalframe[1] * cosf(gimbal_imu.roll) +
  //      this->w_chassis_in_gimbalframe[2] * sinf(gimbal_imu.roll)) +
  //   sinf(this->roll_rel - gimbal_imu.roll) *
  //     (this->w_chassis_in_gimbalframe[2] * cosf(gimbal_imu.roll) -
  //      vyaw_set_in_world * cosf(gimbal_imu.pitch) +
  //      this->w_chassis_in_gimbalframe[1] * sinf(gimbal_imu.roll));
  // pitch_target_relative_speed_filter.update(pitch_target_relative_speed);
  // pitch_target_relative_speed = pitch_target_relative_speed_filter.out;

  // yaw_target_relative_speed =
  //   (sinf(this->roll_rel - gimbal_imu.roll) *
  //    (vpitch_set_in_world - this->w_chassis_in_gimbalframe[1] * cosf(gimbal_imu.roll) +
  //     this->w_chassis_in_gimbalframe[2] * sinf(gimbal_imu.roll))) /
  //     cosf(this->pitch_rel) -
  //   (cosf(this->roll_rel - gimbal_imu.roll) *
  //    (this->w_chassis_in_gimbalframe[2] * cosf(gimbal_imu.roll) -
  //     vyaw_set_in_world * cosf(gimbal_imu.pitch) +
  //     this->w_chassis_in_gimbalframe[1] * sinf(gimbal_imu.roll))) /
  //     cosf(this->pitch_rel);
  // yaw_target_relative_speed_filter.update(yaw_target_relative_speed);
  // yaw_target_relative_speed = yaw_target_relative_speed_filter.out;

  // [解析] ===== 第三段：解算"目标相对角加速度" =====
  // [解析] 思路：角加速度不能简单地对目标角速度差分（那样会掺进大量噪声），
  // [解析]   而是把"枪口在世界系中的期望角加速度"写成若干项，逐项算出来再加起来。
  // [解析]   公式整体形状是： 目标相对角加速度 = T⁻¹ · (A - B - C - D)。
  // [解析]   下面按作者原始的 A/B/C/D 命名逐项解释，读的时候对照变量名看即可。

  //计算A
  // [解析] A = Ṫ·Ė：把"云台相对世界的欧拉角"和"云台相对世界的欧拉角速率"代进去，
  // [解析]   得到的是云台自身转动产生的耦合加速度项（单位 rad/s²）。
  // [解析]   它算的是云台（不是底盘）的运动，用的是 gimbal_imu 的 roll/pitch/yaw 与 vroll/vpitch/vyaw。
  sp::Gimbal::Transform_matrix_rates_multipy_Euler_rates(
    gimbal_imu.roll, gimbal_imu.pitch, gimbal_imu.yaw, gimbal_imu.vroll, gimbal_imu.vpitch,
    gimbal_imu.vyaw, A_Tdot_Edot);

  //计算B
  // [解析] B = 底盘角加速度在云台系下的表示。它是在 update_all_single/dual 里算好存在
  // [解析]   acc_chassis_in_gimbalframe 的，这里只是"抄一份到 B"（保持与推导稿的变量名一致）。
  B_acc_chassis_in_gimbalframe[0] = acc_chassis_in_gimbalframe[0];
  B_acc_chassis_in_gimbalframe[1] = acc_chassis_in_gimbalframe[1];
  B_acc_chassis_in_gimbalframe[2] = acc_chassis_in_gimbalframe[2];

  //计算C
  // [解析] C = w_底盘 × w_相对（叉乘）：两个转动互相耦合产生的加速度项。
  // [解析]   物理直觉：你站在一个正在自转的转盘上同时抬手臂，手臂会有额外的加速度，
  // [解析]   这个叉乘就是 Cloud 在"底盘自转 × 云台相对转动"下的耦合。
  sp::Gimbal::cross_product(this->w_chassis_in_gimbalframe, this->w_relative, C_cross_product);

  //计算D
  // [解析] D = J̇·θ̇：用"云台相对底盘的欧拉角 + 相对角速度"再调一次同一个函数得到。
  // [解析]   和 A 的区别：A 用的是相对世界的量，D 用的是相对底盘的量。
  sp::Gimbal::Transform_matrix_rates_multipy_Euler_rates(
    this->roll_rel, this->pitch_rel, this->yaw_rel, roll_relative_speed, pitch_relative_speed,
    yaw_relative_speed, D_Jdot_E_Motor_dot);

  // [解析] 注意：A 项用的是"云台相对世界"的欧拉角与角速率，而 B/C/D 三项描述的都是相对量，
  // [解析]   两者参考系是否完全一致、是否存在重复扣除，建议对照作者的推导稿再核一遍。
  // [解析]   实测能跑不代表公式一定干净；如果以后云台在自转时出现固定的加速度偏置，先查这里。
  //合起来作为F,简化运算
  // [解析] F = A - B - C - D：把四项合并成"载体系下还需要额外补偿的角加速度"，单位 rad/s²。
  // [解析]   减号的含义是"把不需要云台电机负责的加速度扣掉"：底盘自己的加速度(B)、
  // [解析]   转动耦合产生的加速度(C、D)都不该让云台电机去追，所以做减法。
  F[0] =
    A_Tdot_Edot[0] - B_acc_chassis_in_gimbalframe[0] - C_cross_product[0] - D_Jdot_E_Motor_dot[0];
  F[1] =
    A_Tdot_Edot[1] - B_acc_chassis_in_gimbalframe[1] - C_cross_product[1] - D_Jdot_E_Motor_dot[1];
  F[2] =
    A_Tdot_Edot[2] - B_acc_chassis_in_gimbalframe[2] - C_cross_product[2] - D_Jdot_E_Motor_dot[2];

  // [解析] Final = T⁻¹ · F：把"载体系角加速度"换算回"欧拉角空间"的角加速度。
  // [解析]   注意这里传入的欧拉角是 gimbal_imu.roll/pitch/yaw（云台相对世界），
  // [解析]   所以同一段代码里的两处矩阵运算用的是不同参考系的欧拉角，这是刻意的、也是最难懂的地方。
  //乘以雅各比矩阵T的逆
  sp::Gimbal::transform_omiga_in_body_2_euler_rates(
    F, gimbal_imu.roll, gimbal_imu.pitch, gimbal_imu.yaw, Final[0], Final[1], Final[2]);

  // [解析] ↓ 三行被注释掉的代码：本想把视觉期望加速度直接加进 Final。
  // [解析]   后来改成在下面"投影到电机轴之后"再加（等价于先乘一个 S 矩阵），所以这里注释掉了。
  //给Final加上视觉发来的欧拉角期望加速度
  //后续只考虑后两个分量等价于乘了一个S矩阵
  // Final[1] += acc_pitch_set_in_world;
  // Final[2] += acc_yaw_set_in_world;

  // [解析] 最后一步：把 Final 的 pitch/yaw 分量"投影"到云台电机轴上，得到电机真正要执行的角加速度。
  // [解析] 【为什么还要投影】Final 是在欧拉角空间里的，而 pitch/yaw 电机轴和欧拉角轴并不完全重合，
  // [解析]   两者之间差一个"roll 相关的旋转"+ 一个 pitch 角的缩放，公式里的 cos/sin(roll_rel - imu.roll)
  // [解析]   和除以 cos(pitch_rel) 就是这两个修正。Roll 轴不需要算，因为它是机械固定的。
  // [解析] 注意下面 pitch 用的是 sinf(gimbal_imu.pitch) 而 yaw 用的是 cosf(gimbal_imu.pitch)，
  // [解析]   这不是笔误，而是投影到两个不同轴上的结果。
  //优先滤波再加视觉值
  //千呼万唤始出来兄弟们
  //对这个Final的后两个向量求逆就可以了!!!!!!!!!!!
  pitch_target_relative_acc =
    Final[1] * cosf(this->roll_rel - gimbal_imu.roll) -
    Final[2] * cosf(gimbal_imu.pitch) * sinf(this->roll_rel - gimbal_imu.roll);
  // [解析] 此处滤波的是"由底盘运动推导出来的那部分加速度"，还没有叠加视觉的期望加速度。
  pitch_motor_target_acc_filter.update(pitch_target_relative_acc);
  pitch_target_relative_acc = pitch_motor_target_acc_filter.out;

  // [解析] yaw 轴的投影：注意多除了一个 cos(pitch_rel)，和前面目标角速度那边的道理一样。
  // [解析]   所以这个除零风险同样由前面那个 return 保护住了。
  yaw_target_relative_acc =
    (Final[1] * sinf(this->roll_rel - gimbal_imu.roll) +
     Final[2] * cosf(gimbal_imu.pitch) * cosf(this->roll_rel - gimbal_imu.roll)) /
    cosf(this->pitch_rel);
  yaw_motor_target_acc_filter.update(yaw_target_relative_acc);
  yaw_target_relative_acc = yaw_motor_target_acc_filter.out;

  // [解析] 现在把"视觉给出的期望世界系角加速度"加上去（键鼠模式下这两个参数是 0，加了等于没加）。
  // [解析] 加在滤波之后是刻意的：视觉的期望值本身已经很平滑，不需要再被滤波器拖慢。
  pitch_target_relative_acc +=
    acc_pitch_set_in_world * cosf(this->roll_rel - gimbal_imu.roll) -
    acc_yaw_set_in_world * cosf(gimbal_imu.pitch) * sinf(this->roll_rel - gimbal_imu.roll);
  // [解析] 注意：这一行又把"已经加上视觉值"的结果送进滤波器过滤了一遍（上面刚更新过同一个滤波器）。
  // [解析]   这相当于视觉的期望加速度也被低通滤波了，和上面那句"优先滤波再加视觉值"的字面意思有出入。
  // [解析]   而 yaw 轴在加完之后没有再 update（见下面），两个轴的处理不一致。
  // [解析]   实际效果是 pitch 前馈比 yaw 前馈更"迟钝"一点；这里保持代码原样，只做提醒。
  pitch_motor_target_acc_filter.update(pitch_target_relative_acc);
  pitch_target_relative_acc = pitch_motor_target_acc_filter.out;

  // [解析] yaw 轴加上视觉期望加速度，但没有再调用 update()，所以 yaw 的视觉前馈是"原样直通"的。
  yaw_target_relative_acc +=
    (acc_pitch_set_in_world * sinf(this->roll_rel - gimbal_imu.roll) +
     acc_yaw_set_in_world * cosf(gimbal_imu.pitch) * cosf(this->roll_rel - gimbal_imu.roll)) /
    cosf(this->pitch_rel);
}

// [解析] 【旧版单 IMU 解算，已基本弃用】它用旋转矩阵而不是四元数来算底盘姿态，数学上等价但更啰嗦。
// [解析] 保留它的价值：一是老工程还在用，二是可以用来和新版四元数解算互相验证结果。
// [解析] 与 update_q_chassis2world() 的关键差别（读的时候注意别搞混）：
// [解析]   1) 它不做角度展开（没有 AngleUnwrapper），所以 yaw 跨 ±π 时会跳变；
// [解析]   2) 它用的是已经弃用的 filter0 系列滤波器（alpha 固定 1.0f）来滤波相对角度；
// [解析]   3) 它只输出 R_base2world_ 矩阵和 base_*_in_world 角，不算相对速度、不更新 dq。
void Gimbal::update(
  const sp::Mahony & gimbal_imu, const float & yaw_angle, const float & pitch_angle)
{
  // [解析] 把电机绝对角度减去零点，得到相对角度；limit_angle 折到 (-π, π]。
  // [解析] 乘 sign_yaw_ / sign_pitch_ 是为了处理"电机正转方向与云台右手系相反"的安装方式。
  //云台相对码盘值计算（正方向使用云台系右手定则）
  float yaw_relative_angle = sign_yaw_ * sp::limit_angle(yaw_angle - yaw0_);
  float pitch_relative_angle = sign_pitch_ * sp::limit_angle(pitch_angle - pitch0_);

  // [解析] 用弃用的 filter0 滤波（alpha = 1.0f，等于没滤）。留着是为了不改动历史行为。
  // 使用低通滤波器对角度进行滤波
  yaw_relative_angle_filter0.update(yaw_relative_angle);
  pitch_relative_angle_filter0.update(pitch_relative_angle);

  // [解析] 固定套路：update 之后取 .out 覆盖原变量。
  yaw_relative_angle = yaw_relative_angle_filter0.out;
  pitch_relative_angle = pitch_relative_angle_filter0.out;

  // [解析] 存进公开成员 yaw_fdb_in_joint / pitch_fdb_in_joint，供上层读速度环以外的角度反馈。
  yaw_fdb_in_joint = yaw_relative_angle;
  pitch_fdb_in_joint = pitch_relative_angle;

  // [解析] 下面这一堆 sin/cos 是为"手工展开矩阵乘法"做准备。
  // [解析] 命名规则：s=sin、c=cos、g=gimbal（云台 IMU 的欧拉角）、rel=relative（电机相对角度）、
  // [解析]   p/pitch、r/roll、y/yaw 是轴名。比如 sg_p = sin(gimbal.pitch)、c_rel_y = cos(yaw_relative_angle)。
  // [解析] 为什么要提前算？因为矩阵里每个元素都反复用到这些三角函数值，
  // [解析]   先存成局部变量可以让编译器放进寄存器，比每次调用 sinf 快得多（1ms 周期里很关键）。
  float sg_p = sinf(gimbal_imu.pitch);
  float cg_p = cosf(gimbal_imu.pitch);
  float sg_r = sinf(gimbal_imu.roll);
  float cg_r = cosf(gimbal_imu.roll);
  float sg_y = sinf(gimbal_imu.yaw);
  float cg_y = cosf(gimbal_imu.yaw);

  float s_rel_y = sinf(yaw_relative_angle);
  float c_rel_y = cosf(yaw_relative_angle);
  float s_rel_p = sinf(pitch_relative_angle);
  float c_rel_p = cosf(pitch_relative_angle);

  float sr_sp = sg_r * sg_p;

  // [解析] 下面 9 行是"手工展开的 3×3 矩阵乘法"，把云台姿态 R(云台→世界) 和
  // [解析]   "云台相对底盘"的旋转 R(底盘→云台) 乘在一起，得到 R_base2world_ = R(底盘→世界)。
  // [解析] 【怎么读】Row 0/1/2 分别对应矩阵的第 0/1/2 行；每个元素都是若干 sin/cos 的乘积之和。
  // [解析]   不要试图逐项验证推导，重点关注：行号 = 世界系的轴，列号 = 底盘系的轴，
  // [解析]   所以 R[i][j] 表示"底盘系的第 j 个轴在世界系第 i 个轴上的投影"。
  // [解析] 【和四元数版的等价性】update_q_chassis2world() 用四元数做的其实是同一件事，
  // [解析]   只是那边没有把结果展开成矩阵。两者对同一输入应给出相同姿态，可以互相校验。
  // Row 0
  R_base2world_[0][0] = -(sr_sp * cg_y - sg_y * cg_r) * s_rel_y +
                        (sr_sp * cg_y + sg_r * sg_y) * s_rel_p * c_rel_y +
                        cg_p * cg_y * c_rel_p * c_rel_y;

  R_base2world_[0][1] = (sr_sp * cg_y - sg_y * cg_r) * c_rel_y +
                        (sr_sp * cg_y + sg_r * sg_y) * s_rel_p * s_rel_y +
                        s_rel_y * cg_p * cg_y * c_rel_p;

  R_base2world_[0][2] = (sr_sp * cg_y + sg_r * sg_y) * c_rel_p - s_rel_p * cg_p * cg_y;

  // Row 1
  R_base2world_[1][0] = -(sr_sp * sg_y + cg_r * cg_y) * s_rel_y +
                        (sg_p * sg_y * cg_r - sg_r * cg_y) * s_rel_p * c_rel_y +
                        sg_y * cg_p * c_rel_p * c_rel_y;

  R_base2world_[1][1] = (sr_sp * sg_y + cg_r * cg_y) * c_rel_y +
                        (sg_p * sg_y * cg_r - sg_r * cg_y) * s_rel_p * s_rel_y +
                        sg_y * s_rel_y * cg_p * c_rel_p;

  R_base2world_[1][2] = (sg_p * sg_y * cg_r - sg_r * cg_y) * c_rel_p - sg_y * s_rel_p * cg_p;

  // Row 2
  R_base2world_[2][0] =
    -sg_p * c_rel_p * c_rel_y - sg_r * s_rel_y * cg_p + s_rel_p * cg_p * cg_r * c_rel_y;

  R_base2world_[2][1] =
    -sg_p * s_rel_y * c_rel_p + sg_r * cg_p * c_rel_y + s_rel_p * s_rel_y * cg_p * cg_r;

  R_base2world_[2][2] = sg_p * s_rel_p + cg_p * cg_r * c_rel_p;

  // [解析] 从矩阵反解底盘姿态角，这就是 rotation_matrix_to_euler() 内联在这里的版本：
  // [解析]   yaw = atan2(R[1][0], R[0][0])，pitch = asin(-R[2][0])，roll = atan2(R[2][1], R[2][2])。
  // [解析]   公式对 ZYX 顺序的旋转矩阵是通用的，记住"yaw 看第一列、pitch 看第三行第一列"即可。
  base_yaw_in_world = atan2f(R_base2world_[1][0], R_base2world_[0][0]);
  base_pitch_in_world = asinf(-R_base2world_[2][0]);  // 注意：这里可能需要防 NaN 处理

  // [解析] 安全性兜底：asinf 的输入必须落在 [-1, 1]，否则返回 NaN 并一路污染后面的计算。
  // [解析] 因为矩阵是自己算出来的（数值误差可能让 -R[2][0] 略微越界），所以这里补一个保护。
  // [解析] 注意它是在调用 asinf 之后才判断的：如果真越界了，asinf 已经算出 NaN，
  // [解析]   靠这次赋值把 NaN 覆盖掉 —— 效果正确，但写法上"先算再检查"稍微别扭。
  // 增加安全性：
  if (R_base2world_[2][0] > 1.0f)
    base_pitch_in_world = -sp::SP_PI / 2;
  else if (R_base2world_[2][0] < -1.0f)
    base_pitch_in_world = sp::SP_PI / 2;

  // [解析] roll 用的也是标准公式 atan2(R[2][1], R[2][2])，无需额外保护
  // [解析]   （atan2 对任意输入都有定义，不像 asinf 那样有定义域限制）。
  base_roll_in_world = atan2f(R_base2world_[2][1], R_base2world_[2][2]);
};

// [解析] 【单 IMU 模式的核心，务必重点看】任务：已知"云台相对地面的姿态"和"云台相对底盘的机械角度"，
// [解析]   反推出"底盘相对地面的姿态"。思路就是一次四元数复合：
// [解析]     q_W_C = q_W_G ⊗ q_G_C
// [解析]   其中 W=世界(地面)系、G=云台系、C=底盘系：
// [解析]     q_W_G 由云台 IMU 直接给出（云台→世界），
// [解析]     q_G_C 由两个电机编码器角度构造（底盘→云台），
// [解析]     乘起来就得到底盘→世界的四元数 q_W_C。
// [解析] 【为什么能这么做】因为云台和底盘通过电机刚性相连，两个自由度(yaw/pitch)全都能量出来，
// [解析]   所以"底盘→云台"这个旋转是完全已知的，剩下的就靠云台 IMU 补全。
// [解析] 【函数结尾为什么有 dq】最后还用"本拍四元数 ⊗ 上拍四元数共轭"求出底盘的角速度，
// [解析]   这一步是整个速度反馈的基础，且不能归一化 —— 见函数末尾的详细说明。
void Gimbal::update_q_chassis2world(
  const sp::Mahony & gimbal_imu, const float & yaw_angle, const float & pitch_angle,
  const float & roll0_)
{
  /* ------------------------------------------------
   * Step 0: 编码器角度 → 云台相对底盘欧拉角
   * ------------------------------------------------ */
  // [解析] Step 0：把编码器读数变成"云台相对底盘"的三个角。三个轴的待遇不一样，这里是最容易出错的地方：
  // [解析]   yaw   —— 无限位，会绕圈，所以必须先用 limit_angle 折到 (-π,π] 再送进展开器，
  // [解析]            否则跨圈时的跳变会让后面微分出来的角速度"炸掉"（原注释说的就是这个）。
  // [解析]            注意顺序：先 limit_angle 再 unwrapper.update，反了就没有展开效果。
  // [解析]   pitch —— 有机械限位，转不出 ±π，不会再跳变，所以只折一次不展开（原注释说明）。
  // [解析]   roll  —— 由机械结构固定，直接等于安装横滚角 roll0_，不来自编码器。
  yaw_rel = sign_yaw_ *
            this->yaw_unwrapper_.update(
              sp::limit_angle(yaw_angle - yaw0_));  //这里要展开limit_angle,否则带来的跳变会让w炸掉
  pitch_rel =
    sign_pitch_ * sp::limit_angle(pitch_angle - pitch0_);  //限位原因无所谓不需要展开limit_angle
  roll_rel = roll0_;

  // [解析] 对两个"可控轴"的角度做低通滤波。roll 是安装常数，没有噪声，所以不滤。
  // [解析] 滤完立刻取 .out 覆盖，这是本工程用滤波器的固定写法。
  // 角度低通滤波（只对可控轴）
  yaw_relative_angle_filter.update(yaw_rel);
  pitch_relative_angle_filter.update(pitch_rel);

  yaw_rel = yaw_relative_angle_filter.out;
  pitch_rel = pitch_relative_angle_filter.out;

  /* ------------------------------------------------
   * Step 1: 半角
   * ------------------------------------------------ */
  // [解析] Step 1：算"半角"的六个三角函数值。四元数表示旋转时用的都是"半个角"，
  // [解析]   原因是四元数对旋转的编码是 q = {cos(θ/2), axis * sin(θ/2)}。
  // [解析]   所以下面所有 sin/cos 的输入都是 0.5 * 角度，写成 0.5f 而不是 1/2 是为了避免整数除法。
  // [解析] 变量名同样遵循 s/c + 轴名 的规律：cy=cos(half_yaw)、sp=sin(half_pitch)……
  float half_yaw = 0.5f * yaw_rel;
  float half_pitch = 0.5f * pitch_rel;
  float half_roll = 0.5f * roll_rel;

  float cy = cosf(half_yaw);
  float sy = sinf(half_yaw);
  float cp = cosf(half_pitch);
  float sp = sinf(half_pitch);
  float cr = cosf(half_roll);
  float sr = sinf(half_roll);

  /* ------------------------------------------------
   * Step 2: 构造 q_G_C（底盘 → 云台）
   * ZYX intrinsic: yaw → pitch → roll
   * ------------------------------------------------ */
  // [解析] Step 2：由三个相对角度构造四元数 q_G_C。它是把单轴四元数按
  // [解析]   q_yaw ⊗ q_pitch ⊗ q_roll 顺序乘起来再展开的结果（ZYX 内旋）。
  // [解析] 【重要发现，能省你半小时】这里的 x_GC / y_GC / z_GC 都带一个负号，
  // [解析]   所以 q_G_C 恰好等于 euler_zyx_to_quaternion(yaw_rel, pitch_rel, roll_rel) 的共轭！
  // [解析]   也就是说两个函数对同一组欧拉角给出"互逆"的旋转方向：
  // [解析]     euler_zyx_to_quaternion() 给出"底盘→云台"的反向（即云台→底盘），
  // [解析]     这里手写展开则给出"底盘→云台"。抄公式时千万别把两者的符号混用。
  // [解析] 命名后缀 GC 表示"从 C(底盘) 到 G(云台)"。
  float w_GC = (cy * cp * cr + sy * sp * sr);
  float x_GC = -(cy * cp * sr - sy * sp * cr);
  float y_GC = -(cy * sp * cr + sy * cp * sr);
  float z_GC = -(sy * cp * cr - cy * sp * sr);

  /* ------------------------------------------------
   * Step 3: 读取 q_W_G（云台 → 世界）
   * ------------------------------------------------ */
  // [解析] Step 3：把云台 IMU 的四元数抄到局部变量。用指针 qWG 只是为了少敲几个字，
  // [解析]   后面就直接读 gimbal_imu.q 也一样。顺序仍然是 {w, x, y, z}。
  // [解析]   这个 q 来自 sp::Mahony，由 imu_task 每 1ms 更新一次，这里只读不写。
  const float * qWG = gimbal_imu.q;
  float w_WG = qWG[0];
  float x_WG = qWG[1];
  float y_WG = qWG[2];
  float z_WG = qWG[3];

  /* ------------------------------------------------
   * Step 4: 四元数复合
   * q_W_C = q_W_G ⊗ q_G_C
   * ------------------------------------------------ */
  // [解析] Step 4：把 Hamilton 乘法公式手工展开（等价于调用 quaternion_multiply，
  // [解析]   但这里内联展开，省掉一次函数调用开销）。四个式子的符号顺序不要试图硬记，
  // [解析]   需要时直接对照 quaternion_multiply() 的实现，两边是一致的。
  // [解析] 物理含义：先做"底盘→云台"，再做"云台→世界"，串起来就是"底盘→世界"。
  float w_WC = w_WG * w_GC - x_WG * x_GC - y_WG * y_GC - z_WG * z_GC;
  float x_WC = w_WG * x_GC + x_WG * w_GC + y_WG * z_GC - z_WG * y_GC;
  float y_WC = w_WG * y_GC - x_WG * z_GC + y_WG * w_GC + z_WG * x_GC;
  float z_WC = w_WG * z_GC + x_WG * y_GC - y_WG * x_GC + z_WG * w_GC;

  /* ------------------------------------------------
   * Step 5: 归一化（防数值漂移）
   * ------------------------------------------------ */
  // [解析] Step 5：归一化。四元数表示旋转必须满足"模长 = 1"，但每帧几百次浮点乘法之后
  // [解析]   模长会慢慢偏离 1（数值漂移），不修正的话姿态会越来越歪。
  // [解析]   做法：算出模长 norm，再乘它的倒数。乘倒数比连除 4 次快（除法在 MCU 上很贵）。
  // [解析] 【1e-12f 这个阈值】norm 理论上接近 1，只有计算异常时才会趋近 0；
  // [解析]   加这个判断是为了避免 1/norm 除出 inf（inf 一旦混进姿态就再也回不来了）。
  // [解析] else 分支退回单位四元数 {1,0,0,0}（表示"不旋转"），属于兜底，正常不会走到。
  float norm = sqrtf(w_WC * w_WC + x_WC * x_WC + y_WC * y_WC + z_WC * z_WC);
  if (norm > 1e-12f) {
    float inv = 1.0f / norm;
    w_WC *= inv;
    x_WC *= inv;
    y_WC *= inv;
    z_WC *= inv;
  }
  else {
    // 极端异常兜底
    w_WC = 1.0f;
    x_WC = y_WC = z_WC = 0.0f;
  }

  /* ------------------------------------------------
   * Step 6: 输出（底盘 → 世界）
   * ------------------------------------------------ */
  // [解析] Step 6：先把"上一拍"的四元数备份到 q_last_chassis2world，再写入本拍结果。
  // [解析]   备份是给下面求角速度用的 —— 用"本拍 ⊗ 上拍的共轭"得到两拍之间的旋转增量。
  // [解析]   顺序不能反：一旦先写 q_chassis2world，上一拍的数据就被覆盖了。
  q_last_chassis2world[0] = q_chassis2world[0];
  q_last_chassis2world[1] = q_chassis2world[1];
  q_last_chassis2world[2] = q_chassis2world[2];
  q_last_chassis2world[3] = q_chassis2world[3];

  // [解析] 写入公开成员（也就是最终输出）。q_chassis2world 的含义是"底盘系 → 地面系"。
  q_chassis2world[0] = w_WC;
  q_chassis2world[1] = x_WC;
  q_chassis2world[2] = y_WC;
  q_chassis2world[3] = z_WC;

  // [解析] ↓ 被注释掉的一行：把底盘姿态转成欧拉角。上层实际调用的是 update_all_single()
  // [解析]   里的 quaternion_to_euler(this->q_chassis2world, ...)，功能重复所以这里注释掉了。
  //quaternion_to_euler(q_chassis2world, euler_q);

  // [解析] 下面开始求底盘的角速度，用的是四元数微分法（比直接对欧拉角差分更准确、不受万向锁影响）。
  // [解析] 公式：dq = q_now ⊗ q_last*，即"本拍 ⊗ 上拍的共轭"，
  // [解析]   得到的 dq 表示"从上一拍到本拍的微小旋转"。
  // [解析] 参数 false/true 对应"q1 不取共轭、q2 取共轭"，也就是 q_chassis2world ⊗ q_last*。
  //现在下面这个dq 是等价于在地面系下的底盘w
  quaternion_multiply(q_chassis2world, q_last_chassis2world, dq, false, true);
  // [解析] 关键提醒：dq 不是单位四元数，这里绝对不能对它做归一化！
  // [解析]   归一化会把"旋转有多小"这个信息抹掉，角速度就永远是 0 了。
  //这个dq一定不能归一化, 因为他就不是单位四元数

  // [解析] 泰勒展开近似：对于小角度旋转，q ≈ {1, ω·dt/2}，所以反推 ω = 2 * dq / dt。
  // [解析]   这里把 4 个分量统一乘 2/dt_，乘完之后 dq[1..3] 就是地面系下的角速度（rad/s）。
  // [解析]   【前提条件】必须保证一个周期内转角足够小，即 dt 很小（本项目 1ms）且转速有限。
  // [解析]     如果 dt_ 填错（比如误填 1.0f），算出来的角速度会缩小 1000 倍，速度环直接失效。
  //使用泰勒展开近似四元数增量表示的角速度(必须保证他在1ms内是小量)
  this->dq[0] *= 2.0f / dt_;
  this->dq[1] *= 2.0f / dt_;
  this->dq[2] *= 2.0f / dt_;
  this->dq[3] *= 2.0f / dt_;
}

// [解析] 【双 IMU 模式的相对姿态计算】已有两个 IMU 的绝对姿态，求"云台相对底盘"的旋转。
// [解析] 核心就是一步四元数除法（用共轭实现）：q_gimbal2chassis = q_chassis* ⊗ q_gimbal。
// [解析] 为什么这样算：q_chassis 把底盘系换到世界系，取共轭后就变成"世界系 → 底盘系"；
// [解析]   再用它去接 q_gimbal（云台系 → 世界系），得到的就是"云台系 → 底盘系"，
// [解析]   也就是云台相对底盘的姿态。最后转成欧拉角就是电机角度。
void Gimbal::update_q_gimbal2chassis(const sp::Mahony & gimbal_imu, const sp::Mahony & chassis_imu)
{
  // [解析] 参数 true/false：对第一个参数（底盘四元数）取共轭，第二个不取。
  // [解析]   即 q_chassis* ⊗ q_gimbal。两个布尔值顺序写反是这套代码里最常见的错误来源。
  //已有底盘的姿态四元数,算出底盘和云台之间的相对四元数,并将相对四元数转换成欧拉角
  quaternion_multiply(chassis_imu.q, gimbal_imu.q, q_gimbal2chassis, true, false);
  // [解析] 双 IMU 模式下，底盘姿态就是底盘 IMU 直接给的四元数，复制过来即可（不用反推）。
  q_chassis2world[0] = chassis_imu.q[0];
  q_chassis2world[1] = chassis_imu.q[1];
  q_chassis2world[2] = chassis_imu.q[2];
  q_chassis2world[3] = chassis_imu.q[3];

  // [解析] 归一化防止数值漂移：两个都是"理论模长 1"的四元数，乘完之后可能略微偏离 1。
  // [解析] 与 update_q_chassis2world() 里的归一化完全同一个套路。
  // 归一化防止数值漂移
  float norm = sqrtf(
    q_gimbal2chassis[0] * q_gimbal2chassis[0] + q_gimbal2chassis[1] * q_gimbal2chassis[1] +
    q_gimbal2chassis[2] * q_gimbal2chassis[2] + q_gimbal2chassis[3] * q_gimbal2chassis[3]);
  if (norm > 1e-12f) {
    float inv = 1.0f / norm;
    q_gimbal2chassis[0] *= inv;
    q_gimbal2chassis[1] *= inv;
    q_gimbal2chassis[2] *= inv;
    q_gimbal2chassis[3] *= inv;
  }

  // [解析] 转成欧拉角，顺序是 {roll, pitch, yaw}，存进中转变量 euler_motor，再分别赋给三个 _rel 成员。
  quaternion_to_euler(q_gimbal2chassis, euler_motor);
  // [解析] ↓ 作者留的 TODO：以后也许把"编码器实测角度"和这里 IMU 算出的角度做融合滤波，
  // [解析]   现在二者是独立的两条路（单 IMU 走编码器，双 IMU 走 IMU）。
  //后续考虑是否用电机编码器值和euler_motor进行融合滤波
  roll_rel = euler_motor[0];
  pitch_rel = euler_motor[1];
  // [解析] yaw 一定要过展开器：欧拉角被折算在 (-π, π] 内，云台连续转两圈就会来回跳，
  // [解析]   不展开的话后面微分求角速度会瞬间冒出巨大的尖峰。
  yaw_rel = this->yaw_unwrapper_.update(euler_motor[2]);  // 展开消除±π跳变euler_motor[2];

  // [解析] 三个轴都做低通滤波（双 IMU 模式下 roll 也是可测量量，所以这次 roll 也滤）。
  // [解析] 依旧是"update 完立刻取 .out"的固定写法。
  // 角度低通滤波
  yaw_relative_angle_filter.update(yaw_rel);
  pitch_relative_angle_filter.update(pitch_rel);
  roll_relative_angle_filter.update(roll_rel);
  yaw_rel = yaw_relative_angle_filter.out;
  pitch_rel = pitch_relative_angle_filter.out;
  roll_rel = roll_relative_angle_filter.out;
}
// [解析] 【旧版目标解算，已弃用】需要先调用 update() 把 R_base2world_ 算好，否则矩阵还是单位阵，结果全错。
// [解析] 思路：把世界系下的目标方向用 R_base2world_ 换到底盘系，再反解成电机角度。
// [解析] 下面三个 common_term_* 其实就是"目标单位向量在底盘系的 x/y/z 分量"，
// [解析]   整个式子等价于"用 R_base2world_ 的转置去乘目标向量"（转置 = 逆，因为旋转矩阵正交）。
// [解析] 【和 calc_all_target() 的区别】这个版本不支持速度/加速度前馈，也没有展开器和滤波器，
// [解析]   新代码请用 calc_all_target()。
void Gimbal::calc(float yaw_set_in_world, float pitch_set_in_world)
{
  // [解析] 先把目标 yaw/pitch 的 sin/cos 算好，后面 3 个分量表达式都要用。
  // [解析] 注意 pitch 的 sin 被命名成 sp_val（不是 sp），避免和别处的变量重名。
  float cy = cosf(yaw_set_in_world);
  float sy = sinf(yaw_set_in_world);
  float cp = cosf(pitch_set_in_world);
  float sp_val = sinf(pitch_set_in_world);

  // [解析] 目标向量在底盘系下的 x 分量 = R 矩阵第 0 列与目标向量的点积。
  // [解析] 式子里的 -R_base2world_[2][0] * sp_val 那个负号同样来自本工程"抬头为正"的 pitch 约定。
  float common_term_x =
    R_base2world_[0][0] * cp * cy + R_base2world_[1][0] * sy * cp - R_base2world_[2][0] * sp_val;

  float common_term_y =
    R_base2world_[0][1] * cp * cy + R_base2world_[1][1] * sy * cp - R_base2world_[2][1] * sp_val;

  // [解析] yaw = atan2(分量y, 分量x)，与 calc_all_target() 里的做法一致。
  // [解析] 区别是这里没有套展开器，所以目标角跨 ±π 时会跳变，这也是它被弃用的原因之一。
  yaw_set_in_joint = atan2f(common_term_y, common_term_x);

  // [解析] 目标向量在底盘系下的 z 分量，用来反解 pitch。
  float common_term_z =
    R_base2world_[0][2] * cp * cy + R_base2world_[1][2] * sy * cp - R_base2world_[2][2] * sp_val;

  // [解析] 安全性限幅：asinf 要求输入在 [-1, 1]。因为 R_base2world_ 是手工展开算的，
  // [解析]   浮点误差可能让它略微越界，所以先夹一下再送进 asinf，避免出 NaN。
  // 增加安全性
  if (common_term_z > 1.0f) common_term_z = 1.0f;
  if (common_term_z < -1.0f) common_term_z = -1.0f;

  // [解析] pitch = -asin(z)，负号同样来自"抬头为正"的约定。
  pitch_set_in_joint = -asinf(common_term_z);
}

// [解析] 【最常用的工具函数】把四元数翻译成人能看懂的三轴角度。
// [解析] 输出顺序再次强调：euler[0]=roll、euler[1]=pitch、euler[2]=yaw，单位 rad。
// [解析] 【公式来源】这三个式子是把旋转矩阵与四元数的对应关系（2(wy-xz) 等）代进
// [解析]   ZYX 欧拉角的标准反解公式得到的，不需要自己推导，记住"pitch 用 asin，roll/yaw 用 atan2"即可。
// [解析] 【为什么 pitch 要限幅】pitch 走的是 asinf，定义域必须是 [-1, 1]；
// [解析]   四元数有浮点误差，2(wy-xz) 可能算出 1.0000001，不夹一下就会返回 NaN。
// 四元数转欧拉角 (内旋ZYX顺序: 先yaw->再pitch->最后roll)
// 输入: q[4] = {w, x, y, z} 表示物体系到世界系的四元数
// 输出: euler[3] = {roll, pitch, yaw} 单位：rad

void Gimbal::quaternion_to_euler(const float q[4], float euler[3])
{
  // [解析] 先把四个分量取到局部变量，后面写公式更短。
  float w = q[0];
  float x = q[1];
  float y = q[2];
  float z = q[3];

  // [解析] sin(pitch) = 2(wy - xz)。这个 2 倍系数来自四元数到旋转矩阵的标准换算，
  // [解析]   是公式自带的，不是可以省略的"魔数"。
  float sin_pitch = 2.0f * (w * y - x * z);

  // [解析] 限幅到 [-1, 1]，保证下面的 asinf 不会因为浮点误差收到越界值。
  // [解析] 这是写嵌入式姿态代码的必备习惯：所有 asinf / acosf 的输入都要先夹一下。
  // Clamp for numerical safety
  if (sin_pitch > 1.0f) sin_pitch = 1.0f;
  if (sin_pitch < -1.0f) sin_pitch = -1.0f;

  // [解析] 【万向锁分支】当 |sin(pitch)| 接近 1 时 pitch 已经接近 ±90°，
  // [解析]   此时 roll 和 yaw 的旋转轴重合，数学上无法把两者分开（只有它们的和/差有意义）。
  // [解析]   处理方式：把 roll 直接置 0，把两个角的贡献全部塞进 yaw，即算出的是 yaw+roll 的合成角。
  // [解析]   阈值取 0.999999f 是因为再靠近一点 atan2 就会因分母趋零而丢失精度。
  // [解析]   【对本工程的影响】正常情况下云台 pitch 到不了 ±90°，这个分支基本走不到；
  // [解析]     但如果机械限位被撞坏或者数据异常，云台会突然"甩头"，看到这个现象先查 pitch 是否越限。
  // Gimbal lock check  一瞬间将roll+yaw 全部吸收到yaw
  if (fabsf(sin_pitch) >= 0.999999f) {
    // pitch = ±90°
    // [解析] copysignf(M_PI/2, sin_pitch) 会按 sin_pitch 的符号取 ±π/2，比自己写 if 简洁。
    euler[1] = copysignf(M_PI / 2.0f, sin_pitch);                               // pitch (beta)
    euler[0] = 0.0f;                                                            // roll (theta)
    euler[2] = atan2f(-2.0f * (x * y - w * z), 1.0f - 2.0f * (x * x + z * z));  //yaw = yaw+roll
  }
  else {
    // [解析] 常规分支，三个角的公式都是标准 ZYX 反解：
    // [解析]   pitch = asin(2(wy - xz))、roll = atan2(2(yz + wx), 1 - 2(x²+y²))、
    // [解析]   yaw   = atan2(2(xy + wz), 1 - 2(y²+z²))。
    // [解析]   atan2f(分子, 分母) 自带象限判断，所以角度范围天然是 (-π, π]。
    // Normal case
    euler[1] = asinf(sin_pitch);  // pitch (beta)
    euler[0] = atan2f(2.0f * (y * z + w * x), 1.0f - 2.0f * (x * x + y * y));
    euler[2] = atan2f(2.0f * (x * y + w * z), 1.0f - 2.0f * (y * y + z * z));
  }
}

// [解析] 和 quaternion_to_euler() 是"同一件事的两种输入形式"：这里输入 3×3 矩阵，那边输入四元数。
// [解析] 矩阵约定：R[i][j] 表示"物体系的第 j 个轴在地面系第 i 个轴上的投影"，与 update() 里算出的
// [解析]   R_base2world_ 完全一致，所以两者可以互相替代。
// [解析] 三个角的标准位置（ZYX 顺序）记法：pitch 看 R[2][0]、roll 看 R[2][1] 和 R[2][2]、
// [解析]   yaw 看 R[1][0] 和 R[0][0]。记不住就回来查这三行。
// 旋转矩阵转欧拉角 (ZYX顺序: yaw->pitch->roll)
// 输入: R[3][3] 表示物体相对地面的旋转矩阵
// 输出: euler[3] = {roll, pitch, yaw} 单位：rad
void Gimbal::rotation_matrix_to_euler(const float R[3][3], float euler[3])
{
  // [解析] pitch 的正弦值取 -R[2][0]（带负号），这是 ZYX 顺序下该矩阵元素的固定关系，
  // [解析]   和 quaternion 版本里的 2(wy - xz) 指的是同一个量，符号也一致（都是 -R[2][0] 的定义）。
  // Pitch (绕Y轴旋转)
  float sin_pitch = -R[2][0];

  // [解析] 万向锁判断，处理思路与 quaternion_to_euler() 相同：roll 置 0，角度合成进 yaw。
  // [解析] 注意这里的阈值是 0.99999f（5 个 9），而 quaternion_to_euler() 用的是 0.999999f（6 个 9），
  // [解析]   两个函数不一致。因为只要能拦住 |sin|>1 就不会算出 NaN，所以功能上没影响。
  // 检查万向锁情况
  if (fabsf(sin_pitch) >= 0.99999f) {
    // 万向锁情况
    euler[1] = copysignf(M_PI / 2.0f, sin_pitch);
    euler[0] = 0.0f;                       // 将roll设为0
    euler[2] = atan2f(-R[0][1], R[1][1]);  //
    // [解析] ↑ 万向锁下的 yaw 公式与常规分支不同（用的是 R[0][1] 和 R[1][1]），
    // [解析]   这是把 roll=0 代入后化简出来的结果，不要拿常规公式来验证它。
  }
  else {
    // [解析] 常规分支。这里没有先给 sin_pitch 限幅就调用 asinf，但因为上面的分支
    // [解析]   已经把 |sin_pitch| >= 0.99999 的情况拦掉了，所以输入必然落在定义域内，是安全的。
    euler[1] = asinf(sin_pitch);

    // [解析] Roll：atan2(R[2][1], R[2][2])，ATan2 自带象限判断，无需限幅。
    // Roll (绕X轴旋转)
    euler[0] = atan2f(R[2][1], R[2][2]);

    // [解析] Yaw：atan2(R[1][0], R[0][0])，结果范围 (-π, π]。
    // Yaw (绕Z轴旋转)
    euler[2] = atan2f(R[1][0], R[0][0]);
  }
}

// [解析] 【四元数乘法，本文件用得最多的工具】记住两句话就够用了：
// [解析]   ① 乘积 q1 ⊗ q2 表示"先做 q2 的旋转，再做 q1 的旋转"（和矩阵乘法一样，右边先作用）；
// [解析]   ② 取共轭 q* = {w, -x, -y, -z} 等于"反向旋转"，所以除法 A/B 就写成 A ⊗ B*。
// [解析] 【新手最常犯的错】把 conjugate_q1 / conjugate_q2 传反，结果是"转了两倍"或"转到反方向"，
// [解析]   而且数值看起来还挺正常（不是 NaN 也不是 0），非常难查。改这两个参数前先想清楚方向。
// [解析] 【别名安全】q_result 就算和 q1 / q2 指向同一块内存（原地运算）也没问题，
// [解析]   因为每个 x1/y1... 都已经先取到局部变量了，写入 q_result 不会污染读取。
// 四元数乘法：q_result = q1 ⊗ q2
// 输入/输出格式：q[4] = {w, x, y, z}
// 物理意义：坐标系变换的复合，先应用 q2，再应用 q1
// 例如：q_A_C = q_A_B ⊗ q_B_C（先从C到B，再从B到A）
// conjugate_q1: 是否对 q1 取共轭 (q1* = [w, -x, -y, -z])
// conjugate_q2: 是否对 q2 取共轭 (q2* = [w, -x, -y, -z])
void Gimbal::quaternion_multiply(
  const float q1[4], const float q2[4], float q_result[4], bool conjugate_q1, bool conjugate_q2)
{
  // [解析] 把"是否取共轭"用一个三元表达式就地实现：要共轭就把虚部三个分量取负。
  // [解析] 实部 w 永远不变（共轭只翻转虚部符号），所以 w1/w2 不需要判断。
  // [解析] 这样做的好处是下面写公式时不用再分 4 种情况，代码只有一份。
  // 根据 conjugate 标志决定符号
  float w1 = q1[0];
  float x1 = conjugate_q1 ? -q1[1] : q1[1];
  float y1 = conjugate_q1 ? -q1[2] : q1[2];
  float z1 = conjugate_q1 ? -q1[3] : q1[3];

  float w2 = q2[0];
  float x2 = conjugate_q2 ? -q2[1] : q2[1];
  float y2 = conjugate_q2 ? -q2[2] : q2[2];
  float z2 = conjugate_q2 ? -q2[3] : q2[3];

  // [解析] Hamilton 乘法公式展开（和复数乘法 (a+bi)(c+di) 是同一套逻辑，多了两个虚轴）。
  // [解析] 记忆方法：实部 = 实部乘积 - 三个虚部两两乘积；
  // [解析]   虚部 = "自己的实部×对方虚部 + 自己虚部×对方实部 ± 另外两个虚部的叉乘项"。
  // [解析] 这 4 行公式建议直接背下来（嵌入式姿态代码里到处都在用），不要每次现推。
  // Hamilton 四元数乘法公式
  q_result[0] = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2;  // w
  q_result[1] = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2;  // x
  q_result[2] = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2;  // y
  q_result[3] = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2;  // z
}

// [解析] 【把向量换坐标系，本文件调用次数最多的函数】理解一句话就够：
// [解析]   同一个物理向量（比如重力、底盘角速度），在不同坐标系里有不同的坐标值，
// [解析]   本函数负责把坐标值从"一个系"换算到"另一个系"，物理向量本身没变。
// [解析] 【passive vs active 的区别】原注释特别强调这是 passive rotation：
// [解析]   "坐标系转了"（passive，本函数）和"向量转了"（active）在公式上正好互为共轭，
// [解析]   两者数值结果相反，混淆的话会得到反方向的结果。
// [解析] 【conjugate_q 怎么用】若 q 表示"从 A 系到 B 系"，那不加共轭是 A→B，加了共轭(true)变成 B→A。
// [解析]   代码里大量出现 true，正是因为 q 来自 IMU（body→world），而我们需要把 world 的量换回 body。
// [解析] 【实现技巧】不真的做两次四元数乘法，而是用等价的"t = q_vec × v + w·v，再 v_out = v + 2·q_vec × t"，
// [解析]   运算量大约只有直接展开的一半，在 1ms 控制周期里这个优化是有意义的。
// 四元数坐标变换：v_out = q ⊗ v_in ⊗ q*
// 输入：q[4] = {w, x, y, z} 表示从坐标系A到坐标系B的旋转
//      v_in[3] = {x, y, z} 在坐标系B中的同一个向量的坐标
// 输出：v_out[3] = {x, y, z} 在坐标系A中的同一个向量的坐标向量
// conjugate_q: 是否对 q 取共轭（true 时使用 q* 而不是 q，相当于反向旋转）
// 注意：这是 passive rotation（坐标系变换），不是 active rotation（向量旋转）
void Gimbal::quaternion_frame_transform(
  const float q[4], const float v_in[3], float v_out[3], bool conjugate_q)
{
  // [解析] 与 quaternion_multiply() 同样的处理：要共轭就把虚部取负，实部不动。
  // [解析] 这样只要外面传 true，本函数内部就等价于用 q* 做旋转，即"反向"。
  // 根据 conjugate 标志决定四元数的符号
  float w = q[0];
  float x = conjugate_q ? -q[1] : q[1];
  float y = conjugate_q ? -q[2] : q[2];
  float z = conjugate_q ? -q[3] : q[3];

  // [解析] 把输入向量的三个分量取到局部变量，方便下面展开书写。
  // [解析] 注意 q_vec 指的是四元数的虚部 {x, y, z}，不是"四元数向量"这个新概念。
  float vx = v_in[0], vy = v_in[1], vz = v_in[2];

  // [解析] 下面三步是实现"向量坐标系变换"的高效写法，等价于 v_out = q ⊗ v_in ⊗ q*：
  // [解析]   先把向量当纯虚四元数 {0, vx, vy, vz} 参与两次乘法，再化简掉冗余项，
  // [解析]   就得到了注释里那个公式。数学等价性可以在纸上验算一次，但不验也不影响使用。
  // 优化的四元数-向量旋转公式（避免构造完整四元数）
  // v_out = v_in + 2 * cross(q_vec, cross(q_vec, v_in) + w * v_in)

  // [解析] 第一步：t = q_vec × v_in，标准的叉乘展开（和下面 cross_product() 同一个公式）。
  // 第一步：t = cross(q_vec, v_in) = q_vec × v_in
  float tx = y * vz - z * vy;
  float ty = z * vx - x * vz;
  float tz = x * vy - y * vx;

  // [解析] 第二步：再加上 w * v_in，即 t = q_vec × v_in + w · v_in。
  // [解析] w 是四元数的实部（= cos(θ/2)），这一项是旋转公式化简后剩下的标量贡献。
  // 第二步：t = t + w * v_in
  tx += w * vx;
  ty += w * vy;
  tz += w * vz;

  // [解析] 第三步：v_out = v_in + 2 * (q_vec × t)。那个 2 倍系数来自公式化简，
  // [解析]   是固定的，不要因为"看起来像凑数"就删掉它。
  // 第三步：v_out = v_in + 2 * cross(q_vec, t)
  v_out[0] = vx + 2.0f * (y * tz - z * ty);
  v_out[1] = vy + 2.0f * (z * tx - x * tz);
  v_out[2] = vz + 2.0f * (x * ty - y * tx);
}

// [解析] 三维向量叉乘，纯数学，和坐标系无关。结果向量同时垂直于 v1 和 v2，
// [解析]   长度 = |v1||v2|sin(θ)，方向服从右手定则（右手四指从 v1 弯向 v2，拇指即结果方向）。
// [解析] 【注意】叉乘不满足交换律！v1 × v2 = -(v2 × v1)，参数顺序写反结果整体取负，
// [解析]   在 calc_all_target() 的 C 项里如果顺序搞反，云台自转时会朝反方向漂。
// [解析] 若两个向量平行（或有一个是零向量），结果就是 {0,0,0}。
void Gimbal::cross_product(const float v1[3], const float v2[3], float result[3])
{
  // [解析] 下面三行就是那个"行列式展开"：
  // [解析]   result[0] = v1y*v2z - v1z*v2y  （掩盖 i 后的 2×2 行列式）
  // [解析]   result[1] = v1z*v2x - v1x*v2z  （注意中间那一项有负号，这是展开时符号交替的结果）
  // [解析]   result[2] = v1x*v2y - v1y*v2x
  // 向量叉乘公式：result = v1 × v2
  // | i    j    k   |
  // | v1x  v1y  v1z |
  // | v2x  v2y  v2z |
  result[0] = v1[1] * v2[2] - v1[2] * v2[1];
  result[1] = v1[2] * v2[0] - v1[0] * v2[2];
  result[2] = v1[0] * v2[1] - v1[1] * v2[0];
}

// [解析] 【欧拉运动学方程的正解，对应 T⁻¹ 矩阵】把"陀螺仪量到的载体系角速度"换算成
// [解析]   "yaw/pitch/roll 三轴的角速度"。为什么必须换算：陀螺仪给的是 wx/wy/wz，
// [解析]   它们是绕"载体系三个轴"的转动，而 PID 要控制的是绕 yaw/pitch/roll 轴的转动，
// [解析]   两套轴的方向不同，中间差一个与当前姿态角有关的变换矩阵。
// [解析] 【三行的含义】
// [解析]   vroll  —— roll 轴几乎和载体系 x 轴重合，所以以 wx 为主，另外两项是 pitch/yaw 转动
// [解析]             在 roll 轴上"漏过来"的耦合量（含 tan(pitch)）。
// [解析]   vpitch —— 只有两项，是 wy/wz 在 pitch 轴上的正交分解，没有奇点。
// [解析]   vyaw   —— 分母上的 cos(pitch) 就是万向锁的来源：pitch 接近 ±90° 时结果发散。
// [解析] 【输入】omiga_in_body[3] = {wx, wy, wz}，单位 rad/s，通常直接传 IMU 的 w。
// [解析]        roll/pitch/yaw 是"相对于哪个系的欧拉角"——传相对的角就得到相对该系的角速率。
// [解析] 【输出】vroll/vpitch/vyaw 是引用出参（不是返回值），单位 rad/s，调用者要提供三个 float 变量。
// [解析] 【疑问点】形参 yaw 在这个函数里完全没被用到（公式本身也不需要），它只是为了和逆函数
// [解析]   transform_euler_rates_2_omiga_in_body() 的参数表保持对称。
// [解析]   而且上面那个逆函数里就没有写 (void)yaw，两边风格不一致。
void Gimbal::transform_omiga_in_body_2_euler_rates(
  const float omiga_in_body[3], const float roll, const float pitch, const float yaw, float & vroll,
  float & vpitch, float & vyaw)
{
  // [解析] vroll：主项是 wx，加上 wy、wz 通过 tan(pitch) 和 sin/cos(roll) 耦合过来的部分。
  // [解析]   用 std::sin/std::tan 而不是 sinf/tanf，说明作者两种风格混用（功能相同，只是精度/速度略有差异）。
  vroll = omiga_in_body[0] + omiga_in_body[1] * std::sin(roll) * std::tan(pitch) +
          omiga_in_body[2] * std::cos(roll) * std::tan(pitch);

  // [解析] vpitch：没有奇点，随便什么姿态都能算。这就是为什么云台 pitch 轴永远不会因为
  // [解析]   "解算出 NaN"而失控 —— 会失控的是 vroll / vyaw 那两路。
  vpitch = omiga_in_body[1] * std::cos(roll) - omiga_in_body[2] * std::sin(roll);

  // [解析] vyaw：除以 cos(pitch)。pitch → ±90° 时 cos → 0，vyaw → ±∞。
  // [解析]   所以云台的机械限位必须留出余量，绝不能允许枪口竖直朝上/朝下。
  vyaw = omiga_in_body[1] * std::sin(roll) / std::cos(pitch) +
         omiga_in_body[2] * std::cos(roll) / std::cos(pitch);
}

// [解析] 【上一条的逆运算，对应 T 矩阵（雅可比矩阵）】已知三个欧拉角速率，反求载体系角速度。
// [解析] 【为什么它比正运算讨人喜欢】整个式子全是乘法，没有任何除法，
// [解析]   所以不存在万向锁奇点，pitch 无论多大都能算出有限结果。能用它就别用 T⁻¹。
// [解析] (void)yaw; 是 C++ 里"我知道这个参数没用，请编译器别报警告"的标准写法，
// [解析]   等价于把变量"用掉"一次。看到它就说明后面的公式确实不需要 yaw，不是漏写了。
// [解析] 输入：vroll/vpitch/vyaw（rad/s）+ roll/pitch/yaw（rad）；输出：omiga_in_body[3]（rad/s）。
void Gimbal::transform_euler_rates_2_omiga_in_body(
  const float vroll, const float vpitch, const float vyaw, const float roll, const float pitch,
  const float yaw, float omiga_in_body[3])
{
  (void)yaw;  // yaw 在该变换中不参与，防止未使用警告

  // [解析] wx = vroll - vyaw*sin(pitch)：roll 轴的变化率，减去 yaw 转动在 x 轴上的投影。
  // [解析]   这就是"绕竖直轴自转时，云台 x 轴也会跟着改变朝向"的数学体现。
  omiga_in_body[0] = vroll - vyaw * std::sin(pitch);

  // [解析] wy = vpitch*cos(roll) + vyaw*sin(roll)*cos(pitch)。
  omiga_in_body[1] = vpitch * std::cos(roll) + vyaw * std::sin(roll) * std::cos(pitch);

  // [解析] wz = -vpitch*sin(roll) + vyaw*cos(roll)*cos(pitch)。
  // [解析]   注意 wy / wz 里都有 cos(pitch) 因子：pitch 越大，yaw 的转动在水平面上
  // [解析]   "投影"到载体系 y/z 轴的分量越小，这符合直觉（枪口越竖直，自转越像绕自己转）。
  omiga_in_body[2] = -vpitch * std::sin(roll) + vyaw * std::cos(roll) * std::cos(pitch);
}

// [解析] 【Ṫ·Ė 项，也就是 calc_all_target() 里的 A 项和 D 项】含义是
// [解析]   "变换矩阵对时间求导，再乘上欧拉角速率"，得到的是姿态变化过程中产生的耦合角加速度。
// [解析] 【为什么需要它】如果不考虑这一项，云台在快速转动时会"算出来的目标加速度偏小"，
// [解析]   表现为：大范围甩头时枪口在终点附近过冲或发飘。
// [解析] 【怎么读这个公式】每一项都是"两个速率相乘"（如 vpitch*vyaw），所以量纲天然是 rad²/s²，
// [解析]   再乘上一个由角度决定的系数就得到 rad/s²。系数里没有别的魔数，都是 sin/cos 的乘积。
// [解析] 注意本函数的角速度用 cosf/sinf（float 版），而上面两个逆/正变换用的是 std::sin（double 版），
// [解析]   对结果无影响，只是历史风格不统一。
// [解析] 输入：roll/pitch/yaw（rad）+ vroll/vpitch/vyaw（rad/s）；输出：result[3]（rad/s²）。
void Gimbal::Transform_matrix_rates_multipy_Euler_rates(
  const float roll, const float pitch, const float yaw, const float vroll, const float vpitch,
  const float vyaw, float result[3])
{
  (void)yaw;  // yaw 在该变换中不参与，防止未使用警告
  // [解析] result[0] 是 roll 方向的分量：只有 vpitch*vyaw 这一项，系数 -cos(pitch)。
  // [解析]   公式里那些多余的圆括号（如 (vyaw)）是作者从推导稿复制过来的书写习惯，不影响计算。
  result[0] = -cosf(pitch) * vpitch * (vyaw);
  // [解析] result[1]/result[2] 是 pitch、yaw 方向的分量，结构一样：
  // [解析]   一个 vroll*vpitch 的项，加一个由 vroll / vpitch 与 vyaw 相乘的复合项。
  // [解析]   系数里的 cos/sin(roll)、cos/sin(pitch) 描述了三个轴之间的耦合强度。
  result[1] = -sinf(roll) * vroll * (vpitch) +
              (cosf(roll) * cosf(pitch) * vroll - sinf(roll) * sinf(pitch) * vpitch) * (vyaw);
  result[2] = -cosf(roll) * vroll * (vpitch) +
              (-cosf(roll) * sinf(pitch) * vpitch - sinf(roll) * cosf(pitch) * vroll) * (vyaw);
}

// [解析] 【欧拉角 → 四元数】本文件里唯一做这个转换的函数，构造函数就靠它把安装角变成四元数。
// [解析] 参数（都是 rad）：yaw 绕世界 Z 轴、pitch 绕"转完 yaw 之后的新 Y 轴"、roll 绕"再之后的新 X 轴"，
// [解析]   这就是 ZYX 内旋顺序，和 quaternion_to_euler() 的反解约定严格对应，两者可以来回转换不丢信息。
// [解析] 【四行公式的含义】先把三个单轴旋转各自写成半角四元数，再按 q_yaw ⊗ q_pitch ⊗ q_roll
// [解析]   乘起来并展开，就得到这四行。系数 cy/cp/cr 是 cos(半角)，sy/sp/sr 是 sin(半角)。
// [解析] 【和 update_q_chassis2world() 里手写的那组对比】那边 x/y/z 三个分量全部取了负号，
// [解析]   即那边得到的是本函数结果的共轭（旋转方向相反）。抄公式时千万别把两边混着用。
// 欧拉角(ZYX顺序: yaw->pitch->roll)转四元数
// 输入: yaw, pitch, roll 的含义为: 机体系从与世界系重合开始, 先绕世界Z轴旋转yaw, 再绕新Y轴旋转pitch, 最后绕新X轴旋转roll
// 输出: q[4] = {w, x, y, z}, 表示将机体系下向量变换到世界系下的四元数, 即 v_world = q ⊗ v_body ⊗ q*
void Gimbal::euler_zyx_to_quaternion(float yaw, float pitch, float roll, float q[4])
{
  // [解析] 半角：四元数编码旋转用的是 θ/2，这一步漏掉（直接用整角）是最经典的错误，
  // [解析]   表现为"转 90° 却得到 180° 的效果"，而且低角度时看起来又像是小误差，很难发现。
  // 半角
  float half_yaw = 0.5f * yaw;
  float half_pitch = 0.5f * pitch;
  float half_roll = 0.5f * roll;

  // [解析] 六个三角函数值各算一次，下面四行公式反复使用。
  float cy = cosf(half_yaw);
  float sy = sinf(half_yaw);
  float cp = cosf(half_pitch);
  float sp = sinf(half_pitch);
  float cr = cosf(half_roll);
  float sr = sinf(half_roll);

  // [解析] q[0]=w 是实部，其余三个是虚部。输出顺序固定 {w, x, y, z}，
  // [解析]   与 Mahony、与 gimbal 里的其它所有函数保持一致。
  // ZYX 内旋 (先yaw->再pitch->最后roll) 对应的四元数, 保持与quaternion_to_euler一致
  q[0] = cy * cp * cr + sy * sp * sr;  // w
  q[1] = cy * cp * sr - sy * sp * cr;  // x
  q[2] = cy * sp * cr + sy * cp * sr;  // y
  q[3] = sy * cp * cr - cy * sp * sr;  // z

  // [解析] 归一化：理论上这个公式算出来的模长就是 1，但浮点运算有误差，
  // [解析]   而且如果调用者传了非法角度（比如 NaN），这里也能把结果拉回单位四元数。
  // [解析] 1e-12f 阈值 + else 兜底，与 update_q_chassis2world() 的做法完全一致。
  // 数值安全: 归一化 q，防止由于数值误差导致模长偏离1
  float norm = sqrtf(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
  if (norm > 1e-12f) {
    float inv = 1.0f / norm;
    q[0] *= inv;
    q[1] *= inv;
    q[2] *= inv;
    q[3] *= inv;
  }
  else {
    // [解析] 极端情况（norm 接近 0）直接返回单位四元数 {1,0,0,0}，表示"不旋转"。
    // 极端异常兜底，退回单位四元数
    q[0] = 1.0f;
    q[1] = q[2] = q[3] = 0.0f;
  }
}

// [解析] 【双 IMU 专用：矫正底盘 IMU 的姿态】把底盘 IMU 的读数"绕底盘 X 轴转 install_roll_ 角"，
// [解析]   使其 roll 参考面与云台 IMU 对齐，从而消除两个 IMU 安装角度不同带来的固定偏差。
// [解析] 【为什么是左乘 install_roll_q_ 的共轭】底盘 IMU 量到的是"底盘安装面"的姿态，
// [解析]   我们要的是"车辆底盘真实姿态"，两者之间差一个安装旋转。用共轭左乘即相当于
// [解析]   把这个安装旋转从测量结果里"除掉"（四元数版的"除法"）。
// [解析] 【为什么先备份到临时变量】因为 chassis_imu.q 既是输入又是输出（原地修改），
// [解析]   如果直接把 &chassis_imu.q[0] 当输入传给 quaternion_multiply，
// [解析]   虽然该函数对别名是安全的（先取值再写），但作者选择显式备份，语义更清晰、也更保险。
// [解析] 副作用：会就地修改 chassis_imu.q（姿态）和 chassis_imu.w（角速度）。
// [解析]   注意它会同时改变底盘 IMU 内部积分的参考系，所以整个程序里只应该在
// [解析]   双 IMU 流程的入口处调用一次（update_all_dual 开头），不要重复调用。
void Gimbal::correct_chassis_imu(sp::Mahony & chassis_imu)
{
  // [解析] 两个临时数组：q_temporary 存底盘姿态四元数，w_temporary 存载体系角速度。
  // [解析]   它们是栈上的局部变量（各占 16 / 12 字节），每帧重新分配，开销可以忽略。
  float q_temporary[4];
  float w_temporary[3];
  // [解析] 逐个分量拷贝。这里没有用 for 循环或 memcpy，是为了避免依赖 <cstring> 且让编译器更容易优化。
  q_temporary[0] = chassis_imu.q[0];
  q_temporary[1] = chassis_imu.q[1];
  q_temporary[2] = chassis_imu.q[2];
  q_temporary[3] = chassis_imu.q[3];
  w_temporary[0] = chassis_imu.w[0];
  w_temporary[1] = chassis_imu.w[1];
  w_temporary[2] = chassis_imu.w[2];
  // [解析] 矫正面姿态：install_roll_q_* ⊗ q_chassis（共轭左乘），结果直接写回 chassis_imu.q。
  // [解析] 参数 true/false 表示"对第一个参数取共轭"，即使用 install_roll_q_ 的逆旋转。
  quaternion_multiply(this->install_roll_q_, q_temporary, chassis_imu.q, true, false);
  // [解析] 角速度也要一起矫正：角速度是矢量，必须用同样的旋转把它从"安装面坐标系"
  // [解析]   换到"真实底盘坐标系"，否则姿态对了但角速度还带着安装偏差，速度环会有静态误差。
  quaternion_frame_transform(this->install_roll_q_, w_temporary, chassis_imu.w, true);
}

}  // namespace sp
