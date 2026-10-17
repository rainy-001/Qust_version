// [解析] gimbal.hpp 的全部实现，本质是一堆四元数代数 + 坐标系变换。
// [解析] 前半段是两套更新流程（单/双 IMU）和目标解算，后半段是纯数学小工具（乘 / 换系 / 叉乘 / 欧拉角互转）。
// [解析] 读序建议：先看后半段几个工具函数，再看 update_q_chassis2world()，最后啃 calc_all_target()。
// [解析] 约定：四元数 {w,x,y,z}、欧拉角 {roll,pitch,yaw}（下标不同）；角度 rad、角速度 rad/s。

#include "gimbal.hpp"

#include <cmath>

#include "tools/math_tools/math_tools.hpp"

namespace sp
{

// [解析] reverse_* 的 bool 在这里换算成 ±1.0f 存进 sign_yaw_/sign_pitch_，后面只需乘法就能翻转方向。
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
  // [解析] 只清了 4 个 joint 角和 3 个 base 姿态角；yaw_rel / pitch_rel 等要等第一次 update_* 才有值。
  this->yaw_fdb_in_joint = 0.0f;
  this->pitch_fdb_in_joint = 0.0f;
  this->yaw_set_in_joint = 0.0f;
  this->pitch_set_in_joint = 0.0f;
  this->base_yaw_in_world = 0.0f;
  this->base_pitch_in_world = 0.0f;
  this->base_roll_in_world = 0.0f;

  // 初始化为单位矩阵
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      this->R_base2world_[i][j] = (i == j) ? 1.0f : 0.0f;
    }
  }
  // [解析] 安装横滚角提前转成四元数存好（correct_chassis_imu 每周期要用两次），省掉重复的三角函数。
  euler_zyx_to_quaternion(0.0f, 0.0f, install_roll_, this->install_roll_q_);
}
// [解析] 单 IMU 主流程：反推底盘姿态 → 转欧拉角 → 差分求角加速度 → 算相对角速度并滤波 → 重力补偿。
// [解析] 参数写成 const float & 只是风格（float 传值更快），不要写出依赖"引用会被修改"的代码。
void Gimbal::update_all_single(
  const sp::Mahony & gimbal_imu, const float & yaw_angle, const float & pitch_angle)
{
  //更新电机角度和底盘姿态及角速度
  update_q_chassis2world(gimbal_imu, yaw_angle, pitch_angle, this->gimbal_roll0_);
  // [解析] 转成 {roll,pitch,yaw} 存起来，上层（视觉、翻车判断）直接读角度，不用自己再转一次。
  //转换出底盘姿态欧拉角
  quaternion_to_euler(this->q_chassis2world, this->chassis_euler_in_world);
  // [解析] dq[0] 是残差不是角速度，真正的 wx/wy/wz 在 dq[1..3]（下标差一位，极易写错）。
  //更新底盘的角加速度,并换到云台系表示
  w_last_chassis_in_worldframe[0] = w_chassis_in_worldframe[0];
  w_last_chassis_in_worldframe[1] = w_chassis_in_worldframe[1];
  w_last_chassis_in_worldframe[2] = w_chassis_in_worldframe[2];
  w_chassis_in_worldframe[0] = this->dq[1];
  w_chassis_in_worldframe[1] = this->dq[2];
  w_chassis_in_worldframe[2] = this->dq[3];
  // [解析] diff_vec3 就是 (新值-旧值)/dt 的差分；dt=1ms 会把噪声放大 1000 倍，所以它只用于前馈。
  sp::diff_vec3(
    w_chassis_in_worldframe, w_last_chassis_in_worldframe, acc_chassis_in_worldframe, dt_);
  quaternion_frame_transform(
    gimbal_imu.q, acc_chassis_in_worldframe, acc_chassis_in_gimbalframe, true);

  // [解析] IMU 量到的是"云台相对地面"的角速度，减去底盘在云台系下的分量，才是云台相对底盘的角速度。
  //底盘角速度换到云台系并更新云台相对于底盘的欧拉角变化率,可用作电机角速度反馈值
  quaternion_frame_transform(gimbal_imu.q, w_chassis_in_worldframe, w_chassis_in_gimbalframe, true);
  w_relative[0] = gimbal_imu.w[0] - w_chassis_in_gimbalframe[0];
  w_relative[1] = gimbal_imu.w[1] - w_chassis_in_gimbalframe[1];
  w_relative[2] = gimbal_imu.w[2] - w_chassis_in_gimbalframe[2];
  // [解析] w_relative 是载体系角速度（三轴耦合），必须经 T⁻¹ 才能变成能控制的 yaw/pitch/roll 角速率。
  sp::Gimbal::transform_omiga_in_body_2_euler_rates(
    w_relative, roll_rel, pitch_rel, yaw_rel, roll_relative_speed, pitch_relative_speed,
    yaw_relative_speed);
  // [解析] 滤波器固定用法：update(原始值) 之后必须手动把 .out 抄回变量（下面 6 行都是这个套路）。
  // 对电机反馈速度进行滤波
  roll_relative_speed_filter.update(roll_relative_speed);
  roll_relative_speed = roll_relative_speed_filter.out;
  pitch_relative_speed_filter.update(pitch_relative_speed);
  pitch_relative_speed = pitch_relative_speed_filter.out;
  yaw_relative_speed_filter.update(yaw_relative_speed);
  yaw_relative_speed = yaw_relative_speed_filter.out;

  // [解析] q_chassis2world 表示"底盘系→地面系"，所以换回底盘系同样要用共轭（conjugate=true）。
  //底盘角速度换到底盘系w_chassis_in_chassisframe可用作小陀螺补偿
  quaternion_frame_transform(
    q_chassis2world, w_chassis_in_worldframe, w_chassis_in_chassisframe, true);

  // [解析] 地面系重力方向是 +z（z 轴朝天）；转到云台系后 k_torque[2] 就是 pitch 轴上的重力投影。
  //重力补偿
  static float g[3] = {0.0f, 0.0f, 1.0f};  //地面系下重力方向
  sp::Gimbal::quaternion_frame_transform(
    gimbal_imu.q, g, this->k_torque,
    true);  //将重力方向转换到云台系下,k_torque[2]可以用来做重力补偿
}

// [解析] 双 IMU 流程：结构与 update_all_single 相同，区别只在底盘姿态/角速度的来源。
// [解析] 底盘 IMU 直接量出底盘姿态，不用反推、没有误差累积，但要多标定一个 install_roll。
void Gimbal::update_all_dual(const sp::Mahony & gimbal_imu, sp::Mahony & chassis_imu)
{
  //使用底盘安装roll去矫正chassis_imu的姿态,使其与gimbal_imu在roll轴上对齐,从而获得更准确的底盘姿态
  correct_chassis_imu(chassis_imu);
  //更新等效电机角度
  update_q_gimbal2chassis(gimbal_imu, chassis_imu);
  // [解析] q 的约定是"底盘系 → 地面系"：底盘 IMU 的 w 换到地面系用 conjugate=false，换回云台系用 true。
  //将底盘角速度换到地面系,并微分获得角加速度,再换到云台系表示底盘角加速度
  //转换出底盘姿态欧拉角
  quaternion_to_euler(this->q_chassis2world, this->chassis_euler_in_world);

  w_last_chassis_in_worldframe[0] = w_chassis_in_worldframe[0];
  w_last_chassis_in_worldframe[1] = w_chassis_in_worldframe[1];
  w_last_chassis_in_worldframe[2] = w_chassis_in_worldframe[2];
  quaternion_frame_transform(chassis_imu.q, chassis_imu.w, w_chassis_in_worldframe, false);
  sp::diff_vec3(
    w_chassis_in_worldframe, w_last_chassis_in_worldframe, acc_chassis_in_worldframe, dt_);
  quaternion_frame_transform(
    gimbal_imu.q, acc_chassis_in_worldframe, acc_chassis_in_gimbalframe, true);

  // [解析] 以下到函数结束与 update_all_single() 的后半段逐字相同，含义看那边的注释。
  //底盘角速度换到云台系并更新云台相对于底盘的欧拉角变化率,可用作电机角速度反馈值
  quaternion_frame_transform(gimbal_imu.q, w_chassis_in_worldframe, w_chassis_in_gimbalframe, true);
  w_relative[0] = gimbal_imu.w[0] - w_chassis_in_gimbalframe[0];
  w_relative[1] = gimbal_imu.w[1] - w_chassis_in_gimbalframe[1];
  w_relative[2] = gimbal_imu.w[2] - w_chassis_in_gimbalframe[2];
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

  //底盘角速度换到底盘系w_chassis_in_chassisframe可用作小陀螺补偿
  quaternion_frame_transform(
    q_chassis2world, w_chassis_in_worldframe, w_chassis_in_chassisframe, true);

  //重力补偿
  static float g[3] = {0.0f, 0.0f, 1.0f};  //地面系下重力方向
  sp::Gimbal::quaternion_frame_transform(
    gimbal_imu.q, g, this->k_torque,
    true);  //将重力方向转换到云台系下,k_torque[2]可以用来做重力补偿
}

// [解析] 目标解算分三段：先由世界系目标角反解出"相对底盘"的目标角度（电机要转到的位置），
// [解析] 再算目标角速度（期望速度 - 底盘牵连速度），最后算目标角加速度 T⁻¹·(A-B-C-D) 并投影到电机轴。
// [解析] 公式看着吓人，但每步都只是坐标变换；调试时优先怀疑输入量（目标角、底盘角速度）而不是公式。
void Gimbal::calc_all_target(
  const sp::Mahony & gimbal_imu, float yaw_set_in_world, float pitch_set_in_world,
  float vyaw_set_in_world, float vpitch_set_in_world, float acc_yaw_set_in_world,
  float acc_pitch_set_in_world)
{
  // [解析] 目标向量：z 分量取 -sin(pitch)，因为本工程 pitch 抬头为正而地面系 z 轴朝天；
  // [解析] 也因为这个负号，下面反解 pitch 时写成 asinf(-z)。
  gun_target_vector[0] = cosf(yaw_set_in_world) * cosf(pitch_set_in_world);
  gun_target_vector[1] = sinf(yaw_set_in_world) * cosf(pitch_set_in_world);
  gun_target_vector[2] = -sinf(pitch_set_in_world);
  // [解析] 换到"底盘系"的原因：电机只感知自己与底盘之间的角度，目标必须先落到这个坐标系里。
  quaternion_frame_transform(
    q_chassis2world, gun_target_vector, gun_target_vector_in_chassisframe, true);

  // [解析] 外面套角度展开器：否则目标角从 +179° 跳到 -179° 时，PID 会让云台倒着转 358°。
  //解算云台相对于底盘目标欧拉角
  yaw_target_relative_angle = this->yaw_target_relative_angle_unwrapper.update(
    atan2f(gun_target_vector_in_chassisframe[1], gun_target_vector_in_chassisframe[0]));

  // [解析] 这里没有像 quaternion_to_euler() 那样先限幅；向量是单位向量时安全，q 未归一化时可能出 NaN。
  pitch_target_relative_angle = asinf(-gun_target_vector_in_chassisframe[2]);

  // [解析] 除零保护：pitch_rel 接近 ±90° 时下面那些 1/cos(pitch_rel) 会炸成 ±∞，电机瞬间满电流。
  // [解析] 代价是提前返回——这一帧只更新了目标角度，速度和加速度还停在上一帧；机械限位正常时不会触发。
  if (fabs(cosf(this->pitch_rel)) < 1e-5) {
    return;
    //防止除零爆炸,只更新目标角度,冻结其他目标值沿用上一帧
  }
  // [解析] 目标相对角速度 = 期望的世界系角速度 - 底盘转动带来的牵连速度，再按 roll 差分解到两个轴上。
  // [解析] 式子里一堆 0.0f * cos(...) 是矩阵手工展开的占位项（本来就该是 0），可以直接当 0 看。
  //解算云台相对于底盘目标欧拉角变化率
  pitch_target_relative_speed =
    cosf(this->roll_rel - gimbal_imu.roll) *
      (0.0f - this->w_chassis_in_gimbalframe[1] * cosf(gimbal_imu.roll) +
       this->w_chassis_in_gimbalframe[2] * sinf(gimbal_imu.roll)) +
    sinf(this->roll_rel - gimbal_imu.roll) *
      (this->w_chassis_in_gimbalframe[2] * cosf(gimbal_imu.roll) - 0.0f * cosf(gimbal_imu.pitch) +
       this->w_chassis_in_gimbalframe[1] * sinf(gimbal_imu.roll));
  // [解析] 只滤波"由底盘运动引起的那部分"，视觉给的期望速度加在滤波之后，瞄准不会有延迟。
  pitch_target_relative_speed_filter.update(pitch_target_relative_speed);
  pitch_target_relative_speed = pitch_target_relative_speed_filter.out;
  // [解析] 加号后面：把期望的 vpitch / vyaw 分解到 pitch 轴；yaw 速度也会漏到 pitch 轴，这就是交叉项。
  pitch_target_relative_speed +=
    cosf(this->roll_rel - gimbal_imu.roll) *
      (vpitch_set_in_world - 0.0f * cosf(gimbal_imu.roll) + 0.0f * sinf(gimbal_imu.roll)) +
    sinf(this->roll_rel - gimbal_imu.roll) *
      (0.0f * cosf(gimbal_imu.roll) - vyaw_set_in_world * cosf(gimbal_imu.pitch) +
       0.0f * sinf(gimbal_imu.roll));

  // [解析] yaw 轴多除一个 cos(pitch_rel)：pitch 越大，同样的 yaw 角速度对应的转角变化越大。
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
  yaw_target_relative_speed +=
    (sinf(this->roll_rel - gimbal_imu.roll) *
     (vpitch_set_in_world - 0.0f * cosf(gimbal_imu.roll) + 0.0f * sinf(gimbal_imu.roll))) /
      cosf(this->pitch_rel) -
    (cosf(this->roll_rel - gimbal_imu.roll) *
     (0.0f * cosf(gimbal_imu.roll) - vyaw_set_in_world * cosf(gimbal_imu.pitch) +
      0.0f * sinf(gimbal_imu.roll))) /
      cosf(this->pitch_rel);
  // [解析] 下面一大段注释掉的代码是另一种策略（先合并视觉值再滤波），已弃用，读代码可以直接跳过。
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

  // [解析] 第三段：目标角加速度 = T⁻¹·(A - B - C - D)，四项都是"两个速率相乘"的耦合项，量纲 rad/s²。
  // [解析] 减号的含义是把底盘自身的加速度(B)和转动耦合(C、D)扣掉，不该让云台电机去追。
  //计算A
  sp::Gimbal::Transform_matrix_rates_multipy_Euler_rates(
    gimbal_imu.roll, gimbal_imu.pitch, gimbal_imu.yaw, gimbal_imu.vroll, gimbal_imu.vpitch,
    gimbal_imu.vyaw, A_Tdot_Edot);

  // [解析] B 只是把 update_all_* 里算好的角加速度抄一份，命名保持与推导稿一致。
  //计算B
  B_acc_chassis_in_gimbalframe[0] = acc_chassis_in_gimbalframe[0];
  B_acc_chassis_in_gimbalframe[1] = acc_chassis_in_gimbalframe[1];
  B_acc_chassis_in_gimbalframe[2] = acc_chassis_in_gimbalframe[2];

  //计算C
  sp::Gimbal::cross_product(this->w_chassis_in_gimbalframe, this->w_relative, C_cross_product);

  // [解析] D 和 A 是同一个函数，差别是这里代入"相对底盘"的欧拉角与角速率。
  //计算D
  sp::Gimbal::Transform_matrix_rates_multipy_Euler_rates(
    this->roll_rel, this->pitch_rel, this->yaw_rel, roll_relative_speed, pitch_relative_speed,
    yaw_relative_speed, D_Jdot_E_Motor_dot);

  //合起来作为F,简化运算
  F[0] =
    A_Tdot_Edot[0] - B_acc_chassis_in_gimbalframe[0] - C_cross_product[0] - D_Jdot_E_Motor_dot[0];
  F[1] =
    A_Tdot_Edot[1] - B_acc_chassis_in_gimbalframe[1] - C_cross_product[1] - D_Jdot_E_Motor_dot[1];
  F[2] =
    A_Tdot_Edot[2] - B_acc_chassis_in_gimbalframe[2] - C_cross_product[2] - D_Jdot_E_Motor_dot[2];

  // [解析] 注意这里代入的是云台相对世界的欧拉角（不是 *_rel），而 B/C/D 用的是相对量，参考系不一致。
  //乘以雅各比矩阵T的逆
  sp::Gimbal::transform_omiga_in_body_2_euler_rates(
    F, gimbal_imu.roll, gimbal_imu.pitch, gimbal_imu.yaw, Final[0], Final[1], Final[2]);

  //给Final加上视觉发来的欧拉角期望加速度
  //后续只考虑后两个分量等价于乘了一个S矩阵
  // Final[1] += acc_pitch_set_in_world;
  // Final[2] += acc_yaw_set_in_world;

  // [解析] 最后把 Final 的 pitch/yaw 分量投影到电机轴：差一个 roll 相关的旋转和 cos(pitch_rel) 的缩放。
  // [解析] roll 轴由机械结构固定，所以最终只输出 pitch/yaw 两路目标加速度。
  //优先滤波再加视觉值
  //千呼万唤始出来兄弟们
  //对这个Final的后两个向量求逆就可以了!!!!!!!!!!!
  pitch_target_relative_acc =
    Final[1] * cosf(this->roll_rel - gimbal_imu.roll) -
    Final[2] * cosf(gimbal_imu.pitch) * sinf(this->roll_rel - gimbal_imu.roll);
  pitch_motor_target_acc_filter.update(pitch_target_relative_acc);
  pitch_target_relative_acc = pitch_motor_target_acc_filter.out;

  yaw_target_relative_acc =
    (Final[1] * sinf(this->roll_rel - gimbal_imu.roll) +
     Final[2] * cosf(gimbal_imu.pitch) * cosf(this->roll_rel - gimbal_imu.roll)) /
    cosf(this->pitch_rel);
  yaw_motor_target_acc_filter.update(yaw_target_relative_acc);
  yaw_target_relative_acc = yaw_motor_target_acc_filter.out;

  // [解析] 视觉期望加速度加在滤波之后：视觉值本身够平滑，不需要再被滤波器拖慢。
  pitch_target_relative_acc +=
    acc_pitch_set_in_world * cosf(this->roll_rel - gimbal_imu.roll) -
    acc_yaw_set_in_world * cosf(gimbal_imu.pitch) * sinf(this->roll_rel - gimbal_imu.roll);
  // [解析] 这里又把加过视觉值的结果送进同一个滤波器滤了一遍，而 yaw 轴没有，两轴前馈不一致，保持原样。
  pitch_motor_target_acc_filter.update(pitch_target_relative_acc);
  pitch_target_relative_acc = pitch_motor_target_acc_filter.out;

  yaw_target_relative_acc +=
    (acc_pitch_set_in_world * sinf(this->roll_rel - gimbal_imu.roll) +
     acc_yaw_set_in_world * cosf(gimbal_imu.pitch) * cosf(this->roll_rel - gimbal_imu.roll)) /
    cosf(this->pitch_rel);
}

// [解析] 已弃用的旧版解算：用旋转矩阵而不是四元数，不做角度展开、也不更新 dq 和相对速度。
// [解析] 保留它主要是为了和新版互相验证；新代码请用 update_q_chassis2world()。
void Gimbal::update(
  const sp::Mahony & gimbal_imu, const float & yaw_angle, const float & pitch_angle)
{
  // [解析] 编码器绝对值减零点、再乘符号位，最后 limit_angle 折到 (-π,π]，这就是"相对码盘"的角度。
  //云台相对码盘值计算（正方向使用云台系右手定则）
  float yaw_relative_angle = sign_yaw_ * sp::limit_angle(yaw_angle - yaw0_);
  float pitch_relative_angle = sign_pitch_ * sp::limit_angle(pitch_angle - pitch0_);

  // [解析] 用的是已弃用的 filter0（alpha 固定 1.0f，等于没滤），留着只为不改动历史行为。
  // 使用低通滤波器对角度进行滤波
  yaw_relative_angle_filter0.update(yaw_relative_angle);
  pitch_relative_angle_filter0.update(pitch_relative_angle);

  yaw_relative_angle = yaw_relative_angle_filter0.out;
  pitch_relative_angle = pitch_relative_angle_filter0.out;

  yaw_fdb_in_joint = yaw_relative_angle;
  pitch_fdb_in_joint = pitch_relative_angle;

  // [解析] 先把所有 sin/cos 提到局部变量：3×3 矩阵展开后每个元素都要反复用到它们。
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

  // [解析] 下面 9 行是手工展开的 R_base2world_ = R(云台→世界)·R(底盘→云台)；行=世界系轴，列=底盘系轴。
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

  // [解析] 矩阵反解欧拉角：yaw 看第一列、pitch 看 R[2][0]、roll 看 R[2][1] 与 R[2][2]。
  base_yaw_in_world = atan2f(R_base2world_[1][0], R_base2world_[0][0]);
  base_pitch_in_world = asinf(-R_base2world_[2][0]);  // 注意：这里可能需要防 NaN 处理

  // [解析] asinf 定义域是 [-1,1]，矩阵是算出来的、浮点误差可能越界；这里是先算再覆盖，写法别扭但有效。
  // 增加安全性：
  if (R_base2world_[2][0] > 1.0f)
    base_pitch_in_world = -sp::SP_PI / 2;
  else if (R_base2world_[2][0] < -1.0f)
    base_pitch_in_world = sp::SP_PI / 2;

  base_roll_in_world = atan2f(R_base2world_[2][1], R_base2world_[2][2]);
};

// [解析] 单 IMU 核心：q_W_C = q_W_G ⊗ q_G_C —— 云台 IMU 给"云台→世界"，两个编码器角度给"底盘→云台"，
// [解析] 串起来就是"底盘→世界"。能这么做的前提是云台与底盘刚性相连，这两个自由度全都量得出来。
// [解析] Step 0 的顺序不能反：yaw 要先 limit_angle 再送进展开器；pitch 有机械限位所以只折一次。
// [解析] 结尾用 q_now ⊗ q_last* 求底盘角速度，dq 不是单位四元数，绝对不能归一化。
void Gimbal::update_q_chassis2world(
  const sp::Mahony & gimbal_imu, const float & yaw_angle, const float & pitch_angle,
  const float & roll0_)
{
  /* ------------------------------------------------
   * Step 0: 编码器角度 → 云台相对底盘欧拉角
   * ------------------------------------------------ */
  yaw_rel = sign_yaw_ *
            this->yaw_unwrapper_.update(
              sp::limit_angle(yaw_angle - yaw0_));  //这里要展开limit_angle,否则带来的跳变会让w炸掉
  pitch_rel =
    sign_pitch_ * sp::limit_angle(pitch_angle - pitch0_);  //限位原因无所谓不需要展开limit_angle
  roll_rel = roll0_;

  // 角度低通滤波（只对可控轴）
  yaw_relative_angle_filter.update(yaw_rel);
  pitch_relative_angle_filter.update(pitch_rel);

  yaw_rel = yaw_relative_angle_filter.out;
  pitch_rel = pitch_relative_angle_filter.out;

  /* ------------------------------------------------
   * Step 1: 半角
   * ------------------------------------------------ */
  // [解析] 四元数编码旋转用的是半角（q = {cos(θ/2), axis·sin(θ/2)}），所以下面 sin/cos 的输入都是 0.5×角度。
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
  // [解析] x_GC/y_GC/z_GC 全带负号：这组手写展开等于 euler_zyx_to_quaternion(yaw_rel, ...) 的共轭。
  // [解析] 两个函数对同一组欧拉角给出互逆的旋转方向，抄公式时千万别把两边的符号混用。
  float w_GC = (cy * cp * cr + sy * sp * sr);
  float x_GC = -(cy * cp * sr - sy * sp * cr);
  float y_GC = -(cy * sp * cr + sy * cp * sr);
  float z_GC = -(sy * cp * cr - cy * sp * sr);

  /* ------------------------------------------------
   * Step 3: 读取 q_W_G（云台 → 世界）
   * ------------------------------------------------ */
  const float * qWG = gimbal_imu.q;
  float w_WG = qWG[0];
  float x_WG = qWG[1];
  float y_WG = qWG[2];
  float z_WG = qWG[3];

  /* ------------------------------------------------
   * Step 4: 四元数复合
   * q_W_C = q_W_G ⊗ q_G_C
   * ------------------------------------------------ */
  // [解析] Hamilton 乘法手工展开（省一次函数调用），结果与 quaternion_multiply() 完全一致。
  float w_WC = w_WG * w_GC - x_WG * x_GC - y_WG * y_GC - z_WG * z_GC;
  float x_WC = w_WG * x_GC + x_WG * w_GC + y_WG * z_GC - z_WG * y_GC;
  float y_WC = w_WG * y_GC - x_WG * z_GC + y_WG * w_GC + z_WG * x_GC;
  float z_WC = w_WG * z_GC + x_WG * y_GC - y_WG * x_GC + z_WG * w_GC;

  /* ------------------------------------------------
   * Step 5: 归一化（防数值漂移）
   * ------------------------------------------------ */
  // [解析] 归一化防数值漂移；1e-12f 只是拦住除零，else 分支退回单位四元数兜底，正常走不到。
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
  // [解析] 先备份上一拍再写本拍，顺序反了上一拍的数据就被覆盖，下面求角速度会全错。
  q_last_chassis2world[0] = q_chassis2world[0];
  q_last_chassis2world[1] = q_chassis2world[1];
  q_last_chassis2world[2] = q_chassis2world[2];
  q_last_chassis2world[3] = q_chassis2world[3];

  q_chassis2world[0] = w_WC;
  q_chassis2world[1] = x_WC;
  q_chassis2world[2] = y_WC;
  q_chassis2world[3] = z_WC;

  //quaternion_to_euler(q_chassis2world, euler_q);

  //现在下面这个dq 是等价于在地面系下的底盘w
  // [解析] dq = q_now ⊗ q_last*：表示两拍之间的微小旋转，等价于地面系下的底盘角速度。
  quaternion_multiply(q_chassis2world, q_last_chassis2world, dq, false, true);
  //这个dq一定不能归一化, 因为他就不是单位四元数

  // [解析] 小角度下 q ≈ {1, ω·dt/2}，所以统一乘 2/dt 还原成角速度；dt_ 填错会让结果整体缩放错。
  //使用泰勒展开近似四元数增量表示的角速度(必须保证他在1ms内是小量)
  this->dq[0] *= 2.0f / dt_;
  this->dq[1] *= 2.0f / dt_;
  this->dq[2] *= 2.0f / dt_;
  this->dq[3] *= 2.0f / dt_;
}

// [解析] 双 IMU 的相对姿态：q_gimbal2chassis = q_chassis* ⊗ q_gimbal，即四元数版的"除法"。
// [解析] 随后转成欧拉角就是等效电机角度；q_chassis2world 直接抄底盘 IMU，不用反推。
void Gimbal::update_q_gimbal2chassis(const sp::Mahony & gimbal_imu, const sp::Mahony & chassis_imu)
{
  //已有底盘的姿态四元数,算出底盘和云台之间的相对四元数,并将相对四元数转换成欧拉角
  quaternion_multiply(chassis_imu.q, gimbal_imu.q, q_gimbal2chassis, true, false);
  q_chassis2world[0] = chassis_imu.q[0];
  q_chassis2world[1] = chassis_imu.q[1];
  q_chassis2world[2] = chassis_imu.q[2];
  q_chassis2world[3] = chassis_imu.q[3];

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

  // [解析] 相对四元数转欧拉角，先存进中转变量 euler_motor，再分别赋给三个 _rel 成员。
  quaternion_to_euler(q_gimbal2chassis, euler_motor);
  //后续考虑是否用电机编码器值和euler_motor进行融合滤波
  roll_rel = euler_motor[0];
  pitch_rel = euler_motor[1];
  // [解析] yaw 必须过展开器：欧拉角被折在 (-π,π]，不展开的话微分求角速度会冒出巨大尖峰。
  yaw_rel = this->yaw_unwrapper_.update(euler_motor[2]);  // 展开消除±π跳变euler_motor[2];

  // 角度低通滤波
  yaw_relative_angle_filter.update(yaw_rel);
  pitch_relative_angle_filter.update(pitch_rel);
  roll_relative_angle_filter.update(roll_rel);
  yaw_rel = yaw_relative_angle_filter.out;
  pitch_rel = pitch_relative_angle_filter.out;
  roll_rel = roll_relative_angle_filter.out;
}
// [解析] 旧版目标解算：把世界系目标方向用 R_base2world_ 换到底盘系再反解，要先调用 update()。
// [解析] 三个 common_term_* 就是目标单位向量在底盘系的 x/y/z 分量；此版无速度/加速度前馈和展开器。
void Gimbal::calc(float yaw_set_in_world, float pitch_set_in_world)
{
  float cy = cosf(yaw_set_in_world);
  float sy = sinf(yaw_set_in_world);
  float cp = cosf(pitch_set_in_world);
  float sp_val = sinf(pitch_set_in_world);

  float common_term_x =
    R_base2world_[0][0] * cp * cy + R_base2world_[1][0] * sy * cp - R_base2world_[2][0] * sp_val;

  float common_term_y =
    R_base2world_[0][1] * cp * cy + R_base2world_[1][1] * sy * cp - R_base2world_[2][1] * sp_val;

  yaw_set_in_joint = atan2f(common_term_y, common_term_x);

  float common_term_z =
    R_base2world_[0][2] * cp * cy + R_base2world_[1][2] * sy * cp - R_base2world_[2][2] * sp_val;

  // 增加安全性
  if (common_term_z > 1.0f) common_term_z = 1.0f;
  if (common_term_z < -1.0f) common_term_z = -1.0f;

  pitch_set_in_joint = -asinf(common_term_z);
}

// [解析] 最常用的输出函数：pitch 走 asinf 所以先限幅，±90° 附近进万向锁分支，把 roll+yaw 合成进 yaw。
// [解析] 输出顺序 {roll,pitch,yaw}，与四元数 {w,x,y,z} 的下标不一样，别顺手把 euler[0] 当 yaw。
// 四元数转欧拉角 (内旋ZYX顺序: 先yaw->再pitch->最后roll)
// 输入: q[4] = {w, x, y, z} 表示物体系到世界系的四元数
// 输出: euler[3] = {roll, pitch, yaw} 单位：rad

void Gimbal::quaternion_to_euler(const float q[4], float euler[3])
{
  float w = q[0];
  float x = q[1];
  float y = q[2];
  float z = q[3];

  // [解析] sin(pitch) = 2(wy - xz)：那个 2 倍系数来自四元数→旋转矩阵的标准换算，不是随手写的。
  float sin_pitch = 2.0f * (w * y - x * z);

  // Clamp for numerical safety
  if (sin_pitch > 1.0f) sin_pitch = 1.0f;
  if (sin_pitch < -1.0f) sin_pitch = -1.0f;

  // Gimbal lock check  一瞬间将roll+yaw 全部吸收到yaw
  if (fabsf(sin_pitch) >= 0.999999f) {
    // pitch = ±90°
    euler[1] = copysignf(M_PI / 2.0f, sin_pitch);                               // pitch (beta)
    euler[0] = 0.0f;                                                            // roll (theta)
    euler[2] = atan2f(-2.0f * (x * y - w * z), 1.0f - 2.0f * (x * x + z * z));  //yaw = yaw+roll
  }
  else {
    // Normal case
    euler[1] = asinf(sin_pitch);  // pitch (beta)
    euler[0] = atan2f(2.0f * (y * z + w * x), 1.0f - 2.0f * (x * x + y * y));
    euler[2] = atan2f(2.0f * (x * y + w * z), 1.0f - 2.0f * (y * y + z * z));
  }
}

// [解析] 同一件事的矩阵输入版本：pitch 看 R[2][0]、roll 看 R[2][1]/R[2][2]、yaw 看 R[1][0]/R[0][0]。
// [解析] 万向锁阈值 0.99999f 比四元数版松一位；只要能拦住 |sin|>1 就不会出 NaN，功能上没差别。
// 旋转矩阵转欧拉角 (ZYX顺序: yaw->pitch->roll)
// 输入: R[3][3] 表示物体相对地面的旋转矩阵
// 输出: euler[3] = {roll, pitch, yaw} 单位：rad
void Gimbal::rotation_matrix_to_euler(const float R[3][3], float euler[3])
{
  // Pitch (绕Y轴旋转)
  float sin_pitch = -R[2][0];

  // 检查万向锁情况
  if (fabsf(sin_pitch) >= 0.99999f) {
    // 万向锁情况
    euler[1] = copysignf(M_PI / 2.0f, sin_pitch);
    euler[0] = 0.0f;                       // 将roll设为0
    // [解析] 万向锁下的 yaw 公式换成了 R[0][1]/R[1][1]，是 roll=0 代入后化简的结果，别拿常规公式验证。
    euler[2] = atan2f(-R[0][1], R[1][1]);  //
  }
  else {
    euler[1] = asinf(sin_pitch);

    // Roll (绕X轴旋转)
    euler[0] = atan2f(R[2][1], R[2][2]);

    // Yaw (绕Z轴旋转)
    euler[2] = atan2f(R[1][0], R[0][0]);
  }
}

// [解析] Hamilton 积：q1 ⊗ q2 表示"先 q2 再 q1"；取共轭 = 反向旋转，所以 A/B 要写成 A ⊗ B*。
// [解析] conjugate_q1/q2 传反了结果看着还挺正常（不是 NaN 也不是 0），极难查，改之前先想清方向。
// [解析] 每个分量先取到局部变量，所以 q_result 和 q1/q2 是同一块内存也安全。
// 四元数乘法：q_result = q1 ⊗ q2
// 输入/输出格式：q[4] = {w, x, y, z}
// 物理意义：坐标系变换的复合，先应用 q2，再应用 q1
// 例如：q_A_C = q_A_B ⊗ q_B_C（先从C到B，再从B到A）
// conjugate_q1: 是否对 q1 取共轭 (q1* = [w, -x, -y, -z])
// conjugate_q2: 是否对 q2 取共轭 (q2* = [w, -x, -y, -z])
void Gimbal::quaternion_multiply(
  const float q1[4], const float q2[4], float q_result[4], bool conjugate_q1, bool conjugate_q2)
{
  // 根据 conjugate 标志决定符号
  float w1 = q1[0];
  float x1 = conjugate_q1 ? -q1[1] : q1[1];
  float y1 = conjugate_q1 ? -q1[2] : q1[2];
  float z1 = conjugate_q1 ? -q1[3] : q1[3];

  float w2 = q2[0];
  float x2 = conjugate_q2 ? -q2[1] : q2[1];
  float y2 = conjugate_q2 ? -q2[2] : q2[2];
  float z2 = conjugate_q2 ? -q2[3] : q2[3];

  // Hamilton 四元数乘法公式
  q_result[0] = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2;  // w
  q_result[1] = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2;  // x
  q_result[2] = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2;  // y
  q_result[3] = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2;  // z
}

// [解析] passive rotation：换的是坐标系，不是转向量；v_out = q ⊗ v_in ⊗ q*。
// [解析] conjugate_q=true 表示用 q*，即把"从 A 到 B"变成"从 B 到 A"，这里最容易搞反。
// [解析] 实现上不做两次四元数乘法，用等价的 t = q_vec×v + w·v、v_out = v + 2·q_vec×t，省一半运算。
// 四元数坐标变换：v_out = q ⊗ v_in ⊗ q*
// 输入：q[4] = {w, x, y, z} 表示从坐标系A到坐标系B的旋转
//      v_in[3] = {x, y, z} 在坐标系B中的同一个向量的坐标
// 输出：v_out[3] = {x, y, z} 在坐标系A中的同一个向量的坐标向量
// conjugate_q: 是否对 q 取共轭（true 时使用 q* 而不是 q，相当于反向旋转）
// 注意：这是 passive rotation（坐标系变换），不是 active rotation（向量旋转）
void Gimbal::quaternion_frame_transform(
  const float q[4], const float v_in[3], float v_out[3], bool conjugate_q)
{
  // 根据 conjugate 标志决定四元数的符号
  float w = q[0];
  float x = conjugate_q ? -q[1] : q[1];
  float y = conjugate_q ? -q[2] : q[2];
  float z = conjugate_q ? -q[3] : q[3];

  float vx = v_in[0], vy = v_in[1], vz = v_in[2];

  // 优化的四元数-向量旋转公式（避免构造完整四元数）
  // v_out = v_in + 2 * cross(q_vec, cross(q_vec, v_in) + w * v_in)

  // 第一步：t = cross(q_vec, v_in) = q_vec × v_in
  float tx = y * vz - z * vy;
  float ty = z * vx - x * vz;
  float tz = x * vy - y * vx;

  // 第二步：t = t + w * v_in
  tx += w * vx;
  ty += w * vy;
  tz += w * vz;

  // 第三步：v_out = v_in + 2 * cross(q_vec, t)
  v_out[0] = vx + 2.0f * (y * tz - z * ty);
  v_out[1] = vy + 2.0f * (z * tx - x * tz);
  v_out[2] = vz + 2.0f * (x * ty - y * tx);
}

// [解析] 叉乘不满足交换律：参数顺序写反结果整体取负，C 项搞反会让云台自转时朝反方向漂。
// [解析] 两向量平行（或有零向量）时结果是 {0,0,0}。
void Gimbal::cross_product(const float v1[3], const float v2[3], float result[3])
{
  // 向量叉乘公式：result = v1 × v2
  // | i    j    k   |
  // | v1x  v1y  v1z |
  // | v2x  v2y  v2z |
  result[0] = v1[1] * v2[2] - v1[2] * v2[1];
  result[1] = v1[2] * v2[0] - v1[0] * v2[2];
  result[2] = v1[0] * v2[1] - v1[1] * v2[0];
}

// [解析] T⁻¹（欧拉运动学方程）：载体系角速度 → 三轴欧拉角速率，后三个是引用出参。
// [解析] vroll 含 tan(pitch)、vyaw 含 1/cos(pitch)，pitch 接近 ±90° 时发散，这就是万向锁。
// [解析] 形参 yaw 不参与运算，只是为了和下面的逆函数参数表对称。
void Gimbal::transform_omiga_in_body_2_euler_rates(
  const float omiga_in_body[3], const float roll, const float pitch, const float yaw, float & vroll,
  float & vpitch, float & vyaw)
{
  vroll = omiga_in_body[0] + omiga_in_body[1] * std::sin(roll) * std::tan(pitch) +
          omiga_in_body[2] * std::cos(roll) * std::tan(pitch);

  vpitch = omiga_in_body[1] * std::cos(roll) - omiga_in_body[2] * std::sin(roll);

  vyaw = omiga_in_body[1] * std::sin(roll) / std::cos(pitch) +
         omiga_in_body[2] * std::cos(roll) / std::cos(pitch);
}

// [解析] T 矩阵（上一条的逆）：全是乘法、没有除法，不存在奇点，能用它就别用 T⁻¹。
// [解析] (void)yaw 表示该参数确实不参与运算，不是漏写。
void Gimbal::transform_euler_rates_2_omiga_in_body(
  const float vroll, const float vpitch, const float vyaw, const float roll, const float pitch,
  const float yaw, float omiga_in_body[3])
{
  (void)yaw;  // yaw 在该变换中不参与，防止未使用警告

  // [解析] wx = vroll - vyaw·sin(pitch)：绕竖直轴自转时，云台 x 轴也会跟着改变朝向。
  omiga_in_body[0] = vroll - vyaw * std::sin(pitch);

  omiga_in_body[1] = vpitch * std::cos(roll) + vyaw * std::sin(roll) * std::cos(pitch);

  omiga_in_body[2] = -vpitch * std::sin(roll) + vyaw * std::cos(roll) * std::cos(pitch);
}

// [解析] "变换矩阵导数 × 欧拉角速率" 项（calc_all_target() 的 A 项和 D 项）：每项都是两个速率相乘，量纲 rad/s²。
// [解析] 漏掉它不影响静态精度，但快速甩头时云台会在终点附近过冲或发飘。
void Gimbal::Transform_matrix_rates_multipy_Euler_rates(
  const float roll, const float pitch, const float yaw, const float vroll, const float vpitch,
  const float vyaw, float result[3])
{
  (void)yaw;  // yaw 在该变换中不参与，防止未使用警告
  result[0] = -cosf(pitch) * vpitch * (vyaw);
  result[1] = -sinf(roll) * vroll * (vpitch) +
              (cosf(roll) * cosf(pitch) * vroll - sinf(roll) * sinf(pitch) * vpitch) * (vyaw);
  result[2] = -cosf(roll) * vroll * (vpitch) +
              (-cosf(roll) * sinf(pitch) * vpitch - sinf(roll) * cosf(pitch) * vroll) * (vyaw);
}

// [解析] 欧拉角 → 四元数，四行公式是三个单轴半角四元数按 q_yaw ⊗ q_pitch ⊗ q_roll 展开的结果。
// [解析] 与 quaternion_to_euler() 严格互逆；与 update_q_chassis2world() 里手写的 q_GC 互为共轭。
// 欧拉角(ZYX顺序: yaw->pitch->roll)转四元数
// 输入: yaw, pitch, roll 的含义为: 机体系从与世界系重合开始, 先绕世界Z轴旋转yaw, 再绕新Y轴旋转pitch, 最后绕新X轴旋转roll
// 输出: q[4] = {w, x, y, z}, 表示将机体系下向量变换到世界系下的四元数, 即 v_world = q ⊗ v_body ⊗ q*
void Gimbal::euler_zyx_to_quaternion(float yaw, float pitch, float roll, float q[4])
{
  // 半角
  float half_yaw = 0.5f * yaw;
  float half_pitch = 0.5f * pitch;
  float half_roll = 0.5f * roll;

  float cy = cosf(half_yaw);
  float sy = sinf(half_yaw);
  float cp = cosf(half_pitch);
  float sp = sinf(half_pitch);
  float cr = cosf(half_roll);
  float sr = sinf(half_roll);

  // ZYX 内旋 (先yaw->再pitch->最后roll) 对应的四元数, 保持与quaternion_to_euler一致
  q[0] = cy * cp * cr + sy * sp * sr;  // w
  q[1] = cy * cp * sr - sy * sp * cr;  // x
  q[2] = cy * sp * cr + sy * cp * sr;  // y
  q[3] = sy * cp * cr - cy * sp * sr;  // z

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
    // 极端异常兜底，退回单位四元数
    q[0] = 1.0f;
    q[1] = q[2] = q[3] = 0.0f;
  }
}

// [解析] 把底盘 IMU 的姿态绕底盘 X 轴转 install_roll_，对齐到云台 IMU 的 roll 面，消掉安装偏差。
// [解析] 左乘 install_roll_q_ 的共轭相当于"除掉"这个安装旋转；姿态和角速度要一起转，否则速度环有静差。
// [解析] 整个程序只在 update_all_dual() 入口调用一次，别重复调用。
void Gimbal::correct_chassis_imu(sp::Mahony & chassis_imu)
{
  float q_temporary[4];
  float w_temporary[3];
  q_temporary[0] = chassis_imu.q[0];
  q_temporary[1] = chassis_imu.q[1];
  q_temporary[2] = chassis_imu.q[2];
  q_temporary[3] = chassis_imu.q[3];
  w_temporary[0] = chassis_imu.w[0];
  w_temporary[1] = chassis_imu.w[1];
  w_temporary[2] = chassis_imu.w[2];
  quaternion_multiply(this->install_roll_q_, q_temporary, chassis_imu.q, true, false);
  quaternion_frame_transform(this->install_roll_q_, w_temporary, chassis_imu.w, true);
}

}  // namespace sp
