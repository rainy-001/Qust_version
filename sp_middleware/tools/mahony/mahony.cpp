// [解析] Mahony 互补滤波的实现：一直积分陀螺仪，但用加速度计测到的重力方向不断把它拽回来。
// [解析] 难读的是三处：用四元数估重力方向再与实测叉乘得误差、误差经 PI 加回角速度后积分成四元数、
// [解析]   pitch_geom_calc() 用 x/z 轴的竖直分量判断倒扣，把 pitch 的值域扩到 ±π。

#include "mahony.hpp"

#include <cmath>

#include "tools/math_tools/math_tools.hpp"
namespace sp
{
Mahony::Mahony(float dt, float kp, float ki)
: dt_(dt),
  two_kp_(2 * kp),
  two_ki_(2 * ki),
  inited_(false),
  integral_fbx_(0.0f),
  integral_fby_(0.0f),
  integral_fbz_(0.0f)
{
}

// [解析] 数组版只是转发，拆成 6 个 float 调下面那版（数组作参数会退化成指针）。
void Mahony::update(const float acc[3], const float gyro[3])
{
  update(acc[0], acc[1], acc[2], gyro[0], gyro[1], gyro[2]);
}

// [解析] 每周期调一次（imu_task 1ms）：用加速度计看到的重力方向纠正陀螺仪漂移，结果写进 q 和欧拉角。
void Mahony::update(float ax, float ay, float az, float wx, float wy, float wz)
{
  // [解析] 上电第一拍 q 还是空的，不能积分：先用重力方向定初始姿态，做完直接 return。
  if (!inited_) {
    inited_ = true;
    init(ax, ay, az);
    return;
  }
  w[0] = wx;
  w[1] = wy;
  w[2] = wz;
  // [解析] 必须先备份：下面 wx/wy/wz 会被误差改写成修正后的角速度，而算欧拉角速率要用原始值。
  float gx = wx;
  float gy = wy;
  float gz = wz;

  // Normalise accelerometer measurement
  float norm = 1.0f / std::sqrt(ax * ax + ay * ay + az * az);
  ax *= norm;
  ay *= norm;
  az *= norm;

  // Estimated direction of gravity and vector perpendicular to magnetic flux
  // [解析] 用当前四元数估出重力方向（旋转矩阵第三行，这里只存了一半）；halfvz 用单位四元数恒等式化简过。
  // [解析] 取一半是为了和叉乘得到的 halfex/halfey/halfez 同尺度，丢掉的 2 倍由 two_kp_/two_ki_ 补回。
  float halfvx = this->q[1] * this->q[3] - this->q[0] * this->q[2];
  float halfvy = this->q[0] * this->q[1] + this->q[2] * this->q[3];
  float halfvz = this->q[0] * this->q[0] - 0.5f + this->q[3] * this->q[3];

  // Error is sum of cross product between estimated and measured direction of gravity
  // [解析] 两个方向重合时叉乘为 0 就不修正；不重合时叉乘方向指示该绕哪根轴转、大小≈夹角（sinθ≈θ）。
  float halfex = (ay * halfvz - az * halfvy);
  float halfey = (az * halfvx - ax * halfvz);
  float halfez = (ax * halfvy - ay * halfvx);

  // Compute and apply integral feedback if enabled
  // [解析] ki > 0 才启用：把持续存在的误差累加进角速度，正好抵消陀螺仪的固定零偏；本项目默认 ki = 0。
  if (two_ki_ > 0.0f) {
    // integral error scaled by Ki
    integral_fbx_ += two_ki_ * halfex * dt_;
    integral_fby_ += two_ki_ * halfey * dt_;
    integral_fbz_ += two_ki_ * halfez * dt_;

    // apply integral feedback
    wx += integral_fbx_;
    wy += integral_fby_;
    wz += integral_fbz_;
  }
  else {
    // prevent integral windup
    // [解析] ki = 0 时清零积分器，防止它偷偷累成一个大值（windup），否则以后一开 ki 会瞬间弹进去。
    integral_fbx_ = 0.0f;
    integral_fby_ = 0.0f;
    integral_fbz_ = 0.0f;
  }

  // Apply proportional feedback
  // [解析] 比例项是 pitch/roll 收敛的主力；到这里 wx/wy/wz 已经不是原始角速度了。
  wx += two_kp_ * halfex;
  wy += two_kp_ * halfey;
  wz += two_kp_ * halfez;

  // Integrate rate of change of quaternion
  // pre-multiply common factors
  // [解析] 先乘公共因子 0.5·dt，下面 4 行加法就不用重复乘；这是一阶欧拉积分，dt 越大截断误差越大。
  wx *= 0.5f * dt_;
  wy *= 0.5f * dt_;
  wz *= 0.5f * dt_;

  // [解析] 四元数微分方程 q̇ = 0.5·q⊗(0,ω) 展开成 4 个分量，先放临时变量，归一化前不要写回 q。
  float q0 = this->q[0] + (-this->q[1] * wx - this->q[2] * wy - this->q[3] * wz);
  float q1 = this->q[1] + (this->q[0] * wx + this->q[2] * wz - this->q[3] * wy);
  float q2 = this->q[2] + (this->q[0] * wy - this->q[1] * wz + this->q[3] * wx);
  float q3 = this->q[3] + (this->q[0] * wz + this->q[1] * wy - this->q[2] * wx);

  this->q_last[0] = this->q[0];
  this->q_last[1] = this->q[1];
  this->q_last[2] = this->q[2];
  this->q_last[3] = this->q[3];

  // Normalise quaternion
  // [解析] 必须归一化：欧拉积分和浮点误差会让模长偏离 1，而后面所有姿态计算都假定它是单位四元数。
  norm = 1.0f / std::sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  this->q[0] = q0 * norm;
  this->q[1] = q1 * norm;
  this->q[2] = q2 * norm;
  this->q[3] = q3 * norm;

  //只是用到了1=q[0]^2+q[1]^2+q[2]^2+q[3]^2这个关系简化了一些计算
  //以后的同学在用ai理解mahony算法的时候会更容易,形式更加相同
  //之后要加万向锁保护  ,我的理想是加根据历史记录进行保护
  // [解析] 四元数 → ZYX 欧拉角：yaw/roll 用 atan2（能判象限），pitch 用 asin 只有 ±π/2，倒扣时会翻回来。
  // [解析] 分母写成 1−2(...) 而不是旋转矩阵原元素，是利用 w²+x²+y²+z²=1 化简过的，与作者注释对应。
  this->yaw = std::atan2(
    2.0f * (this->q[0] * this->q[3] + this->q[1] * this->q[2]),
    1.0f - 2.0f * (this->q[2] * this->q[2] + this->q[3] * this->q[3]));
  this->pitch = std::asin(2.0f * (this->q[0] * this->q[2] - this->q[1] * this->q[3]));
  this->roll = std::atan2(
    2.0f * (this->q[0] * this->q[1] + this->q[2] * this->q[3]),
    1.0f - 2.0f * (this->q[1] * this->q[1] + this->q[2] * this->q[2]));

  //计算欧拉角变化率
  // [解析] 传进去的是原始角速度 gx/gy/gz，不是上面被修正过的 wx/wy/wz。
  culculate_yaw_pitch_roll_rates(gx, gy, gz, this->roll, this->pitch, this->yaw);

  //将pitch进行值域扩充,解决串腿翻倒后自启问题
  pitch_geom_calc();
}

// [解析] 由载体系角速度反解 ZYX 欧拉角速率，结果写进 vroll/vpitch/vyaw（陀螺仪测的是绕车体三轴，不是欧拉角变化率）。
// [解析] 公式里有 tan/cos(pitch)，pitch 接近 ±90°（万向锁）时会发散；参数 yaw 在函数体里一次都没用到。
void Mahony::culculate_yaw_pitch_roll_rates(
  float wx, float wy, float wz, float roll, float pitch, float yaw)
{
  // [解析] 三行分别是 roll / pitch / yaw 速率：逐级把角速度投影到对应轴上，yaw 那行还要多除一个 cos(pitch)。
  this->vroll = wx + wy * std::sin(roll) * std::tan(pitch) + wz * std::cos(roll) * std::tan(pitch);
  this->vpitch = wy * std::cos(roll) - wz * std::sin(roll);
  this->vyaw = wy * std::sin(roll) / std::cos(pitch) + wz * std::cos(roll) / std::cos(pitch);
}

// [解析] 上电初始对准：静止时重力最可靠，只用加速度计定初始 roll/pitch，yaw 固定 0（无磁力计就没有绝对方位）。
void Mahony::init(float ax, float ay, float az)
{
  float norm = 1.0f / std::sqrt(ax * ax + ay * ay + az * az);
  ax *= norm;
  ay *= norm;
  az *= norm;

  // [解析] 严格公式里 pitch 的分母是 sqrt(ay²+az²)，这里用 az 近似，车身大横滚时初始值会偏（上电放平即可）。
  float pitch0 = std::atan2(-ax, az);
  float roll0 = std::atan2(ay, az);
  float yaw0 = 0.0f;
  // 需要注意的是:上电后在确定底盘相对于地面的初始yaw值的时候 要在mahony的init阶段直接将yaw_relative_angle赋值给底盘系imu的yaw0(正负自己确定)!!!

  // [解析] 四元数公式里用的都是半角，所以每个角都先乘 0.5；c*2 / s*2 就是 cos(角/2) / sin(角/2)。
  float cr2 = std::cos(roll0 * 0.5f);
  float cp2 = std::cos(pitch0 * 0.5f);
  float cy2 = std::cos(yaw0 * 0.5f);
  float sr2 = std::sin(roll0 * 0.5f);
  float sp2 = std::sin(pitch0 * 0.5f);
  float sy2 = std::sin(yaw0 * 0.5f);

  // [解析] 欧拉角 → 四元数的 ZYX 标准公式：三个角唯一对应一个单位四元数，照抄即可。
  float q0 = cr2 * cp2 * cy2 + sr2 * sp2 * sy2;
  float q1 = sr2 * cp2 * cy2 - cr2 * sp2 * sy2;
  float q2 = cr2 * sp2 * cy2 + sr2 * cp2 * sy2;
  float q3 = cr2 * cp2 * sy2 - sr2 * sp2 * cy2;

  norm = 1.0f / std::sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  this->q[0] = q0 * norm;
  this->q[1] = q1 * norm;
  this->q[2] = q2 * norm;
  this->q[3] = q3 * norm;

  this->yaw = yaw0;
  this->pitch = pitch0;
  this->roll = roll0;
}

// [解析] 把 pitch 从 asin 的 ±π/2 扩展到 ±π：asin 丢掉了"翻过去的那一半"，用 world_z 的符号补回 ±π。
// [解析] 普通 pitch 在倒扣瞬间会跳变，串腿/翻车自启逻辑要的是这个扩展值。
void Mahony::pitch_geom_calc()
{
  // [解析] pitch_geom 没有初值，第一次调用时这一行读到的是未初始化内存，首拍 vpitch_geom 会出现一个毛刺。
  this->pitch_geom_last = this->pitch_geom;

  quaternion_frame_transform_for_mahony(
    this->q, this->base_x, this->world_x, false);  //将base系的x轴转换到world系
  quaternion_frame_transform_for_mahony(
    this->q, this->base_z, this->world_z, false);  //将base系的z轴转换到world系

  // [解析] world_z[2] ≈ +1 表示直立、≈ −1 表示完全倒扣，是下面判断象限的依据。
  float pitch_init = std::asin(
    -world_x[2]);  //初始pitch角度(云台x轴在world系下的投影与world系xy平面的夹角)(低头为正)

  //接下来结合z轴判断pitch角度的象限
  if (world_z[2] >= 0.0f) {
    this->pitch_geom = pitch_init;
  }
  else {
    // [解析] 已倒扣时 asin 的结果要接上 ±π 才连续：world_x[2] ≥ 0（车头朝上）走 −π 一侧，
    // [解析]   否则走 +π 一侧，两边在 ±π 处刚好接上。
    if (world_x[2] >= 0.0f) {
      this->pitch_geom = -M_PI - pitch_init;  //抬头倒扣
    }
    else {
      this->pitch_geom = M_PI - pitch_init;  //低头倒扣
    }  //这里仍然避免不了+-2pi的跳变,这就没关系了,至少我们先把值域扩展到了+-pi
  }

  // 计算几何pitch角及其微分
  // [解析] 直接差分求角速度：±π 交界那一拍会算出一个巨大的假速度，噪声也比 vpitch 大。
  this->vpitch_geom = (this->pitch_geom - this->pitch_geom_last) / dt_;
}

// [解析] 内部存的是 2×kp / 2×ki，所以这里也必须乘 2，否则改完的效果跟构造时传入的差一倍。
void Mahony::set_kp(float kp) { this->two_kp_ = 2.0f * kp; }

void Mahony::set_ki(float ki) { this->two_ki_ = 2.0f * ki; }

//专门给mahony写的四元数换系,以免老是引用gimbal类
// 四元数坐标变换：v_out = q ⊗ v_in ⊗ q*
// 输入：q[4] = {w, x, y, z} 表示从坐标系A到坐标系B的旋转
//      v_in[3] = {x, y, z} 在坐标系B中的同一个向量的坐标
// 输出：v_out[3] = {x, y, z} 在坐标系A中的同一个向量的坐标向量
// conjugate_q: 是否对 q 取共轭（true 时使用 q* 而不是 q，相当于反向旋转）
// 注意：这是 passive rotation（坐标系变换），不是 active rotation（向量旋转）
// [解析] 用四元数把一个向量换系，v_out = q ⊗ v_in ⊗ q*；conjugate_q = true 时相当于反着转回去。
void Mahony::quaternion_frame_transform_for_mahony(
  const float q[4], const float v_in[3], float v_out[3], bool conjugate_q)
{
  // 根据 conjugate 标志决定四元数的符号
  float w = q[0];
  float x = conjugate_q ? -q[1] : q[1];
  float y = conjugate_q ? -q[2] : q[2];
  float z = conjugate_q ? -q[3] : q[3];

  float vx = v_in[0], vy = v_in[1], vz = v_in[2];

  // [解析] 这个等价变形只要两次叉乘（约 12 次乘法），比直接算 q⊗v⊗q* 省一半以上，1kHz 下值得。
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

}  // namespace sp
