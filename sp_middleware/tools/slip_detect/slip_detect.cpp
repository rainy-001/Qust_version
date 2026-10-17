// [解析] 滑移检测的实现：对 4 个轮子各算一遍轮速 → 轮位处的地面速度 → 投影到滚动方向 → 滑移率，
// [解析]   再低通滤波、换算成功率惩罚系数，最后统计整体打滑状态。
// [解析] 没有 volatile / 互斥，假定只有底盘控制一个任务在调它。

/**
 * @file    slip_detect.cpp
 * @brief   滑移率检测模块实现
 *
 * 滑移率定义:
 *   λ_i = (v_wheel_i - v_ground_i) / max(|v_wheel_i|, |v_ground_i|, ε)
 *
 *   其中:
 *     v_wheel_i = ω_i × r            — 编码器推算的轮子线速度
 *     v_ground_i = V_chassis · e_i + ωz_true × (r_pos × e_i)
 *                = (vx - ωz·y_i)·cosθ_i + (vy + ωz·x_i)·sinθ_i
 *                — 底盘真实运动在轮子滚动方向上的投影
 *
 * 惩罚函数:
 *   slip_penalty = clamp(1 - penalty_gain × |λ|, 0.05, 1.0)
 */

#include "slip_detect.hpp"

#include <algorithm>
#include <cmath>

namespace sp
{

// [解析] 只存参数并把输出、滤波状态清零；{0, 1, 0, 0} 的顺序由 WheelSlipData 的成员顺序决定。
SlipDetect::SlipDetect(
  float half_length, float half_width, float alpha_slip, float penalty_gain, float slip_threshold)
: l_(half_length),
  w_(half_width),
  alpha_slip_(alpha_slip),
  penalty_gain_(penalty_gain),
  slip_threshold_(slip_threshold),
  any_slipping(false),
  avg_slip_ratio(0.0f),
  slip_wz(0.0f),
  inited_(false)
{
  for (int i = 0; i < 4; i++) {
    wheel_slip[i] = {0.0f, 1.0f, 0.0f, 0.0f};
    filtered_slip_[i] = 0.0f;
  }
}

// [解析] 核心循环：四个轮子逐个算，中间量都是局部变量，结果写进 wheel_slip[] 和整体统计。
void SlipDetect::update(
  float imu_wz, float chassis_vx, float chassis_vy, const float wheel_speeds[4],
  const float wheel_angles[4], float wheel_radius)
{
  // ---- 轮位坐标 (车体坐标系: X向前, Y向左) ----
  //   [0]=RF: (x=+l, y=-w)
  //   [1]=LF: (x=+l, y=+w)
  //   [2]=LR: (x=-l, y=+w)
  //   [3]=RR: (x=-l, y=-w)
  // [解析] 四个轮位在车体系下的坐标（y 轴向左为正），顺序 [RF, LF, LR, RR]，是刚体速度场里的半径向量。
  const float px[4] = {+l_, +l_, -l_, -l_};
  const float py[4] = {-w_, +w_, +w_, -w_};

  // ---- 角速度滑移量 (整体指标) ----
  // chassis_wz 没有直接传入，从轮速估算意义不大，
  // 这里用 imu_wz 和轮速的差异来估计
  // 注意：slip_wz 需要外部配合 chassis.wz 来计算
  // 这里保留为整体观测指标
  // [解析] 作者的待办：代码既没用到 chassis_wz，也没给 slip_wz 赋过值，所以它永远是构造函数里的 0。

  float slip_sum = 0.0f;
  int slip_count = 0;
  any_slipping = false;

  for (int i = 0; i < 4; i++) {
    // --- 1. 轮子线速度 (编码器) ---
    float v_wheel = wheel_speeds[i] * wheel_radius;
    wheel_slip[i].v_wheel = v_wheel;

    // --- 2. 底盘在轮位处的真实线速度 ---
    //     v_at_wheel_x = vx - ωz × y_i   (刚体速度场)
    //     v_at_wheel_y = vy + ωz × x_i
    // [解析] 刚体速度场 v(点) = v(质心) + ω × r：ω=(0,0,ωz)、r=(px,py,0)，展开就是下面的 −ωz·py 与 +ωz·px。
    float v_gnd_x = chassis_vx - imu_wz * py[i];
    float v_gnd_y = chassis_vy + imu_wz * px[i];

    // --- 3. 投影到轮子滚动方向 ---
    // [解析] 轮子只有滚动方向由电机驱动（垂直方向是被动滑动），所以把车速投影到轮子朝向：v·e = vx·cosθ + vy·sinθ。
    float cos_a = std::cos(wheel_angles[i]);
    float sin_a = std::sin(wheel_angles[i]);
    float v_ground = v_gnd_x * cos_a + v_gnd_y * sin_a;
    wheel_slip[i].v_ground = v_ground;

    // --- 4. 滑移率计算 ---
    // [解析] 分母取绝对值较大者，|λ| 天然落在 [−1, 1] 内，下面的钳制只是防御性写法。
    float max_v = std::max(std::abs(v_wheel), std::abs(v_ground));
    float raw_slip = 0.0f;

    // 低速死区: 速度太低时滑移率无意义 (噪声主导)
    // [解析] 低速死区：速度低于 5 cm/s 时分母极小，量化误差和 IMU 噪声会被放大成 ±1 的假滑移，车一停就狂降功率。
    constexpr float kSpeedDeadZone = 0.05f;  // 5 cm/s
    if (max_v > kSpeedDeadZone) {
      // [解析] λ = (轮速 − 地面速度) / max：正 = 轮子转得比车快（空转），负 = 比车慢（抱死）。
      raw_slip = (v_wheel - v_ground) / max_v;
      // 钳制到 [-1, 1]
      if (raw_slip > 1.0f) raw_slip = 1.0f;
      if (raw_slip < -1.0f) raw_slip = -1.0f;
    }

    // --- 5. 低通滤波 ---
    // [解析] 一阶低通 filtered = α·raw + (1−α)·filtered_last：原始滑移率抖得厉害，直接拿来罚功率会让动力一抽一抽。
    // [解析] 第一拍没有历史值，直接取 raw，否则从 0 慢慢爬升的过程会被误判成"正在打滑"。
    if (!inited_) {
      filtered_slip_[i] = raw_slip;
    }
    else {
      filtered_slip_[i] =
        alpha_slip_ * raw_slip + (1.0f - alpha_slip_) * filtered_slip_[i];
    }
    wheel_slip[i].slip_ratio = filtered_slip_[i];

    // --- 6. 惩罚系数 ---
    // [解析] penalty = 1 − gain·|λ|：取绝对值是因为空转和抱死都该罚；默认 gain = 1.5 相当激进。
    float abs_slip = std::abs(filtered_slip_[i]);
    wheel_slip[i].slip_penalty = 1.0f - penalty_gain_ * abs_slip;
    // 钳制: 不完全切断功率，保留最低 5%
    // [解析] 下限 0.05 留了底线：掐到 0 会让这个轮瞬间失去姿态控制；上限 1.0 防止异常输入把权重放大。
    if (wheel_slip[i].slip_penalty < 0.05f) {
      wheel_slip[i].slip_penalty = 0.05f;
    }
    if (wheel_slip[i].slip_penalty > 1.0f) {
      wheel_slip[i].slip_penalty = 1.0f;
    }

    // --- 7. 整体状态 ---
    // [解析] 任意一轮 |λ| 超阈值就置 any_slipping，同时累加 |λ| 供最后求平均。
    if (abs_slip > slip_threshold_) {
      any_slipping = true;
    }
    slip_sum += abs_slip;
    slip_count++;
  }

  inited_ = true;
  avg_slip_ratio = (slip_count > 0) ? (slip_sum / static_cast<float>(slip_count)) : 0.0f;
}

// [解析] 把滑移惩罚作用到功率分配权重上并重新归一化。HKUST 算法假定四个权重和为 1，
// [解析]   不归一化就等于白丢一份功率；归一化后打滑轮省下的份额自动补给抓地轮。
void SlipDetect::apply_slip_penalty(const float K_original[4], float K_corrected[4])
{
  // 每个轮子的权重乘以对应的滑移惩罚系数
  for (int i = 0; i < 4; i++) {
    K_corrected[i] = K_original[i] * wheel_slip[i].slip_penalty;
  }

  // 重新归一化: 释放的功率额度按比例自动流向不打滑的轮子
  float sum = K_corrected[0] + K_corrected[1] + K_corrected[2] + K_corrected[3];

  // [解析] 乘倒数少 3 次除法；1e-6 的容差是避免除以一个极小数把权重放大成天文数字。
  if (sum > 1e-6f) {
    float inv_sum = 1.0f / sum;
    for (int i = 0; i < 4; i++) {
      K_corrected[i] *= inv_sum;
    }
  }
  else {
    // 所有轮子都严重打滑 → 均分，让每个轮子至少有一些基础力矩维持姿态
    for (int i = 0; i < 4; i++) {
      K_corrected[i] = 0.25f;
    }
  }
}

// [解析] 与 apply_slip_penalty() 分工：那个是轮间重新分配，这个是整体降功率，四轮全滑时才用。
float SlipDetect::get_global_slip_scale() const
{
  // 平均滑移率超过阈值时，输出整体限幅系数
  // 映射: avg_slip > 0.3 开始限幅, avg_slip = 0.8 时限到 0.3
  if (avg_slip_ratio < slip_threshold_) {
    return 1.0f;
  }

  // 线性插值: [threshold, 0.8] → [1.0, 0.3]
  // [解析] 把 avg_slip 从 slip_threshold_ 线性映射到 [1.0, 0.3]；threshold 传 0.8 会除以 0，作者没做保护。
  float t = (avg_slip_ratio - slip_threshold_) / (0.8f - slip_threshold_);
  if (t > 1.0f) t = 1.0f;

  float scale = 1.0f - t * 0.7f;  // 最多降到 0.3
  return scale;
}

}  // namespace sp
