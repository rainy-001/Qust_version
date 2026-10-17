// [解析] 舵轮底盘"打滑检测"：拿编码器推算的轮速和 IMU 测出的真实车速比较，算出四轮滑移率，
// [解析]   再换算成功率惩罚系数，把功率从空转的轮子挪给还抓着地的轮子。
// [解析] 全仓库没有业务代码调用它（属预留能力）；轮序固定 [RF, LF, LR, RR]，与 chassis_para.hpp 的 move_t 一致。

/**
 * @file    slip_detect.hpp
 * @brief   舵轮底盘滑移率检测模块
 *
 * 通过比较陀螺仪真实角速度与轮式里程计推算角速度，
 * 估计每个轮子的滑移率，为功率分配提供惩罚系数。
 *
 * 核心原理：
 *   λ_i = (v_wheel_i - v_ground_i) / max(|v_wheel_i|, |v_ground_i|, ε)
 *   其中 v_ground_i 使用陀螺仪真实 wz 投影计算
 *
 * 轮序约定（与 chassis_para.hpp 中 move_t 一致）：
 *   [0]=RF(右前), [1]=LF(左前), [2]=LR(左后), [3]=RR(右后)
 */

#ifndef SP__SLIP_DETECT_HPP
#define SP__SLIP_DETECT_HPP

namespace sp
{

/**
 * @brief 单个轮子的滑移数据
 */
// [解析] 一个轮子的"体检报告"，4 个成员全是只读输出；成员顺序与 .cpp 里的聚合初始化 {0, 1, 0, 0} 严格对应。
struct WheelSlipData
{
  float slip_ratio;   ///< 滑移率 [-1, 1]  正=驱动空转, 负=制动抱死
  float slip_penalty; ///< 功率惩罚系数 [0, 1]  1=无惩罚, ~0=几乎切断
  float v_wheel;      ///< 轮子线速度 (m/s)  来自编码器
  float v_ground;     ///< 地面真实速度投影 (m/s)  来自IMU+运动学
};

// [解析] 一个对象管理整车的四轮打滑检测：每控制周期 update() 一次，功率分配前调 apply_slip_penalty()。
// [解析] 用的是 imu_wz 而不是底盘自己算的 wz —— 打滑恰恰意味着轮式里程计的 wz 已经不可信。
class SlipDetect
{
public:
  /**
   * @param half_length   底盘前后轮距的一半 (m)
   * @param half_width    底盘左右轮距的一半 (m)
   * @param alpha_slip    滑移率低通滤波系数 (0~1), 越大响应越快但越噪声敏感
   * @param penalty_gain  惩罚增益 (>0), 越大对滑移的惩罚越激进
   * @param slip_threshold 滑移判定阈值, |λ|超过此值认为打滑
   */
  // [解析] 初始化列表的顺序和成员声明顺序并不一致（实际按声明顺序走），各成员互相独立所以无副作用。
  SlipDetect(
    float half_length, float half_width, float alpha_slip = 0.08f,
    float penalty_gain = 1.5f, float slip_threshold = 0.25f);

  // [解析] 只读输出：四轮明细、是否有轮打滑、平均 |λ|；slip_wz 目前恒为 0（没人给它赋值）。
  // ---- 只读输出 ----
  WheelSlipData wheel_slip[4]; ///< 四轮滑移数据 [RF, LF, LR, RR]
  bool any_slipping;           ///< 是否有任意轮子在打滑
  float avg_slip_ratio;        ///< 四轮平均绝对滑移率 [0, 1]
  float slip_wz;               ///< 角速度滑移量: chassis.wz - imu_wz (rad/s)

  /**
   * @brief 核心更新函数 —— 每个控制周期调用一次
   *
   * @param imu_wz         陀螺仪真实Z轴角速度 (rad/s)  — imu.vyaw 或 imu.w[2]
   * @param chassis_vx     轮式里程计X轴线速度 (m/s)    — chassis.vx
   * @param chassis_vy     轮式里程计Y轴线速度 (m/s)    — chassis.vy
   * @param wheel_speeds   四轮编码器转速 (rad/s)       — move_data.current_drive_speed
   * @param wheel_angles   四轮当前舵角 (rad)           — move_data.current_pivot_angle
   * @param wheel_radius   轮子半径 (m)
   */
  // [解析] 核心更新：一次性算完四个轮子的滑移率与惩罚；参数单位必须对上（rad/s、m/s、rad、m）。
  void update(
    float imu_wz, float chassis_vx, float chassis_vy, const float wheel_speeds[4],
    const float wheel_angles[4], float wheel_radius);

  /**
   * @brief 对功率分配权重施加滑移惩罚并重新归一化
   *
   * 用法：在 HKUST 算法算出 K_original[4] 之后、解二次方程之前调用。
   * 不打滑的轮子 K 值几乎不变，打滑轮 K 值被压缩，释放的权重自动流向抓地轮。
   *
   * @param K_original   HKUST 算法原始分配权重 [4]
   * @param K_corrected  施加惩罚并归一化后的修正权重 [4]
   */
  // [解析] 权重乘惩罚后重新归一化：打滑轮让出的份额按比例流向抓地轮，四个权重之和仍为 1。
  void apply_slip_penalty(const float K_original[4], float K_corrected[4]);

  /**
   * @brief 获取当前整体滑移限幅系数（用于全部驱动轮统一降功率）
   * @return [0.3, 1.0]  1.0=不限幅, 越小限幅越狠
   */
  // [解析] 整体限幅系数 [0.3, 1.0]，四轮都滑、轮间再分配也没用时用它统一降功率。
  float get_global_slip_scale() const;

private:
  // 底盘几何
  // [解析] l_/w_ 是轮距的一半，四个轮位 (+l,−w)(+l,+w)(−l,+w)(−l,−w) 由它们拼出来。
  const float l_, w_;

  // 滤波与增益参数
  const float alpha_slip_;
  const float penalty_gain_;
  const float slip_threshold_;

  // 初始化标志
  bool inited_;

  // 内部低通状态
  // [解析] 第一次 update() 没有滤波历史，直接令 filtered = raw，避免从 0 爬升被误判成打滑。
  float filtered_slip_[4];
};

}  // namespace sp

#endif  // SP__SLIP_DETECT_HPP
