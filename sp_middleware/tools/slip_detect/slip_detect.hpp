// [解析] ==========================================================================
// [解析] 【文件】sp_middleware/tools/slip_detect/slip_detect.hpp
// [解析] 【作用】舵轮底盘"打滑检测"模块：拿编码器推算的轮速和 IMU 测出的真实车速做比较，
// [解析]         算出四个轮子各自的滑移率，再把它换算成"功率惩罚系数"，
// [解析]         用来把功率从空转的轮子挪给还抓着地的轮子。
// [解析] 【被谁用】全仓库搜索 "SlipDetect" 只在它自己的 .hpp/.cpp 里出现，
// [解析]           目前没有任何业务代码 new/定义过它（属于预留能力，等功率分配模块接入）。
// [解析]           按设计意图，应该在底盘功率分配（HKUST 算法）之前调用：
// [解析]           apply_slip_penalty() 修正每个轮的权重，get_global_slip_scale() 做整体限幅。
// [解析] 【关键点】
// [解析]   1) 轮序固定为 [0]=RF 右前, [1]=LF 左前, [2]=LR 左后, [3]=RR 右后，
// [解析]      与 sp_middleware 里 chassis_para.hpp 的 move_t 一致，顺序传错就会张冠李戴。
// [解析]   2) 滑移率 λ 的正负含义：正 = 轮子转得比车快（驱动空转），负 = 轮子转得比车慢（刹车抱死）。
// [解析]   3) 这里用的是 imu_wz（陀螺仪 z 轴角速度）而不是底盘自己算的 wz，
// [解析]      因为"打滑"恰恰意味着轮式里程计算出来的 wz 已经不可信了。
// [解析] ==========================================================================

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
// [解析] WheelSlipData：一个轮子的"体检报告"，4 个 float 全是只读输出。
// [解析] 成员顺序和 .cpp 里 wheel_slip[i] = {0.0f, 1.0f, 0.0f, 0.0f}; 这句初始化是严格对应的，
// [解析]   所以千万不要在这个结构体中间插成员，否则那句初始化会整体错位。
struct WheelSlipData
{
  // [解析] slip_ratio：滑移率 λ，范围 [-1, 1]，量纲为 1（无单位）。
  // [解析]   0    = 轮速与车速完全一致，没打滑；
  // [解析]   正   = 轮子转得比车快 → 驱动空转（油门给大了/地面滑）；
  // [解析]   负   = 轮子转得比车慢 → 制动抱死（刹车过猛/轮胎锁死）。
  // [解析]   这里是"低通滤波之后"的值，所以不会突变。
  float slip_ratio;   ///< 滑移率 [-1, 1]  正=驱动空转, 负=制动抱死
  // [解析] slip_penalty：功率惩罚系数，范围 [0.05, 1]（注意不是 [0,1]，是有下限的）。
  // [解析]   1    = 完全信任这个轮子，权重不变；
  // [解析]   0.05 = 几乎切断这个轮子的动力，但留 5% 底线，避免完全失去控制。
  // [解析]   算法：1 - penalty_gain × |λ|，再钳到下限 0.05。
  float slip_penalty; ///< 功率惩罚系数 [0, 1]  1=无惩罚, ~0=几乎切断
  // [解析] v_wheel：编码器推算的轮子线速度，单位 m/s。算法：轮子角速度(rad/s) × 轮子半径(m)。
  float v_wheel;      ///< 轮子线速度 (m/s)  来自编码器
  // [解析] v_ground：由 IMU 真实角速度和底盘线速度，按刚体速度场投影到"轮子滚动方向"得到的
  // [解析]   地面真实速度分量，单位 m/s。它是"这个轮子理论上应该跑多快"，作为对比基准。
  float v_ground;     ///< 地面真实速度投影 (m/s)  来自IMU+运动学
};

// [解析] SlipDetect 类：一个对象管理整台车的四轮打滑检测。
// [解析] 典型用法（每控制周期一次）：
// [解析]   1) slip.update(imu.vyaw, chassis.vx, chassis.vy, speeds, angles, 0.06f);
// [解析]   2) 功率分配前：slip.apply_slip_penalty(K_hkust, K_fixed);
// [解析]   3) 需要整体降功率时：scale = slip.get_global_slip_scale();
class SlipDetect
{
public:
  // [解析] 构造函数：只把几何尺寸和滤波/增益参数存进 const 成员，不做任何计算。
  // [解析]   half_length  —— 前后轮距的一半，单位 m。例如轴距 0.6m 就填 0.3f。
  // [解析]   half_width   —— 左右轮距的一半，单位 m。这两个值用来算"轮子在车体上的坐标"。
  // [解析]   alpha_slip   —— 一阶低通滤波系数 (0~1]，默认 0.08f。
  // [解析]                   公式 filtered = α·raw + (1-α)·filtered_last，
  // [解析]                   α 越大响应越快但噪声越大，α 越小越平滑但更滞后。
  // [解析]                   0.08 对应大约 12 个周期的时间常数，属于"很平滑"的取值。
  // [解析]   penalty_gain —— 惩罚增益 (>0)，默认 1.5f。|λ| 每 1.0，权重就砍掉 1.5 倍；
  // [解析]                   越大惩罚越激进（更容易把打滑轮的动力掐掉）。
  // [解析]   slip_threshold —— 判定阈值，默认 0.25f。|λ| 超过它才算"这个轮子在打滑"。
  // [解析] 注意：这些参数都是 const 成员，构造之后就改不了了（想改只能重建对象）。
  /**
   * @param half_length   底盘前后轮距的一半 (m)
   * @param half_width    底盘左右轮距的一半 (m)
   * @param alpha_slip    滑移率低通滤波系数 (0~1), 越大响应越快但越噪声敏感
   * @param penalty_gain  惩罚增益 (>0), 越大对滑移的惩罚越激进
   * @param slip_threshold 滑移判定阈值, |λ|超过此值认为打滑
   */
  SlipDetect(
    float half_length, float half_width, float alpha_slip = 0.08f,
    float penalty_gain = 1.5f, float slip_threshold = 0.25f);

  // ---- 只读输出 ----
  // [解析] wheel_slip：四个轮子的详细滑移数据（结构体见上），顺序 [RF, LF, LR, RR]。
  WheelSlipData wheel_slip[4]; ///< 四轮滑移数据 [RF, LF, LR, RR]
  // [解析] any_slipping：本次 update() 里只要有一个轮的 |λ| 超过 slip_threshold_ 就为 true。
  // [解析]   适合做"是否触发打滑保护"的开关判断。
  bool any_slipping;           ///< 是否有任意轮子在打滑
  // [解析] avg_slip_ratio：四轮 |λ| 的算术平均，范围 [0, 1]，反映"整体打滑程度"。
  float avg_slip_ratio;        ///< 四轮平均绝对滑移率 [0, 1]
  // [解析] slip_wz：本意是"角速度滑移量 chassis.wz - imu_wz"，单位 rad/s。
  // [解析] 注意：当前实现里它只被初始化为 0，update() 并没有写它（因为 update 没有收
  // [解析]   chassis.wz 参数），所以现在读它永远是 0，属于作者留的待办。
  float slip_wz;               ///< 角速度滑移量: chassis.wz - imu_wz (rad/s)

  // [解析] update()：核心更新函数，每个控制周期调用一次，一次性算完四个轮子的滑移率与惩罚。
  // [解析] 所有输入都是"当前时刻的测量值"，函数没有返回值，结果写在上面的公共成员里。
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
  // [解析] 参数补充说明（新手最容易搞错的单位）：
  // [解析]   imu_wz       —— 旋转角速度，右手定则，逆时针为正，单位 rad/s。
  // [解析]   chassis_vx/vy—— 车体系下的线速度：x 向前、y 向左，单位 m/s（轮式里程计给的）。
  // [解析]   wheel_speeds —— 轮子"滚动"的角速度，单位 rad/s（不是 rpm！用之前要换算）。
  // [解析]   wheel_angles —— 舵轮当前朝向（舵角）相对车体 x 轴的夹角，单位 rad。
  // [解析]   wheel_radius —— 轮子半径，单位 m（本项目麦轮/舵轮一般是 0.06f 左右）。
  // [解析] 副作用：会更新 wheel_slip[]、any_slipping、avg_slip_ratio，并把 inited_ 置为 true。
  void update(
    float imu_wz, float chassis_vx, float chassis_vy, const float wheel_speeds[4],
    const float wheel_angles[4], float wheel_radius);

  // [解析] apply_slip_penalty()：给功率分配权重乘上滑移惩罚，再整体归一化。
  // [解析] 效果：不打滑的轮子权重几乎不变；打滑的轮子权重被压小，省下来的功率按比例
  // [解析]   自动分给还抓着地的轮子，所以四个权重之和仍然是 1。
  /**
   * @brief 对功率分配权重施加滑移惩罚并重新归一化
   *
   * 用法：在 HKUST 算法算出 K_original[4] 之后、解二次方程之前调用。
   * 不打滑的轮子 K 值几乎不变，打滑轮 K 值被压缩，释放的权重自动流向抓地轮。
   *
   * @param K_original   HKUST 算法原始分配权重 [4]
   * @param K_corrected  施加惩罚并归一化后的修正权重 [4]
   */
  // [解析] 输入/输出：K_original 是只读输入（4 个权重，一般加起来是 1）；
  // [解析]   K_corrected 是输出数组，调用者负责准备好 4 个 float 的空间，函数会覆盖它。
  // [解析] 注意：K_corrected 允许和 K_original 是同一个数组（原地修正也能算对，
  // [解析]   因为第一步只是逐个乘惩罚系数，不依赖别的元素）。
  void apply_slip_penalty(const float K_original[4], float K_corrected[4]);

  // [解析] get_global_slip_scale()：把"整体打滑程度"映射成一个全局限幅系数。
  // [解析] const 成员函数，不修改对象状态；返回 [0.3, 1.0]，1.0 表示不限幅。
  /**
   * @brief 获取当前整体滑移限幅系数（用于全部驱动轮统一降功率）
   * @return [0.3, 1.0]  1.0=不限幅, 越小限幅越狠
   */
  float get_global_slip_scale() const;

private:
  // 底盘几何
  // [解析] l_、w_：前后轮距一半、左右轮距一半，单位 m（const，构造时定死）。
  // [解析]   轮位坐标就是靠它们拼出来的：右前(+l,-w)、左前(+l,+w)、左后(-l,+w)、右后(-l,-w)。
  const float l_, w_;

  // 滤波与增益参数
  // [解析] 三个参数同构造函数说明，全部是 const，构造后不可改。
  const float alpha_slip_;
  const float penalty_gain_;
  const float slip_threshold_;

  // 初始化标志
  // [解析] inited_：第一次 update() 时低通滤波器还没有历史值，所以第一次直接令
  // [解析]   filtered = raw（跳过滤波），避免从 0 慢慢爬上来产生的假"打滑"。
  bool inited_;

  // 内部低通状态
  // [解析] filtered_slip_：四个轮子各自的滤波状态（上一次的输出），跨周期保留。
  float filtered_slip_[4];
};

}  // namespace sp

#endif  // SP__SLIP_DETECT_HPP
