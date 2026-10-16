// [解析] ==========================================================================
// [解析] 【文件】sp_middleware/tools/slip_detect/slip_detect.cpp
// [解析] 【作用】滑移检测的实现：核心就一个循环，对 4 个轮子分别做
// [解析]   "算轮速 → 算该轮位置的地面速度 → 投影到滚动方向 → 相减得滑移率 →
// [解析]    低通滤波 → 换算成功率惩罚系数"，最后再统计整体打滑状态。
// [解析] 【看代码的重点】
// [解析]   1) px[]/py[] 四个轮位的坐标是怎么来的（车体系：x 向前、y 向左）；
// [解析]   2) v_gnd_x = vx - ωz·y、v_gnd_y = vy + ωz·x 这两行是刚体速度场公式；
// [解析]   3) 为什么分母取 max(|v_wheel|, |v_ground|)（保证 |λ| ≤ 1）；
// [解析]   4) 低速死区 kSpeedDeadZone 为什么必须有（速度太小时噪声会把 λ 顶到 ±1）。
// [解析] 【注意】本文件没有使用 volatile / 互斥，设计上假定只有底盘控制一个任务在调用它。
// [解析] ==========================================================================

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

// [解析] 构造函数：把几何尺寸和参数存进 const 成员，并把输出/滤波状态清零。
// [解析]   几何：l_ = 前后轮距一半，w_ = 左右轮距一半，单位 m（用来算四个轮子在哪）。
// [解析]   参数：alpha_slip_（低通系数，越大越快）、penalty_gain_（惩罚强度）、
// [解析]         slip_threshold_（判定打滑的 |λ| 门槛）。
// [解析]   输出：any_slipping = false、avg_slip_ratio = 0、slip_wz = 0（上电先给个安全值）。
// [解析]   状态：inited_ = false，让第一次 update() 直接采信原始滑移率、不做滤波。
// [解析] 小提醒：这个初始化列表的顺序（l_、w_、alpha_… 之后才 any_slipping）
// [解析]   和头文件里的成员声明顺序（any_slipping 等 public 成员在前）并不一致，
// [解析]   实际初始化顺序永远按"声明顺序"走。这里各成员互相独立，所以没有副作用，
// [解析]   但开了 -Wreorder 的编译器会报个警告，知道原因就行。
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
  // [解析] 四个轮子依次初始化为"没打滑、不惩罚"的默认状态。
  // [解析]   wheel_slip[i] = {0.0f, 1.0f, 0.0f, 0.0f}; 是按结构体成员顺序的聚合初始化：
  // [解析]     slip_ratio = 0.0（无滑移）、slip_penalty = 1.0（权重不变）、
  // [解析]     v_wheel = 0.0、v_ground = 0.0。
  // [解析]   ★ 这四个数的顺序由 WheelSlipData 的成员顺序决定，改结构体就要回来改这里。
  // [解析]   filtered_slip_ 是低通滤波器的历史状态，清零即可。
  for (int i = 0; i < 4; i++) {
    wheel_slip[i] = {0.0f, 1.0f, 0.0f, 0.0f};
    filtered_slip_[i] = 0.0f;
  }
}

// [解析] update()：本模块的核心，一个循环搞定四个轮子的滑移检测。
// [解析] 参数（全部是当前时刻的测量值，单位务必对上，否则算出来的 λ 毫无意义）：
// [解析]   imu_wz       —— 陀螺仪 z 轴角速度，单位 rad/s（逆时针为正）。
// [解析]   chassis_vx/vy—— 车体系线速度，x 向前、y 向左，单位 m/s（轮式里程计给的）。
// [解析]   wheel_speeds —— 四轮"滚动方向"的角速度，单位 rad/s（不是 rpm，用前要换算）。
// [解析]   wheel_angles —— 四轮舵角（轮子朝向相对车体 x 轴的夹角），单位 rad。
// [解析]   wheel_radius —— 轮子半径，单位 m。
// [解析] 副作用：更新 wheel_slip[0..3]、any_slipping、avg_slip_ratio，并把 inited_ 置 true。
// [解析] 算法流程（对应下面 7 个编号注释）：
// [解析]   ① 轮速 = 轮子角速度 × 半径；② 用刚体速度场算轮位处的车速；③ 投影到轮子滚动方向；
// [解析]   ④ 相减除以较大者得到滑移率；⑤ 低通滤波；⑥ 换算成功效惩罚系数；⑦ 统计整体状态。
void SlipDetect::update(
  float imu_wz, float chassis_vx, float chassis_vy, const float wheel_speeds[4],
  const float wheel_angles[4], float wheel_radius)
{
  // ---- 轮位坐标 (车体坐标系: X向前, Y向左) ----
  //   [0]=RF: (x=+l, y=-w)
  //   [1]=LF: (x=+l, y=+w)
  //   [2]=LR: (x=-l, y=+w)
  //   [3]=RR: (x=-l, y=-w)
  // [解析] px/py 就是四个轮子在车体系里的坐标（单位 m），下标顺序 [RF, LF, LR, RR]。
  // [解析]   右前在 x 正、y 负；左前在 x 正、y 正，依此类推（y 轴向左为正，所以左边是 +w）。
  // [解析]   这两个数组是后面刚体速度场公式的"半径向量"，顺序千万不能乱。
  const float px[4] = {+l_, +l_, -l_, -l_};
  const float py[4] = {-w_, +w_, +w_, -w_};

  // ---- 角速度滑移量 (整体指标) ----
  // chassis_wz 没有直接传入，从轮速估算意义不大，
  // 这里用 imu_wz 和轮速的差异来估计
  // 注意：slip_wz 需要外部配合 chassis.wz 来计算
  // 这里保留为整体观测指标
  // [解析] 注意：上面这段是作者的说明，读起来像"功能待补充"。
  // [解析]   实际代码里确实既没有用到 chassis_wz，也没有给 slip_wz 赋过值，
  // [解析]   所以 slip_wz 会永远是构造函数里那个 0（想用的话得自己按注释补上）。

  // [解析] slip_sum 累加四轮 |λ| 用来求平均；slip_count 记录有效轮数。
  // [解析] any_slipping 先清掉，本次循环里只要有一个轮超阈值就会被置 true。
  float slip_sum = 0.0f;
  int slip_count = 0;
  any_slipping = false;

  // [解析] 循环四次，每次处理一个轮子。i 的含义见上面 px/py：0=RF 右前, 1=LF 左前,
  // [解析]   2=LR 左后, 3=RR 右后。所有中间量都是局部变量，下次循环自动丢弃。
  for (int i = 0; i < 4; i++) {
    // --- 1. 轮子线速度 (编码器) ---
    // [解析] 角速度 × 半径 = 线速度，单位 m/s。这是"轮子自己以为自己跑了多快"。
    // [解析] 注意符号：rad/s 为正代表车前进方向滚动；若编码器装反整体符号会反，
    // [解析]   那样算出来的 λ 会整体变号，看上去就像所有轮子都在反着打滑。
    float v_wheel = wheel_speeds[i] * wheel_radius;
    wheel_slip[i].v_wheel = v_wheel;

    // --- 2. 底盘在轮位处的真实线速度 ---
    //     v_at_wheel_x = vx - ωz × y_i   (刚体速度场)
    //     v_at_wheel_y = vy + ωz × x_i
    // [解析] 这两行是刚体运动学：v(某点) = v(质心) + ω × r，ω=(0,0,ωz)，r=(px,py,0)，
    // [解析]   算出来 ω × r = (−ωz·py, +ωz·px, 0)，于是就有了下面的形式。
    // [解析] 物理意义：车不仅整体平移，还在旋转，所以每个轮子所在位置的速度各不相同
    // [解析]   （离旋转中心越远的轮子跑得越快，这就是转弯时内外轮速度差的原因）。
    float v_gnd_x = chassis_vx - imu_wz * py[i];
    float v_gnd_y = chassis_vy + imu_wz * px[i];

    // --- 3. 投影到轮子滚动方向 ---
    // [解析] 舵轮/麦轮的轮子只有"滚动方向"上的速度由驱动电机产生（垂直方向靠被动滑动），
    // [解析]   所以要把车速向量投影到轮子的滚动方向上：v·e = vx·cosθ + vy·sinθ。
    // [解析] wheel_angles[i] 是轮子朝向相对车体 x 轴的夹角，单位 rad。
    // [解析] 注意：本项目约定角度是从车体 x 轴量起；如果实际标定时是从 y 轴量起，
    // [解析]   cos/sin 要对调，否则投影结果是错的（这个约定必须和底盘运动学保持一致）。
    float cos_a = std::cos(wheel_angles[i]);
    float sin_a = std::sin(wheel_angles[i]);
    float v_ground = v_gnd_x * cos_a + v_gnd_y * sin_a;
    wheel_slip[i].v_ground = v_ground;

    // --- 4. 滑移率计算 ---
    // [解析] 分母取两者绝对值的较大者：这样 |v_wheel − v_ground| ≤ max，滑移率天然落在
    // [解析]   [−1, 1] 内（下面的钳制其实是防御性写法，正常情况下不会触发）。
    float max_v = std::max(std::abs(v_wheel), std::abs(v_ground));
    float raw_slip = 0.0f;

    // 低速死区: 速度太低时滑移率无意义 (噪声主导)
    // [解析] 低速死区：车速和轮速都低于 0.05 m/s（5 cm/s）时，索性认定"没打滑"（λ=0）。
    // [解析]   原因：速度接近 0 时分母极小，编码器量化误差和 IMU 噪声都会被放大成 ±1 的
    // [解析]   假滑移，导致车一停下来就疯狂降功率。这个阈值就是为了挡住这种假阳性。
    constexpr float kSpeedDeadZone = 0.05f;  // 5 cm/s
    if (max_v > kSpeedDeadZone) {
      // [解析] 滑移率定义式：λ = (轮速 − 地面速度) / max(|轮速|, |地面速度|)。
      // [解析]   正 = 轮子转得比车快（空转），负 = 轮子转得比车慢（抱死）。
      raw_slip = (v_wheel - v_ground) / max_v;
      // 钳制到 [-1, 1]
      // [解析] 双保险：即使上面的推导保证不越界，也再钳一次，防止异常输入传到惩罚系数里。
      if (raw_slip > 1.0f) raw_slip = 1.0f;
      if (raw_slip < -1.0f) raw_slip = -1.0f;
    }

    // --- 5. 低通滤波 ---
    // [解析] 一阶低通（指数加权平均）：filtered = α·raw + (1−α)·filtered_last。
    // [解析]   目的：滑移率原始值抖动很大（编码器量化 + IMU 噪声），直接拿去惩罚功率
    // [解析]   会让动力一抽一抽的。α 越大越跟手但越抖，α 越小越平滑但越迟钝（默认 0.08）。
    // [解析] 第一次调用时没有历史值，直接令 filtered = raw，跳过递推：
    // [解析]   否则从 0 慢慢往上爬的过程会被误判成"正在打滑"。
    if (!inited_) {
      filtered_slip_[i] = raw_slip;
    }
    else {
      filtered_slip_[i] =
        alpha_slip_ * raw_slip + (1.0f - alpha_slip_) * filtered_slip_[i];
    }
    wheel_slip[i].slip_ratio = filtered_slip_[i];

    // --- 6. 惩罚系数 ---
    // [解析] 惩罚公式：penalty = 1 − gain × |λ|。用绝对值是因为空转和抱死都应该被惩罚。
    // [解析]   默认 gain = 1.5，即 |λ| ≈ 0.67 时惩罚就压到下限了，属于比较激进的取值。
    float abs_slip = std::abs(filtered_slip_[i]);
    wheel_slip[i].slip_penalty = 1.0f - penalty_gain_ * abs_slip;
    // 钳制: 不完全切断功率，保留最低 5%
    // [解析] 下限 0.05：再滑也不把动力完全掐掉，否则车会瞬间失去姿态控制能力。
    // [解析] 上限 1.0：防止 |λ| 出现异常小时把权重放大到 1 以上（那样归一化后别的轮会更小）。
    if (wheel_slip[i].slip_penalty < 0.05f) {
      wheel_slip[i].slip_penalty = 0.05f;
    }
    if (wheel_slip[i].slip_penalty > 1.0f) {
      wheel_slip[i].slip_penalty = 1.0f;
    }

    // --- 7. 整体状态 ---
    // [解析] 只要有一个轮子的 |λ| 超过阈值，any_slipping 就是 true（本周期内一旦置真不再清）。
    // [解析] slip_sum 累加 |λ|，循环结束后除以轮数得到 avg_slip_ratio。
    if (abs_slip > slip_threshold_) {
      any_slipping = true;
    }
    slip_sum += abs_slip;
    slip_count++;
  }

  // [解析] 循环结束，说明四个轮子都算完了，以后就不再需要"首次直通"的特殊处理。
  inited_ = true;
  // [解析] 求平均。这里的三目运算符其实是防御性写法：slip_count 在循环里必定累加到 4，
  // [解析]   永远不可能为 0；写成这样大概是怕以后改成"跳过某些轮子"的写法。
  avg_slip_ratio = (slip_count > 0) ? (slip_sum / static_cast<float>(slip_count)) : 0.0f;
}

// [解析] apply_slip_penalty()：把滑移惩罚作用到功率分配权重上，并重新归一化。
// [解析] 为什么需要"归一化"？因为 HKUST 功率分配算法默认四个权重加起来是 1（代表总功率的
// [解析]   分配比例）。乘完惩罚后总和会小于 1，如果直接送进去，车就相当于白丢了一部分功率；
// [解析]   重新归一化等于把打滑轮省下的那份功率按比例补给抓地轮，总功率不浪费。
// [解析] 参数：K_original 只读输入（4 个权重）；K_corrected 输出（调用者准备好的 4 个 float）。
// [解析] 副作用：只写 K_corrected，不修改对象内部状态。
void SlipDetect::apply_slip_penalty(const float K_original[4], float K_corrected[4])
{
  // 每个轮子的权重乘以对应的滑移惩罚系数
  // [解析] 逐元素相乘。slip_penalty ∈ [0.05, 1]，所以这一步只会把权重压小、不会放大。
  for (int i = 0; i < 4; i++) {
    K_corrected[i] = K_original[i] * wheel_slip[i].slip_penalty;
  }

  // 重新归一化: 释放的功率额度按比例自动流向不打滑的轮子
  float sum = K_corrected[0] + K_corrected[1] + K_corrected[2] + K_corrected[3];

  // [解析] 1e-6 是很小的容差：只有在"四个轮子权重几乎全是 0"时才走 else。
  // [解析]   用 1e-6 而不是直接判 > 0，是为了避免除以一个极小值把权重放大成天文数字。
  if (sum > 1e-6f) {
    float inv_sum = 1.0f / sum;
    // [解析] 乘以倒数（而不是每个都做除法），少 3 次浮点除法，在 1kHz 的控制循环里更划算。
    for (int i = 0; i < 4; i++) {
      K_corrected[i] *= inv_sum;
    }
  }
  else {
    // 所有轮子都严重打滑 → 均分，让每个轮子至少有一些基础力矩维持姿态
    // [解析] 均分 0.25 是一种"保底策略"：此时已经无法分辨哪个轮子抓地更好，
    // [解析]   与其全给某一个轮子，不如四轮平均，至少还能维持车体姿态不失控。
    for (int i = 0; i < 4; i++) {
      K_corrected[i] = 0.25f;
    }
  }
}

// [解析] get_global_slip_scale()：把整体打滑程度映射成一个"全局降功率系数"。
// [解析] 与 apply_slip_penalty() 的区别：那个是"轮间重新分配"（总量不变），
// [解析]   这个是"整体限幅"（总量一起变小），用于四个轮子都滑、再怎么分配也没用的场合。
// [解析] 返回值：范围 [0.3, 1.0]，1.0 = 完全不限幅。const 函数，不改对象状态。
float SlipDetect::get_global_slip_scale() const
{
  // 平均滑移率超过阈值时，输出整体限幅系数
  // 映射: avg_slip > 0.3 开始限幅, avg_slip = 0.8 时限到 0.3
  // [解析] 注意：上面这行注释里的 0.3 是"举例用的默认阈值"（默认 slip_threshold_ = 0.25），
  // [解析]   实际门槛由成员 slip_threshold_ 决定，不要把它俩混为一谈。
  // [解析] 没超过阈值 → 直接返回 1.0（不限幅），这是绝大多数时间的正常分支。
  if (avg_slip_ratio < slip_threshold_) {
    return 1.0f;
  }

  // 线性插值: [threshold, 0.8] → [1.0, 0.3]
  // [解析] t 是归一化位置：avg_slip_ratio 从 slip_threshold_ 涨到 0.8 时，t 从 0 变到 1。
  // [解析] 分母 (0.8f - slip_threshold_) 默认是 0.55；如果构造时把 threshold 传成 0.8，
  // [解析]   这里就会除以 0 → t 变成 inf 或 nan（作者没有做保护，传参时避开 0.8 即可）。
  float t = (avg_slip_ratio - slip_threshold_) / (0.8f - slip_threshold_);
  // [解析] 上限钳到 1：平均滑移率超过 0.8 以后就不再继续加重惩罚，保持最低 0.3。
  if (t > 1.0f) t = 1.0f;

  // [解析] 把 t ∈ [0,1] 线性映射到 [1.0, 0.3]：scale = 1 − 0.7t。
  // [解析]   0.7 就是"最多砍掉 70% 功率"，这个下限是经验值：砍到 0.3 以下基本就动不了了。
  float scale = 1.0f - t * 0.7f;  // 最多降到 0.3
  return scale;
}

}  // namespace sp
