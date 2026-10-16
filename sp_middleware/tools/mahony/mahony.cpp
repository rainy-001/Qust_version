// [解析] ==========================================================================
// [解析] 【文件】sp_middleware/tools/mahony/mahony.cpp
// [解析] 【作用】Mahony 互补滤波姿态解算的全部实现：一边积分陀螺仪，一边用加速度计纠偏。
// [解析] 【看代码的重点】这是全工程数学味最重的文件，最难看懂的是这四块：
// [解析]   1) update() 里"用四元数估计出重力方向 → 与实测重力做叉乘 → 得到姿态误差"这一步；
// [解析]   2) 误差乘以 kp（比例项）和 ki（积分项）后，反过来加到陀螺仪读数上，再积分成四元数；
// [解析]   3) 四元数微分方程 q̇ = 0.5 · q ⊗ (0, ω) 展开出来的那 4 行加法；
// [解析]   4) pitch_geom_calc() 用 x/z 轴在世界系的投影判断"是否倒扣"，把 pitch 值域扩到 ±π。
// [解析] 【大白话】陀螺仪短期准、长期会漂；加速度计长期准、短期抖而且怕车体加速度。
// [解析]   Mahony 干的事就是：一直相信陀螺仪积分，但用加速度计测到的重力方向不断把它"拽"回来。
// [解析] ==========================================================================

#include "mahony.hpp"

#include <cmath>

#include "tools/math_tools/math_tools.hpp"
namespace sp
{
// [解析] 构造函数：初始化列表 `: 成员(值)` 的写法等价于在函数体里逐个赋值，只是效率更高。
// [解析]   dt_      = dt              采样周期（s），全类共用，const 不能改
// [解析]   two_kp_  = 2 * kp          Mahony 公式里的 2 倍系数提前乘好
// [解析]   two_ki_  = 2 * ki          同上
// [解析]   inited_  = false           表示"还没做初始对准"，第一次 update() 会走 init()
// [解析]   integral_fbx_/fby_/fbz_ = 0  积分项清零，防止上电就带着历史误差
// [解析] 注意：函数体是空的 —— 构造函数不访问硬件、不做计算，这样上电顺序更可控。
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

// [解析] 数组版 update()：纯粹是个"转发函数"，把两个数组拆成 6 个 float 再调真正的实现。
// [解析] 这么写是因为数组做参数会退化成指针，拆成单变量后编译器更容易优化、也更好读。
// [解析] acc = {ax, ay, az} 单位 m/s²；gyro = {wx, wy, wz} 单位 rad/s。
void Mahony::update(const float acc[3], const float gyro[3])
{
  update(acc[0], acc[1], acc[2], gyro[0], gyro[1], gyro[2]);
}

// [解析] update()：姿态解算主函数，每个采样周期调用一次（本项目 imu_task 里 1ms 一次）。
// [解析] 参数：ax/ay/az 加速度计读数（单位 m/s²，含重力，载体系）；
// [解析]       wx/wy/wz 陀螺仪读数（单位 rad/s，载体系）。
// [解析] 返回值：无。结果写进 q、yaw/pitch/roll、vyaw/vpitch/vroll、w、q_last 等公共成员。
// [解析] 整体思路可以概括成 5 步，读代码时照着这 5 步对：
// [解析]   ① 上电第一拍：只用加速度计定初始姿态，然后直接 return；
// [解析]   ② 把加速度归一化，得到"实测重力方向"；
// [解析]   ③ 用当前四元数算出"估计重力方向"，两者叉乘得到姿态误差；
// [解析]   ④ 误差经 PI 反馈加到陀螺仪角速度上 —— 这就是"纠偏"；
// [解析]   ⑤ 用修正后的角速度积分四元数、归一化，最后换算成欧拉角。
void Mahony::update(float ax, float ay, float az, float wx, float wy, float wz)
{
  // [解析] 上电后第一次调用不能积分：那时 q 里还没有任何有效姿态。
  // [解析] 所以先用加速度计（重力方向）定出初始 roll/pitch，做完直接 return。
  // [解析] inited_ 被置 true，所以这段只会执行一次 —— 这是很常用的一次性初始化写法。
  if (!inited_) {
    inited_ = true;
    init(ax, ay, az);
    return;
  }
  // [解析] 把陀螺仪原始读数存进公共成员 w[]，供外部查看（只读输出）。
  // [解析] 注意存的时机：必须在下面被 PI 修正"之前"，所以 w[] 是纯原始值、未融合。
  // [解析] 下标含义：w[0]=绕车体 x 轴（侧倾方向）、w[1]=绕 y 轴（俯仰方向）、w[2]=绕 z 轴（偏航）。
  w[0] = wx;
  w[1] = wy;
  w[2] = wz;
  // [解析] ★ 很关键的一手备份：下面 wx/wy/wz 会被姿态误差"改写"成修正后的角速度，
  // [解析]   但本函数末尾算欧拉角速率时用的是"原始角速度"，所以先复制一份到 gx/gy/gz。
  // [解析]   忘了备份就会出现"用修正值算欧拉角速率"的隐性错误。
  float gx = wx;
  float gy = wy;
  float gz = wz;

  // Normalise accelerometer measurement
  // [解析] 归一化：把加速度缩成"长度为 1 的单位向量"，只保留"重力指向哪边"这个方向信息。
  // [解析]   为什么不直接用原始值？因为加速度的模会随车体运动变化（弹跳、加速都会变），
  // [解析]   而我们只关心方向。norm = 1/|a| 是"乘倒数代替除法"，在无 FPU 除法的场景更快。
  // [解析] 潜在风险：若 |a| 极小（自由落体或传感器异常），算出的 norm 会爆成 inf；
  // [解析]   本工程没做保护，因为正常行驶时 |a| ≈ 9.8，不会触发。
  float norm = 1.0f / std::sqrt(ax * ax + ay * ay + az * az);
  ax *= norm;
  ay *= norm;
  az *= norm;

  // Estimated direction of gravity and vector perpendicular to magnetic flux
  // [解析] 用当前四元数"估计"重力方向 —— 也就是把世界系的竖直向量转到载体系下。
  // [解析]   标准做法是取旋转矩阵的第三行：v = (2(xz-wy), 2(wx+yz), 2(w²+z²)-1)，
  // [解析]   代码里加了 half 前缀，说明存的是它的一半：
  // [解析]     halfvx = x·z − w·y          ← 对应 2(xz−wy) 的一半
  // [解析]     halfvy = w·x + y·z          ← 对应 2(wx+yz) 的一半
  // [解析]     halfvz = w² − 0.5 + z²      ← 用了单位四元数恒等式 w²+x²+y²+z²=1 化简
  // [解析]   取一半是为了和下面叉乘得到的 halfex/halfey/halfez 保持同尺度，
  // [解析]   丢掉的 2 倍最后由 kp/ki 补回来（这就是 two_kp_ / two_ki_ 的来历）。
  // [解析] 注意：变量名叫 "vector perpendicular to magnetic flux" 是从原始 Madgwick 代码
  // [解析]   抄来的（那边还有磁力计），本工程只用加速度计，把它当"重力方向估计"看即可。
  float halfvx = this->q[1] * this->q[3] - this->q[0] * this->q[2];
  float halfvy = this->q[0] * this->q[1] + this->q[2] * this->q[3];
  float halfvz = this->q[0] * this->q[0] - 0.5f + this->q[3] * this->q[3];

  // Error is sum of cross product between estimated and measured direction of gravity
  // [解析] 叉乘"实测重力方向 × 估计重力方向的一半"，得到姿态误差向量：
  // [解析]   姿态完全正确时两个方向重合，叉乘为 0 → 不加修正；
  // [解析]   不重合时叉乘结果垂直于两者，方向告诉我们"该绕哪根轴转"，大小 ≈ 夹角
  // [解析]   （小角度下 sinθ ≈ θ），正好可以当修正量。
  // [解析] 展开的就是右手定则的叉乘公式：e = a × halfv。
  float halfex = (ay * halfvz - az * halfvy);
  float halfey = (az * halfvx - ax * halfvz);
  float halfez = (ax * halfvy - ay * halfvx);

  // Compute and apply integral feedback if enabled
  // [解析] 只有 ki > 0 才启用积分项。ki 的作用是消掉陀螺仪的"零偏"：
  // [解析]   陀螺静止时输出并不是 0，而是一个固定的小常数，长期积分就会把姿态带偏；
  // [解析]   积分项会把持续存在的误差慢慢累加进角速度，正好抵消这个零偏。
  // [解析] 本项目默认 ki = 0（见 mahony.hpp 构造函数默认参数），所以平时走 else 分支。
  if (two_ki_ > 0.0f) {
    // integral error scaled by Ki
    // [解析] 积分累加：误差 × 2ki × dt。这是矩形法数值积分，dt 不准积分就不准。
    integral_fbx_ += two_ki_ * halfex * dt_;
    integral_fby_ += two_ki_ * halfey * dt_;
    integral_fbz_ += two_ki_ * halfez * dt_;

    // apply integral feedback
    // [解析] 把积分结果加进角速度，等价于对陀螺仪说"你偏了这么多，我帮你补回来"。
    wx += integral_fbx_;
    wy += integral_fby_;
    wz += integral_fbz_;
  }
  else {
    // prevent integral windup
    // [解析] ki = 0 时把积分器清零，防止它偷偷累成一个很大的值（积分饱和/windup），
    // [解析]   否则以后一开 ki，积攒的那一大坨会瞬间弹进角速度里。
    integral_fbx_ = 0.0f;
    integral_fby_ = 0.0f;
    integral_fbz_ = 0.0f;
  }

  // Apply proportional feedback
  // [解析] 比例项：误差 × 2kp 直接加进角速度，是让 pitch/roll 收敛的主力。
  // [解析]   kp 大 → 姿态被很快拽向重力方向，但车体加速度和振动也会被当成误差 → 抖；
  // [解析]   kp 小 → 主要靠陀螺仪积分，平滑但会慢慢漂。默认 0.5f 是折中。
  // [解析] ★ 注意：到这里 wx/wy/wz 已经不是原始角速度，而是"修正后的角速度"了，
  // [解析]   所以函数开头才要备份 gx/gy/gz（下面算欧拉角速率要用原始值）。
  wx += two_kp_ * halfex;
  wy += two_kp_ * halfey;
  wz += two_kp_ * halfez;

  // Integrate rate of change of quaternion
  // pre-multiply common factors
  // [解析] 四元数微分方程：q̇ = 0.5 · q ⊗ (0, ωx, ωy, ωz)。
  // [解析]   这三行先把公共因子 0.5·dt 乘进角速度，后面 4 行加法就不用重复乘了。
  // [解析]   整个更新用的是"一阶欧拉积分"：q_new = q_old + q̇·dt，简单但要求 dt 足够小
  // [解析]   （本项目 1ms，误差可以忽略；dt 越大截断误差越大）。
  wx *= 0.5f * dt_;
  wy *= 0.5f * dt_;
  wz *= 0.5f * dt_;

  // [解析] 把四元数乘法 q ⊗ (0, ω) 展开成 4 个分量：q0=w, q1=x, q2=y, q3=z。
  // [解析] 这些是"新四元数的临时值"，还没归一化，所以先不要往 this->q 里写。
  float q0 = this->q[0] + (-this->q[1] * wx - this->q[2] * wy - this->q[3] * wz);
  float q1 = this->q[1] + (this->q[0] * wx + this->q[2] * wz - this->q[3] * wy);
  float q2 = this->q[2] + (this->q[0] * wy - this->q[1] * wz + this->q[3] * wx);
  float q3 = this->q[3] + (this->q[0] * wz + this->q[1] * wy - this->q[2] * wx);

  // [解析] 在覆盖 q 之前，先把"上一次的四元数"备份到 q_last，供外部做增量/角速度估计。
  // [解析] 注意：init() 里不会写 q_last，所以上电后第一次完整 update() 之前它是未初始化值。
  this->q_last[0] = this->q[0];
  this->q_last[1] = this->q[1];
  this->q_last[2] = this->q[2];
  this->q_last[3] = this->q[3];

  // Normalise quaternion
  // [解析] 归一化：把新四元数除以自身模长，强制满足 w²+x²+y²+z²=1。
  // [解析]   为什么必须做？欧拉积分 + 浮点误差会让模长慢慢偏离 1，而后面所有姿态计算
  // [解析]   （欧拉角、重力方向估计）都假定它是单位四元数；不归一化姿态会越算越歪。
  // [解析] 这里复用了上面的变量 norm，属于"就地复用"，省一个变量。
  norm = 1.0f / std::sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  this->q[0] = q0 * norm;
  this->q[1] = q1 * norm;
  this->q[2] = q2 * norm;
  this->q[3] = q3 * norm;

  //只是用到了1=q[0]^2+q[1]^2+q[2]^2+q[3]^2这个关系简化了一些计算
  //以后的同学在用ai理解mahony算法的时候会更容易,形式更加相同
  //之后要加万向锁保护  ,我的理想是加根据历史记录进行保护
  // [解析] 下面三行是"四元数 → 欧拉角"的标准换算，采用 ZYX 内旋（先 yaw、再 pitch、最后 roll）：
  // [解析]   yaw   = atan2(2(wz+xy), 1−2(y²+z²))   —— 绕竖直轴，范围 (−π, π]
  // [解析]   pitch = asin (2(wy−xz))               —— 抬头为负/低头为正（取决于安装方向），范围 [−π/2, π/2]
  // [解析]   roll  = atan2(2(wx+yz), 1−2(x²+y²))   —— 左右侧倾，范围 (−π, π]
  // [解析] 为什么用 atan2 而不是 atan？因为 atan2(y, x) 会同时看分子分母的符号，
  // [解析]   能正确判断象限，范围覆盖整个圆；atan 只能给出 (−π/2, π/2)。
  // [解析] 为什么分母写成 "1 − 2(...)" 而不是矩阵里的原始元素？见上面作者注释：
  // [解析]   利用单位四元数恒等式 w²+x²+y²+z²=1 把 R33、R11 化简过，形式更统一。
  // [解析] 注意：pitch 用 asin，天然只能表示 ±90°；倾倒/倒扣超过 90° 时它会翻回来，
  // [解析]   这就是"万向锁"的表现，作者在注释里也写了后面要靠 pitch_geom 兜底。
  this->yaw = std::atan2(
    2.0f * (this->q[0] * this->q[3] + this->q[1] * this->q[2]),
    1.0f - 2.0f * (this->q[2] * this->q[2] + this->q[3] * this->q[3]));
  this->pitch = std::asin(2.0f * (this->q[0] * this->q[2] - this->q[1] * this->q[3]));
  this->roll = std::atan2(
    2.0f * (this->q[0] * this->q[1] + this->q[2] * this->q[3]),
    1.0f - 2.0f * (this->q[1] * this->q[1] + this->q[2] * this->q[2]));

  //计算欧拉角变化率
  // [解析] 用"原始"陀螺仪值 gx/gy/gz（不是被 PI 修正过的 wx/wy/wz）反解欧拉角速率，
  // [解析]   再带上刚算出来的 roll/pitch/yaw。结果写进 vroll/vpitch/vyaw。
  // [解析] 也可以直接对欧拉角做差分，但那会把噪声放大；反解法更平滑，代价是
  // [解析]   公式里有 1/cos(pitch)，接近万向锁时会发散。
  culculate_yaw_pitch_roll_rates(gx, gy, gz, this->roll, this->pitch, this->yaw);

  //将pitch进行值域扩充,解决串腿翻倒后自启问题
  // [解析] 计算扩展值域版本的 pitch（pitch_geom）及其微分 vpitch_geom。
  // [解析]   用途：串腿/翻车后自动爬起时，普通 pitch 会在倒扣瞬间跳变，扩展版不会。
  pitch_geom_calc();
}

// [解析] culculate_yaw_pitch_roll_rates()：由"载体系角速度 + 欧拉角"反解出欧拉角速率。
// [解析] 参数：wx/wy/wz 载体系角速度 (rad/s)；roll/pitch/yaw 当前欧拉角 (rad)。
// [解析] 结果直接写进成员 vroll/vpitch/vyaw（所以函数没有返回值）。
// [解析] 为什么要有这个函数：陀螺仪测的是"绕车体三轴"的角速度，而控制云台速度环需要的是
// [解析]   "欧拉角的变化率"。两者不是一回事，必须用运动学矩阵换算。
// [解析] 注意参数 yaw 在函数体里其实一次都没用到 —— 这不是笔误：
// [解析]   ZYX 欧拉角的三个速率只跟 roll、pitch 和角速度有关，与 yaw 本身无关，
// [解析]   保留这个参数只是为了调用处传参形式统一。
void Mahony::culculate_yaw_pitch_roll_rates(
  float wx, float wy, float wz, float roll, float pitch, float yaw)
{
  // [解析] 第一行（roll 速率）：三项分别是"绕车体 x 轴自转"、"绕 y 轴俯仰耦合到 roll"、
  // [解析]   "绕 z 轴偏航耦合到 roll"。系数里有 tan(pitch)，所以 pitch 接近 ±90° 时会发散。
  this->vroll = wx + wy * std::sin(roll) * std::tan(pitch) + wz * std::cos(roll) * std::tan(pitch);
  // [解析] 第二行（pitch 速率）：把载体系的 wy/wz 按 roll 角投影到"俯仰轴"上，最简单的一项。
  this->vpitch = wy * std::cos(roll) - wz * std::sin(roll);
  // [解析] 第三行（yaw 速率）：同理投影到"竖直轴"，但要注意是绕世界系竖直轴转，
  // [解析]   所以要再除以 cos(pitch) —— 车越接近立起来，同样的角速度对应的 yaw 变化越大。
  // [解析]   ★ 万向锁就出在这里：pitch → ±90° 时 1/cos(pitch) → ∞，vyaw 会爆掉。
  this->vyaw = wy * std::sin(roll) / std::cos(pitch) + wz * std::cos(roll) / std::cos(pitch);
}

// [解析] init()：上电后的初始对准。只看加速度计（重力方向），因为静止时重力最可靠。
// [解析] 参数：ax/ay/az 加速度计原始读数，单位 m/s²（含重力）。
// [解析] 结果：直接写 this->q（初始四元数）和 yaw/pitch/roll。
// [解析] 注意：init() 不会写 q_last / pitch_geom / vyaw 等成员，所以它们在第一拍仍是未初始化值。
void Mahony::init(float ax, float ay, float az)
{
  // [解析] 同样先归一化，把加速度变成单位向量（只关心重力方向，不关心大小）。
  float norm = 1.0f / std::sqrt(ax * ax + ay * ay + az * az);
  ax *= norm;
  ay *= norm;
  az *= norm;

  // [解析] 由重力方向反推初始俯仰/横滚角：
  // [解析]   pitch0 = atan2(−ax, az)：车头朝上仰时 ax 会变化，所以俯仰看 ax 与 az 的比。
  // [解析]   roll0  = atan2( ay, az)：左右侧倾看 ay 与 az 的比。
  // [解析]   严格公式里 pitch 的分母应该是 sqrt(ay²+az²)，这里用 az 近似；小角度下差别可忽略，
  // [解析]   但车体横滚很大时初始 pitch 会有偏差（上电时把车放平就没事）。
  // [解析]   yaw0 直接固定为 0：没有磁力计就无从得知绝对方位，只能"上电时刻 = yaw 零点"。
  float pitch0 = std::atan2(-ax, az);
  float roll0 = std::atan2(ay, az);
  float yaw0 = 0.0f;
  // 需要注意的是:上电后在确定底盘相对于地面的初始yaw值的时候 要在mahony的init阶段直接将yaw_relative_angle赋值给底盘系imu的yaw0(正负自己确定)!!!
  // [解析] ↑ 作者留下的待办：如果底盘要和云台/场地对齐，需要在这里把已经知道的相对 yaw
  // [解析]   赋给 yaw0（正负号按自己的坐标系约定定）。现在恒为 0，说明当前没有做这件事。

  // [解析] 下面 6 行是"半角三角函数"：把每个角都除以 2（四元数公式里用的都是半角）。
  // [解析]   c*2 = cos(角/2)，s*2 = sin(角/2)，*代表 roll/pitch/yaw。
  float cr2 = std::cos(roll0 * 0.5f);
  float cp2 = std::cos(pitch0 * 0.5f);
  float cy2 = std::cos(yaw0 * 0.5f);
  float sr2 = std::sin(roll0 * 0.5f);
  float sp2 = std::sin(pitch0 * 0.5f);
  float sy2 = std::sin(yaw0 * 0.5f);

  // [解析] 欧拉角 → 四元数的标准公式（ZYX 内旋顺序）。背不下来也没关系，
  // [解析]   只要知道"这三个角唯一的对应着一个单位四元数"，照抄公式即可。
  float q0 = cr2 * cp2 * cy2 + sr2 * sp2 * sy2;
  float q1 = sr2 * cp2 * cy2 - cr2 * sp2 * sy2;
  float q2 = cr2 * sp2 * cy2 + sr2 * cp2 * sy2;
  float q3 = cr2 * cp2 * sy2 - sr2 * sp2 * cy2;

  // [解析] 归一化后写入成员 q（虽然理论上已经单位化，但浮点运算后仍要再归一化一次保险）。
  norm = 1.0f / std::sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  this->q[0] = q0 * norm;
  this->q[1] = q1 * norm;
  this->q[2] = q2 * norm;
  this->q[3] = q3 * norm;

  // [解析] 顺手把欧拉角也写成初始值，这样上层第一次读就有意义（不用等下一拍 update）。
  this->yaw = yaw0;
  this->pitch = pitch0;
  this->roll = roll0;
}

// [解析] pitch_geom_calc()：把 pitch 的值域从 [−π/2, π/2] 扩展到 [−π, π]，也就是"几何俯仰角"。
// [解析] 为什么需要：普通 pitch 是用 asin 算的，只能表示 ±90°。车"串腿"翻倒、云台倒扣时，
// [解析]   pitch 会突然翻个方向跳变，自启逻辑会误判，所以这里换一种算法来区分"正着"还是"倒扣"。
// [解析] 思路：把车体的 x 轴、z 轴用四元数转到世界系，看它们的竖直分量（下标 [2]）的符号，
// [解析]   就能知道车头抬了多高、车是不是倒扣，从而把角度延拓到 ±π。
void Mahony::pitch_geom_calc()
{
  // [解析] 先把上一拍的结果存起来，本函数末尾要用它做差分算角速度。
  // [解析] 注意：pitch_geom 在头文件里没有默认初值，所以本函数"第一次被调用"时，
  // [解析]   这一行读到的是未初始化的内存值，会导致第一次算出的 vpitch_geom 是一个大尖峰。
  // [解析]   影响只有一拍（1ms），之后就被真实值覆盖了，但看波形时会看到这个毛刺。
  this->pitch_geom_last = this->pitch_geom;

  // [解析] 把车体 x 轴 (1,0,0) 转到世界系，得到 world_x。最后一个参数 false 表示"不取共轭"，
  // [解析]   即按 q 的正向旋转（载体系 → 世界系）来算。
  quaternion_frame_transform_for_mahony(
    this->q, this->base_x, this->world_x, false);  //将base系的x轴转换到world系
  // [解析] 同理把车体 z 轴 (0,0,1) 转到世界系得到 world_z。world_z[2] 就是"车顶朝上还是朝下"：
  // [解析]   ≈ +1 正常直立，≈ −1 完全倒扣，中间值表示倾斜。
  quaternion_frame_transform_for_mahony(
    this->q, this->base_z, this->world_z, false);  //将base系的z轴转换到world系

  // [解析] asin(−world_x[2])：world_x[2] 是车体 x 轴在世界系里的"竖直分量"，
  // [解析]   车头抬得越高它越大，取负号后就成了"低头为正"的俯仰角，值域 [−π/2, π/2]。
  // [解析]   asin 的定义域是 [−1,1]，而 world_x[2] 理论上就是余弦/正弦值，不会越界。
  float pitch_init = std::asin(
    -world_x[2]);  //初始pitch角度(云台x轴在world系下的投影与world系xy平面的夹角)(低头为正)

  //接下来结合z轴判断pitch角度的象限
  // [解析] 若 world_z[2] ≥ 0，说明车体 z 轴还朝上（没倒扣），pitch_init 本身就是正确答案。
  if (world_z[2] >= 0.0f) {
    this->pitch_geom = pitch_init;
  }
  else {
    // [解析] 已经倒扣（world_z[2] < 0），此时 asin 给出的角度丢了"翻过去的那一半"，
    // [解析]   需要加上 ±π 把它接回到连续的区间上：
    // [解析]     world_x[2] ≥ 0（车头指向上方）→ 走 −π 一侧，得到负的扩展角（抬头倒扣）；
    // [解析]     world_x[2] < 0（车头指向下方）→ 走 +π 一侧，得到正的扩展角（低头倒扣）。
    // [解析]   这样在倒扣边界处两边的取值刚好在 ±π 相接，中间过程单调连续。
    if (world_x[2] >= 0.0f) {
      this->pitch_geom = -M_PI - pitch_init;  //抬头倒扣
    }
    else {
      this->pitch_geom = M_PI - pitch_init;  //低头倒扣
    }  //这里仍然避免不了+-2pi的跳变,这就没关系了,至少我们先把值域扩展到了+-pi
    // [解析] ↑ 作者注释的意思：扩展到 ±π 之后，在 ±π 交界处仍会有一次"跳变"（π 与 −π 是同一个
    // [解析]   方向但数值不连续），这是用角度表示旋转天生的问题，除非改用四元数/旋转矩阵。
  }

  // 计算几何pitch角及其微分
  // [解析] 直接差分求角速度：(本次 − 上次) / dt，单位 rad/s。
  // [解析] 缺点和所有差分一样会把噪声放大，而且 pitch_geom 在 ±π 跳变的那一拍会算出一个
  // [解析]   巨大的假速度；好处是不依赖三角公式，倒扣时也不会发散。
  this->vpitch_geom = (this->pitch_geom - this->pitch_geom_last) / dt_;
}

// [解析] set_kp()：动态修改比例增益。注意内部存的是 2×kp，和构造函数里的处理保持一致，
// [解析]   所以这里也必须乘 2，否则改完 kp 的效果会跟构造时传的参数不一样（差一倍）。
void Mahony::set_kp(float kp) { this->two_kp_ = 2.0f * kp; }

// [解析] set_ki()：动态修改积分增益，同样乘 2 保持尺度一致。
// [解析]   注意：从 0 改成非 0 的那一刻，积分器会开始累加；从非 0 改回 0 时积分器会在
// [解析]   下一拍 update() 的 else 分支里被清零。
void Mahony::set_ki(float ki) { this->two_ki_ = 2.0f * ki; }

//专门给mahony写的四元数换系,以免老是引用gimbal类
// 四元数坐标变换：v_out = q ⊗ v_in ⊗ q*
// 输入：q[4] = {w, x, y, z} 表示从坐标系A到坐标系B的旋转
//      v_in[3] = {x, y, z} 在坐标系B中的同一个向量的坐标
// 输出：v_out[3] = {x, y, z} 在坐标系A中的同一个向量的坐标向量
// conjugate_q: 是否对 q 取共轭（true 时使用 q* 而不是 q，相当于反向旋转）
// 注意：这是 passive rotation（坐标系变换），不是 active rotation（向量旋转）
// [解析] 本函数干的事：用四元数把一个向量"换系"（v_out = q ⊗ v_in ⊗ q*）。
// [解析] 参数：q[4] 单位四元数 {w,x,y,z}（一般传本对象的 this->q，即载体系→世界系）；
// [解析]       v_in[3] 输入向量；v_out[3] 输出向量（要先准备好 3 个 float 的空间）；
// [解析]       conjugate_q = true 时用 q 的共轭，等于反着转回去。
// [解析] 返回值无；只写 v_out，不改对象状态，也不改 q 和 v_in（都是 const 指针）。
// [解析] 注意：上面那行"这是 passive rotation（坐标系变换），不是 active rotation（向量旋转）"
// [解析]   的说法容易把人绕晕：代码实现的其实就是教科书里的 v_out = q⊗v⊗q*，也就是
// [解析]   "用旋转 q 去转一个向量"的公式。两种说法只是解释角度不同（换坐标系描述一个向量，
// [解析]   等价于把坐标系反向转），不必纠结名字，记住调用方式就行：
// [解析]   pitch_geom_calc() 里传入载体系下的 base_x，得到的就是世界系下的 world_x。
// [解析] 另注：函数开头注释写的是"v_in 在 B 系、输出在 A 系"，而实际调用方向是
// [解析]   "输入载体系（可理解成 A 系）、输出世界系（B 系）"，两处约定对不上，
// [解析]   以实际用法为准（本工程其他地方也是这么用的）。
void Mahony::quaternion_frame_transform_for_mahony(
  const float q[4], const float v_in[3], float v_out[3], bool conjugate_q)
{
  // 根据 conjugate 标志决定四元数的符号
  // [解析] 单位四元数的共轭就是把虚部（x,y,z）取反、实部 w 不变。
  // [解析] 这里用三目运算符一次性完成，省掉 if-else 分支，代价只是可读性差一点。
  float w = q[0];
  float x = conjugate_q ? -q[1] : q[1];
  float y = conjugate_q ? -q[2] : q[2];
  float z = conjugate_q ? -q[3] : q[3];

  // [解析] 把输入向量取到局部变量，方便下面反复用（也避免通过指针反复访存）。
  float vx = v_in[0], vy = v_in[1], vz = v_in[2];

  // 优化的四元数-向量旋转公式（避免构造完整四元数）
  // v_out = v_in + 2 * cross(q_vec, cross(q_vec, v_in) + w * v_in)
  // [解析] 直接算 q⊗v⊗q* 需要两次四元数乘法（16 次以上乘法），这个等价变形只要两次叉乘
  // [解析]   （12 次乘法），在 1kHz 的循环里能省不少时间。数学上两者完全等价。
  // [解析]   q_vec 指四元数的虚部 (x,y,z)，w 是实部。

  // 第一步：t = cross(q_vec, v_in) = q_vec × v_in
  // [解析] 标准叉乘展开：t = (y·vz − z·vy, z·vx − x·vz, x·vy − y·vx)。
  float tx = y * vz - z * vy;
  float ty = z * vx - x * vz;
  float tz = x * vy - y * vx;

  // 第二步：t = t + w * v_in
  // [解析] 把实部那一项补进来，此时 t 就等于 (q_vec × v_in + w·v_in)。
  tx += w * vx;
  ty += w * vy;
  tz += w * vz;

  // 第三步：v_out = v_in + 2 * cross(q_vec, t)
  // [解析] 再叉乘一次并乘 2，加回原向量 —— 这就是上面那个优化公式的最后一步。
  // [解析] 结果就是"把 v_in 按 q 旋转"之后的向量（分量顺序仍是 x,y,z，没有 w）。
  v_out[0] = vx + 2.0f * (y * tz - z * ty);
  v_out[1] = vy + 2.0f * (z * tx - x * tz);
  v_out[2] = vz + 2.0f * (x * ty - y * tx);
}

}  // namespace sp
