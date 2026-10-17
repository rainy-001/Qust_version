// [解析] 舵轮底盘运动学的实现：init() 记舵零位、calc()+convert() 正解、update() 反解。
// [解析] 三处最费解：calc() 里自转贡献的切向速度、convert() 的"就近转 vs 轮子反转"、update() 符号与 calc() 相反。

#include "swerve.hpp"

#include "tools/math_tools/math_tools.hpp"

namespace sp
{
Swerve::Swerve(float wheel_radius, float half_length, float half_width, bool invert_pivot)
: r_(wheel_radius), l_(half_length), w_(half_width), sign_(invert_pivot ? -1.0f : 1.0f)
{
}

// [解析] 记四个舵的安装零位；调用瞬间四个轮子必须同向，否则之后算出的舵角全盘错乱。
void Swerve::init(float yaw_lf, float yaw_lr, float yaw_rf, float yaw_rr)
{
  yaw_offset_lf_ = yaw_lf;
  yaw_offset_lr_ = yaw_lr;
  yaw_offset_rf_ = yaw_rf;
  yaw_offset_rr_ = yaw_rr;
}

// [解析] 正解第一步：v_轮 = v_底盘 + ω × r_轮，展开成下面 4 行（w_ 半轮距、l_ 半轴距，wz 逆时针为正）。
void Swerve::calc(
  float vx, float vy, float wz, float yaw_lf, float yaw_lr, float yaw_rf, float yaw_rr)
{
  // 计算中间变量: 各轮速度向量
  float v_lf[2] = {vx - wz * w_, vy + wz * l_};
  float v_lr[2] = {vx - wz * w_, vy - wz * l_};
  float v_rf[2] = {vx + wz * w_, vy + wz * l_};
  float v_rr[2] = {vx + wz * w_, vy - wz * l_};

  // 计算各舵转角和各舵角度
  convert(v_lf, yaw_lf, yaw_offset_lf_, this->angle_lf, this->speed_lf);
  convert(v_lr, yaw_lr, yaw_offset_lr_, this->angle_lr, this->speed_lr);
  convert(v_rf, yaw_rf, yaw_offset_rf_, this->angle_rf, this->speed_rf);
  convert(v_rr, yaw_rr, yaw_offset_rr_, this->angle_rr, this->speed_rr);
}

// [解析] 单轮换算：速度矢量(m/s) -> 舵目标角度 + 轮目标转速(rad/s)，纯计算、结果走引用参数。
void Swerve::convert(const float v[2], float yaw, float yaw_offset, float & angle, float & speed)
{
  // ref: https://github.com/rm-controls/rm_controllers/blob/master/rm_chassis_controllers/src/swerve.cpp

  // [解析] 速度恰好为 0 时保持舵角、转速给 0；不特判则 atan2(0,0) 会让停着的车乱摆舵。
  if (v[0] == 0.0f && v[1] == 0.0f) {
    angle = yaw;
    speed = 0.0f;
    return;
  }

  float v_angle = std::atan2(v[1], v[0]);
  // [解析] 反向 180° 也作为候选：舵转 180° 与"轮子反转"等效，离当前舵角近就选它。
  float v_angle_flipped = limit_angle(v_angle + SP_PI);
  float pivot_angle = sign_ * limit_angle(yaw - yaw_offset);

  float a = limit_angle(v_angle - pivot_angle);
  float b = limit_angle(v_angle_flipped - pivot_angle);
  // [解析] 就近原则：比较 |a| 与 |b|（a、b 带符号，直接比大小会判错）。
  float pivot_angle_set = (std::abs(a) < std::abs(b)) ? v_angle : v_angle_flipped;

  angle = limit_angle(sign_ * pivot_angle_set + yaw_offset);
  // [解析] 输出轮速 = |v|/r_ * cos(a)，cos(a) 是舵轮底盘的精髓：
  // [解析]   选中反向候选时 a≈±π、cos≈-1，速度自动变负 —— 舵不用真转 180°，靠轮子反转等效实现。
  // [解析]   走正向时 a 是"舵还没转过去的偏差角"，cos<1 顺手做了"没对准先降速"的补偿，转到位才回到 1。
  speed = std::sqrt(v[0] * v[0] + v[1] * v[1]) / r_ * std::cos(a) ;//* std::cos(a) * std::cos(a);
}

// [解析] 反解：四个轮的实测转速/舵角 -> 底盘实际速度，用于速度估计、打滑判断、超功率保护。
void Swerve::update(
  float speed_lf, float speed_lr, float speed_rf, float speed_rr, float angle_lf, float angle_lr,
  float angle_rf, float angle_rr)
{
  // 角度修正
  // [解析] 减安装零位得到轮子相对底盘的朝向；这里不乘 sign_，符号翻转并到下面 v_xx_y 的负号里。
  float theta_lf = limit_angle(angle_lf - yaw_offset_lf_);
  float theta_lr = limit_angle(angle_lr - yaw_offset_lr_);
  float theta_rf = limit_angle(angle_rf - yaw_offset_rf_);
  float theta_rr = limit_angle(angle_rr - yaw_offset_rr_);

  // 各轮线速度分解（车体坐标系）
  // [解析] 轮转速 -> 轮缘线速度 speed_xx*r_（m/s），再按 theta_xx 分解成 x/y；y 前面那个负号与 convert() 的 (cos,+sin) 相反，两处不能只改一处。
  this->v_lf_x = speed_lf * r_ * std::cos(theta_lf);
  this->v_lf_y = - speed_lf * r_ * std::sin(theta_lf);

  this->v_lr_x = speed_lr * r_ * std::cos(theta_lr);
  this->v_lr_y = - speed_lr * r_ * std::sin(theta_lr);

  this->v_rf_x = speed_rf * r_ * std::cos(theta_rf);
  this->v_rf_y = - speed_rf * r_ * std::sin(theta_rf);

  this->v_rr_x = speed_rr * r_ * std::cos(theta_rr);
  this->v_rr_y = - speed_rr * r_ * std::sin(theta_rr);

  // 位置向量
  const float pos[4][2] = {
    {+l_, +w_},  // LF
    {-l_, +w_},  // LR
    {+l_, -w_},  // RF
    {-l_, -w_},  // RR
  };

  // 平均线速度
  float vx_sum = (v_lf_x + v_lr_x + v_rf_x + v_rr_x) / 4.0f;
  float vy_sum = (v_lf_y + v_lr_y + v_rf_y + v_rr_y) / 4.0f;

  // 平均角速度
  // [解析] 单轮反解 ω = (v_x*p_y - v_y*p_x)/(l²+w²) = (r × v)/|r|²，逐轮解出后再平均；正方向与 calc() 的 wz 相反。
  this->wz_lf = (v_lf_x * pos[0][1] - v_lf_y * pos[0][0]) / (l_ * l_ + w_ * w_);
  this->wz_lr = (v_lr_x * pos[1][1] - v_lr_y * pos[1][0]) / (l_ * l_ + w_ * w_);
  this->wz_rf = (v_rf_x * pos[2][1] - v_rf_y * pos[2][0]) / (l_ * l_ + w_ * w_);
  this->wz_rr = (v_rr_x * pos[3][1] - v_rr_y * pos[3][0]) / (l_ * l_ + w_ * w_);
  float wz_sum = (wz_lf + wz_lr + wz_rf + wz_rr) / 4.0f;

  // [解析] 发布 vx/vy/wz；vy、wz 与 calc() 输入的正方向相反，别不加处理引回去做闭环（会成正反馈）。
  this->vx = vx_sum;
  this->vy = vy_sum;
  this->wz = wz_sum;
}

}  // namespace sp