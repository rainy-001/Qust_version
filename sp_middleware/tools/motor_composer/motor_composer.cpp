// [解析] MotorComposer 的实现：每个函数一行公式，靠"乘方向符号"把两台电机的方向统一起来。
// [解析] 角度/速度取平均（描述同一位姿）、力矩相加（总出力）；calc_speeds 是速度的反向拆分，成员与参数同名所以 this-> 不能省。

#include "motor_composer.hpp"

namespace sp
{
// [解析] const 成员只能在初始化列表里赋值，所以 ±1 符号在这里一次定死。
MotorComposer::MotorComposer(bool reverse_l, bool reverse_r)
: sign_l_((reverse_l) ? -1.0f : 1.0f), sign_r_((reverse_r) ? -1.0f : 1.0f)
{
}

// [解析] 记零位：只在机械停在你期望的零位时调用一次，否则等效角度会带一个固定偏差。
void MotorComposer::init_angles(float angle_l, float angle_r)
{
  angle_l0_ = angle_l;
  angle_r0_ = angle_r;
}

// [解析] (sign_l*(angle_l-零位_l) + sign_r*(angle_r-零位_r)) / 2，结果 rad、相对零位。
void MotorComposer::calc_angle(float angle_l, float angle_r)
{
  this->angle = (sign_l_ * (angle_l - angle_l0_) + sign_r_ * (angle_r - angle_r0_)) / 2;
}

void MotorComposer::calc_speed(float speed_l, float speed_r)
{
  this->speed = (sign_l_ * speed_l + sign_r_ * speed_r) / 2;
}

// [解析] 力矩取"和"不是平均：两台电机推同一根轴，总出力才是等效电机的出力。
void MotorComposer::calc_torque(float torque_l, float torque_r)
{
  this->torque = sign_l_ * torque_l + sign_r_ * torque_r;
}

void MotorComposer::calc_speeds(float speed)
{
  this->speed_l = sign_l_ * speed;
  this->speed_r = sign_r_ * speed;
}

}  // namespace sp
