// [解析] DDT_Motor 的实现：把 8 字节反馈解析成转速/力矩/角度，把力矩反算成协议里的整数电流值。
// [解析] 两个量纲常量：MAX_RAW = 16383 是指令满量程（2^14-1），RAW_TO_CURRENT = 55/32767 是 raw->安培。
// [解析] 反馈是大端（高字节在前，和大疆电机一致，和 super_cap 的小端相反）；角度是折回 (-PI, PI] 的单圈角。

#include "ddt_motor.hpp"

#include "tools/math_tools/math_tools.hpp"

namespace sp
{
// [解析] 限幅用 16383 而换算用 32767，两处口径不同，别以为这两个数字该相等。
constexpr float DDT_MOTOR_MAX_RAW = 16383.0f;
constexpr float DDT_MOTOR_RAW_TO_CURRENT = 55.0f / 32767.0f;

DDT_Motor::DDT_Motor(uint8_t motor_id, float torque_const)
: rx_id(0x96 + motor_id),
  tx_id((motor_id < 5) ? 0x32 : 0x33),
  motor_id_(motor_id),
  torque_const_(torque_const)
{
}

// [解析] 转速原始值是 0.1rpm 定点数；角度用 /32767 归一化，所以原始值 0~65535 会算到约 4*PI，靠 limit_angle 折回一圈。
void DDT_Motor::read(const uint8_t * data)
{
  int16_t speed_int = (data[0] << 8) | data[1];
  int16_t current_int = (data[2] << 8) | data[3];
  uint16_t angle_int = (data[4] << 8) | data[5];

  this->speed = speed_int * 0.1f * 2 * sp::SP_PI / 60.0f;
  this->torque = current_int * DDT_MOTOR_RAW_TO_CURRENT * torque_const_;
  this->angle = sp::limit_angle(angle_int / 32767.0f * 2 * sp::SP_PI);
  this->error = data[6];
  this->mode = data[7];
}

// [解析] 和 RM_Motor 一样由 4 台共用一帧，下标 = (id-1)%4*2 且高字节在前；4 台都写完再统一 send()。
void DDT_Motor::write(uint8_t * data) const
{
  data[(motor_id_ - 1) % 4 * 2 + 0] = cmd_raw_ >> 8;
  data[(motor_id_ - 1) % 4 * 2 + 1] = cmd_raw_;
}

// [解析] read() 的逆运算：力矩 / Kt 得安培，再 / 每 raw 多少安培 得 raw，最后限幅到 ±16383。
void DDT_Motor::cmd(float value)
{
  float raw = value / torque_const_ / DDT_MOTOR_RAW_TO_CURRENT;
  cmd_raw_ = sp::limit_max(raw, DDT_MOTOR_MAX_RAW);
}

}  // namespace sp
