// [解析] DM_Motor 的实现：read() 把 8 字节反馈拆成物理量，write 系列把物理量打包成 8 字节。
// [解析] 两处最容易看错：read() 里 data[4] 被劈成两半(高 4 位归速度、低 4 位归力矩)；
// [解析] write() 里 Kp/Kd 全填 0，力矩实际落在注释标成 Kd 的 data[6]、data[7] 上。

#include "dm_motor.hpp"

#include <cstring>

#include "tools/math_tools/math_tools.hpp"

namespace sp
{
// [解析] 只做初始化列表赋值(const 成员只能这样赋)；cmd_torque_/cmd_velocity_ 没有初值，首次 write() 前必须先调一次 cmd()。
DM_Motor::DM_Motor(
  uint16_t can_id, uint16_t master_id, float pmax, float vmax, float tmax, bool multi_circle)
: rx_id(master_id),
  tx_id(can_id),
  tx_id_vel(0x200 + can_id),
  pmax_(pmax),
  vmax_(vmax),
  tmax_(tmax),
  has_read_(false),
  circle_(0),
  multi_circle_(multi_circle)
{
}

bool DM_Motor::is_open() const { return has_read_; }

// [解析] 100ms 没反馈就判掉线(正常 1kHz 回报，相当于连丢 100 帧)；用无符号相减兼容 HAL_GetTick() 回绕。
bool DM_Motor::is_alive(uint32_t now_ms) const
{
  return is_open() && (now_ms - last_read_ms_ < 100);
}

//multi_circle是true就用多圈
// [解析] 反馈帧字节划分：data[0] 高 4 位错误码，data[1..2] 位置(16 位)，data[3] + data[4] 高 4 位速度(12 位)，
// [解析] data[4] 低 4 位 + data[5] 力矩(12 位)，data[6]/data[7] 是 MOS 管与转子温度(℃)。
void DM_Motor::read(uint8_t * data, uint32_t stamp_ms)
{
  last_read_ms_ = stamp_ms;

  this->error = data[0] >> 4;
  this->speed = uint_to_float((data[3] << 4) | (data[4] >> 4), -vmax_, vmax_, 12);
  this->torque = uint_to_float(((data[4] & 0xF) << 8) | data[5], -tmax_, tmax_, 12);

  this->angle_raw_ = uint_to_float((data[1] << 8) | data[2], -pmax_, pmax_, 16);

  if (!has_read_) {
    has_read_ = true;
    last_angle_raw_ = angle_raw_;
  }
  // [解析] 单圈角在 ±pmax_ 处跳变：本帧比上帧大过 +pmax_ 说明反转跨了边界(circle_--)，小过 -pmax_ 说明正转跨了边界。
  if (angle_raw_ - last_angle_raw_ > pmax_)
    circle_--;
  else if (angle_raw_ - last_angle_raw_ < -pmax_)
    circle_++;

  last_angle_raw_ = angle_raw_;
  this->angle = multi_circle_ ? angle_raw_ + circle_ * 2 * pmax_ : angle_raw_;

  this->T_MOS = data[6];
  this->T_Rotor = data[7];
}

// [解析] 纯力矩模式：Kp/Kd 全置 0，MIT 公式只剩 t_ff，力矩落在 data[6] 高 4 位 + data[7]。
// [解析] data[0..2] 没有赋值，靠 Kp=Kd=0 让 can.tx_data 里的残留值不起作用。
void DM_Motor::write(uint8_t * data) const
{
  auto torque_uint = float_to_uint(cmd_torque_, -tmax_, tmax_, 12);
  data[3] = 0x00;              // Kp[11:8]
  data[4] = 0x00;              // Kp[7:0]
  data[5] = 0x00;              // Kd[11:4]
  data[6] = torque_uint >> 8;  // Kd[3:0]
  data[7] = torque_uint;
}

// [解析] 速度模式不走定点数：直接把 float 的 4 个字节拷进去，靠 Cortex-M4 小端与协议的低字节在前一致。
void DM_Motor::write_velocity(uint8_t * data) const
{
  // 浮点数占用 4 个字节，直接内存拷贝即可满足“低位在前”的小端要求
  std::memcpy(data, &cmd_velocity_, sizeof(float));

  // 保险起见，将后4个字节清零（CAN帧通常发8字节，尽管速度模式只用前4字节）
  data[4] = 0x00;
  data[5] = 0x00;
  data[6] = 0x00;
  data[7] = 0x00;
}

// [解析] 使能/失能/清错是固定帧：前 7 字节 0xFF，最后一字节是命令码(0xFC 使能 / 0xFD 失能 / 0xFB 清错)。
// [解析] 上电后必须先发一次使能帧，否则电机不会输出力矩。
void DM_Motor::write_enable(uint8_t * data) const
{
  data[0] = 0xFF;
  data[1] = 0xFF;
  data[2] = 0xFF;
  data[3] = 0xFF;
  data[4] = 0xFF;
  data[5] = 0xFF;
  data[6] = 0xFF;
  data[7] = 0xFC;
}

void DM_Motor::write_disable(uint8_t * data) const
{
  data[0] = 0xFF;
  data[1] = 0xFF;
  data[2] = 0xFF;
  data[3] = 0xFF;
  data[4] = 0xFF;
  data[5] = 0xFF;
  data[6] = 0xFF;
  data[7] = 0xFD;
}

void DM_Motor::write_clear_error(uint8_t * data) const
{
  data[0] = 0xFF;
  data[1] = 0xFF;
  data[2] = 0xFF;
  data[3] = 0xFF;
  data[4] = 0xFF;
  data[5] = 0xFF;
  data[6] = 0xFF;
  data[7] = 0xFB;
}

// [解析] cmd() 只缓存目标值，不发送；limit_max 把超量程的力矩静默截断到 ±tmax_。
void DM_Motor::cmd(float torque) { cmd_torque_ = limit_max(torque, tmax_); }

void DM_Motor::cmd_velocity(float velocity) { cmd_velocity_ = limit_max(velocity, vmax_); }

// [解析] 只做限幅和缓存，换算留到 write_mit()；kp 量程 0~500、kd 量程 0~5 是协议定的。
// [解析] limit_max 只压上界，负数要再单独拉回 0(负增益会让系统发散)。
void DM_Motor::cmd_mit(float p_des, float v_des, float kp, float kd, float t_ff)
{
  cmd_p_des_ = limit_max(p_des, pmax_);
  cmd_v_des_ = limit_max(v_des, vmax_);

  cmd_kp_ = limit_max(kp, 500.0f);
  if (cmd_kp_ < 0.0f) cmd_kp_ = 0.0f;
  cmd_kd_ = limit_max(kd, 5.0f);
  if (cmd_kd_ < 0.0f) cmd_kd_ = 0.0f;

  cmd_t_ff_ = limit_max(t_ff, tmax_);
}

// [解析] 等价于 cmd_mit(0, velocity, 0, kd, 0)：位置环关掉，只留速度环。
void DM_Motor::cmd_mit_velocity(float velocity, float kd)
{
  cmd_mit(0.0f, velocity, 0.0f, kd, 0.0f);
}

// [解析] 位分配：data[0..1]=p_des(16 位)，data[2]=v_des[11:4]，data[3]=v_des[3:0]|Kp[11:8]，
// [解析] data[4]=Kp[7:0]，data[5]=Kd[11:4]，data[6]=Kd[3:0]|t_ff[11:8]，data[7]=t_ff[7:0]。
void DM_Motor::write_mit(uint8_t * data) const
{
  // [解析] 拼包套路：&0xF 切出 4 位、&0xFF 切出 8 位，再用 << >> 挪到目标位置，最后 | 合并。
  uint16_t p_int = float_to_uint(cmd_p_des_, -pmax_, pmax_, 16);
  uint16_t v_int = float_to_uint(cmd_v_des_, -vmax_, vmax_, 12);
  uint16_t kp_int = float_to_uint(cmd_kp_, 0.0f, 500.0f, 12);
  uint16_t kd_int = float_to_uint(cmd_kd_, 0.0f, 5.0f, 12);
  uint16_t t_int = float_to_uint(cmd_t_ff_, -tmax_, tmax_, 12);

  data[0] = (p_int >> 8) & 0xFF;                           // p_des[15:8]
  data[1] = p_int & 0xFF;                                  // p_des[7:0]
  data[2] = (v_int >> 4) & 0xFF;                           // v_des[11:4]
  data[3] = ((v_int & 0xF) << 4) | ((kp_int >> 8) & 0xF);  // v_des[3:0] | Kp[11:8]
  data[4] = kp_int & 0xFF;                                 // Kp[7:0]
  data[5] = (kd_int >> 4) & 0xFF;                          // Kd[11:4]
  data[6] = ((kd_int & 0xF) << 4) | ((t_int >> 8) & 0xF);  // Kd[3:0] | t_ff[11:8]
  data[7] = t_int & 0xFF;                                  // t_ff[7:0]
}

}  // namespace sp