// [解析] RM_Motor 的实现：按 ID/型号算收发帧 ID，把 8 字节反馈换算成物理量，把力矩反算成 raw。
// [解析] 编码器 13 位，一圈 8192 个计数；力矩系数里带着 hpp 的 *10/*20/*3，改的时候必须两处一起改。

#include "rm_motor.hpp"

#include "tools/math_tools/math_tools.hpp"

namespace sp
{
// [解析] 满量程 raw：M2006 ±10000，M3508/GM6020 电流 ±16384，GM6020 电压 ±25000 对应 ±24V。
constexpr int16_t M2006_MAX_RAW = 10000;
constexpr int16_t M3508_MAX_RAW = 16384;
constexpr int16_t GM6020_MAX_RAW = 16384;
constexpr int16_t GM6020_V_MAX_RAW = 25000;

// [解析] raw -> N·m：手册力矩 / 减速比 * 放大系数 / 满量程 raw。M2006/M3508 的减速比在这里已经除过一次，
// [解析] read() 里又乘回来，两次正好抵消，所以算出的力矩数值与减速比无关。
constexpr float M2006_RAW_TO_TORQUE = 0.18f / M2006_P36 * 10 / M2006_MAX_RAW;
constexpr float M3508_RAW_TO_TORQUE = 0.3f / M3508_P19 * 20 / M3508_MAX_RAW;
constexpr float GM6020_RAW_TO_TORQUE = 0.741f * 3 / GM6020_MAX_RAW;

// [解析] 上面系数的倒数，供 cmd() 反算；它们不会随上面自动变，改系数必须两边一起改。
// [解析] 最后一行是电压模式：25000 / 24V ≈ 1041.7 raw/V，cmd(12) 差不多就是半量程。
constexpr float M2006_CMD_TO_RAW = 1.0f / M2006_RAW_TO_TORQUE;
constexpr float M3508_CMD_TO_RAW = 1.0f / M3508_RAW_TO_TORQUE;
constexpr float GM6020_CMD_TO_RAW = 1.0f / GM6020_RAW_TO_TORQUE;
constexpr float GM6020_V_CMD_TO_RAW = GM6020_V_MAX_RAW / 24.0f;

// [解析] 反馈帧 ID：M2006/M3508 = 0x200+id，GM6020 两种模式共用 0x204+id，同总线不能挂同 ID 的两版 6020。
uint16_t get_rx_id(uint8_t motor_id, RM_Motors motor_type)
{
  switch (motor_type) {
    case RM_Motors::M2006:
    case RM_Motors::M3508:
      return motor_id + 0x200;
    default:
      return motor_id + 0x204;
  }
}

// [解析] 控制帧 ID：一帧 8 字节只装 4 台电机，所以 1~4 号一组、5~8 号另一组。
// [解析] 6020 电流 0x1FE/0x2FE、电压 0x1FF/0x2FF，两组只差最低位；M2006/M3508 是 0x200/0x1FF。
uint16_t get_tx_id(uint8_t motor_id, RM_Motors motor_type)
{
  switch (motor_type) {
    case RM_Motors::GM6020:
      return motor_id < 5 ? 0x1FE : 0x2FE;
    case RM_Motors::GM6020_V:
      return motor_id < 5 ? 0x1FF : 0x2FF;
    default:
      return motor_id < 5 ? 0x200 : 0x1FF;
  }
}

// [解析] 按型号取 raw->N·m 系数；6020 两种模式反馈帧相同，共用 default。
float get_raw_to_torque(RM_Motors motor_type)
{
  switch (motor_type) {
    case RM_Motors::M2006:
      return M2006_RAW_TO_TORQUE;
    case RM_Motors::M3508:
      return M3508_RAW_TO_TORQUE;
    default:
      return GM6020_RAW_TO_TORQUE;
  }
}

// [解析] 按型号取 指令->raw 系数；这里 6020 的电流/电压模式必须分开，不能像上面那样共用 default。
float get_cmd_to_raw(RM_Motors motor_type)
{
  switch (motor_type) {
    case RM_Motors::M2006:
      return M2006_CMD_TO_RAW;
    case RM_Motors::M3508:
      return M3508_CMD_TO_RAW;
    case RM_Motors::GM6020:
      return GM6020_CMD_TO_RAW;
    default:
      return GM6020_V_CMD_TO_RAW;
  }
}

// [解析] 按型号取满量程，cmd() 用它限幅；超量程发出去会触发驱动板保护，甚至按最大值猛冲。
int16_t get_max_raw(RM_Motors motor_type)
{
  switch (motor_type) {
    case RM_Motors::M2006:
      return M2006_MAX_RAW;
    case RM_Motors::M3508:
      return M3508_MAX_RAW;
    case RM_Motors::GM6020:
      return GM6020_MAX_RAW;
    default:
      return GM6020_V_MAX_RAW;
  }
}

// [解析] 初始化顺序由 hpp 里的声明顺序决定，与这里写的顺序无关；rx_id/tx_id 是 const，只能在这里赋值。
// [解析] 故意不初始化 last_ecd_（由 read() 第一帧补齐）和 last_read_ms_（is_alive() 里有 is_open() 短路）。
RM_Motor::RM_Motor(uint8_t motor_id, RM_Motors motor_type, float ratio, bool multi_circle)
: rx_id(get_rx_id(motor_id, motor_type)),
  tx_id(get_tx_id(motor_id, motor_type)),
  angle(0),
  speed(0),
  torque(0),
  temperature(0),
  motor_id_(motor_id),
  motor_type_(motor_type),
  ratio_(ratio),
  multi_circle_(multi_circle),
  has_read_(false),
  circle_(0),
  cmd_raw_(0)
{
}

bool RM_Motor::is_open() const { return has_read_; }

// [解析] 用减法比较而不是比大小，定时器回绕时才正确；100ms 允许连丢 100 帧，误报和发现速度都能兼顾。
bool RM_Motor::is_alive(uint32_t now_ms) const
{
  return is_open() && (now_ms - last_read_ms_ < 100);
}

// [解析] 解析一帧反馈（大端，data[0] 是高字节）：[0..1] 编码器 [2..3] rpm [4..5] 电流 raw [6] 温度。
// [解析] 跑在 CAN 接收中断里，每行都要快，不要在这里加打印等阻塞操作。
void RM_Motor::read(uint8_t * data, uint32_t stamp_ms)
{
  // 更新时间戳
  last_read_ms_ = stamp_ms;

  // 数据解析
  uint16_t angle_ecd = (data[0] << 8) | data[1];
  int16_t speed_rpm = (data[2] << 8) | data[3];
  int16_t current_raw = (data[4] << 8) | data[5];

  // 首次读取时, 初始化last_ecd_
  if (!has_read_) {
    has_read_ = true;
    last_ecd_ = angle_ecd;
  }

  // 多圈编码
  // [解析] 编码器 0~8191 跨 0 回绕时差值会突变，所以用半圈 4096 做阈值判方向：差值是大正数说明往回绕了，
  // [解析] 大负数说明往前绕。前提是两次读之间转不过半圈，否则会少数一圈（角度突然跳 2*PI）。
  if (angle_ecd - last_ecd_ > 4096)
    circle_--;
  else if (angle_ecd - last_ecd_ < -4096)
    circle_++;
  last_ecd_ = angle_ecd;

  // [解析] 减 4095 把 0~8191 平移到正负对称（零位偏 1 个计数），再 /8192*2*PI 换成弧度、/ratio_ 到输出轴。
  float angle_rad = float(angle_ecd - 4095) / 8192 * 2 * SP_PI / ratio_;

  // 更新公有属性
  this->angle = multi_circle_ ? angle_rad + circle_ * 2 * SP_PI / ratio_ : angle_rad;
  this->speed = float(speed_rpm) / 60 * 2 * SP_PI / ratio_;
  this->torque = float(current_raw) * get_raw_to_torque(motor_type_) * ratio_;
  this->temperature = data[6];
}

// [解析] 这一帧由 4 台电机共用，每台占 2 字节：下标 = (id-1)%4*2，所以各写各的、最后统一 send() 一次。
// [解析] 高字节在前（大端），和 read() 的拼法对称；负数右移补符号位，赋给 uint8_t 截断后仍能正确还原。
void RM_Motor::write(uint8_t * data) const
{
  data[(motor_id_ - 1) % 4 * 2 + 0] = cmd_raw_ >> 8;
  data[(motor_id_ - 1) % 4 * 2 + 1] = cmd_raw_;
}

// [解析] 只更新 cmd_raw_，不发帧；value 是输出轴力矩，先 / ratio_ 换到转子侧，再乘系数换成 raw。
void RM_Motor::cmd(float value)
{
  auto raw = value / ratio_ * get_cmd_to_raw(motor_type_);
  auto max_raw = get_max_raw(motor_type_);

  if (raw > max_raw) raw = max_raw;
  if (raw < -max_raw) raw = -max_raw;

  // 更新私有属性cmd_raw_, 待write()使用
  cmd_raw_ = raw;
}

}  // namespace sp
