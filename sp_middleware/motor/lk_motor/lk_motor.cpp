
// [解析] lk_motor.hpp 的实现：型号参数查表、反馈字节 -> 物理量、物理量 -> 指令字节、多圈角度累计。
// [解析] 链路：CAN 接收中断里 read() 解析反馈；控制任务里 cmd_xxx() + write() + can.send() 下发。
// [解析] 三处最容易看不懂：小端拼字节必须强转 int16、多圈靠"差值过半圈"判跨零、组帧时整型按小端拆字节。
// [解析] 型号相关量纲全在构造函数里算好存成 const 成员，这里的函数只做换算和拼字节，运行期不查表。
#include "lk_motor.hpp"

#include "tools/math_tools/math_tools.hpp"

namespace sp
{
// int32 数据域的限幅值, 取 2e9(略小于 INT32_MAX)避免浮点转整数溢出
constexpr float LK_MAX_RAW_I32 = 2.0e9f;

// [解析] 这 4 个是"按型号查表"：switch 命中返回常量，default 兜底成另一种型号，
// [解析] 所以新增型号忘加 case 不会报错、只会悄悄用默认值——改型号参数时记得回来补。
float get_torque_const(LK_Motors motor_type)
{
  switch (motor_type) {
    case LK_Motors::MHF9025:
      return MHF9025_TORQUE_CONST;
    default:
      return MG4005_TORQUE_CONST;
  }
}

float get_model_ratio(LK_Motors motor_type)
{
  switch (motor_type) {
    case LK_Motors::MHF9025:
      return MHF9025_P1;
    default:
      return MG4005_P10;
  }
}

float get_max_current(LK_Motors motor_type)
{
  switch (motor_type) {
    case LK_Motors::MHF9025:
      return MHF9025_MAX_CURRENT;
    default:
      return MG4005_MAX_CURRENT;
  }
}

float get_iq_scale(LK_Motors motor_type)
{
  // 手册 P6: MG 电机 iq 分辨率 (66/4096 A)/LSB; MF 电机 iq 分辨率 (33/4096 A)/LSB
  // MHF 属于 MF 家族(带 H 中空轴), 所以用 MF 的分辨率
  switch (motor_type) {
    case LK_Motors::MHF9025:
      return LK_MF_IQ_SCALE;
    default:
      return LK_MG_IQ_SCALE;
  }
}

// [解析] 上限 = min(峰值电流 ÷ iq 系数, 2048)：前者受电流限制，2048 是协议硬天花板(iq_raw 只用 ±2048)。
// [解析] 返回的是幅值：MHF9025 由电流定(≈856)，只有电流很大的型号才会撞上 2048。
int16_t get_max_raw(LK_Motors motor_type)
{
  auto raw = get_max_current(motor_type) / get_iq_scale(motor_type);
  if (raw > 2048.0f) raw = 2048.0f;  // 协议上限
  return static_cast<int16_t>(raw);
}

float get_ecd_range(LK_Motors motor_type)
{
  // 两个型号都按 16bit(65536) 处理:
  //   MHF9025: 实测左拨杆上档转一圈, encoder_raw_max 扫满 65535 ✓
  //   MG4005 : 按原中间件(它就是为 MG4005 写的)的写法 —— encoder/65536、±32768 翻转
  // 手册 P6 那句"14bit 0~16383"是举例说明不同型号位宽, 不代表这两个型号就是 14bit
  // [解析] 返回值是角度换算的分母(16384/32768/65536)，两个分支现在都返回 65536，等于暂时没有分支。
  // [解析] 要用"协议位宽"而不是"编码器硬件位宽"：MHF9025 硬件是 19bit 磁编，协议只按 16bit 上报。
  switch (motor_type) {
    case LK_Motors::MHF9025:
      return LK_ECD_RANGE_16BIT;
    default:
      return LK_ECD_RANGE_16BIT;
  }
}

// [解析] const 成员只能在初始化列表里给值，所以型号相关量纲在这里一次算好，运行期不再查表。
// [解析] 初始化顺序按成员"声明顺序"而非这里的书写顺序；ratio_total_ 自己重算，不依赖 ratio_model_。
LK_Motor::LK_Motor(uint8_t motor_id, LK_Motors motor_type, float ratio, bool multi_circle)
: rx_id(LK_RX_ID_BASE + motor_id),
  tx_id(LK_TX_ID_BASE + motor_id),
  motor_id_(motor_id),
  motor_type_(motor_type),
  ratio_model_(get_model_ratio(motor_type)),
  ratio_added_(ratio),
  ratio_total_(get_model_ratio(motor_type) * ratio),
  torque_const_(get_torque_const(motor_type)),
  iq_scale_(get_iq_scale(motor_type)),
  max_raw_(get_max_raw(motor_type)),
  ecd_range_(get_ecd_range(motor_type)),
  multi_circle_(multi_circle)
{
}

bool LK_Motor::is_open() const { return has_read_; }

bool LK_Motor::is_alive(uint32_t now_ms) const
{
  // [解析] 100ms 是掉线阈值；用无符号差值 now_ms - last_read_ms_ 判断，计时器回绕归零也不会误判。
  return is_open() && (now_ms - last_read_ms_ < 100);
}

// ---------------------------- 反馈解析 ----------------------------

// [解析] 分发器：调用者必须已按 rx_id 过滤(本函数不再查 ID)，进来先刷时间戳再按 data[0] 分派。
// [解析] 0x9A/0x9B 走 read_state1；0x9C 以及所有控制指令(0xA1~0xA8)的反馈帧数据域同格式，都走 read_state2。
// [解析] 0x80/0x81/0x88 的反馈不带状态数据，但时间戳已刷新，所以"发使能 -> 收到空帧"后 is_alive() 即为真。
void LK_Motor::read(const uint8_t * data, uint32_t stamp_ms)
{
  // 能进入本函数的帧都来自本电机的 CAN ID(LK_RX_ID_BASE + motor_id), 因此都算"在线"
  last_read_ms_ = stamp_ms;

  const uint8_t cmd = data[0];

  if (cmd == LK_CMD_STATE1 || cmd == LK_CMD_CLEAR_ERROR) {
    read_state1(data);
  }
  else if (cmd == LK_CMD_STATE2 || (cmd >= LK_CMD_TORQUE && cmd <= LK_CMD_ANGLE_INCREMENT_SPEED)) {
    // 控制指令的反馈帧与0x9C的数据域布局一致
    read_state2(data);
  }
  // 0x80 关闭 / 0x81 停止 / 0x88 使能 的反馈帧不带状态数据, 只用于确认在线
}

// [解析] 布局(手册 P5)：data[1] 温度、data[6] 电机状态(0x00 才是开启)、data[7] 错误标志。
// [解析] 温度必须强转 int8_t：0xFB 当无符号是 251℃，当有符号才是 -5℃。
// [解析] 0x9B(清错误) 的反馈布局与 0x9A 相同，所以两者共用这一个函数。
void LK_Motor::read_state1(const uint8_t * data)
{
  if (data[0] != LK_CMD_STATE1 && data[0] != LK_CMD_CLEAR_ERROR) return;

  this->temp = static_cast<int8_t>(data[1]);
  this->motor_state = data[6];
  this->error_state = data[7];
}

void LK_Motor::read_state2(const uint8_t * data)
{
  const uint8_t cmd = data[0];
  if (
    cmd != LK_CMD_STATE2 && (cmd < LK_CMD_TORQUE || cmd > LK_CMD_ANGLE_INCREMENT_SPEED)) return;

  // 数据域(减速箱前的电机侧量), 见手册 P6:
  //   data[1]    温度, int8, 1℃/LSB
  //   data[2..3] iq 原始值(int16, 小端); MF 系列 33/4096 A/LSB, MG 系列 66/4096 A/LSB
  //   data[4..5] 转速, int16, 小端, 1dps/LSB
  //   data[6..7] 单圈编码器, uint16, 小端(满量程见 get_ecd_range: 两个型号都按 65536)
  // [解析] 小端 = 低字节在前：(data[3] << 8) | data[2]；不加强转 int16_t，负数会变成 65408 这种大正数。
  const int16_t iq_raw = static_cast<int16_t>((data[3] << 8) | data[2]);
  const int16_t speed_raw = static_cast<int16_t>((data[5] << 8) | data[4]);
  const uint16_t encoder = static_cast<uint16_t>((data[7] << 8) | data[6]);

  // 首次读取时初始化 last_ecd_
  if (!has_read_) {
    has_read_ = true;
    last_ecd_ = encoder;
  }

  // 多圈计数: 过 0 点时 ±满量程 跳变
  // [解析] 先扩成 int32 再相减：两个 uint16 直接减，65535 -> 5 会得到大正数而不是 -65530。
  // [解析] 差值超过半圈就当"反向跨过 0 点"，step_ 减 1；小于 -半圈则正向跨，step_ 加 1。
  const int32_t delta = static_cast<int32_t>(encoder) - static_cast<int32_t>(last_ecd_);
  if (delta > static_cast<int32_t>(ecd_range_ / 2.0f))
    step_--;
  else if (delta < -static_cast<int32_t>(ecd_range_ / 2.0f))
    step_++;
  last_ecd_ = encoder;

  // 电机侧多圈角度 / 单圈角度, 单位: rad
  // [解析] 单圈角度 = encoder / ecd_range × 2π；除数是 float，所以不会整除丢精度。
  const float motor_angle_single = encoder / ecd_range_ * 2.0f * SP_PI;
  const float motor_angle =
    motor_angle_single + static_cast<float>(step_) * 2.0f * SP_PI;

  // 更新公有属性(均为减速箱后的输出端量)
  // [解析] 控制指令的反馈帧也带温度，所以这里再写一次(与 read_state1 同源)。
  this->temp = static_cast<int8_t>(data[1]);
  this->encoder_raw = encoder;
  if (encoder > encoder_raw_max) this->encoder_raw_max = encoder;
  // [解析] 换算链：iq_raw × iq 系数 = 相电流(电机侧) -> ×转矩常数 -> ×总减速比 = 输出端力矩(估计值)。
  // [解析] 速度反过来要除减速比；encoder_raw_max 只记上电以来最大值，用来现场核实编码器位宽。
  this->current = iq_raw * iq_scale_;
  this->torque = this->current * torque_const_ * ratio_total_;
  this->speed = speed_raw * SP_PI / 180.0f / ratio_total_;
  this->multicycle_angle = motor_angle / ratio_total_;
  // [解析] multi_circle 为 true 直接用多圈值(连续)；false 则把单圈值归一到 (-π, π]。
  // [解析] 单圈值在 ±π 处会跳变，拿它做位置环，电机经过该点会"以为"目标差了一大截。
  this->angle =
    multi_circle_ ? this->multicycle_angle : limit_angle(motor_angle_single / ratio_total_);
}

// ---------------------------- 指令帧 ----------------------------
// 所有指令帧都按 data[0] 指令字 + data[1..7] 数据 的格式填充完整8字节
// [解析] 这些 write_xxx() 只填 8 个字节，不发送；发出去还得 can.send(tx_id)。
// [解析] 不带参数的指令(0x9A/0x9C/0x9B/0x88/0x80/0x81) data[1..7] 全部补 0，协议要求数据域固定 8 字节。
// [解析] 0x9A/0x9C 是"报一次状态"的查询帧，发完等电机回话；0x88 使能必须先发，否则后面控制帧都不起作用。

void LK_Motor::write_state1(uint8_t * data) const
{
  data[0] = LK_CMD_STATE1;
  data[1] = 0x00;
  data[2] = 0x00;
  data[3] = 0x00;
  data[4] = 0x00;
  data[5] = 0x00;
  data[6] = 0x00;
  data[7] = 0x00;
}

void LK_Motor::write_state2(uint8_t * data) const
{
  data[0] = LK_CMD_STATE2;
  data[1] = 0x00;
  data[2] = 0x00;
  data[3] = 0x00;
  data[4] = 0x00;
  data[5] = 0x00;
  data[6] = 0x00;
  data[7] = 0x00;
}

void LK_Motor::write_clear_error(uint8_t * data) const
{
  data[0] = LK_CMD_CLEAR_ERROR;
  data[1] = 0x00;
  data[2] = 0x00;
  data[3] = 0x00;
  data[4] = 0x00;
  data[5] = 0x00;
  data[6] = 0x00;
  data[7] = 0x00;
}

void LK_Motor::write_open(uint8_t * data) const
{
  data[0] = LK_CMD_TURN_ON;
  data[1] = 0x00;
  data[2] = 0x00;
  data[3] = 0x00;
  data[4] = 0x00;
  data[5] = 0x00;
  data[6] = 0x00;
  data[7] = 0x00;
}

void LK_Motor::write_turn_on(uint8_t * data) const { write_open(data); }

void LK_Motor::write_turn_off(uint8_t * data) const
{
  data[0] = LK_CMD_TURN_OFF;
  data[1] = 0x00;
  data[2] = 0x00;
  data[3] = 0x00;
  data[4] = 0x00;
  data[5] = 0x00;
  data[6] = 0x00;
  data[7] = 0x00;
}

void LK_Motor::write_stop(uint8_t * data) const
{
  data[0] = LK_CMD_STOP;
  data[1] = 0x00;
  data[2] = 0x00;
  data[3] = 0x00;
  data[4] = 0x00;
  data[5] = 0x00;
  data[6] = 0x00;
  data[7] = 0x00;
}

void LK_Motor::write_torque(uint8_t * data) const
{
  // data[4..5] = iqControl(int16, 小端), ±2048 对应 MF ±16.5A / MG ±33A
  // [解析] cmd_raw_ 是 cmd_torque() 算好并限幅过的 iq 原始值，这里只拆字节：& 0xFF 取低 8 位放 data[4]。
  // [解析] int16 负数是补码，算术右移后 & 0xFF 拿到的仍是正确字节，所以负力矩也能照原样下发。
  data[0] = LK_CMD_TORQUE;
  data[1] = 0x00;
  data[2] = 0x00;
  data[3] = 0x00;
  data[4] = static_cast<uint8_t>(cmd_raw_ & 0xFF);
  data[5] = static_cast<uint8_t>((cmd_raw_ >> 8) & 0xFF);
  data[6] = 0x00;
  data[7] = 0x00;
}

void LK_Motor::write_velocity(uint8_t * data) const
{
  // 手册 P10, 速度闭环控制命令 0xA2:
  //   data[2..3] = iqControl 转矩电流限制(int16, 小端), ±2048 对应 MF ±16.5A / MG ±33A
  //   data[4..7] = speedControl(int32, 小端), 0.01dps/LSB
  // 注意: 这里必须给一个合理的转矩电流限制, 不能留 0, 否则电机可能没有力矩输出
  // [解析] data[2..3] 是"速度环最多用多大电流"，必须给非 0 值(这里用型号上限 max_raw_)：
  // [解析] 留 0 会被理解成"限 0A"，电机收到指令却不转，是最难 debug 的坑之一。
  // [解析] data[4..7] 是 int32：按小端拆 4 个字节，低字节在前，顺序不能反。
  const auto speed_raw = cmd_velocity_raw_;
  const int16_t iq_limit = max_raw_;

  data[0] = LK_CMD_VELOCITY;
  data[1] = 0x00;
  data[2] = static_cast<uint8_t>(iq_limit & 0xFF);
  data[3] = static_cast<uint8_t>((iq_limit >> 8) & 0xFF);
  data[4] = static_cast<uint8_t>(speed_raw & 0xFF);
  data[5] = static_cast<uint8_t>((speed_raw >> 8) & 0xFF);
  data[6] = static_cast<uint8_t>((speed_raw >> 16) & 0xFF);
  data[7] = static_cast<uint8_t>((speed_raw >> 24) & 0xFF);
}

void LK_Motor::write_position(uint8_t * data) const
{
  // data[1]    = 旋转方向, 0x00 顺时针, 0x01 逆时针
  // data[4..7] = 单圈位置(uint32, 小端), 电机侧 0.01°, 范围 0 ~ 36000
  // cmd_position_ 是输出端"度": 先乘总减速比换成电机侧角度, 再归一化到单圈[0,360)
  // [解析] 先乘减速比再 fmod：电机侧单圈最多 3600°，顺序反了会超出协议 0~36000 的范围。
  // [解析] fmod 对负数返回负值(-10)，必须加 360 折回 [0,360)，否则转 uint32 会变成天文数字。
  // [解析] 因为做了单圈归一化，cmd_position(370°) 与 cmd_position(10°) 完全等价。
  float motor_deg = std::fmod(cmd_position_ * ratio_total_, 360.0f);
  if (motor_deg < 0.0f) motor_deg += 360.0f;

  const auto angle_raw = static_cast<uint32_t>(motor_deg * 100.0f);

  data[0] = LK_CMD_POSITION;
  data[1] = cmd_direction_;
  data[2] = 0x00;
  data[3] = 0x00;
  data[4] = static_cast<uint8_t>(angle_raw & 0xFF);
  data[5] = static_cast<uint8_t>((angle_raw >> 8) & 0xFF);
  data[6] = static_cast<uint8_t>((angle_raw >> 16) & 0xFF);
  data[7] = static_cast<uint8_t>((angle_raw >> 24) & 0xFF);
}

void LK_Motor::write_angle_increment(uint8_t * data) const
{
  // data[4..7] = 角度增量(int32, 小端), 电机侧 0.01°
  // cmd_angle_ 是输出端"度", 乘总减速比得到电机侧角度
  // [解析] limit_max(raw, ±2e9) 不能省：超大浮点直接转 int32 属未定义行为；增量可负，不做归一化。
  const float raw = cmd_angle_ * ratio_total_ * 100.0f;
  const auto angle_raw = static_cast<int32_t>(limit_max(raw, LK_MAX_RAW_I32));

  data[0] = LK_CMD_ANGLE_INCREMENT;
  data[1] = 0x00;
  data[2] = 0x00;
  data[3] = 0x00;
  data[4] = static_cast<uint8_t>(angle_raw & 0xFF);
  data[5] = static_cast<uint8_t>((angle_raw >> 8) & 0xFF);
  data[6] = static_cast<uint8_t>((angle_raw >> 16) & 0xFF);
  data[7] = static_cast<uint8_t>((angle_raw >> 24) & 0xFF);
}

void LK_Motor::write_angle_increment2(uint8_t * data) const
{
  // data[2..3] = 最大速度(uint16, 小端), 电机侧 1°/s
  // data[4..7] = 角度增量(int32, 小端), 电机侧 0.01°
  // [解析] 0xA8 比 0xA7 只多一个"最大速度"字段，角度增量部分算法完全相同。
  // [解析] speed_dps = 输出端度/s × 总减速比 = 电机侧度/s；本字段标度是 1°/s(不像角度那样 ×100)。
  // [解析] 该字段是 uint16，先夹到 [0, 65535] 再转：负数不夹会变成 65535 这种巨大速度。
  const float angle = cmd_angle_ * ratio_total_ * 100.0f;
  const auto angle_raw = static_cast<int32_t>(limit_max(angle, LK_MAX_RAW_I32));

  float speed_dps = cmd_max_speed_ * ratio_total_;
  if (speed_dps < 0.0f) speed_dps = 0.0f;
  if (speed_dps > 65535.0f) speed_dps = 65535.0f;
  const auto speed_raw = static_cast<uint16_t>(speed_dps);

  data[0] = LK_CMD_ANGLE_INCREMENT_SPEED;
  data[1] = 0x00;
  data[2] = static_cast<uint8_t>(speed_raw & 0xFF);
  data[3] = static_cast<uint8_t>((speed_raw >> 8) & 0xFF);
  data[4] = static_cast<uint8_t>(angle_raw & 0xFF);
  data[5] = static_cast<uint8_t>((angle_raw >> 8) & 0xFF);
  data[6] = static_cast<uint8_t>((angle_raw >> 16) & 0xFF);
  data[7] = static_cast<uint8_t>((angle_raw >> 24) & 0xFF);
}

// [解析] 按最近一次 cmd_xxx() 设的 cmd_mode_ 选组帧函数，顺序反了发的就是上一种模式的指令。
// [解析] default 走力矩：对象刚构造时 cmd_mode_ 就是 0xA1，这个默认值对调试最安全。
void LK_Motor::write(uint8_t * data) const
{
  switch (cmd_mode_) {
    case LK_CMD_VELOCITY:
      write_velocity(data);
      break;
    case LK_CMD_POSITION:
      write_position(data);
      break;
    case LK_CMD_ANGLE_INCREMENT:
      write_angle_increment(data);
      break;
    case LK_CMD_ANGLE_INCREMENT_SPEED:
      write_angle_increment2(data);
      break;
    default:
      write_torque(data);
      break;
  }
}

// ---------------------------- 指令缓存 ----------------------------
// [解析] 这一组只"记录目标值 + 记住模式"，不换算、不组帧、不发送；换算统一推迟到 write_xxx()。
// [解析] 每次调用都会改写 cmd_mode_，所以"最后一次调用"决定 write() 发什么。

void LK_Motor::cmd(float torque) { cmd_torque(torque); }

void LK_Motor::cmd_torque(float torque)
{
  // 输出端力矩 -> 电机侧转矩电流 -> iq 原始值(换算系数按系列区分)
  // [解析] 换算链与 read_state2 正好相反：输出端力矩 ÷总减速比 -> ÷转矩常数 -> ÷iq 系数 = iq 原始值。
  // [解析] 限幅是削顶不是等比缩放，给太大会一直顶在最大力矩上；先限幅再强转 int16 才不会溢出。
  const float current = torque / ratio_total_ / torque_const_;
  float raw = current / iq_scale_;

  if (raw > max_raw_) raw = max_raw_;
  if (raw < -max_raw_) raw = -max_raw_;

  cmd_raw_ = static_cast<int16_t>(raw);
  cmd_mode_ = LK_CMD_TORQUE;
}

void LK_Motor::cmd_velocity(float speed)
{
  // 输出端 rad/s -> 电机侧 0.01°/s
  // [解析] rad/s ×(180/π) 得 °/s，再 ×总减速比换成电机侧，最后 ×100 变成协议要的 0.01°/s 计数。
  const float raw = speed * ratio_total_ * 180.0f / SP_PI * 100.0f;
  cmd_velocity_raw_ = static_cast<int32_t>(limit_max(raw, LK_MAX_RAW_I32));
  cmd_mode_ = LK_CMD_VELOCITY;
}

void LK_Motor::cmd_position(float position, uint8_t direction)
{
  // position: 输出端 度(单圈), 组帧时会乘减速比并归一化到电机侧的单圈 0~360°
  // [解析] 这里只原样存输出端角度，乘减速比和归一化都留到 write_position() 做(存/算分离)。
  cmd_position_ = position;
  cmd_direction_ = direction;
  cmd_mode_ = LK_CMD_POSITION;
}

void LK_Motor::cmd_angle_increment(float angle)
{
  // angle: 输出端 度
  // [解析] 原样存输出端角度：可正可负、可超 360°，组帧时乘减速比后按 int32 下发，不做单圈归一化。
  cmd_angle_ = angle;
  cmd_mode_ = LK_CMD_ANGLE_INCREMENT;
}

void LK_Motor::cmd_angle_speed(float angle, float max_speed)
{
  // angle: 输出端 度; max_speed: 输出端 度/s
  // [解析] 想用带限速的 0xA8 必须走这个函数；cmd_angle_increment() 发的是 0xA7，不限速。
  cmd_angle_ = angle;
  cmd_max_speed_ = max_speed;
  cmd_mode_ = LK_CMD_ANGLE_INCREMENT_SPEED;
}

}  // namespace sp
