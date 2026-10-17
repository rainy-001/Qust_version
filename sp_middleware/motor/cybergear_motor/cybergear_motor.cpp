// [解析] CyberGear_Motor 的实现：read() 拆 4 种反馈帧，write() 按 communication_type 填 5 类命令帧。
// [解析] 难点在 read()：除了 8 字节数据，29 位扩展帧 ID 自身还要切成 类型(5)/模式(2)/故障(6)/电机 ID(8)/主机 ID(8)。
// [解析] 命令帧的数据区大多是固定值，只有运控帧(0x01)和写参数(0x12)真的带数据。

#include "cybergear_motor.hpp"

#include "tools/math_tools/math_tools.hpp"

namespace sp
{
CyberGear_Motor::CyberGear_Motor(
  uint8_t master_id, uint8_t motor_id, float pmax, float vmax, float tmax)
: master_id(master_id), motor_id(motor_id), pmax_(pmax), vmax_(vmax), tmax_(tmax)
{
}

bool CyberGear_Motor::is_open() const { return has_read_; }

// [解析] 500ms 没反馈算掉线(比达妙的 100ms 宽松，CyberGear 反馈周期慢)；无符号相减兼容 tick 回绕。
bool CyberGear_Motor::is_alive(uint32_t now_ms) const
{
  return is_open() && (now_ms - last_read_ms_ < 500);
}

// [解析] 帧头字段：bit28~24 通信类型、bit23~22 模式、bit21~16 故障、bit15~8 电机 ID、bit7~0 主机 ID。
void CyberGear_Motor::read(uint32_t header, uint8_t * data, uint32_t stamp_ms)
{
  has_read_ = true;
  last_read_ms_ = stamp_ms;
  feedback_communication_type = ((header) >> 24) & 0x1F;  //先读取帧类型，只取低五位

  //正常工作时的反馈
  if (feedback_communication_type == Communication_Type_MotorRequest) {
    this->feedback_master_id = header & 0xFF;    //指明这一帧发给哪台主机的(取低8位)
    this->feedback_can_id = header >> 8 & 0xFF;  //指明这一帧是哪个电机发的(取bit 8-15)
    this->status_error = (header >> 16) & 0x3F;  //只取低六位
    this->mode = (header >> 22) & 0x03;          //只取低两位
    // [解析] 数据区 3 组 16 位定点数：data[0..1] 角度、data[2..3] 速度、data[4..5] 力矩，都是高字节在前；data[6..7] 是摄氏度×10。
    this->angle = uint_to_float(data[0] << 8 | data[1], -pmax_, pmax_, 16);
    this->speed = uint_to_float(data[2] << 8 | data[3], -vmax_, vmax_, 16);
    this->torque = uint_to_float(data[4] << 8 | data[5], -tmax_, tmax_, 16);
    this->temperature = float((data[6] << 8 | data[7]) & 0xFFFF) / 10.0f;
  }
  // [解析] 注意 0x15/0x11 是"低 8 位=电机 ID、bit15~8=主机 ID"，和 0x02 正好相反，这里照抄最容易错。
  //故障反馈
  else if (feedback_communication_type == Communication_Type_ErrorFeedback) {
    this->feedback_can_id = header & 0xFF;                                                // 取低8位
    this->feedback_master_id = (header >> 8) & 0xFF;                                      // 取低8位
    // [解析] 故障位域最高到 bit16，所以只取低 17 位(0x0001FFFF)。
    this->error = (data[0] << 24 | data[1] << 16 | data[2] << 8 | data[3]) & 0x0001FFFF;  //取低17位
    this->temp_warning = (data[4] << 24 | data[5] << 16 | data[6] << 8 | data[7]) & 0xFFFFFFFF;
  }
  // [解析] 索引是低字节在前(小端)，和上面角度/速度的高字节在前相反；写反了会读到别的参数。
  //单个参数读取的反馈
  else if (feedback_communication_type == Communication_Type_GetSingleParameter) {
    this->feedback_can_id = header & 0xFF;            // 取低8位
    this->feedback_master_id = (header >> 8) & 0xFF;  // 取低8位
    this->index = (data[1] << 8 | data[0]) & 0xFFFF;
    // [解析] run_mode 是 0~3 的枚举，协议只给 1 个字节，且按有符号解释(0xFF 会变成 -1)。
    if (this->index == 0x7005) {
      this->parameter = (int8_t)(data[4] & 0xFF);
    }
    else {
      this->parameter = data[4] << 24 | data[5] << 16 | data[6] << 8 | data[7];
    }
  }
  // [解析] 低 8 位是结果状态，0xFE 表示成功；电机 ID 在 bit15~8，取法和 0x02/0x15 又不一样。
  //获取设备ID的反馈
  else if (feedback_communication_type == Communication_Type_GetID) {
    this->feedback_can_id = (header >> 8) & 0xFF;  // 取低8位
    this->ID_gotten = (header & 0xFF) == 0xFE;     //读到0xFE说明获取ID成功
    // 组装64位MCU_ID：必须先使用static_cast提升为uint64_t再移位，避免32位提升后左移>31导致未定义行为
    this->MCU_ID = (static_cast<uint64_t>(data[0]) << 56) | (static_cast<uint64_t>(data[1]) << 48) |
                   (static_cast<uint64_t>(data[2]) << 40) | (static_cast<uint64_t>(data[3]) << 32) |
                   (static_cast<uint64_t>(data[4]) << 24) | (static_cast<uint64_t>(data[5]) << 16) |
                   (static_cast<uint64_t>(data[6]) << 8) | (static_cast<uint64_t>(data[7]));
  }
}

//CyberGear的一堆命令（真的多，不一定都用得到
//运动控制模式，只发送力矩，不使用目标角度、角速度、Kp、Kd
// [解析] 纯力矩：位置/速度/Kp/Kd 一律置 0，控制律退化成 t_ff；这里的量程写死成 ±4π 和 ±30。
// [解析] torque 没有限幅，超出 ±tmax_ 时 float_to_uint 的结果会溢出 16 位，调用前自己先限。
void CyberGear_Motor::cmd(float torque)
{
  communication_type = Communication_Type_MotionControl;
  tar_torque = float_to_uint(torque, -tmax_, tmax_, 16);
  tar_position_ = float_to_uint(0, -4 * sp::SP_PI, 4 * sp::SP_PI, 16);
  tar_speed_ = float_to_uint(0, -30.0f, 30.0f, 16);
  Kp_ = float_to_uint(0, 0.0f, 500.0f, 16);
  Kd_ = float_to_uint(0, 0.0f, 5.0f, 16);
}
//获取设备ID命令
void CyberGear_Motor::cmd_get_id() { communication_type = Communication_Type_GetID; }
//设置电机CAN ID命令
void CyberGear_Motor::cmd_set_can_id() { communication_type = Communication_Type_CanID; }
//使能电机命令
void CyberGear_Motor::cmd_motor_enable() { communication_type = Communication_Type_MotorEnable; }
//停止电机命令
void CyberGear_Motor::cmd_motor_stop() { communication_type = Communication_Type_MotorStop; }
//设置当前位置为零点命令
void CyberGear_Motor::cmd_set_pos_zero() { communication_type = Communication_Type_SetPosZero; }
//读取单个参数命令
void CyberGear_Motor::cmd_get_single_parameter(uint16_t index)
{
  communication_type = Communication_Type_GetSingleParameter;
  index_ = index;
}
//设置单个参数命令
// [解析] parameter_ 是 uint32_t，float 赋过去是截断(0.5 变 0)，写小数参数要留意。
void CyberGear_Motor::cmd_set_single_parameter(uint16_t index, float parameter)
{
  communication_type = Communication_Type_SetSingleParameter;
  index_ = index;
  parameter_ = parameter;
}

//把能写到data里的内容传到can发送数据中
// [解析] 只是总出口：按 communication_type 选分支把数据区 8 个字节填好，帧头由 can.send_ext() 负责拼。
// [解析] 运控帧的力矩不在数据区，而在扩展帧 ID 里，所以这里只管位置/速度/Kp/Kd。
void CyberGear_Motor::write(uint8_t * data) const
{
  // [解析] 8 字节按 16 位两两分组(大端)：位置、速度、Kp、Kd。
  if (communication_type == Communication_Type_MotionControl) {
    // data[0] = 0;
    // data[1] = 0;
    // data[2] = 0;
    // data[3] = 0;
    // data[4] = 0;
    // data[5] = 0;
    // data[6] = 0;
    // data[7] = 0;
    //理论上这里发的都是0
    data[0] = tar_position_ >> 8 & 0xFF;
    data[1] = tar_position_ & 0xFF;
    data[2] = tar_speed_ >> 8 & 0xFF;
    data[3] = tar_speed_ & 0xFF;
    data[4] = Kp_ >> 8 & 0xFF;
    data[5] = Kp_ & 0xFF;
    data[6] = Kd_ >> 8 & 0xFF;
    data[7] = Kd_ & 0xFF;
  }
  // [解析] 取 ID(0x00)、使能(0x03)、改 CAN ID(0x07) 数据区全 0，格式相同所以合成一个分支。
  else if (
    communication_type == Communication_Type_GetID ||
    communication_type == Communication_Type_MotorEnable ||
    communication_type == Communication_Type_CanID) {
    data[0] = 0x00;
    data[1] = 0x00;
    data[2] = 0x00;
    data[3] = 0x00;
    data[4] = 0x00;
    data[5] = 0x00;
    data[6] = 0x00;
    data[7] = 0x00;
  }
  // [解析] 停止(0x04)、置零位(0x06) 要求 data[0]=0x01 作为执行标志，漏了电机收到也不动。
  else if (
    communication_type == Communication_Type_MotorStop ||
    communication_type == Communication_Type_SetPosZero) {
    data[0] = 0x01;
    data[1] = 0x00;
    data[2] = 0x00;
    data[3] = 0x00;
    data[4] = 0x00;
    data[5] = 0x00;
    data[6] = 0x00;
    data[7] = 0x00;
  }
  // [解析] 读参数：只发 2 字节索引，小端(低字节在前)，必须和 read() 里的拼法配套。
  else if (communication_type == Communication_Type_GetSingleParameter) {
    data[0] = index_ & 0xFF;
    data[1] = (index_ >> 8) & 0xFF;
    data[2] = 0x00;
    data[3] = 0x00;
    data[4] = 0x00;
    data[5] = 0x00;
    data[6] = 0x00;
    data[7] = 0x00;
  }
  // [解析] 写参数：前 2 字节索引(小端)，后 4 字节参数值(大端)。
  else if (communication_type == Communication_Type_SetSingleParameter) {
    data[0] = index_ & 0xFF;
    data[1] = (index_ >> 8) & 0xFF;
    data[2] = 0x00;
    data[3] = 0x00;
    if (index_ != run_mode) {
      data[4] = parameter_ >> 24 & 0xFF;
      data[5] = (parameter_ >> 16) & 0xFF;
      data[6] = (parameter_ >> 8) & 0xFF;
      data[7] = parameter_ & 0xFF;
    }
    // [解析] run_mode 只占 1 个字节；若按 4 字节大端发，电机读到的模式是 0，永远设不上。
    else if (index_ == run_mode) {
      data[4] = (uint8_t)(parameter_);
      data[5] = 0x00;
      data[6] = 0x00;
      data[7] = 0x00;
    }
  }
  // [解析] 不属于这 5 个分支时 data 一个字节都不写，会保留上次的内容，发送前先确认 communication_type 有效。
}

}  // namespace sp
