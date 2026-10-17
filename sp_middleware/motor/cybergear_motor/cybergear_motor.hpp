// [解析] 小米 CyberGear 微电机封装：解析 4 种反馈帧，打包运控/使能/停止/读写参数等命令帧。
// [解析] 与达妙电机的关键差别：命令类型放在 29 位扩展帧 ID 的高 5 位里而不是数据区，所以 read() 要连帧头一起解析。
// [解析] 发送靠 can.send_ext(communication_type, tar_torque, motor_id, master_id)，8 字节数据由 write() 填进 can.tx_data。
// [解析] 下面一大堆宏都是协议规定的编号(通信类型 0x00~0x15、参数索引 0x7005~0x7018)，不能凭感觉改。

#ifndef SP__CYBERGEAR_MOTOR_HPP
#define SP__CYBERGEAR_MOTOR_HPP

#include <cstdint>
//气动cybergearID 0x06 0x00
// [解析] 主机 0x00 / 电机 0x01 是本项目用的默认 ID；电机出厂默认 0x7F，可用 cmd_set_can_id() 改。
#define Master_CAN_ID 0x00
#define CyberGear_CAN_ID 0x01
// [解析] 三个量程宏决定定点数换算(±12 N·m、±30 rad/s、±4π rad)；CYBERGEAR_MAX_SPEED 取 30.0f，与旁边注释的 31.42 rad/s 略有出入。
// [解析] CYBERGEAR_MAX_POSITION 没加括号，直接参与运算要自己补：(1.0f / (CYBERGEAR_MAX_POSITION))。
#define CYBERGEAR_MAX_TORQUE 12.0f            // N·m
#define CYBERGEAR_MAX_SPEED 30.0f             // 300 rpm = 31.42 rad/s
#define CYBERGEAR_MAX_POSITION 3.141593f * 4  // 2圈

//CyberGear支持的 communication_type 类型
// [解析] 通信类型表：0x00 取 ID、0x01 运控、0x02 电机反馈、0x03 使能、0x04 停止、0x06 置零位、0x07 改 CAN ID、0x11/0x12 读写参数、0x15 故障反馈。
#define Communication_Type_GetID 0x00          //反馈，获取设备ID：获取设备的ID和64位MCU唯一标识符
#define Communication_Type_MotionControl 0x01  //控制，运动控制指令
#define Communication_Type_MotorRequest 0x02   //反馈，电机反馈数据
#define Communication_Type_MotorEnable 0x03    //控制，电机使能运行
#define Communication_Type_MotorStop 0x04      //控制，电机停止运行
#define Communication_Type_SetPosZero 0x06     //控制，设置电机机械零位
#define Communication_Type_CanID 0x07          //更改电机CAN ID
#define Communication_Type_Control_Mode 0x0C
#define Communication_Type_GetSingleParameter 0x11  //反馈，获取单个参数
#define Communication_Type_SetSingleParameter 0x12  //控制，设置单个参数
#define Communication_Type_ErrorFeedback 0x15       //反馈，故障反馈帧

//电机运行模式参数的index
// [解析] 参数索引相当于电机内部的寄存器地址，用 Get/SetSingleParameter 读写；run_mode 决定电机听哪种指令
// [解析] (0 运控 / 1 位置 / 2 速度 / 3 电流)，写它时协议只允许 1 个字节，见 .cpp 里的特判。
#define run_mode 0x7005       //运行模式参数 0：运控模式，1：位置模式，2：速度模式，3：电流模式
#define iq_ref 0x7006         //电流模式Iq指令
#define spd_ref 0x700A        //转速模式转速指令
#define imit_torque 0x700B    //转矩限制
#define cur_kp 0x7010         //电流的Kp
#define cur_ki 0x7011         //电流的Ki
#define cur_filt_gain 0x7014  //电流滤波系数filter_gain
#define loc_ref 0x7016        //位置模式角度指令
#define limit_spd 0x7017      //位置模式速度设置
#define limit_cur 0x7018      //速度位置模式电流设置

namespace sp
{

// [解析] 一个对象 = 一台 CyberGear 电机；用法与 DM_Motor 类似，但每条命令前必须先调 cmd_xxx() 设好 communication_type。
class CyberGear_Motor
{
public:
  // [解析] 只存参数；pmax/vmax/tmax 必须和电机内部设置一致，否则定点数换算出来的值整体偏移。
  CyberGear_Motor(uint8_t master_id, uint8_t motor_id, float pmax, float vmax, float tmax);

  // [解析] 一帧扩展帧 ID 里同时装着"发给哪台电机"和"来自哪个主机"，这样一条总线才能挂多台电机、多个主机而不串。
  //主机id为master_id, 电机id为motor_id,电机接收控制帧的时候要指明来自哪个主机，电机反馈的时候要指明是哪台电机
  const uint8_t master_id;  //代表主机的ID
  const uint8_t motor_id;   //代表电机的ID

  // [解析] 这些值是 2 的幂，多种故障同时出现时相加/按位或(读到 3 = 欠压+过流)，判断要用 & 而不是 ==。
  //可以工作但状态异常，0：正常，1：欠压，2：过流，4：过温，8：磁编码器故障，16：HALL编码故障，32：未标定 多个错误相加
  uint8_t status_error = 0;
  //工作模式状态，0：Reset模式[复位]，1：Cali模式[标定]，2：Motor模式[运行]
  //初次使能电机会进入Reset模式，需要再次使能设定为Motor模式
  uint8_t mode = 0;

  //无法工作的故障帧反馈内容
  // bit16:A相电流采样过流
  // bit15~bit8:过载故障
  // bit7:编码器未标定
  // bit5:C相电流采样过流
  // bit4:B相电流采样过流
  // bit3:过压故障
  // bit2:欠压故障
  // bit1:驱动芯片故障
  // bit0:电机过温故障，默认80度
  uint32_t error = 0;
  // 只读! 警告温度，单位: 摄氏度*10
  uint32_t temp_warning = 0;

  // [解析] read() 从帧头解出的三个字段：发送方电机 ID、目标主机 ID、反馈类型(决定数据区按哪种格式解析)。
  //反馈的电机id
  uint8_t feedback_can_id;
  //反馈的主机id
  uint8_t feedback_master_id;
  //反馈的通信类型
  uint8_t feedback_communication_type;

  float angle = 0;        // 只读! 当前角度，[0~65535]对应(-4π，4π)
  float speed = 0;        // 只读! 当前角速度，[0~65535]对应(-30 rad/s，30 rad/s)
  float torque = 0;       // 只读! 当前力矩，[0~65535]对应(-12 N·m，12 N·m)
  float temperature = 0;  // 只读! 当前温度，Temp(摄氏度)*10

  uint16_t index = 0;      //参数索引
  float parameter = 0;     //参数值
  uint64_t MCU_ID = 0;     // 只读! MCU唯一标识符
  bool ID_gotten = false;  // 只读! 是否成功获取到设备ID

  // [解析] is_alive() 要求 500ms 内有反馈(比达妙的 100ms 宽松，CyberGear 反馈周期慢)；无符号相减兼容 tick 回绕。
  bool is_open() const;
  bool is_alive(uint32_t now_ms) const;

  //读取电机反馈数据
  // [解析] 比 DM_Motor::read() 多一个 header 参数：电机 ID/主机 ID/模式/故障都藏在 29 位帧头里。
  void read(uint32_t header, uint8_t * data, uint32_t stamp_ms);
  //把结构体的内容传到can发送数据中
  // [解析] 按 communication_type 选一种格式填满 8 字节；帧头(类型、力矩、ID)由 can.send_ext() 负责拼。
  void write(uint8_t * data) const;
  //设置通信类型，并将各种数据传入Motor结构体
  // [解析] cmd_xxx() 一律只设 communication_type 并缓存参数，不发帧；真正发送要再调 write() + can.send_ext()。
  //发送力矩
  void cmd(float torque);
  //获取设备ID
  void cmd_get_id();
  //设置电机CAN ID
  void cmd_set_can_id();
  //使能电机
  void cmd_motor_enable();
  //停止电机
  void cmd_motor_stop();
  //设置当前位置为零点
  void cmd_set_pos_zero();
  //读取单个参数
  void cmd_get_single_parameter(uint16_t index);
  //设置单个参数
  // [解析] parameter 存进 uint32_t 时小数被直接截断(0.5 -> 0)，写 cur_kp/cur_ki 这类小数参数要留意。
  void cmd_set_single_parameter(uint16_t index, float parameter);

  // [解析] 这两个 public 成员就是给 send_ext() 用的：类型决定命令，tar_torque 是已换算好的 16 位定点数。
  //需要传递的参数
  uint16_t tar_torque = 0;
  uint8_t communication_type = 0;

private:
  const float pmax_;
  const float vmax_;
  const float tmax_;

  bool has_read_ = false;
  uint32_t last_read_ms_;

  //可以发给电机的变量,不需要使用
  // [解析] 三个目标值都已换算成 16 位定点数：65535 对应 +量程，32767 对应 0，0 对应 -量程。
  uint16_t tar_position_ = 0;  //目标位置[0~65535]对应(-4π~4π)
  uint16_t tar_speed_ = 0;     //目标角速度[0~65535]对应(-30rad/s~30rad/s)
  uint16_t Kp_ = 0;            //Kp [0~65535]对应(0.0~500.0)
  uint16_t Kd_ = 0;            //Kd [0~65535]对应(0.0~5.0)

  uint16_t index_;      //参数索引
  uint32_t parameter_;  //参数值
};

}  // namespace sp
#endif