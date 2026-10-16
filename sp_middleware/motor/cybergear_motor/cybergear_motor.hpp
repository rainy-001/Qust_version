// [解析] ==========================================================================
// [解析] 【文件】sp_middleware/motor/cybergear_motor/cybergear_motor.hpp
// [解析] 【作用】封装小米 CyberGear 微电机(小米微电机)：解析它各种反馈帧，
// [解析]         并把"运动控制/使能/停止/读写参数"等命令打包成 CAN 的 8 字节数据。
// [解析] 【被谁用】配合 sp_middleware/io/can/can.hpp 的 send_ext() 使用：
// [解析]          CAN 帧头里的 communication_type 由本类的成员变量 communication_type 提供，
// [解析]           数据由本类的 write() 填进 can.tx_data。
// [解析] 【关键点】和达妙电机最大的区别：CyberGear 把"要干什么(命令类型)"放在 CAN 的
// [解析]           扩展帧 ID(29 位)里，而不是数据里，所以读反馈时要连帧头 header 一起解析。
// [解析]           本文件里大量的 #define 都是协议规定的编号，不能凭感觉改。
// [解析] 【阅读顺序】先看本文件了解"有哪些命令和参数编号"，再看 cybergear_motor.cpp 看实现。
// [解析] ==========================================================================

#ifndef SP__CYBERGEAR_MOTOR_HPP
#define SP__CYBERGEAR_MOTOR_HPP

#include <cstdint>
// [解析] 需要 uint8_t / uint16_t / uint32_t / uint64_t 这些固定宽度整型。
// [解析] 注意：下面这一大堆 #define 都在全局作用域(不在 namespace sp 里)，名字比较通用，
// [解析]   如果别的文件也定义了同名的宏，会互相覆盖，使用时留意。
//气动cybergearID 0x06 0x00
// [解析] 上面这行原注释说明：这一组默认 ID 是给"气动那台 CyberGear"用的(主机 0x00、电机 0x01)。
// [解析] 注意 Master_CAN_ID / CyberGear_CAN_ID 是宏而不是 const 变量，
// [解析]   所以它们没有类型，参与运算时按 int 处理。
#define Master_CAN_ID 0x00
// [解析] ↑ 主机(STM32)自己的 ID，取值 0x00~0x3F(6 位)。电机回包时会带上它。
#define CyberGear_CAN_ID 0x01
// [解析] ↑ 电机的 CAN ID，出厂默认 0x7F，可用 cmd_set_can_id() 改成 0x00~0x3F。
#define CYBERGEAR_MAX_TORQUE 12.0f            // N·m
// [解析] ↑ 力矩量程 ±12 N·m，16 位整数映射：整数 0 -> -12 N·m，32767 附近 -> 0，65535 -> +12 N·m。
#define CYBERGEAR_MAX_SPEED 30.0f             // 300 rpm = 31.42 rad/s
// [解析] ↑ 速度量程 ±30 rad/s(注意：旁边原注释写"300 rpm = 31.42 rad/s"，
// [解析]   实测宏值取的是 30.0f 而不是 31.42f，两者略有出入，见下面"注意"说明)。
#define CYBERGEAR_MAX_POSITION 3.141593f * 4  // 2圈
// [解析] ↑ 位置量程 ±4π rad = ±2 圈：协议里角度范围 [0~65535] 对应 (-4π, 4π)。
// [解析] 注意：这个宏没有加括号，直接展开成 `3.141593f * 4`。
// [解析]   单独用没问题，但如果写成 `1.0f / CYBERGEAR_MAX_POSITION`，
// [解析]   展开后会变成 `1.0f / 3.141593f * 4`(先除后乘)，结果就不是想要的倒数了。
// [解析] 注意：CYBERGEAR_MAX_SPEED 的宏值是 30.0f，而原注释说 300rpm=31.42rad/s，
// [解析]   二者不一致。30.0f 会让"整数满量程"对应的实际转速偏小约 4.5%，
// [解析]   若发现速度读数总比真实值小一点，可以从这里找原因。

//CyberGear支持的 communication_type 类型
// [解析] ↓ 这就是"通信类型"编号表，相当于命令字。它有三个用途：
// [解析]   1) 发送时：写进扩展帧 ID 的最高 5 位，告诉电机"这条命令是干什么的"；
// [解析]   2) 接收时：从帧头里取出来，判断"这条反馈是哪种反馈"；
// [解析]   3) 本类的成员变量 communication_type 就存这个值，write() 再据此决定怎么填数据。
// [解析] 标"控制"的是主机发给电机，标"反馈"的是电机回给主机。
#define Communication_Type_GetID 0x00          //反馈，获取设备ID：获取设备的ID和64位MCU唯一标识符
#define Communication_Type_MotionControl 0x01  //控制，运动控制指令
#define Communication_Type_MotorRequest 0x02   //反馈，电机反馈数据
// [解析] ↑ 最常用的反馈帧：里面有角度、速度、力矩、温度，read() 主要就是在处理它。
#define Communication_Type_MotorEnable 0x03    //控制，电机使能运行
// [解析] ↑ 使能后电机不会立刻进入运行模式，第一次使能会先做磁编码器标定(见 mode 成员说明)。
#define Communication_Type_MotorStop 0x04      //控制，电机停止运行
#define Communication_Type_SetPosZero 0x06     //控制，设置电机机械零位
// [解析] ↑ 把电机"当前所在位置"记为 0 点。装车找零位时用它，注意别在限位处误发。
#define Communication_Type_CanID 0x07          //更改电机CAN ID
#define Communication_Type_Control_Mode 0x0C
// [解析] ↑ 0x0C 原注释没写说明，属于"控制(设置)模式"类命令。
// [解析] 注意：cybergear_motor.cpp 的 write() 里没有为 0x0C 写分支，
// [解析]   所以只把 communication_type 设成它，data[] 不会被赋值(会残留上一次的数据)。
#define Communication_Type_GetSingleParameter 0x11  //反馈，获取单个参数
#define Communication_Type_SetSingleParameter 0x12  //控制，设置单个参数
// [解析] ↑ 读/写参数都靠"参数索引 + 参数值"这一对，索引表见下面的 0x7005~0x7018。
#define Communication_Type_ErrorFeedback 0x15       //反馈，故障反馈帧
// [解析] ↑ 电机"无法工作"时才发这一帧，里面的 error 位域见类成员 error 的注释。
// [解析] 0x05、0x08~0x0B、0x0D~0x10、0x13、0x14 等编号协议里没有使用或被保留，别乱填。

//电机运行模式参数的index
// [解析] ↓ 这一组是"参数索引"，可以理解成电机内部的寄存器地址：
// [解析]   读写单个参数时，先把索引发过去(SetSingleParameter / GetSingleParameter)，
// [解析]   再跟上要写/读回的值。0x70xx 这一段是协议分配给"运行参数"的编号区。
// [解析] 注意：这些宏名(run_mode、cur_kp ...)非常短且位于全局作用域，
// [解析]   很容易和别处的同名宏/变量冲突，用的时候最好心里有数。
#define run_mode 0x7005       //运行模式参数 0：运控模式，1：位置模式，2：速度模式，3：电流模式
// [解析] ↑ 最关键的参数：它决定电机听哪种指令。
// [解析]   0 运控模式(MotionControl)：用 0x01 命令发 位置/速度/Kp/Kd/力矩；
// [解析]   1 位置模式：只用 loc_ref 给目标角度；2 速度模式：只用 spd_ref 给目标转速；
// [解析]   3 电流模式：只用 iq_ref 给电流指令。写这个参数时只发 1 个字节(见 .cpp 里的特判)。
#define iq_ref 0x7006         //电流模式Iq指令
// [解析] ↑ 电流环的 q 轴(转矩轴)电流指令，是换算过的整数值而不是安培，
// [解析]   具体怎么折算成电流请看说明书；实际使用时一般先设 limit_cur 再给 iq_ref。
#define spd_ref 0x700A        //转速模式转速指令
// [解析] ↑ 速度模式的目标转速，单位 rad/s。
#define imit_torque 0x700B    //转矩限制
// [解析] ↑ 转矩限制值。注意：协议文档里的名字是 limit_torque，这里少了开头的 l，
// [解析]   疑似笔误，但宏的值 0x700B 是对的，用起来没有影响(只是名字看着奇怪)。
#define cur_kp 0x7010         //电流的Kp
#define cur_ki 0x7011         //电流的Ki
// [解析] ↑ 电流环的 PI 参数：kp 决定响应快慢，ki 决定能不能消掉稳态误差。
#define cur_filt_gain 0x7014  //电流滤波系数filter_gain
// [解析] ↑ 电流采样低通滤波的系数，越小越平滑但越迟钝。
#define loc_ref 0x7016        //位置模式角度指令
// [解析] ↑ 位置模式的目标角度，单位 rad。
#define limit_spd 0x7017      //位置模式速度设置
// [解析] ↑ 位置模式下允许的最大转速，用来限制"冲目标点"的速度。
#define limit_cur 0x7018      //速度位置模式电流设置
// [解析] ↑ 速度/位置模式下允许的最大电流，是最直接的一道"力矩安全阀"。

namespace sp
{

// [解析] CyberGear_Motor 类：一个对象 = 一台小米 CyberGear 电机。
// [解析] 它和 sp::DM_Motor 的用法几乎一样，但有一个重要差别：
// [解析]   发命令前要先调用一次 cmd_xxx() 设置"通信类型"，write() 才会知道该按哪种格式填数据。
// [解析] 典型流程：
// [解析]   1) 构造：sp::CyberGear_Motor m(0x00, 0x01, 4*PI, 30.0f, 12.0f);
// [解析]   2) 使能：m.cmd_motor_enable(); m.write(can.tx_data);
// [解析]          can.send_ext(0x03, 0, m.motor_id, m.master_id);
// [解析]   3) 控制：m.cmd(力矩); m.write(can.tx_data); can.send_ext(0x01, m.tar_torque, ...);
// [解析]   4) 反馈：中断里收到扩展帧后 if (id 匹配) m.read(can.rx_id, can.rx_data, 时间戳);
class CyberGear_Motor
{
public:
  // [解析] 构造函数：只保存参数，不碰硬件。
  // [解析] master_id：主机(STM32)自己的 ID，范围 0x00~0x3F。
  // [解析] motor_id ：电机的 CAN ID，出厂默认 0x7F，可通过 cmd_set_can_id() 修改。
  // [解析] pmax/vmax/tmax：三个物理量的量程上限，必须和电机内部设置一致，
  // [解析]   否则定点数换算出来的角度/速度/力矩全是错的(和达妙电机同理)。
  CyberGear_Motor(uint8_t master_id, uint8_t motor_id, float pmax, float vmax, float tmax);

  //主机id为master_id, 电机id为motor_id,电机接收控制帧的时候要指明来自哪个主机，电机反馈的时候要指明是哪台电机
  // [解析] 上面这行原注释是重点：一帧扩展帧 ID 里同时装着"发给哪台电机"和"来自哪个主机"，
  // [解析]   这样一条总线上才能挂多台电机、多个主机而不会串。
  const uint8_t master_id;  //代表主机的ID
  // [解析] ↑ const：创建后不可改。
  const uint8_t motor_id;   //代表电机的ID
  // [解析] ↑ 发送时用它填帧头；收到反馈时用 feedback_can_id 与本值比较，确认"是不是我的帧"。

  //可以工作但状态异常，0：正常，1：欠压，2：过流，4：过温，8：磁编码器故障，16：HALL编码故障，32：未标定 多个错误相加
  // [解析] 注意这些值是"按位"的(1/2/4/8/16/32 都是 2 的幂)：同时出现多种异常时是相加/按位或，
  // [解析]   例如读到 3 表示同时欠压+过流，读到 10(8+2) 表示磁编码器故障+过流。
  // [解析]   判断某一种故障要用 `status_error & 4` 这种按位与，不能用 == 直接比较。
  uint8_t status_error = 0;
  //工作模式状态，0：Reset模式[复位]，1：Cali模式[标定]，2：Motor模式[运行]
  //初次使能电机会进入Reset模式，需要再次使能设定为Motor模式
  // [解析] 上面两行原注释解释了一个新手常见坑：只发一次使能命令的话，电机可能停在标定状态不转，
  // [解析]   需要再发一次(或收到 mode==2 之后再发控制指令)。
  uint8_t mode = 0;

  //无法工作的故障帧反馈内容
  // [解析] ↓ 这个位域只在收到"故障反馈帧"(communication_type = 0x15)时才会被填充，
  // [解析]   和上面的 status_error 是两个来源、两套含义，不要混用。
  // [解析]   注意它的第 16 位存在(共 17 位)，所以判断时不能只看 16 位。
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
  // [解析] ↑ 单位是 0.1℃，所以读到 800 表示 80.0℃；想用摄氏度要自己除以 10。
  // [解析]   这个量通常表示"温度警告阈值"或"当前温度×10"，具体以说明书为准。
  uint32_t temp_warning = 0;

  //反馈的电机id
  uint8_t feedback_can_id;
  // [解析] ↑ read() 时从帧头里解出来的"发送方电机 ID"，用来核对是不是本电机的帧。
  //反馈的主机id
  uint8_t feedback_master_id;
  //反馈的通信类型
  uint8_t feedback_communication_type;
  // [解析] ↑ read() 解出的反馈类型(0x02/0x15/0x11/0x00)，决定按哪种格式解析这 8 个字节。

  float angle = 0;        // 只读! 当前角度，[0~65535]对应(-4π，4π)
  // [解析] ↑ 单位 rad。这句"只读"注释里的 [0~65535] 指的是 CAN 数据里的原始整数范围，
  // [解析]   不是 angle 成员的取值范围 —— angle 已经被换算成 rad 了(约 -12.57 ~ +12.57)。
  float speed = 0;        // 只读! 当前角速度，[0~65535]对应(-30 rad/s，30 rad/s)
  // [解析] ↑ 单位 rad/s，同理范围由宏 CYBERGEAR_MAX_SPEED(30.0f) 决定。
  float torque = 0;       // 只读! 当前力矩，[0~65535]对应(-12 N·m，12 N·m)
  // [解析] ↑ 单位 N·m，量程来自宏 CYBERGEAR_MAX_TORQUE(12.0f)。
  float temperature = 0;  // 只读! 当前温度，Temp(摄氏度)*10
  // [解析] ↑ 数据里是"摄氏度×10"的整数，read() 里已经除以 10，所以这个成员是真正的摄氏度。

  uint16_t index = 0;      //参数索引
  // [解析] ↑ 读参数反馈里回显的索引(如 0x7005)，用来确认"这是我要读的那个参数"。
  float parameter = 0;     //参数值
  // [解析] ↑ 上面那个索引对应的值。注意这里用 float 保存，而协议里可能是 8 位或 32 位整数。
  uint64_t MCU_ID = 0;     // 只读! MCU唯一标识符
  // [解析] ↑ 64 位的芯片唯一 ID，用来区分"同型号的不同电机"，比 CAN ID 更可靠。
  bool ID_gotten = false;  // 只读! 是否成功获取到设备ID

  // [解析] 是否收到过反馈(判断电机是否连上)。
  bool is_open() const;
  // [解析] 通信是否正常：距上次反馈不超过 500ms(见 .cpp)。参数 now_ms 为当前毫秒时间戳。
  // [解析] 注意这个超时是 500ms，比达妙电机的 100ms 宽松得多。
  bool is_alive(uint32_t now_ms) const;

  //读取电机反馈数据
  // [解析] header 是 CAN 扩展帧 ID(can.rx_id)，data 是 8 字节数据(can.rx_data)，
  // [解析] stamp_ms 是当前毫秒时间戳。注意它比 DM_Motor::read() 多一个 header 参数，
  // [解析] 因为 CyberGear 把大量信息放在了帧头里。
  void read(uint32_t header, uint8_t * data, uint32_t stamp_ms);
  //把结构体的内容传到can发送数据中
  // [解析] write()：根据当前 communication_type，把对应内容打包进 data(一般是 can.tx_data)。
  void write(uint8_t * data) const;
  //设置通信类型，并将各种数据传入Motor结构体
  // [解析] ↓ 这些 cmd_xxx() 函数有一个共同点：它们只负责"设置通信类型 + 缓存参数"，
  // [解析]   并不会发送。真正发出去还要再调用 write() 填数据、can.send_ext() 发帧。
  // [解析] 可以理解成：cmd_xxx() = 决定"要说什么"，write()+send_ext() = 真正"说出去"。
  //发送力矩
  // [解析] cmd(力矩)：切到运控模式，并把目标力矩存起来。单位 N·m，超过 ±tmax_ 会被截断。
  // [解析] 注意：它同时把目标位置、目标速度、Kp、Kd 全部清零，也就是"只出力矩、不加闭环"。
  void cmd(float torque);
  //获取设备ID
  // [解析] 只设置通信类型为 0x00。结果看 ID_gotten 和 MCU_ID。
  void cmd_get_id();
  //设置电机CAN ID
  // [解析] 只设置通信类型为 0x07。改 ID 要配合 cmd_set_single_parameter 或专用帧，见说明书。
  void cmd_set_can_id();
  //使能电机
  // [解析] 只设置通信类型为 0x03。发出去后电机开始标定，可能不会立刻转。
  void cmd_motor_enable();
  //停止电机
  // [解析] 只设置通信类型为 0x04。这是最常用的"急停"手段。
  void cmd_motor_stop();
  //设置当前位置为零点
  // [解析] 只设置通信类型为 0x06，把电机当前机械位置记为 0 点。
  void cmd_set_pos_zero();
  //读取单个参数
  // [解析] index 取上面定义的 0x7005/0x700A/... 之一；结果回显在成员 index 与 parameter 里。
  void cmd_get_single_parameter(uint16_t index);
  //设置单个参数
  // [解析] index 同上；parameter 是要写入的值，单位随参数而定(角度 rad、转速 rad/s、Kp 无量纲...)。
  // [解析] 注意：内部的 parameter_ 是 uint32_t，而这里的形参是 float，
  // [解析]   赋值时小数部分会被直接截断(例如 0.5 变成 0)。写 cur_kp/cur_ki 这类
  // [解析]   0~1 的小数参数时要特别小心，建议先确认说明书要求的格式。
  void cmd_set_single_parameter(uint16_t index, float parameter);

  //需要传递的参数
  // [解析] 下面两个是 public 的，因为发 CAN 帧时外层(通常是 can_task)需要读它们：
  // [解析]   can.send_ext(m.communication_type, m.tar_torque, m.motor_id, m.master_id);
  uint16_t tar_torque = 0;
  // [解析] ↑ 已转换成 16 位定点数的目标力矩(不是 N·m 的浮点值)，供 send_ext() 拼进帧头。
  uint8_t communication_type = 0;
  // [解析] ↑ 本次要发送的命令类型，write() 就靠它选择填数据的方式。

private:
  // [解析] 量程副本(单位分别是 rad、rad/s、N·m)，由构造函数传入，const 不可改。
  const float pmax_;
  const float vmax_;
  const float tmax_;

  // [解析] has_read_：是否收到过反馈；last_read_ms_：最近一次反馈的毫秒时间戳。
  bool has_read_ = false;
  uint32_t last_read_ms_;

  //可以发给电机的变量,不需要使用
  // [解析] 原注释说"不需要使用"，意思是从类外面不需要管它们：
  // [解析]   它们由 cmd() 计算好，write() 再取出来用。
  uint16_t tar_position_ = 0;  //目标位置[0~65535]对应(-4π~4π)
  // [解析] ↑ 无符号整数，65535↔+4π、32767 附近↔0、0↔-4π(定点数惯例)。
  uint16_t tar_speed_ = 0;     //目标角速度[0~65535]对应(-30rad/s~30rad/s)
  uint16_t Kp_ = 0;            //Kp [0~65535]对应(0.0~500.0)
  // [解析] ↑ 注意这里用 16 位而不是达妙电机的 12 位，因为 CyberGear 协议给 Kp 留了 2 个字节。
  uint16_t Kd_ = 0;            //Kd [0~65535]对应(0.0~5.0)

  uint16_t index_;      //参数索引
  // [解析] ↑ cmd_get/set_single_parameter() 传进来的索引，write() 时发给电机。
  uint32_t parameter_;  //参数值
  // [解析] ↑ 要写入的参数值。注意它是整型，浮点参数会被截断(见上面的"注意")。
};

}  // namespace sp
#endif