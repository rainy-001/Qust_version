
// [解析] 瓴控(LK)系列电机封装(MHF9025 直驱 / MG4005 带 10:1 行星)：CAN 的 8 字节反馈 -> 物理量，
// [解析] 上层给的目标值 -> 符合厂家《电机CAN总线通讯协议 V2.36》的 8 字节指令帧。
// [解析] 用法照 applications/can_task.cpp 的 GM6020：cmd_xxx() 存目标 -> write(can.tx_data) 组帧 ->
// [解析] can.send(tx_id) 才真正发出；反馈在 CAN 中断里 if (can.rx_id == m.rx_id) m.read(can.rx_data, stamp)。
// [解析] 坑：angle/speed/torque/multicycle_angle 已换到输出端(除过总减速比)，current 是减速箱前相电流。
#ifndef SP__LK_MOTOR_HPP
#define SP__LK_MOTOR_HPP

#include <cstdint>

namespace sp
{
// ============================ 型号参数 ============================
// 术语约定(与数据手册一致):
//   P:            型号自带减速箱的减速比, 直驱型号为 1.0
//   TORQUE_CONST: 电机侧(减速箱前)转矩常数, 单位 N·m/A
//   MAX_CURRENT:  电机侧(减速箱前)峰值相电流, 单位 A
// 注意: 这里的转矩常数是"电机侧"的, 输出端(减速箱后)力矩 = 相电流 * 转矩常数 * 总减速比

// MHF9025: 直驱(无减速箱)
//   额定 24V / 3.46A / 130rpm / 2.79N·m, 堵转 6.9A / 5.8N·m, 最高 280rpm
//   编码器硬件是 19bit 磁编, 但 CAN 协议里按 16bit 上报(见 get_ecd_range)
constexpr float MHF9025_P1 = 1.0f;
constexpr float MHF9025_TORQUE_CONST = 0.81f;  // N·m/A
constexpr float MHF9025_MAX_CURRENT = 6.90f;   // A, 堵转电流

// MG4005: 4005 + 自带 10:1 行星减速箱(PG4210)
//   额定 24V / 1.6A / 253rpm / 1N·m, 堵转 4A / 2.5N·m, 最高 316rpm
//   编码器同样按 16bit 上报(与"为 MG4005 写的"原中间件一致: encoder/65536、±32768 翻转)
constexpr float MG4005_P10 = 10.0f;
constexpr float MG4005_TORQUE_CONST = 0.06f;  // N·m/A, 电机侧(减速箱前)
constexpr float MG4005_MAX_CURRENT = 4.00f;   // A, 堵转电流

// ---------------------------- 协议常量 ----------------------------
// 依据: 瓴控《电机CAN总线通讯协议 V2.36》
//
// iq 原始值为 int16, 范围 ±2048, 但对应的实际转矩电流分系列不同(手册 P6/P9):
//   MF 系列(含 MHF...): ±2048 <-> ±16.5A  即 33/4096 A/LSB
//   MG 系列:            ±2048 <-> ±33A    即 66/4096 A/LSB
// 故电流换算系数改成按型号取(get_iq_scale)
// [解析] iq 是真正产生力矩的交轴电流，上报的是 int16 计数：真实电流(A) = iq_raw × 本型号系数。
constexpr float LK_MF_IQ_SCALE = 33.0f / 4096.0f;  // 0.008057 A/LSB
constexpr float LK_MG_IQ_SCALE = 66.0f / 4096.0f;  // 0.016113 A/LSB

// 单圈编码器满量程: 协议 0x9C 里 data[6..7] 是 uint16, 所以满量程只可能 <= 65536
//   14bit -> 16384, 15bit -> 32768, 16bit -> 65536
// 当前两个型号都按 16bit(65536):
//   MHF9025 —— 已实测: 左拨杆上档转一整圈, encoder_raw_max 扫满 65535
//   MG4005  —— 沿用原中间件(它就是为 MG4005 写的)的 encoder/65536
// 换型号时如果角度成 1/4、1/2 偏差, 或转一圈 encoder_raw_max 只到 16383/32767,
// 就改 get_ecd_range() 里对应型号的返回值
// [解析] ecd_range 是角度换算的分母：单圈弧度 = encoder / ecd_range × 2π，两个型号目前都取 65536。
constexpr float LK_ECD_RANGE_14BIT = 16384.0f;
constexpr float LK_ECD_RANGE_15BIT = 32768.0f;
constexpr float LK_ECD_RANGE_16BIT = 65536.0f;

// CAN ID 基址(11位标准帧), 手册 P5:
//   命令报文标识符 = 0x140 + ID(1~32)
//   回复报文标识符 = 0x140 + ID(1~32)
constexpr uint16_t LK_RX_ID_BASE = 0x140;
constexpr uint16_t LK_TX_ID_BASE = 0x140;
// [解析] 收发共用一个 ID，靠 data[0] 指令字区分方向与内容，和 RM 电机(收发 ID 不同)的习惯不一样。

// 指令字(数据域 data[0]), 按手册命令编号排列
// [解析] 0x9A/0x9C 是查询，0x80/0x81/0x88 是开关机，0xA1~0xA8 是控制；控制指令的反馈帧数据域与 0x9C 同格式。
constexpr uint8_t LK_CMD_STATE1 = 0x9A;                // 1.  读状态1(温度/电压/电机状态/错误)
constexpr uint8_t LK_CMD_CLEAR_ERROR = 0x9B;           // 2.  清除错误标志
constexpr uint8_t LK_CMD_STATE2 = 0x9C;                // 3.  读状态2(温度/iq/转速/编码器)
constexpr uint8_t LK_CMD_TURN_OFF = 0x80;              // 5.  电机关闭
constexpr uint8_t LK_CMD_TURN_ON = 0x88;               // 6.  电机运行(使能)
constexpr uint8_t LK_CMD_STOP = 0x81;                  // 7.  电机停止
constexpr uint8_t LK_CMD_TORQUE = 0xA1;                // 10. 转矩(电流)闭环控制
constexpr uint8_t LK_CMD_VELOCITY = 0xA2;              // 11. 速度闭环控制
constexpr uint8_t LK_CMD_POSITION = 0xA5;              // 14. 单圈位置闭环控制1
constexpr uint8_t LK_CMD_ANGLE_INCREMENT = 0xA7;       // 16. 增量位置闭环控制1
constexpr uint8_t LK_CMD_ANGLE_INCREMENT_SPEED = 0xA8;  // 17. 增量位置闭环控制2(带限速)

// [解析] 型号决定减速比/转矩常数/iq 系数，构造时一次查好存成 const 成员，运行期不再分支。
enum class LK_Motors
{
  MHF9025,  // 9025 直驱, 自带减速比 1.0
  MG4005    // 4005 + 自带 10:1 行星减速箱
};

// [解析] 下面 6 个是"按型号查参数"的自由函数，只被 LK_Motor 构造函数调用一次。
// [解析] 写法统一是 switch + default 兜底：新增型号忘了加 case 不会编译报错，只会悄悄退化成默认值。
float get_torque_const(LK_Motors motor_type);
float get_model_ratio(LK_Motors motor_type);
float get_max_current(LK_Motors motor_type);
// iq 原始值 -> 实际转矩电流的换算系数, 单位 A/LSB(MF 与 MG 系列不同, 见协议常量注释)
float get_iq_scale(LK_Motors motor_type);
// 协议下发的 iq 原始值上限(取 型号峰值电流 与 协议上限 中较小者)
int16_t get_max_raw(LK_Motors motor_type);
// 单圈编码器满量程: 当前 MHF9025 / MG4005 都取 16bit(65536)
// (MHF9025 已实测转一圈 encoder_raw_max 扫满 65535; MG4005 与原中间件一致)
float get_ecd_range(LK_Motors motor_type);

// [解析] 一个对象 = 一台电机：既存反馈结果(只读状态)，也存待发指令(cmd_* 缓存)。
// [解析] 标准用法三步走：cmd_xxx() 存目标值 -> write(can.tx_data) 组帧 -> can.send(tx_id) 才算发出。
class LK_Motor
{
public:
  // [解析] 构造只换算并存配置，不碰硬件、不发帧；motor_id 必须和电机拨码设的 ID 一致，否则收发 ID 全错。
  // motor_id: 电机ID, 取值 1 ~ 32, 控制帧与反馈帧的 CAN ID 都是 0x140 + motor_id
  // motor_type: 电机型号, 取值见 `LK_Motors`, 型号自带减速箱的减速比包含在型号参数里
  // ratio: 额外加装的行星减速箱减速比(直驱/不加装时填 1.0)
  //        总减速比 = 型号自带减速比 * ratio
  // multi_circle: true 时 angle 为多圈累计角度, false 时 angle 为单圈角度(-π, π]
  LK_Motor(uint8_t motor_id, LK_Motors motor_type, float ratio = 1.0f, bool multi_circle = true);

  const uint16_t rx_id;  // 电机反馈帧ID = LK_RX_ID_BASE + motor_id
  const uint16_t tx_id;  // 电机控制帧ID = LK_TX_ID_BASE + motor_id

  // ---------------------------- 只读状态 ----------------------------
  // [解析] read()/read_state2() 在 CAN 中断里写这批成员、任务里读，没加锁，最多读到"旧一拍"。
  // [解析] 单位：current 是减速箱前相电流，torque/angle/speed/multicycle_angle 是输出端量。
  uint8_t motor_state = 0x10;    // 电机状态, 来自0x9A反馈(0x00开启/0x10关闭)
  uint8_t error_state = 0x00;    // 错误标志, 来自0x9A反馈
  uint16_t encoder_raw = 0;      // 单圈编码器原始值, 用来实测该型号的编码器位宽
  uint16_t encoder_raw_max = 0;  // 上电以来编码器原始值最大值(转一整圈应扫到 满量程-1)
  int8_t temp = 0;               // 摄氏度
  float current = 0;             // 相电流(减速箱前), 单位: A
  float torque = 0;              // 输出端力矩, 单位: N·m
  float speed = 0;             // 输出端转速, 单位: rad/s
  float angle = 0;             // 输出端角度, 单位: rad, 是否多圈由 multi_circle 决定
  float multicycle_angle = 0;  // 输出端多圈累计角度, 单位: rad

  // 总减速比 = 型号自带减速比 * 额外加装的减速比
  float ratio() const { return ratio_total_; }

  // [解析] 下面几个是内联查询接口，给上层做单位换算/限幅用，几乎没有调用开销。
  uint8_t motor_id() const { return motor_id_; }
  LK_Motors motor_type() const { return motor_type_; }
  float model_ratio() const { return ratio_model_; }  // 型号自带减速比
  float added_ratio() const { return ratio_added_; }  // 额外加装的减速比
  int16_t max_raw() const { return max_raw_; }        // 力矩指令的原始值限幅

  // [解析] is_open() = 收到过至少一帧反馈(不是"发过使能帧")；is_alive() = 最近 100ms 内收到过反馈。
  // [解析] now_ms 必须和 read() 传入的时间戳同源(本项目用 osKernelSysTick())，否则判断无效。
  bool is_open() const;
  bool is_alive(uint32_t now_ms) const;

  // ---------------------- 与RM/DM电机对齐的接口 ----------------------
  // [解析] 这三个是统一门面，让上层不必区分电机型号；cmd() 就是 cmd_torque() 的别名。
  // 反馈解析统一入口: 按 data[0] 自动分发到 read_state1/read_state2
  void read(const uint8_t * data, uint32_t stamp_ms);
  // 控制帧统一出口: 按最近一次 cmd_* 设置的模式组帧(默认力矩模式)
  void write(uint8_t * data) const;
  // 力矩控制, 单位: N·m(输出端)
  void cmd(float torque);

  // ---------------------------- 反馈解析 ----------------------------
  // [解析] 一般交给 read() 自动分发；直接调用时自己保证 data[0] 与函数对得上。
  void read_state1(const uint8_t * data);  // 0x9A / 0x9B
  void read_state2(const uint8_t * data);  // 0x9C / 0xA1 ~ 0xA8

  // ---------------------------- 指令帧 ----------------------------
  // 以下函数每次都会把8字节数据域完整填充, 可直接 send()
  // [解析] write_xxx() 只填 8 个字节，不发送；发出去还要 can.send(tx_id)。
  // [解析] 不带参数的指令 data[1..7] 一律补 0；write_open() 与 write_turn_on() 是同一件事(都是 0x88)。
  void write_state1(uint8_t * data) const;       // 0x9A 读取状态1
  void write_state2(uint8_t * data) const;       // 0x9C 读取状态2
  void write_clear_error(uint8_t * data) const;  // 0x9B 清除错误
  void write_open(uint8_t * data) const;         // 0x88 使能
  void write_turn_on(uint8_t * data) const;      // 0x88 使能
  void write_turn_off(uint8_t * data) const;     // 0x80 关闭
  void write_stop(uint8_t * data) const;         // 0x81 停止
  void write_torque(uint8_t * data) const;       // 0xA1
  void write_velocity(uint8_t * data) const;     // 0xA2
  void write_position(uint8_t * data) const;     // 0xA5
  void write_angle_increment(uint8_t * data) const;       // 0xA7
  void write_angle_increment2(uint8_t * data) const;      // 0xA8

  // ---------------------------- 指令缓存 ----------------------------
  // [解析] cmd_xxx() 只记录目标值和模式，换算/组帧/发送都推迟到 write()——只调 cmd 忘了 send 是常见坑。
  // [解析] 力矩参数是输出端 N·m：÷总减速比 -> ÷转矩常数 -> ÷iq 系数 得 iq 原始值，再按 max_raw_ 对称削顶；
  // [解析] 削顶不是等比缩放，指令给太大会一直顶在最大力矩上。传 0 就是不出力，适合做测试起点。
  // 力矩模式, 单位: N·m(输出端), 超过型号峰值转矩会被限幅
  void cmd_torque(float torque);
  // 速度模式, 单位: rad/s(输出端) —— 和反馈 speed 同单位, 方便直接做闭环比较
  void cmd_velocity(float speed);
  // 单圈绝对位置模式, 单位: 度(输出端, 内部会归一化到单圈)
  // direction: 0x00 顺时针, 0x01 逆时针
  void cmd_position(float position, uint8_t direction = 0x00);
  // 角度增量模式, 单位: 度(输出端)
  void cmd_angle_increment(float angle);
  // 角度增量模式(带最大速度限制), 单位: 度, 度/s(均为输出端)
  void cmd_angle_speed(float angle, float max_speed = 0.0f);

private:
  // [解析] 私有成员三类：① 构造时按型号算好的 const 配置 ② 在线判断与多圈计数 ③ cmd_* 指令缓存。
  const uint8_t motor_id_;
  const LK_Motors motor_type_;
  const float ratio_model_;  // 型号自带减速比
  const float ratio_added_;  // 额外加装的减速比
  const float ratio_total_;  // 总减速比
  const float torque_const_;
  const float iq_scale_;  // iq 原始值 -> 转矩电流, 单位 A/LSB
  const int16_t max_raw_;
  const float ecd_range_;
  const bool multi_circle_;

  bool has_read_ = false;
  uint32_t last_read_ms_ = 0;

  int32_t step_ = 0;      // 多圈计数
  uint16_t last_ecd_ = 0;

  // 待发送指令缓存
  uint8_t cmd_mode_ = LK_CMD_TORQUE;
  int16_t cmd_raw_ = 0;
  int32_t cmd_velocity_raw_ = 0;  // 电机侧 0.01°/s
  float cmd_position_ = 0.0f;     // 单圈位置, 输出端 度
  uint8_t cmd_direction_ = 0x00;
  float cmd_angle_ = 0.0f;        // 角度增量, 输出端 度
  float cmd_max_speed_ = 0.0f;    // 最大速度, 输出端 度/s
};

}  // namespace sp

#endif  // SP__LK_MOTOR_HPP
