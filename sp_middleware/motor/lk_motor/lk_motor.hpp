// [解析] ==========================================================================
// [解析] 【文件】sp_middleware/motor/lk_motor/lk_motor.hpp
// [解析] 【作用】封装瓴控(LK)系列电机(本工程用的是 MHF9025 直驱、MG4005+10:1 行星减速箱)：
// [解析]         1) 把 CAN 收到的 8 字节原始反馈翻译成温度/电流/力矩/转速/角度；
// [解析]         2) 把上层给的目标值打包成符合厂家《电机CAN总线通讯协议 V2.36》的 8 字节帧。
// [解析] 【被谁用】它属于电控中间件，本仓库里暂时没有调用点(搜索 lk_motor 只在本目录出现)。
// [解析]           使用时照 applications/can_task.cpp 里 GM6020 的套路：
// [解析]             全局对象 sp::LK_Motor m(1, sp::LK_Motors::MHF9025);
// [解析]             发：m.cmd_torque(x); m.write(can.tx_data); can.send(m.tx_id);
// [解析]             收：CAN 中断里 if (can.rx_id == m.rx_id) m.read(can.rx_data, 毫秒时间戳);
// [解析]           它也能当 sp::JointMotorController<MotorType> 的模板参数
// [解析]           (tools/joint/joint_controller.hpp 只要求电机有 angle/speed/torque 和 cmd())。
// [解析] 【关键点】最容易踩的 4 个坑：
// [解析]   1) 单位与坐标系：反馈里带"输出端"字样的量(angle/speed/torque/multicycle_angle)
// [解析]      都已经除过/乘过总减速比，而 current 是电机侧(减速箱前)的相电流，别混用；
// [解析]   2) 定点数量纲：角度 0.01°、速度 0.01dps 或 1dps、编码器是小端 uint16、
// [解析]      力矩/电流用 iq 原始值 int16(±2048)，每个都有自己的"标度因数"；
// [解析]   3) MF 与 MG 两个系列的 iq 换算系数不一样(33/4096 与 66/4096)，选错型号力矩差一倍；
// [解析]   4) cmd_xxx() 只是"把目标值记下来"，真正组帧在 write()，最后别忘了 can.send()。
// [解析] 【阅读顺序】先看本文件了解"有什么功能"，再看 lk_motor.cpp 了解"怎么实现的"。
// [解析] ==========================================================================

#ifndef SP__LK_MOTOR_HPP
#define SP__LK_MOTOR_HPP

#include <cstdint>
// [解析] 这个头文件只用到固定宽度整型(uint8_t/uint16_t/int16_t/int32_t)。
// [解析] 用它而不是 int，是为了让"占几个字节"完全确定——CAN 报文是按字节拼的，不能含糊。

namespace sp
{
// [解析] namespace sp：本工程所有中间件都放在这个命名空间里，避免和 STM32 HAL 库、标准库重名。
// ============================ 型号参数 ============================
// 术语约定(与数据手册一致):
//   P:            型号自带减速箱的减速比, 直驱型号为 1.0
//   TORQUE_CONST: 电机侧(减速箱前)转矩常数, 单位 N·m/A
//   MAX_CURRENT:  电机侧(减速箱前)峰值相电流, 单位 A
// 注意: 这里的转矩常数是"电机侧"的, 输出端(减速箱后)力矩 = 相电流 * 转矩常数 * 总减速比
// [解析] ↑ 这条换算链后面反复要用，先记住它：
// [解析]   输出端力矩(N·m) = 相电流(A) × 转矩常数(N·m/A) × 总减速比
// [解析]   总减速比 = 型号自带减速比(比如 MG4005 的 10) × 额外加装的行星减速比(构造函数传的 ratio)
// [解析]   减速比越大 → 输出端越有劲、但转得越慢，这是"用转速换力矩"。
// [解析] 为什么这些参数不做成一张数组表？因为要按型号做编译期查表，写成常量最直观、零运行开销。

// MHF9025: 直驱(无减速箱)
//   额定 24V / 3.46A / 130rpm / 2.79N·m, 堵转 6.9A / 5.8N·m, 最高 280rpm
//   编码器硬件是 19bit 磁编, 但 CAN 协议里按 16bit 上报(见 get_ecd_range)
constexpr float MHF9025_P1 = 1.0f;
// [解析] 1.0f 就是减速比 1（"P1"表示减速箱减速比 P=1），直驱型号电机轴即输出轴。
// [解析] 末尾的 f 是必须的：不加 f 写成 1.0 会被当成 double，可能触发精度/警告问题。
constexpr float MHF9025_TORQUE_CONST = 0.81f;  // N·m/A
// [解析] 0.81 N·m/A = 厂家给的转矩常数：每 1A 相电流(电机侧)产生 0.81 N·m 力矩。
// [解析] 所以同一台电机电流翻倍，力矩也翻倍；这个值由磁钢和绕组决定，是电机的固有属性。
constexpr float MHF9025_MAX_CURRENT = 6.90f;   // A, 堵转电流
// [解析] 6.90 A 是"堵转电流"(电机被卡住不转时的电流)，也是允许的峰值。
// [解析] 代码用它算力矩指令限幅：命令超过这个电流就被切掉，防止过流烧电机/触发保护。
// [解析] 注意这是"电机侧"电流；由于减速比 1.0，输出端能拿到的最大力矩约 6.9×0.81 ≈ 5.6 N·m。

// MG4005: 4005 + 自带 10:1 行星减速箱(PG4210)
//   额定 24V / 1.6A / 253rpm / 1N·m, 堵转 4A / 2.5N·m, 最高 316rpm
//   编码器同样按 16bit 上报(与"为 MG4005 写的"原中间件一致: encoder/65536、±32768 翻转)
constexpr float MG4005_P10 = 10.0f;
// [解析] 10.0：4005 电机出厂就带 10:1 行星减速箱，力矩×10、转速÷10。
constexpr float MG4005_TORQUE_CONST = 0.06f;  // N·m/A, 电机侧(减速箱前)
// [解析] 0.06 N·m/A 同样是"电机侧"的！输出端力矩 = 电流 × 0.06 × 总减速比。
// [解析] 例：1A 电流 → 电机侧 0.06 N·m → 输出端 0.06×10 = 0.6 N·m。别拿 0.06 当输出端力矩用。
constexpr float MG4005_MAX_CURRENT = 4.00f;   // A, 堵转电流

// ---------------------------- 协议常量 ----------------------------
// [解析] 从这一行往下都是"协议常量"，数值来源是厂家的 CAN 通讯协议手册(HAL 无关)。
// [解析] 这些数字一个都不能随便改：改错了电机要么不响应，要么按错误量纲动作(很危险)。
// 依据: 瓴控《电机CAN总线通讯协议 V2.36》
//
// iq 原始值为 int16, 范围 ±2048, 但对应的实际转矩电流分系列不同(手册 P6/P9):
//   MF 系列(含 MHF...): ±2048 <-> ±16.5A  即 33/4096 A/LSB
//   MG 系列:            ±2048 <-> ±33A    即 66/4096 A/LSB
// 故电流换算系数改成按型号取(get_iq_scale)
// [解析] 什么是 iq？iq 是"交轴电流"，也就是真正产生力矩的那部分电流，闭环控制在用它。
// [解析] 电机上报的 iq 不是安培，而是一个 int16 计数值(iq_raw)，需要乘标度因数才变成安培。
constexpr float LK_MF_IQ_SCALE = 33.0f / 4096.0f;  // 0.008057 A/LSB
// [解析] 33/4096 ≈ 0.008057 A/LSB：MF 系列 iq_raw 每 +1，电流增加约 8.06 mA。
// [解析] 为什么分母是 4096？因为 iq_raw 的范围是 ±2048，正负合起来 4096 份平分满量程 33A。
constexpr float LK_MG_IQ_SCALE = 66.0f / 4096.0f;  // 0.016113 A/LSB
// [解析] 66/4096 ≈ 0.016113 A/LSB，正好是 MF 的两倍：同样的 iq_raw，MG 系列电流/力矩大一倍。
// [解析] 这就是"按型号取系数"的原因——用错系数，力矩会整体差 2 倍，调 PID 会彻底找不到北。

// 单圈编码器满量程: 协议 0x9C 里 data[6..7] 是 uint16, 所以满量程只可能 <= 65536
//   14bit -> 16384, 15bit -> 32768, 16bit -> 65536
// 当前两个型号都按 16bit(65536):
//   MHF9025 —— 已实测: 左拨杆上档转一整圈, encoder_raw_max 扫满 65535
//   MG4005  —— 沿用原中间件(它就是为 MG4005 写的)的 encoder/65536
// 换型号时如果角度成 1/4、1/2 偏差, 或转一圈 encoder_raw_max 只到 16383/32767,
// 就改 get_ecd_range() 里对应型号的返回值
constexpr float LK_ECD_RANGE_14BIT = 16384.0f;
constexpr float LK_ECD_RANGE_15BIT = 32768.0f;
constexpr float LK_ECD_RANGE_16BIT = 65536.0f;
// [解析] 这三个常量是"单圈编码器满量程"的候选值，等于 2 的位数次方：2^14/2^15/2^16。
// [解析] 它是角度换算的分母：单圈角度(rad) = encoder / ecd_range × 2π。
// [解析] 为什么两个型号都选 65536？注释里给了依据：MHF9025 是实测(转一圈 encoder_raw_max
// [解析]   扫满 65535)，MG4005 是沿用原中间件。手册里"14bit 0~16383"只是举例说明位宽有差异。
// [解析] 疑似不一致点：MHF9025 的硬件其实是 19bit 磁编，协议里只按 16bit 上报，
// [解析]   所以这里必须用协议位宽而不是编码器硬件位宽，否则角度会算错。

// CAN ID 基址(11位标准帧), 手册 P5:
//   命令报文标识符 = 0x140 + ID(1~32)
//   回复报文标识符 = 0x140 + ID(1~32)
constexpr uint16_t LK_RX_ID_BASE = 0x140;
constexpr uint16_t LK_TX_ID_BASE = 0x140;
// [解析] 0x140 = 320(十进制)。电机 ID 从 1 开始，所以 1 号电机的收发 ID 都是 0x141。
// [解析] 本型号"命令帧"和"回复帧"共用同一个 ID(只是方向相反)，靠数据域第 1 字节(data[0])
// [解析]   里的"指令字"区分内容——这跟 RM 电机(收发用不同 ID)的习惯不同，注意别照搬。
// [解析] 收发都叫 BASE 是因为值相同；拆成两个常量是为了将来万一厂家改了也不必满代码搜。

// 指令字(数据域 data[0]), 按手册命令编号排列
// [解析] 指令字 = 数据域的第 1 个字节，含义是"这一帧想干什么"的命令编号，由厂家规定。
// [解析] 发送时由 write_xxx() 填进 data[0]；接收时 read() 读 data[0] 决定用哪套解析规则。
// [解析] 下面的注释里"1. 2. 3. …"是手册里的命令编号(缺号是手册里没用到/本工程没实现的命令)。
constexpr uint8_t LK_CMD_STATE1 = 0x9A;                // 1.  读状态1(温度/电压/电机状态/错误)
constexpr uint8_t LK_CMD_CLEAR_ERROR = 0x9B;           // 2.  清除错误标志
constexpr uint8_t LK_CMD_STATE2 = 0x9C;                // 3.  读状态2(温度/iq/转速/编码器)
constexpr uint8_t LK_CMD_TURN_OFF = 0x80;              // 5.  电机关闭
constexpr uint8_t LK_CMD_TURN_ON = 0x88;               // 6.  电机运行(使能)
constexpr uint8_t LK_CMD_STOP = 0x81;                  // 7.  电机停止
// [解析] 0x80(关闭) / 0x81(停止) / 0x88(使能) 是"开关机类"指令：
// [解析]   使能后电机才有力矩输出；关闭/停止后即使发控制帧也不会动，一般用 0x88 做上电初始化。
constexpr uint8_t LK_CMD_TORQUE = 0xA1;                // 10. 转矩(电流)闭环控制
constexpr uint8_t LK_CMD_VELOCITY = 0xA2;              // 11. 速度闭环控制
constexpr uint8_t LK_CMD_POSITION = 0xA5;              // 14. 单圈位置闭环控制1
constexpr uint8_t LK_CMD_ANGLE_INCREMENT = 0xA7;       // 16. 增量位置闭环控制1
constexpr uint8_t LK_CMD_ANGLE_INCREMENT_SPEED = 0xA8;  // 17. 增量位置闭环控制2(带限速)
// [解析] ↑ 控制指令字在协议里不是连续的(0xA3/0xA4/0xA6 是别的功能)，但本工程用到的
// [解析]   0xA1~0xA8 恰好是连续一段，所以 cpp 里用 `cmd >= LK_CMD_TORQUE && cmd <= 0xA8`
// [解析]   一口气判断"这是控制类指令的反馈帧"。这是本文件里最需要留意的"范围判断"。

// [解析] 型号枚举：告诉中间件"这台电机是哪一款"，构造函数靠它查减速比/转矩常数/电流系数。
// [解析] enum class 是 C++ 的"强类型枚举"：使用时必须写全 LK_Motors::MHF9025，
// [解析]   不能像老式 enum 那样直接写 MHF9025，因此既不会和别的同名符号撞车，
// [解析]   也逼着你把型号名字写正确(写错立刻编译报错，而不是悄悄按默认型号运行)。
enum class LK_Motors
{
  MHF9025,  // 9025 直驱, 自带减速比 1.0
  MG4005    // 4005 + 自带 10:1 行星减速箱
};

// [解析] 下面 6 个是"按型号查参数"的普通函数(不属于任何类)。
// [解析] LK_Motor 构造函数会挨个调用它们，并把结果存成自己的 const 成员；
// [解析] 于是运行期(尤其是中断里的 read())不用再判断型号，也少一层分支。
float get_torque_const(LK_Motors motor_type);
// [解析] 返回该型号的转矩常数，单位 N·m/A，指的是"电机侧(减速箱前)"。
float get_model_ratio(LK_Motors motor_type);
// [解析] 返回该型号自带减速箱的减速比(直驱 1.0；MG4005 的 10.0)，无量纲。
float get_max_current(LK_Motors motor_type);
// [解析] 返回该型号电机侧峰值(堵转)电流，单位 A，用来算出力矩指令的限幅值。
// iq 原始值 -> 实际转矩电流的换算系数, 单位 A/LSB(MF 与 MG 系列不同, 见协议常量注释)
float get_iq_scale(LK_Motors motor_type);
// [解析] 返回 A/LSB 标度因数：真实电流(A) = iq_raw × 该系数。MF 与 MG 相差一倍。
// 协议下发的 iq 原始值上限(取 型号峰值电流 与 协议上限 中较小者)
int16_t get_max_raw(LK_Motors motor_type);
// [解析] 返回力矩指令能下发的最大 iq 原始值(对称限幅，即实际范围 [-返回值, +返回值])。
// [解析] 来历：峰值电流 ÷ iq 系数 = 按电流算出的上限，再和协议硬上限 2048 取小。
// [解析] 例：MHF9025 是 6.90 / 0.008057 ≈ 856，小于 2048，所以上限由电流决定；
// [解析]     若某型号电流很大，就会撞上 2048 这个协议天花板。
// 单圈编码器满量程: 当前 MHF9025 / MG4005 都取 16bit(65536)
// (MHF9025 已实测转一圈 encoder_raw_max 扫满 65535; MG4005 与原中间件一致)
float get_ecd_range(LK_Motors motor_type);
// [解析] 返回"转一整圈编码器数多少个数"，只会是 16384 / 32768 / 65536 之一。
// [解析] 换型号后如果角度成 1/4、1/2 倍的偏差，或实测 encoder_raw_max 只到 16383/32767，
// [解析]   就要回来改 cpp 里 get_ecd_range() 对应型号的返回值。

// [解析] LK_Motor 类：一个对象 = 一台瓴控电机。
// [解析] 它同时装着两类数据：① 反馈结果(只读状态成员) ② 待发指令(cmd_* 缓存)。
// [解析] 因此标准用法是"三步走"：cmd_xxx() 存目标值 → write() 组帧 → can.send() 真正发送。
// [解析] 典型使用流程(照 applications/can_task.cpp 里 GM6020 的套路套过来)：
// [解析]   1) 全局对象: sp::LK_Motor m(1, sp::LK_Motors::MHF9025);   // 1 号电机
// [解析]   2) 使能:     m.write_open(can.tx_data); can.send(m.tx_id);  // 发 0x88 使能帧
// [解析]   3) 控制:     m.cmd_torque(1.0f); m.write(can.tx_data); can.send(m.tx_id);
// [解析]   4) 反馈:     CAN 中断里 if (can.rx_id == m.rx_id) m.read(can.rx_data, stamp_ms);
class LK_Motor
{
public:
  // motor_id: 电机ID, 取值 1 ~ 32, 控制帧与反馈帧的 CAN ID 都是 0x140 + motor_id
  // motor_type: 电机型号, 取值见 `LK_Motors`, 型号自带减速箱的减速比包含在型号参数里
  // ratio: 额外加装的行星减速箱减速比(直驱/不加装时填 1.0)
  //        总减速比 = 型号自带减速比 * ratio
  // multi_circle: true 时 angle 为多圈累计角度, false 时 angle 为单圈角度(-π, π]
  LK_Motor(uint8_t motor_id, LK_Motors motor_type, float ratio = 1.0f, bool multi_circle = true);
  // [解析] 构造函数只把参数换算成配置常量存进成员，不碰硬件、不发任何 CAN 帧，随时可以构造。
  // [解析] 参数单位说明：
  // [解析]   motor_id：1~32，必须和电机拨码/上位机设置的 ID 一致，否则收发 ID 全错(收不到任何反馈)。
  // [解析]   motor_type：决定减速比、转矩常数、电流系数(见 LK_Motors)。
  // [解析]   ratio：额外加装减速箱的减速比，直驱/不加装填 1.0；比如再加一级 2:1 就填 2.0。
  // [解析]   multi_circle：只影响 angle 成员的语义(多圈累计 or 单圈)，multicycle_angle 始终是多圈。
  // [解析] 后两个参数有默认值，所以最简写法就是 LK_Motor m(1, LK_Motors::MHF9025);

  const uint16_t rx_id;  // 电机反馈帧ID = LK_RX_ID_BASE + motor_id
  const uint16_t tx_id;  // 电机控制帧ID = LK_TX_ID_BASE + motor_id
  // [解析] 这两个 ID 设计成 const：对象一旦建好就固定，避免被别处误改导致"发到别人家电机上"。
  // [解析] 代价是改不了——若电机 ID 拨码变了，只能重新构造一个对象。

  // ---------------------------- 只读状态 ----------------------------
  // [解析] 下面这批成员是"反馈结果"，由 read()/read_state2() 在 CAN 接收中断里更新。
  // [解析] ★ 单位与坐标系(最容易混的地方)：
  // [解析]   current 是电机侧(减速箱前)相电流；angle/speed/torque/multicycle_angle 都已换算到
  // [解析]   输出端(减速箱后)，即上层拿到的就是"关节上"的量，一般不需要再乘减速比。
  // [解析] ★ 中断在写、任务在读，这些 float 没有加锁；实践中读到的值最多"旧一拍"，
  // [解析]   做闭环完全够用。若要求极严，可在任务里先读进局部变量再参与运算。
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
  // [解析] encoder_raw / encoder_raw_max 是留给你的"排查工具"：
  // [解析]   上电后用手把输出端慢慢转整整一圈，看 encoder_raw_max 最大能到多少，
  // [解析]   就能确认这台电机的编码器在协议里到底是 14/15/16 位，从而决定 get_ecd_range()。
  // [解析] angle 与 multicycle_angle 的区别：multi_circle=true 时两者数值相同(连续多圈值)；
  // [解析]   false 时 angle 被 limit_angle() 归一到单圈 (-π, π]，每转一圈会在 ±π 处跳变一次，
  // [解析]   直接拿它做位置环会导致电机在跳变点"抽一下"，此时应该用 multicycle_angle。
  // [解析] torque 是"估计值"而不是实测值：由电流×转矩常数×总减速比算来，作为力矩反馈够用。
  // [解析] temp 是 int8 摄氏度，范围 -128~127；不接电机时读到的是它内部温度，不必惊慌。

  // 总减速比 = 型号自带减速比 * 额外加装的减速比
  float ratio() const { return ratio_total_; }
  // [解析] 下面这一组都是"查询接口"(只读)：函数体写在头文件里会自动内联，几乎没有调用开销。
  // [解析] 它们主要给上层做单位换算和限幅用，例如上层想知道"力矩对应多少电流上限"。

  uint8_t motor_id() const { return motor_id_; }
  LK_Motors motor_type() const { return motor_type_; }
  float model_ratio() const { return ratio_model_; }  // 型号自带减速比
  float added_ratio() const { return ratio_added_; }  // 额外加装的减速比
  int16_t max_raw() const { return max_raw_; }        // 力矩指令的原始值限幅
  // [解析] ratio() 给出的是"总减速比"，用它可以把电机侧的量换算到输出端(乘)，或反向(除)。
  // [解析] max_raw() 是力矩指令能下发的最大 iq 原始值(±)，cmd_torque() 内部就是按它限幅的。

  bool is_open() const;
  bool is_alive(uint32_t now_ms) const;
  // [解析] is_open()：是否至少收到过一帧有效反馈(即电机已被使能并开始回报)。没收到前为 false。
  // [解析] is_alive(now_ms)：最近 100ms 内收到过反馈吗？掉线检测用(超时阈值写死在 cpp 里)。
  // [解析]   参数 now_ms 必须和 read() 传入的时间戳同源(都用 osKernelSysTick())，否则判断无效。

  // ---------------------- 与RM/DM电机对齐的接口 ----------------------
  // 反馈解析统一入口: 按 data[0] 自动分发到 read_state1/read_state2
  void read(const uint8_t * data, uint32_t stamp_ms);
  // 控制帧统一出口: 按最近一次 cmd_* 设置的模式组帧(默认力矩模式)
  void write(uint8_t * data) const;
  // 力矩控制, 单位: N·m(输出端)
  void cmd(float torque);
  // [解析] 这三个是"统一门面"，目的是让上层代码不必区分电机型号(和 RM/DM 电机写法一致)：
  // [解析]   read()  → 收帧解析入口，自动按 data[0] 分发；
  // [解析]   write() → 发帧组帧出口，按最近一次 cmd_xxx 决定的模式生成 8 字节；
  // [解析]   cmd()   → 等价于 cmd_torque()。
  // [解析] read() 的 stamp_ms 是"收到这一帧时的系统毫秒数"，用于 is_alive() 和 last_read_ms_。

  // ---------------------------- 反馈解析 ----------------------------
  void read_state1(const uint8_t * data);  // 0x9A / 0x9B
  void read_state2(const uint8_t * data);  // 0x9C / 0xA1 ~ 0xA8
  // [解析] read_state1 解析"读状态1/清错误"的反馈：只有温度、开关状态、错误码。
  // [解析] read_state2 解析"读状态2 及所有控制指令"的反馈：温度/iq/转速/编码器(数据量大)。
  // [解析] 两者一般不用直接调，交给 read() 自动分发即可；直接调时自己保证 data[0] 对得上。

  // ---------------------------- 指令帧 ----------------------------
  // 以下函数每次都会把8字节数据域完整填充, 可直接 send()
  // [解析] 这些 write_xxx() 只负责把 8 个字节填好，**不发送**！发出去还要 can.send(tx_id)。
  // [解析] 参数 data 是长度至少 8 的缓冲区(最常用 can.tx_data)，可以理解为"往这里贴字节"。
  // [解析] 全部是 const 函数：组帧只读成员(指令缓存)、不改对象状态，多任务调用也不会互相破坏。
  // [解析] 0x9A/0x9B/0x9C/0x80/0x81/0x88 这几条指令不带参数，数据域剩余字节一律补 0，
  // [解析]   因为协议规定数据域固定 8 字节，不能只发前 1 个字节。
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
  // [解析] 注意 write_open() 与 write_turn_on() 是同一件事(都发 0x88)，保留两个名字只是
  // [解析]   为了迁就不同人的叫法，实际用哪个都行。

  // ---------------------------- 指令缓存 ----------------------------
  // [解析] 这一节是"设定目标值"的函数：只把数值记进成员变量，不做换算、不组帧、更不发送。
  // [解析] 真正的单位换算和字节打包推迟到 write_xxx() 里做。好处是控制频率和发送频率可以不同；
  // [解析] 代价是"调了 cmd_xxx() 却忘了 write()+send()"时电机毫无反应——新手最常犯的错。
  // 力矩模式, 单位: N·m(输出端), 超过型号峰值转矩会被限幅
  void cmd_torque(float torque);
  // [解析] 参数是"输出端"力矩(N·m)：内部会先除以总减速比换成电机侧力矩，再换成电流、iq 原始值，
  // [解析] 最后按 max_raw_ 限幅(正负对称)。传 0 就是"不出力"，是安全测试起点。
  // 速度模式, 单位: rad/s(输出端) —— 和反馈 speed 同单位, 方便直接做闭环比较
  void cmd_velocity(float speed);
  // [解析] 用 rad/s 而不是 rpm，就是为了和反馈 speed 同单位，写 PID 时不用来回换算。
  // [解析] 注意：组帧时还会附上一个转矩电流上限(见 write_velocity)，否则电机可能没力矩。
  // 单圈绝对位置模式, 单位: 度(输出端, 内部会归一化到单圈)
  // direction: 0x00 顺时针, 0x01 逆时针
  void cmd_position(float position, uint8_t direction = 0x00);
  // [解析] 位置是"单圈绝对角度"：内部会乘减速比换到电机侧，再归一化到 [0, 360) 度。
  // [解析] 因为归一化了，输入 370 度和 10 度等价；想要"转半圈"请用 cmd_angle_increment()。
  // 角度增量模式, 单位: 度(输出端)
  void cmd_angle_increment(float angle);
  // [解析] 相对当前位置转多少度，正数一个方向、负数反向；可以大于 360(会连续转多圈)。
  // 角度增量模式(带最大速度限制), 单位: 度, 度/s(均为输出端)
  void cmd_angle_speed(float angle, float max_speed = 0.0f);
  // [解析] 比 cmd_angle_increment() 多一个"最大速度"限制，电机侧按 1°/s 上报(uint16)。
  // [解析] max_speed 默认 0.0f：协议里 0 通常表示"不限速"(取决于电机固件)，需要限速就显式填。

private:
  // [解析] 私有成员分三类，看懂这三类就懂了这个类的全部结构：
  // [解析]   ① 配置(带 const)：构造时按型号算好，之后只读，运行期不产生额外分支；
  // [解析]   ② 运行状态：has_read_/last_read_ms_ 管"在线判断"，step_/last_ecd_ 管"多圈计数"；
  // [解析]   ③ 指令缓存(cmd_*)：cmd_xxx() 写、write() 读，是"组帧"的原料。
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
  // [解析] ratio_total_ = ratio_model_ × ratio_added_，是"输出端 ⇄ 电机侧"换算的唯一钥匙：
  // [解析]   电机侧转 10 圈 = 输出端转 1 圈(MG4005 且不加装时)，所有反馈/指令都用它换算。

  bool has_read_ = false;
  uint32_t last_read_ms_ = 0;
  // [解析] has_read_：是否已经收到过第一帧反馈(首次反馈时要初始化 last_ecd_，见 cpp 里的说明)。
  // [解析] last_read_ms_：最近一次收到反馈的系统毫秒时刻，is_alive() 用它判断掉线。

  int32_t step_ = 0;      // 多圈计数
  uint16_t last_ecd_ = 0;
  // [解析] step_ 记录"编码器已经越过 0 点几次"，正负表示方向；
  // [解析] last_ecd_ 保存上一次的编码器原始值，用来算两次之间的差值，判断有没有跨圈。
  // [解析] 多圈角度 = 单圈角度 + step_ × 2π，这条式子后面 cpp 里会用。

  // 待发送指令缓存
  // [解析] cmd_mode_ 决定 write() 走哪个分支，默认是力矩模式(构造函数里被初始化成 0xA1)。
  uint8_t cmd_mode_ = LK_CMD_TORQUE;
  int16_t cmd_raw_ = 0;
  int32_t cmd_velocity_raw_ = 0;  // 电机侧 0.01°/s
  float cmd_position_ = 0.0f;     // 单圈位置, 输出端 度
  uint8_t cmd_direction_ = 0x00;
  float cmd_angle_ = 0.0f;        // 角度增量, 输出端 度
  float cmd_max_speed_ = 0.0f;    // 最大速度, 输出端 度/s
  // [解析] cmd_raw_ 已经是换算好的 iq 原始值(=力矩→电流→iq)，组帧时直接拼字节，单位 LSB。
  // [解析] 其余缓存都保留"物理量"原样(输出端度/度每秒)，推迟到组帧时才做乘减速比等换算。
};

}  // namespace sp

#endif  // SP__LK_MOTOR_HPP
