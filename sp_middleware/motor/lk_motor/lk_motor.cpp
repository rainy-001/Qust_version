// [解析] ==========================================================================
// [解析] 【文件】sp_middleware/motor/lk_motor/lk_motor.cpp
// [解析] 【作用】lk_motor.hpp 里声明的函数的具体实现：型号参数查表、反馈解析(字节->物理量)、
// [解析]         指令组帧(物理量->字节)、多圈角度累计。
// [解析] 【被谁用】由 lk_motor.hpp 的使用者间接调用，通常是 CAN 接收中断(read)和一个控制任务
// [解析]         (cmd_xxx + write + can.send)。本仓库当前没有调用点，用法见 hpp 的【被谁用】。
// [解析] 【看代码的重点】4 个最容易看不懂的地方，下面都逐段标注：
// [解析]   1) iq 原始值 -> 电流 -> 力矩 的换算链，以及为什么系数要按 MF/MG 系列区分；
// [解析]   2) 小端字节拼接 (data[3] << 8) | data[2] 是什么意思、为什么必须强转 int16_t；
// [解析]   3) 多圈计数：为什么差值大于半圈就认为"跨 0 点了"，step_ 加减 1 的含义；
// [解析]   4) 组帧时 `& 0xFF` / `>> 8` 这类位运算，是把整数按小端拆成字节的标准写法。
// [解析] ==========================================================================

#include "lk_motor.hpp"

#include "tools/math_tools/math_tools.hpp"
// [解析] 用到里面的 SP_PI(圆周率)、limit_angle()(把角度归一到 (-π, π])、
// [解析] limit_max()(把值限幅到 [-max, max])，见 tools/math_tools/math_tools.hpp。

namespace sp
{
// [解析] 下面这个常量是"int32 数据域"的限幅值：协议里速度/角度增量用 int32 表示，
// [解析] 若上层给了超大值，浮点转 int32 会溢出(未定义行为)，所以先夹到 ±2.0e9 再转。
// int32 数据域的限幅值, 取 2e9(略小于 INT32_MAX)避免浮点转整数溢出
constexpr float LK_MAX_RAW_I32 = 2.0e9f;

// [解析] 下面 4 个 get_xxx() 都是"按型号查表"，写法统一：switch 命中就返回该型号常量，
// [解析] default 兜底返回另一种型号的值。这样即使以后新增型号忘了加 case，也只会"退化成默认值"
// [解析] 而不会编译不过(好处是稳，坏处是错了不报错——新增型号时记得回来补 case)。
float get_torque_const(LK_Motors motor_type)
{
  switch (motor_type) {
    case LK_Motors::MHF9025:
      return MHF9025_TORQUE_CONST;  // [解析] 直驱 9025：0.81 N·m/A
    default:
      return MG4005_TORQUE_CONST;   // [解析] 其它(含 MG4005)：0.06 N·m/A
  }
}

float get_model_ratio(LK_Motors motor_type)
{
  switch (motor_type) {
    case LK_Motors::MHF9025:
      return MHF9025_P1;    // [解析] 直驱，减速比 1.0
    default:
      return MG4005_P10;    // [解析] MG4005 自带 10:1
  }
}

float get_max_current(LK_Motors motor_type)
{
  switch (motor_type) {
    case LK_Motors::MHF9025:
      return MHF9025_MAX_CURRENT;  // [解析] 6.90 A
    default:
      return MG4005_MAX_CURRENT;   // [解析] 4.00 A
  }
}

float get_iq_scale(LK_Motors motor_type)
{
  // 手册 P6: MG 电机 iq 分辨率 (66/4096 A)/LSB; MF 电机 iq 分辨率 (33/4096 A)/LSB
  // MHF 属于 MF 家族(带 H 中空轴), 所以用 MF 的分辨率
  // [解析] 这里就是"型号决定量纲"的具体落点：同样的 iq_raw，MF 与 MG 换算出的电流差一倍。
  switch (motor_type) {
    case LK_Motors::MHF9025:
      return LK_MF_IQ_SCALE;  // [解析] 33/4096 ≈ 0.008057 A/LSB
    default:
      return LK_MG_IQ_SCALE;  // [解析] 66/4096 ≈ 0.016113 A/LSB
  }
}

int16_t get_max_raw(LK_Motors motor_type)
{
  // [解析] 先用"峰值电流 ÷ 电流系数"算出按电流允许的原始值上限(即 856 或 248 这种数)。
  auto raw = get_max_current(motor_type) / get_iq_scale(motor_type);
  if (raw > 2048.0f) raw = 2048.0f;  // 协议上限
  // [解析] ↑ 2048 是协议的硬天花板(iq_raw 是 int16 但厂家只用 ±2048 这一段)，
  // [解析]   两个上限取小，保证"既不超电流，也不超协议"。
  return static_cast<int16_t>(raw);
  // [解析] 强转 int16_t 是安全的：上面的 raw 一定落在 0~2048，绝不会溢出。
  // [解析] 注意返回值只表示"幅值"，实际使用会写成 ±max_raw_。
}

float get_ecd_range(LK_Motors motor_type)
{
  // 两个型号都按 16bit(65536) 处理:
  //   MHF9025: 实测左拨杆上档转一圈, encoder_raw_max 扫满 65535 ✓
  //   MG4005 : 按原中间件(它就是为 MG4005 写的)的写法 —— encoder/65536、±32768 翻转
  // 手册 P6 那句"14bit 0~16383"是举例说明不同型号位宽, 不代表这两个型号就是 14bit
  // [解析] 这个函数只可能返回 16384/32768/65536，它是"编码器计数 -> 弧度"的分母：
  // [解析]   单圈弧度 = encoder / ecd_range × 2π。返回 65536 意味着 1 个计数 ≈ 0.0000959 rad。
  switch (motor_type) {
    case LK_Motors::MHF9025:
      return LK_ECD_RANGE_16BIT;  // [解析] 16bit 满量程 65536
    default:
      return LK_ECD_RANGE_16BIT;  // [解析] 同样是 65536(见上面注释的依据)
      // [解析] 注意：两个分支返回值一模一样，这个 switch 目前等于"没分支"。
      // [解析]   保留结构是为了将来测试出 MG4005 其实是 14bit 时可以只改一行。
  }
}

// [解析] 下面是构造函数的"初始化列表"：冒号后面这些 xxx(值) 会在函数体执行之前完成赋值。
// [解析] 之所以必须用初始化列表(而不是在函数体里赋值)，是因为这些成员都带 const，
// [解析] const 成员只能在初始化时给值，之后再也不能改。
// [解析] 顺序提醒：成员的初始化顺序是"按声明顺序"而不是按这里写的顺序，
// [解析]   本文件里 ratio_total_ 用到 get_model_ratio() 重新算了一遍，与 ratio_model_ 无关，
// [解析]   所以即使顺序不同也不会出问题(这是一个容易忽略的细节)。
LK_Motor::LK_Motor(uint8_t motor_id, LK_Motors motor_type, float ratio, bool multi_circle)
: rx_id(LK_RX_ID_BASE + motor_id),
  // [解析] 反馈帧 ID = 0x140 + 电机 ID，接收中断里用它匹配"这帧是不是我的"。
  tx_id(LK_TX_ID_BASE + motor_id),
  // [解析] 控制帧 ID 与反馈帧同值(协议规定共用 0x140+ID)，发送时传给 can.send()。
  motor_id_(motor_id),
  motor_type_(motor_type),
  ratio_model_(get_model_ratio(motor_type)),
  // [解析] 型号自带减速比：MHF9025=1.0，MG4005=10.0。
  ratio_added_(ratio),
  ratio_total_(get_model_ratio(motor_type) * ratio),
  // [解析] 总减速比 = 自带 × 加装。所有"输出端 ⇄ 电机侧"换算都乘/除这个数。
  torque_const_(get_torque_const(motor_type)),
  iq_scale_(get_iq_scale(motor_type)),
  max_raw_(get_max_raw(motor_type)),
  ecd_range_(get_ecd_range(motor_type)),
  multi_circle_(multi_circle)
  // [解析] 上面几行把"型号相关的所有量纲"一次性算好存起来，运行期不再查表。
{
}
// [解析] 函数体是空的：构造阶段不碰硬件、不发 CAN 帧，所以随时(包括上电初始化时)都可以构造对象。

bool LK_Motor::is_open() const { return has_read_; }
// [解析] "打开"的判据是"收到过反馈"而不是"发过使能帧"：只有电机真的回话才算它已在运行。

bool LK_Motor::is_alive(uint32_t now_ms) const
{
  return is_open() && (now_ms - last_read_ms_ < 100);
  // [解析] 100 是掉线超时阈值，单位 ms：超过 100ms 没收到反馈就认为电机掉线。
  // [解析] 这里写成 now_ms - last_read_ms_ 而不是 last_read_ms_ + 100 < now_ms：
  // [解析]   利用无符号整数的"回绕"特性，即使 osKernelSysTick() 溢出归零，差值依然是对的，
  // [解析]   这是嵌入式判断超时的标准写法。
  // [解析] 调用方传进来的 now_ms 必须和 read() 时的时间戳来自同一个计时器(本项目用 osKernelSysTick())。
}

// ---------------------------- 反馈解析 ----------------------------
// [解析] 数据流：CAN 中断收到帧 -> 上层比对 rx_id -> 调 read(data, 时间戳)。
// [解析] read() 是"分发器"，按 data[0] 指令字决定交给 read_state1 还是 read_state2 解析。

void LK_Motor::read(const uint8_t * data, uint32_t stamp_ms)
{
  // 能进入本函数的帧都来自本电机的 CAN ID(LK_RX_ID_BASE + motor_id), 因此都算"在线"
  // [解析] 所以第一件事就是把"最近收到反馈的时刻"刷新，is_alive() 依赖它。
  // [解析] 注意：这里默认调用者已经用 rx_id 过滤过了(read 内部不再查 ID)。
  // [解析]   如果你偷懒把所有帧都喂进来，别的电机的数据会被当成自己的，角度会乱跳。
  last_read_ms_ = stamp_ms;

  const uint8_t cmd = data[0];
  // [解析] 先取出来存成局部变量，避免下面重复写 data[0]；也顺便说明"第 1 字节是指令字"。

  if (cmd == LK_CMD_STATE1 || cmd == LK_CMD_CLEAR_ERROR) {
    read_state1(data);
    // [解析] 0x9A(读状态1) 与 0x9B(清错误) 的反馈数据域布局相同，共用一个解析函数。
  }
  else if (cmd == LK_CMD_STATE2 || (cmd >= LK_CMD_TORQUE && cmd <= LK_CMD_ANGLE_INCREMENT_SPEED)) {
    // 控制指令的反馈帧与0x9C的数据域布局一致
    read_state2(data);
    // [解析] 条件等价于 cmd == 0x9C || (0xA1 <= cmd <= 0xA8)：任何一条控制指令发下去，
    // [解析] 电机都会用"和 0x9C 一样格式"的状态帧回话，所以统一按 read_state2 解析。
  }
  // 0x80 关闭 / 0x81 停止 / 0x88 使能 的反馈帧不带状态数据, 只用于确认在线
  // [解析] ↑ 这类帧这里故意不解析内容，但它上面已经刷新了 last_read_ms_，
  // [解析]   所以"发使能帧 -> 收到空反馈"之后 is_alive() 就会变成 true。
}

void LK_Motor::read_state1(const uint8_t * data)
{
  if (data[0] != LK_CMD_STATE1 && data[0] != LK_CMD_CLEAR_ERROR) return;
  // [解析] 防御性检查：单独调用本函数时若指令字不对就直接返回，免得读错位置覆盖状态。
  // [解析] 数据域布局(手册 P5)：data[1] 温度(int8, ℃)、data[6] 电机状态、data[7] 错误标志。

  this->temp = static_cast<int8_t>(data[1]);
  // [解析] data[1] 是 uint8_t，温度可能为负(比如 -5℃ = 0xFB)，必须强转成有符号的 int8_t；
  // [解析] 不转的话 0xFB 会被当成 251℃，读出来完全不对。这是"有符号/无符号"的经典坑。
  this->motor_state = data[6];
  // [解析] 电机状态：0x00 表示已开启(使能)，0x10 表示已关闭。注意是"0 才是开"，别反着理解。
  this->error_state = data[7];
  // [解析] 错误标志位(过压/欠压/过流/堵转等按位定义)，为 0 表示正常；非 0 时可发 0x9B 清除错误。
}

void LK_Motor::read_state2(const uint8_t * data)
{
  const uint8_t cmd = data[0];
  if (
    cmd != LK_CMD_STATE2 && (cmd < LK_CMD_TORQUE || cmd > LK_CMD_ANGLE_INCREMENT_SPEED)) return;
  // [解析] 同样先校验指令字：只接受 0x9C 和 0xA1~0xA8 这一段，其它帧直接丢弃。

  // 数据域(减速箱前的电机侧量), 见手册 P6:
  //   data[1]    温度, int8, 1℃/LSB
  //   data[2..3] iq 原始值(int16, 小端); MF 系列 33/4096 A/LSB, MG 系列 66/4096 A/LSB
  //   data[4..5] 转速, int16, 小端, 1dps/LSB
  //   data[6..7] 单圈编码器, uint16, 小端(满量程见 get_ecd_range: 两个型号都按 65536)
  // [解析] "小端"= 低位字节在前：data[2] 是低 8 位、data[3] 是高 8 位，所以拼成
  // [解析]   (data[3] << 8) | data[2]。<< 8 把高字节移到高 8 位，| 再把低字节接上。
  const int16_t iq_raw = static_cast<int16_t>((data[3] << 8) | data[2]);
  // [解析] 拼出来的是一个"16 位无符号含义"的数，例如 0xFF80 等于 65408；
  // [解析] 强转成 int16_t 之后它才变成 -128。不加强转的话电流会被当成 +65408 这种离谱值。
  // [解析] (iq 可能为负：电机反转或回馈制动时，转矩电流就是负的。)
  const int16_t speed_raw = static_cast<int16_t>((data[5] << 8) | data[4]);
  // [解析] 转速原始值，单位 1dps(度/秒)，是"电机侧"的转速(减速箱前)，后面要除减速比。
  const uint16_t encoder = static_cast<uint16_t>((data[7] << 8) | data[6]);
  // [解析] 单圈编码器计数，0 ~ 65535(16bit 满量程)，转一圈从 0 走到 65535 再回 0。
  // [解析] 这里拼完转成 uint16_t，因为它一定是正数(角度位置没有负值)。

  // 首次读取时初始化 last_ecd_
  if (!has_read_) {
    has_read_ = true;
    last_ecd_ = encoder;
    // [解析] 第一帧只用来"打底"：把 last_ecd_ 设成当前值，step_ 保持 0。
    // [解析] 否则第一次算 delta 时 last_ecd_ 还是 0，会被误判成"跨了半圈"。
  }

  // 多圈计数: 过 0 点时 ±满量程 跳变
  // [解析] 编码器只有单圈信息，转第二圈时数值会从 65535 突然掉回 0(或者反过来)。
  // [解析] 判断方法：两次采样差值若超过"半圈"，说明其实是反向跨过 0 点，用 step_ 补一圈回来。
  const int32_t delta = static_cast<int32_t>(encoder) - static_cast<int32_t>(last_ecd_);
  // [解析] 强转 int32_t 再相减：两个 uint16 直接相减结果仍是 uint16(或提升为 int)，
  // [解析] 遇到 65535 -> 5 这种跨圈会得到很大的正数而不是 -65530，因此必须先扩成 32 位有符号。
  if (delta > static_cast<int32_t>(ecd_range_ / 2.0f))
    step_--;
  // [解析] delta 大于半圈(32768)：只可能是 last_ecd_ 接近 0 而 encoder 接近满量程，
  // [解析]   即电机反向跨过了 0 点，所以圈数减 1。
  else if (delta < -static_cast<int32_t>(ecd_range_ / 2.0f))
    step_++;
  // [解析] delta 小于 -半圈：说明是"从接近满量程跳到接近 0"的正向跨越，圈数加 1。
  // [解析] 副作用：如果两个采样间隔内电机转得超过半圈(高速场景)，这套判断会数错圈，
  // [解析]   所以多圈角度只适合中低速使用；高速时请用单圈 angle 或外接其他传感器。
  last_ecd_ = encoder;
  // [解析] 保存本次值，给下一帧做差用。

  // 电机侧多圈角度 / 单圈角度, 单位: rad
  const float motor_angle_single = encoder / ecd_range_ * 2.0f * SP_PI;
  // [解析] 单圈角度换算：encoder/ecd_range 得到 0~1 的比例，再乘 2π 变成弧度，范围 [0, 2π)。
  // [解析] 注意这里除的是 float(ecd_range_ 是 65536.0f)，所以是浮点除法，不会整除丢精度。
  const float motor_angle =
    motor_angle_single + static_cast<float>(step_) * 2.0f * SP_PI;
  // [解析] 多圈角度 = 单圈角度 + 圈数 × 2π。step_ 只在这个函数里变化，所以是连续的(不会跳变)。
  // [解析] 以上两个都是"电机侧(减速箱前)"的角度，下面统一除减速比换成输出端。

  // 更新公有属性(均为减速箱后的输出端量)
  this->temp = static_cast<int8_t>(data[1]);
  // [解析] 控制指令的反馈帧里也带温度，所以这里再写一次(和 read_state1 的值同源)。
  this->encoder_raw = encoder;
  if (encoder > encoder_raw_max) this->encoder_raw_max = encoder;
  // [解析] 只记录历史最大值(上电后不自动清零)：手动转一整圈就能看出满量程，用来核实编码器位宽。
  this->current = iq_raw * iq_scale_;
  // [解析] iq_raw(LSB) × 系数(A/LSB) = 相电流(A)，这是"电机侧"电流(减速箱前)。
  this->torque = this->current * torque_const_ * ratio_total_;
  // [解析] 换算链的最后一步：电流 → 电机侧力矩(×转矩常数) → 输出端力矩(×总减速比)。
  // [解析] 例：MG4005 拿到 1A、减速比 10 → 0.06×10 = 0.6 N·m。这是"估算值"，不是实测力矩。
  this->speed = speed_raw * SP_PI / 180.0f / ratio_total_;
  // [解析] 电机侧转速(度/秒) → 先乘 π/180 换成 rad/s → 再除以总减速比换成输出端转速。
  // [解析] 减速箱"减速"的直观体现：输出端转速 = 电机侧 ÷ 减速比，越小越有劲。
  this->multicycle_angle = motor_angle / ratio_total_;
  // [解析] 多圈累计角度换到输出端(除以减速比)。这个量是连续的，适合做位置环的反馈。
  this->angle =
    multi_circle_ ? this->multicycle_angle : limit_angle(motor_angle_single / ratio_total_);
  // [解析] 三目运算符：multi_circle 为 true 就直接给多圈值(连续、无跳变)；
  // [解析]   为 false 时给单圈值，并用 limit_angle() 把 [0,2π) 映射成 (-π, π]。
  // [解析] ★ 单圈值在 ±π 处会跳变，若拿它做位置环，电机经过该点会突然"以为"目标差了一大截。
}

// ---------------------------- 指令帧 ----------------------------
// 所有指令帧都按 data[0] 指令字 + data[1..7] 数据 的格式填充完整8字节
// [解析] 这一节的函数是"打包器"：把成员里缓存的指令翻译成 8 个字节，写进调用者给的 data。
// [解析] ★ 它们都不会发送！发送流程是：write_xxx(&can.tx_data) -> can.send(tx_id)。
// [解析] 下面的 0x9A/0x9C/0x9B/0x88/0x80/0x81 六条指令不带参数，data[1..7] 全部填 0x00，
// [解析]   因为协议规定数据域固定 8 字节，多出的字节必须显式写 0。

void LK_Motor::write_state1(uint8_t * data) const
{
  data[0] = LK_CMD_STATE1;
  // [解析] 0x9A = "请把状态1(温度/电压/电机状态/错误码)报给我"，属于查询帧，发完等反馈。
  data[1] = 0x00;
  data[2] = 0x00;
  data[3] = 0x00;
  data[4] = 0x00;
  data[5] = 0x00;
  data[6] = 0x00;
  data[7] = 0x00;
  // [解析] ↑ data[1..7] 保留字节，协议要求填 0。7 行看似啰嗦，但这样最不容易漏字节。
}

void LK_Motor::write_state2(uint8_t * data) const
{
  data[0] = LK_CMD_STATE2;
  // [解析] 0x9C = "请上报状态2(iq/转速/编码器)"，是获取角度反馈最常用的查询帧。
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
  // [解析] 0x9B = 清除错误标志；error_state 非 0 时先排查原因再用它清标志，别盲目清。
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
  // [解析] 0x88 = 使能(电机运行)。上电后必须先发它，否则后面所有控制帧都不起作用。
  data[1] = 0x00;
  data[2] = 0x00;
  data[3] = 0x00;
  data[4] = 0x00;
  data[5] = 0x00;
  data[6] = 0x00;
  data[7] = 0x00;
}

void LK_Motor::write_turn_on(uint8_t * data) const { write_open(data); }
// [解析] 只是 write_open() 的别名(同一个 0x88)，两个名字都留着迁就不同人的叫法。

void LK_Motor::write_turn_off(uint8_t * data) const
{
  data[0] = LK_CMD_TURN_OFF;
  // [解析] 0x80 = 关闭电机(断电，不输出力矩)。急停/收车时应该发这条。
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
  // [解析] 0x81 = 停止(与"关闭"语义相近但不完全相同：停止侧重中断当前运动)，
  // [解析] 具体差异以厂家手册为准；工程里最常用的是 0x88 使能 / 0x80 关闭这一对。
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
  // [解析] cmd_raw_ 是 cmd_torque() 事先算好并限幅过的 iq 原始值，这里只负责拆成两个字节。
  data[0] = LK_CMD_TORQUE;
  // [解析] 0xA1 = 转矩(电流)闭环：给多少电流就出多少力矩，是最常用的底层控制方式。
  data[1] = 0x00;
  data[2] = 0x00;
  data[3] = 0x00;
  data[4] = static_cast<uint8_t>(cmd_raw_ & 0xFF);
  // [解析] & 0xFF 取低 8 位 = 低字节。因为协议是小端，低字节放在前面的 data[4]。
  data[5] = static_cast<uint8_t>((cmd_raw_ >> 8) & 0xFF);
  // [解析] >> 8 取高 8 位 = 高字节，放 data[5]。注意先移位再 & 0xFF，顺序不能反。
  // [解析] cmd_raw_ 是 int16(有符号)，右移在负数上是"算术右移"，& 0xFF 之后拿到的正是
  // [解析]   补码的低/高字节，拼回去仍然表示同一个负数，所以负力矩也能正确下发。
  data[6] = 0x00;
  data[7] = 0x00;
}

void LK_Motor::write_velocity(uint8_t * data) const
{
  // 手册 P10, 速度闭环控制命令 0xA2:
  //   data[2..3] = iqControl 转矩电流限制(int16, 小端), ±2048 对应 MF ±16.5A / MG ±33A
  //   data[4..7] = speedControl(int32, 小端), 0.01dps/LSB
  // 注意: 这里必须给一个合理的转矩电流限制, 不能留 0, 否则电机可能没有力矩输出
  // [解析] 速度模式下电机内部自己跑速度环，但"最多用多大电流"仍要主机告诉它：
  // [解析]   data[2..3] 就是这个"电流天花板"，这里取型号允许的最大值 max_raw_。
  // [解析] ★ 最容易踩的坑：把这两字节留 0(以为"不限")，实际含义变成"限 0 A"，
  // [解析]   电机看着收指令却不转，很难 debug。
  const auto speed_raw = cmd_velocity_raw_;
  // [解析] auto 推导为 int32_t：速度指令是 32 位定点数(0.01°/s 每 LSB)，范围比 int16 大得多。
  const int16_t iq_limit = max_raw_;
  // [解析] 用型号峰值电流对应的原始值作为速度环的电流上限，既保护电机又不至于太软。

  data[0] = LK_CMD_VELOCITY;
  data[1] = 0x00;
  data[2] = static_cast<uint8_t>(iq_limit & 0xFF);
  data[3] = static_cast<uint8_t>((iq_limit >> 8) & 0xFF);
  // [解析] ↑ 与 write_torque() 相同的"小端拆两字节"写法：低字节在前。
  data[4] = static_cast<uint8_t>(speed_raw & 0xFF);
  data[5] = static_cast<uint8_t>((speed_raw >> 8) & 0xFF);
  data[6] = static_cast<uint8_t>((speed_raw >> 16) & 0xFF);
  data[7] = static_cast<uint8_t>((speed_raw >> 24) & 0xFF);
  // [解析] ↑ 32 位量就拆 4 个字节，顺序是从最低字节到最高字节(小端)：
  // [解析]   data[4]=bit0~7, data[5]=bit8~15, data[6]=bit16~23, data[7]=bit24~31。
  // [解析]   凡是协议里写 int32/uint32 的字段，拆字节都是这四行的套路(角度增量那两处同理)。
}

void LK_Motor::write_position(uint8_t * data) const
{
  // data[1]    = 旋转方向, 0x00 顺时针, 0x01 逆时针
  // data[4..7] = 单圈位置(uint32, 小端), 电机侧 0.01°, 范围 0 ~ 36000
  // cmd_position_ 是输出端"度": 先乘总减速比换成电机侧角度, 再归一化到单圈[0,360)
  // [解析] std::fmod 是"浮点取余"，用来把任意角度折进一圈之内：比如 370 -> 10。
  // [解析] 顺序不能反：先乘减速比再取余，得到的才是"电机侧"的单圈角度；
  // [解析]   若先取余(输出端 [-360,360))再乘减速比，结果会超出 360°(电机侧最多要 3600°)，
  // [解析]   而协议这个字段只允许 0~36000 计数，超出就是非法指令。
  float motor_deg = std::fmod(cmd_position_ * ratio_total_, 360.0f);
  if (motor_deg < 0.0f) motor_deg += 360.0f;
  // [解析] fmod 对负数返回负值(-10 % 360 = -10)，加 360 把它映射到 [0, 360)，
  // [解析]   这样后续转成 uint32 才不会被截断成 0(负数转无符号是灾难性的)。
  // [解析] 副作用提醒：因为做了单圈归一化，cmd_position(370°) 与 cmd_position(10°) 完全等价。

  const auto angle_raw = static_cast<uint32_t>(motor_deg * 100.0f);
  // [解析] 度 × 100 = 0.01° 的计数(标度因数 100 LSB/度)：例如 10° -> 1000。
  // [解析] 范围 0 ~ 36000(即 0°~360°)，远远小于 uint32 上限，所以不会溢出。

  data[0] = LK_CMD_POSITION;
  // [解析] 0xA5 = 单圈位置闭环控制：电机自己跑到"指定的单圈角度"并停在那里。
  data[1] = cmd_direction_;
  // [解析] 方向字节：0x00 顺时针 / 0x01 逆时针，由 cmd_position() 的第二个参数决定。
  data[2] = 0x00;
  data[3] = 0x00;
  data[4] = static_cast<uint8_t>(angle_raw & 0xFF);
  data[5] = static_cast<uint8_t>((angle_raw >> 8) & 0xFF);
  data[6] = static_cast<uint8_t>((angle_raw >> 16) & 0xFF);
  data[7] = static_cast<uint8_t>((angle_raw >> 24) & 0xFF);
  // [解析] ↑ 又是"32 位量小端拆 4 字节"：低字节在前，和 write_velocity() 完全同一套路。
}

void LK_Motor::write_angle_increment(uint8_t * data) const
{
  // data[4..7] = 角度增量(int32, 小端), 电机侧 0.01°
  // cmd_angle_ 是输出端"度", 乘总减速比得到电机侧角度
  // [解析] 增量模式不需要归一化(可以连续转多圈)，但要限幅，避免浮点转 int32 溢出。
  const float raw = cmd_angle_ * ratio_total_ * 100.0f;
  // [解析] 输出端角度 × 总减速比 = 电机侧角度；再 ×100 换成 0.01° 计数。
  const auto angle_raw = static_cast<int32_t>(limit_max(raw, LK_MAX_RAW_I32));
  // [解析] limit_max(raw, ±2e9) 把它夹在 int32 可表示范围内(且小于 INT32_MAX=2147483647)，
  // [解析]   否则超大浮点数转 int32 是未定义行为，可能得到任意值甚至负数。
  // [解析] 注意这里是 int32(可负)：负角度代表反向转，所以不能再加 360 之类的归一化操作。

  data[0] = LK_CMD_ANGLE_INCREMENT;
  // [解析] 0xA7 = 增量位置闭环控制1：相对"当前位置"转过指定的角度。
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
  // [解析] 0xA8 与 0xA7 的唯一区别：多了一个"最大速度"字段，用来限制这次运动有多快。
  const float angle = cmd_angle_ * ratio_total_ * 100.0f;
  const auto angle_raw = static_cast<int32_t>(limit_max(angle, LK_MAX_RAW_I32));
  // [解析] 与 write_angle_increment() 里的算法一模一样：输出端度 -> 电机侧 0.01° 计数。

  float speed_dps = cmd_max_speed_ * ratio_total_;
  // [解析] 输出端 度/s × 总减速比 = 电机侧 度/s(dps)。因为减速箱减速，电机侧一定更快。
  if (speed_dps < 0.0f) speed_dps = 0.0f;
  // [解析] 协议里这个字段是 uint16(无符号)，负数必须先夹到 0，否则会变成 65535 那种巨大速度。
  if (speed_dps > 65535.0f) speed_dps = 65535.0f;
  // [解析] 上限就是 uint16 的最大值 65535(单位 1°/s)，超过它就会被截断成错误的小数值。
  const auto speed_raw = static_cast<uint16_t>(speed_dps);
  // [解析] 注意本字段的标度因数是 1°/s 每 LSB(不像角度那样 ×100)，所以直接强转即可。
  // [解析] 所以填 100 表示"最多 100°/s(电机侧)"；要限速到 0 就等于不限速(取决于固件实现)。

  data[0] = LK_CMD_ANGLE_INCREMENT_SPEED;
  // [解析] 0xA8 = 增量位置闭环控制2(带限速)，做"慢速小角度微调"时很有用。
  data[1] = 0x00;
  data[2] = static_cast<uint8_t>(speed_raw & 0xFF);
  data[3] = static_cast<uint8_t>((speed_raw >> 8) & 0xFF);
  // [解析] ↑ uint16 只拆两个字节(低字节在前)，别照着 32 位那套多拆两位。
  data[4] = static_cast<uint8_t>(angle_raw & 0xFF);
  data[5] = static_cast<uint8_t>((angle_raw >> 8) & 0xFF);
  data[6] = static_cast<uint8_t>((angle_raw >> 16) & 0xFF);
  data[7] = static_cast<uint8_t>((angle_raw >> 24) & 0xFF);
}

void LK_Motor::write(uint8_t * data) const
{
  // [解析] 统一出口：按"最近一次 cmd_xxx() 设的模式"选一个 write_xxx() 去组帧。
  // [解析] 所以调用顺序很重要——先 cmd_xxx()，再 write()；反了发的就是上一种模式的指令。
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
      // [解析] default 兜底走力矩模式：对象刚构造时 cmd_mode_ = LK_CMD_TORQUE 也落在这里，
      // [解析]   意思是"没特意设置模式时，默认按力矩控制发"，这个默认值对调试最安全。
  }
}

// ---------------------------- 指令缓存 ----------------------------
// [解析] 这一组函数只"记录目标值 + 记住模式"，不换算、不组帧、不发送。
// [解析] 换算和打包统一放在 write_xxx() 里做，好处是同一份目标值可以被反复组帧(比如高频重发)。
// [解析] ★ 每次调用都会把 cmd_mode_ 改成自己的模式，所以"最后一次调用"决定 write() 发什么。

void LK_Motor::cmd(float torque) { cmd_torque(torque); }
// [解析] 统一接口别名：和 RM/DM 电机一样都叫 cmd()，方便 JointMotorController 之类的模板复用。

void LK_Motor::cmd_torque(float torque)
{
  // 输出端力矩 -> 电机侧转矩电流 -> iq 原始值(换算系数按系列区分)
  // [解析] 换算链(和 read_state2 里的力矩计算正好相反)：
  // [解析]   输出端力矩 ÷ 总减速比 = 电机侧力矩；÷ 转矩常数 = 相电流(A)；÷ iq 系数 = iq 原始值。
  const float current = torque / ratio_total_ / torque_const_;
  // [解析] 注意是"除"减速比：输出端只要一点点力矩，电机侧就要出好几倍(减速箱放大力矩)。
  float raw = current / iq_scale_;
  // [解析] 除以 A/LSB 系数得到整数计数；这里用 float 保存，限幅完了再转 int16。
  // [解析] 例：MG4005 想输出 1.5 N·m → 电机侧 0.15 N·m → 2.5A → 2.5/0.016113 ≈ 155 LSB
  // [解析]   (没超过它的上限 248，所以能原样下发；再多就会被下面两行削掉)。

  if (raw > max_raw_) raw = max_raw_;
  if (raw < -max_raw_) raw = -max_raw_;
  // [解析] 正负对称限幅到型号允许的最大 iq 原始值，保护电机(超了会过流保护/发热)。
  // [解析] 限幅是"削顶"而不是"按比例缩放"，所以指令给太大时电机会一直顶在最大力矩上。

  cmd_raw_ = static_cast<int16_t>(raw);
  // [解析] 先限幅再强转，所以不会溢出；截断小数部分只带来 1LSB(约 8~16mA)的误差，可以忽略。
  cmd_mode_ = LK_CMD_TORQUE;
  // [解析] 顺手把模式切回力矩控制，之后 write() 就会走 write_torque()。
}

void LK_Motor::cmd_velocity(float speed)
{
  // 输出端 rad/s -> 电机侧 0.01°/s
  // [解析] 单位换算一次看全：rad/s ×(180/π)→ °/s，再 × 总减速比 → 电机侧 °/s，
  // [解析]   最后 ×100 换成协议要求的 0.01°/s 计数。顺序无所谓，都是乘法。
  const float raw = speed * ratio_total_ * 180.0f / SP_PI * 100.0f;
  cmd_velocity_raw_ = static_cast<int32_t>(limit_max(raw, LK_MAX_RAW_I32));
  // [解析] 先限幅再转 int32，防止超大指令导致浮点转整型溢出(未定义行为)。
  cmd_mode_ = LK_CMD_VELOCITY;
  // [解析] 提醒：组帧时还会附上电流上限(见 write_velocity)，所以这里不必再管电流。
}

void LK_Motor::cmd_position(float position, uint8_t direction)
{
  // position: 输出端 度(单圈), 组帧时会乘减速比并归一化到电机侧的单圈 0~360°
  cmd_position_ = position;
  // [解析] 原样存下(输出端度)，不做归一化——归一化留到 write_position() 做，保持"存/算分离"。
  cmd_direction_ = direction;
  // [解析] 只对 0xA5 位置模式有意义：0x00 顺时针、0x01 逆时针。传别的值等于让电机自己解释。
  cmd_mode_ = LK_CMD_POSITION;
}

void LK_Motor::cmd_angle_increment(float angle)
{
  // angle: 输出端 度
  cmd_angle_ = angle;
  // [解析] 相对增量(可正可负、可超过 360°)，组帧时乘减速比后按 int32 下发，不做单圈归一化。
  cmd_mode_ = LK_CMD_ANGLE_INCREMENT;
}

void LK_Motor::cmd_angle_speed(float angle, float max_speed)
{
  // angle: 输出端 度; max_speed: 输出端 度/s
  cmd_angle_ = angle;
  cmd_max_speed_ = max_speed;
  // [解析] 两个值一次都存下来，组帧时分别填到 data[4..7] 和 data[2..3]。
  cmd_mode_ = LK_CMD_ANGLE_INCREMENT_SPEED;
  // [解析] 想用"带限速"的增量模式，必须调这个函数；调 cmd_angle_increment() 是不会限速的。
}

}  // namespace sp
