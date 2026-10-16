// [解析] ==========================================================================
// [解析] 【文件】sp_middleware/motor/dm_motor/dm_motor.cpp
// [解析] 【作用】dm_motor.hpp 里那些函数的具体实现：
// [解析]          read() 把 CAN 的 8 字节翻译成物理量，write 系列把物理量打包成 8 字节。
// [解析] 【看代码的重点】三个地方最容易看不懂，下面都逐行标注了：
// [解析]   1) read() 里 "data[3]<<4 | data[4]>>4" 这种跨字节拼位；
// [解析]   2) uint_to_float / float_to_uint 的定点数换算(量程怎么变成整数)；
// [解析]   3) write() 里 Kp=0/Kd=0 时，为什么力矩被塞进了 data[6]、data[7]。
// [解析] ==========================================================================

#include "dm_motor.hpp"

#include <cstring>
// [解析] <cstring> 是为了 write_velocity() 里的 std::memcpy —— 把 float 的 4 个字节原样拷进数组。

#include "tools/math_tools/math_tools.hpp"
// [解析] 这里提供两个关键工具：
// [解析]   uint_to_float(整数, min, max, 位数)  整数 -> 物理量(读反馈时用)
// [解析]   float_to_uint(物理量, min, max, 位数) 物理量 -> 整数(发指令时用)
// [解析]   limit_max(x, max) 把 x 限制到 [-max, +max]
// [解析] 换算公式(见 math_tools.cpp)：
// [解析]   整数 -> 物理量: norm = 整数 / (2^位数 - 1);  物理量 = norm*(max-min) + min
// [解析]   物理量 -> 整数: norm = (物理量-min)/(max-min); 整数 = norm*(2^位数 - 1)
// [解析] 也就是说：整数 0 对应最小物理量 min，整数满量程对应 max。

namespace sp
{
// [解析] 构造函数：冒号后面这一串叫"初始化列表"，会在函数体执行之前按声明顺序完成赋值。
// [解析] 由于 rx_id / tx_id / pmax_ 等都是 const，它们只能在初始化列表里赋值，不能在函数体里写。
// [解析] 函数体是空的：构造函数不操作硬件，使能电机要靠 write_enable() + CAN 发送。
DM_Motor::DM_Motor(
  uint16_t can_id, uint16_t master_id, float pmax, float vmax, float tmax, bool multi_circle)
: rx_id(master_id),
  tx_id(can_id),
  tx_id_vel(0x200 + can_id),
  // [解析] ↑ 速度模式的发送 ID = can_id + 0x200，达妙协议规定，不是随便写的。
  pmax_(pmax),
  vmax_(vmax),
  tmax_(tmax),
  has_read_(false),
  // [解析] ↑ 刚创建时当然还没收到过反馈，所以是 false。
  circle_(0),
  // [解析] ↑ 多圈计数从 0 开始。
  multi_circle_(multi_circle)
{
}
// [解析] 注意：cmd_torque_ / cmd_velocity_ 没有出现在初始化列表里，定义处也没写默认值，
// [解析]   所以它们在第一次调用 cmd()/cmd_velocity() 之前是"未初始化的随机值"。
// [解析]   如果上电后不先调一次 cmd(0) 就直接 write()，发出去的力矩是随机的。
// [解析]   这是原代码的写法，了解即可，不要在这里改。

// [解析] 是否收到过反馈帧 —— 判断电机"在不在线"的最基本情况。
bool DM_Motor::is_open() const { return has_read_; }

// [解析] 通信超时判断：距上次收到反馈不到 100ms 才算活着。
// [解析] 100ms 的来历：电机正常反馈频率是 1kHz(1ms 一帧)，100ms 相当于"连续丢了 100 帧"，
// [解析]   到这个程度基本可以确定是线掉了或电机断电了，而不是偶发的丢包。
// [解析] 时间戳用无符号数相减 (now_ms - last_read_ms_) 而不是比较大小，
// [解析]   这样即使 HAL_GetTick() 溢出回绕，算出来的差仍然是正确的。
bool DM_Motor::is_alive(uint32_t now_ms) const
{
  return is_open() && (now_ms - last_read_ms_ < 100);
}

// [解析] read()：把一帧 8 字节反馈解析成物理量。调用时机：CAN 中断里 rx_id 匹配之后。
// [解析] 参数 data：指向收到的 8 字节原始数据(通常是 can.rx_data)，长度必须是 8。
// [解析] 参数 stamp_ms：当前毫秒时间戳(通常是 HAL_GetTick())，用于 is_alive() 判断掉线。
// [解析] 本函数用到的 8 个字节划分如下(均为"高位在前"的大端式拼法)：
// [解析]   data[0] 高 4 位 = 错误码，低 4 位本实现没有使用
// [解析]   data[1]、data[2]        = 位置，16 位整数
// [解析]   data[3] + data[4]高 4 位 = 速度，12 位整数
// [解析]   data[4]低 4 位 + data[5] = 力矩，12 位整数
// [解析]   data[6] = MOS 管温度，data[7] = 转子温度(单位都是 ℃，整数)
// [解析] 注意：不同批次的达妙固件/说明书对反馈帧的字节划分有过差异(有的资料把位置写成
// [解析]   跨 data[0]低4位~data[2]高4位)。若实测角度/速度明显不对，优先核对说明书。
//multi_circle是true就用多圈
void DM_Motor::read(uint8_t * data, uint32_t stamp_ms)
{
  // [解析] 先记录时间戳：只要进了这个函数就算"收到过帧"，is_alive() 靠它续命。
  last_read_ms_ = stamp_ms;

  // [解析] >> 4 是取出高 4 位：data[0] 的高半字节就是错误码，低半字节与错误无关。
  this->error = data[0] >> 4;
  // [解析] 速度是 12 位：先由 data[3] 左移 4 位当高 8 位，再或上 data[4] 的高 4 位当低 4 位。
  // [解析] 注意 data[4] 这一个字节被"劈成两半"：高 4 位归速度，低 4 位归力矩。
  this->speed = uint_to_float((data[3] << 4) | (data[4] >> 4), -vmax_, vmax_, 12);
  // [解析] 力矩同样 12 位：data[4] 的低 4 位当高 4 位，data[5] 当低 8 位。
  // [解析] 顺序必须是先 <<8 再 | ，否则高低字节就装反了。
  this->torque = uint_to_float(((data[4] & 0xF) << 8) | data[5], -tmax_, tmax_, 12);

  // [解析] 位置是 16 位，直接由 data[1]/data[2] 拼成，量程 ±pmax_。
  // [解析] 这里先存进 angle_raw_(原始单圈角)，等下面处理完多圈再算 angle。
  this->angle_raw_ = uint_to_float((data[1] << 8) | data[2], -pmax_, pmax_, 16);

  // [解析] 第一帧特殊处理：此时上一帧的角度还是 0，直接做差会凭空多算/少算一圈，
  // [解析] 所以第一次只把 last_angle_raw_ 对齐到当前值，不累加圈数。
  if (!has_read_) {
    has_read_ = true;
    last_angle_raw_ = angle_raw_;
  }
  // [解析] ↓ 多圈的核心：单圈角度在 +pmax 和 -pmax 处会"跳变"。
  // [解析]   正向转过边界时：上一帧≈+pmax，这一帧≈-pmax，差值≈-2*pmax，一定是很大的负数
  // [解析]     -> 判定为"又往前转了一圈" -> circle_++ (圈数加一)
  // [解析]   反向转过边界时差值≈+2*pmax -> circle_-- 。
  // [解析]   阈值取 pmax_(而不是 2*pmax_)是留余量：正常一次转动带来的角度变化远小于 pmax。
  if (angle_raw_ - last_angle_raw_ > pmax_)
    circle_--;
  else if (angle_raw_ - last_angle_raw_ < -pmax_)
    circle_++;

  // [解析] 记住本帧角度，下一帧比拼要用。
  last_angle_raw_ = angle_raw_;
  // [解析] 单圈模式直接用 angle_raw_；多圈模式则把圈数折算成角度累加上去：
  // [解析]   一圈 = 2*pmax_(因为范围是 -pmax ~ +pmax，跨度正好是一整圈)。
  this->angle = multi_circle_ ? angle_raw_ + circle_ * 2 * pmax_ : angle_raw_;

  // [解析] 温度是整数直接用，不需要任何换算(协议里单位已经是 ℃)。
  this->T_MOS = data[6];
  this->T_Rotor = data[7];
}

// [解析] write()：MIT 模式的"精简版"打包，只发力矩。
// [解析] 思路是让位置环/速度环全部失效：Kp = 0、Kd = 0、p_des = 0、v_des = 0，
// [解析]   此时电机的 MIT 公式 t = Kp*(p_des-p) + Kd*(v_des-v) + t_ff 只剩 t_ff，
// [解析]   于是"力矩控制"就等于直接给 t_ff。这就是纯力矩模式的做法。
// [解析] 参数 data：输出缓冲区，一般直接传 can.tx_data(长度必须 ≥ 8)。
void DM_Motor::write(uint8_t * data) const
{
  // [解析] 先把力矩物理量转成 12 位整数：-tmax_ -> 0，0 -> 约 2048，+tmax_ -> 4095。
  auto torque_uint = float_to_uint(cmd_torque_, -tmax_, tmax_, 12);
  // [解析] 下面 5 行就是在按协议摆位。原始注释里的 Kp/Kd 是"该位置在协议中占的字段名"，
  // [解析] 但因为 Kp、Kd 都取 0，这些字节实际全被填 0。
  // [解析] 注意：原注释把 data[6]、data[7] 标成 Kd[3:0]，其实这两个字节装的是力矩
  // [解析]   t_ff[11:8] 和 t_ff[7:0](t_ff 就是本函数要发的力矩)，看代码时不要被注释带偏。
  data[3] = 0x00;              // Kp[11:8]
  data[4] = 0x00;              // Kp[7:0]
  data[5] = 0x00;              // Kd[11:4]
  data[6] = torque_uint >> 8;  // Kd[3:0]
  // [解析] ↑ 取力矩的高 4 位。因为 12 位整数右移 8 位后最大只有 0xF，
  // [解析]   自然落在低 4 位(t_ff[11:8])，高 4 位(Kd[3:0])保持 0，所以不用再 & 0xF。
  data[7] = torque_uint;
  // [解析] ↑ 力矩的低 8 位。赋值给 uint8_t 会自动截断，等价于 torque_uint & 0xFF。
  // [解析] 注意：data[0]、data[1]、data[2] 这里没有被赋值(它们对应 p_des 和 v_des 高位)。
  // [解析]   因为是同一个 can.tx_data 反复复用，里面可能残留上一次 write_mit() 的数据；
  // [解析]   不过 Kp、Kd 都为 0，残留的目标位置/速度不会产生任何作用，所以是安全的。
}

// [解析] write_velocity()：速度模式的打包方式完全不同 —— 它不用定点数，直接把 float 原样发。
void DM_Motor::write_velocity(uint8_t * data) const
{
  // 浮点数占用 4 个字节，直接内存拷贝即可满足“低位在前”的小端要求
  // [解析] STM32F407 的 Cortex-M4 是"小端"CPU：float 在内存里就是低字节在前，
  // [解析] 而达妙速度模式协议要求的正是"低字节在前"，两者一致，所以 memcpy 即可，
  // [解析] 不需要手动拆字节。换成大端 CPU 这么写就是错的。
  // [解析] 注意：这里假设 data 至少有 4 字节可写；调用方必须传长度 ≥8 的数组。
  std::memcpy(data, &cmd_velocity_, sizeof(float));

  // 保险起见，将后4个字节清零（CAN帧通常发8字节，尽管速度模式只用前4字节）
  // [解析] 清掉后 4 字节只是"好看"：一帧 CAN 固定发 8 字节，不清零就会把旧数据一起发出去。
  data[4] = 0x00;
  data[5] = 0x00;
  data[6] = 0x00;
  data[7] = 0x00;
}

// [解析] ↓ 下面三个函数打包的是"特殊命令帧"，它们不控制运动，只控制电机的开关状态。
// [解析] 帧格式：前 7 个字节固定为 0xFF，最后 1 个字节是命令码。
// [解析] 用 0xFF 这种固定图案是为了让电机一眼认出"这不是运动控制帧，是命令帧"。
// [解析] 命令码：0xFC = 使能，0xFD = 失能，0xFB = 清除错误(均见达妙说明书)。
// [解析] 这三个数值属于协议常量，一个字节都不能改，改了就是另一条命令。
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
  // [解析] ↑ 0xFC = 使能。上电后必须先发这一帧，电机才进入"能输出力矩"的状态。
}

// [解析] 失能帧：与使能帧只差最后 1 个字节(0xFD)。发完电机立刻卸力、自由转动。
// [解析] 安全提示：调试时若发现电机失控，第一时间发这一帧，比断电更快。
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
  // [解析] ↑ 0xFD = 失能。
}

// [解析] 清错帧：error 里出现 8/9/A... 等故障后，先排除物理原因，再发这一帧清标志。
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
  // [解析] ↑ 0xFB = 清除错误。
}

// [解析] cmd()：只是"记下"力矩目标，并不立刻发送；真正发送要再调 write() + can.send()。
// [解析] limit_max(x, max) = 把 x 限制到 [-max, +max]，所以超出量程的值会被静默截断：
// [解析]   传 100 N·m 而 tmax=7 时，实际发出的是 7 N·m，不会报错也不会警告。
void DM_Motor::cmd(float torque) { cmd_torque_ = limit_max(torque, tmax_); }

// [解析] cmd_velocity()：记下速度目标，单位 rad/s，同样被静默限制到 ±vmax_。
void DM_Motor::cmd_velocity(float velocity) { cmd_velocity_ = limit_max(velocity, vmax_); }

// [解析] cmd_mit()：一次性设置 MIT 的五个参数，并顺便做"上下限保护"。
// [解析] 这里只做缓存和限幅，不做转换；转换在 write_mit() 里统一完成。
// [解析] 为什么 kp 上限是 500、kd 上限是 5？
// [解析]   因为协议里这两个字段用 12 位整数传输，说明书把它们的物理量程定为 0~500 和 0~5，
// [解析]   超量程的部分根本没法表示，所以在这里先砍掉。
void DM_Motor::cmd_mit(float p_des, float v_des, float kp, float kd, float t_ff)
{
  cmd_p_des_ = limit_max(p_des, pmax_);
  cmd_v_des_ = limit_max(v_des, vmax_);

  // [解析] limit_max(kp, 500) 已经保证 kp ≤ 500，下面再补一句把负数拉回 0，
  // [解析] 因为协议只允许 kp ∈ [0, 500]，负数没有意义(负 kp 会让系统发散)。
  cmd_kp_ = limit_max(kp, 500.0f);
  if (cmd_kp_ < 0.0f) cmd_kp_ = 0.0f;
  // [解析] kd 同理，量程 0~5。kd 太小会抖，太大会"发软"，实际值要靠试出来的。
  cmd_kd_ = limit_max(kd, 5.0f);
  if (cmd_kd_ < 0.0f) cmd_kd_ = 0.0f;

  cmd_t_ff_ = limit_max(t_ff, tmax_);
  // [解析] ↑ 前馈力矩，用来抵消重力之类的已知恒定负载，限幅到 ±tmax。
}

// [解析] 速度模式的便捷封装：等价于"位置环关掉、只留速度环"的 MIT 控制。
// [解析] p_des=0、kp=0 是关键 —— 位置环一旦有增益，电机就会一边追速度一边拉回原点。
void DM_Motor::cmd_mit_velocity(float velocity, float kd)
{
  cmd_mit(0.0f, velocity, 0.0f, kd, 0.0f);
}

// [解析] write_mit()：MIT 模式的完整打包 —— 本文件里位运算最密集的地方，先看整体结构：
// [解析] 总共 8 个字节要装下 5 个参数，而它们加起来的位宽是 16+12+12+12+12 = 64 位，正好 8 字节，
// [解析] 所以必须"一个字节劈成两半用"，这就是 data[3] 和 data[6] 里同时出现两个参数的原因。
// [解析] 字节分配表(从上到下即 data[0] → data[7])：
// [解析]   data[0]                 = p_des[15:8]      目标位置高 8 位
// [解析]   data[1]                 = p_des[7:0]       目标位置低 8 位
// [解析]   data[2]                 = v_des[11:4]      目标速度高 8 位
// [解析]   data[3] 高4位|低4位     = v_des[3:0] | Kp[11:8]
// [解析]   data[4]                 = Kp[7:0]
// [解析]   data[5]                 = Kd[11:4]
// [解析]   data[6] 高4位|低4位     = Kd[3:0] | t_ff[11:8]
// [解析]   data[7]                 = t_ff[7:0]
// [解析] 记忆技巧：位置最长(16 位)单独占前两个字节，其余四个 12 位参数两两"共享"一个字节。
void DM_Motor::write_mit(uint8_t * data) const
{
  // [解析] 五次转换，把物理量变成整数。注意每个参数的量程上下限：
  // [解析]   位置：[-pmax_, +pmax_]，16 位(所以精度最高)
  // [解析]   速度：[-vmax_, +vmax_]，12 位
  // [解析]   Kp  ：[0, 500]，从 0 开始(没有负增益)，12 位
  // [解析]   Kd  ：[0, 5]，12 位
  // [解析]   力矩：[-tmax_, +tmax_]，12 位
  uint16_t p_int = float_to_uint(cmd_p_des_, -pmax_, pmax_, 16);
  uint16_t v_int = float_to_uint(cmd_v_des_, -vmax_, vmax_, 12);
  uint16_t kp_int = float_to_uint(cmd_kp_, 0.0f, 500.0f, 12);
  uint16_t kd_int = float_to_uint(cmd_kd_, 0.0f, 5.0f, 12);
  uint16_t t_int = float_to_uint(cmd_t_ff_, -tmax_, tmax_, 12);

  // [解析] >>8 取高 8 位；&0xFF 是为了"只保留低 8 位"，防止高位残留(虽然 >>8 后高位本已为 0)。
  data[0] = (p_int >> 8) & 0xFF;                           // p_des[15:8]
  data[1] = p_int & 0xFF;                                  // p_des[7:0]
  // [解析] ↑ 低位直接赋给 uint8_t，会自动截断成低 8 位。
  data[2] = (v_int >> 4) & 0xFF;                           // v_des[11:4]
  // [解析] ↑ v_des 是 12 位，只能放进 8 位的槽，所以右移 4 位丢掉最低 4 位；
  // [解析]   丢掉的这 4 位马上在下一行"回收利用"，放进 data[3] 的高 4 位。
  data[3] = ((v_int & 0xF) << 4) | ((kp_int >> 8) & 0xF);  // v_des[3:0] | Kp[11:8]
  // [解析] ↑ 这一个字节塞了两个参数：高 4 位是 v_des 的最低 4 位，低 4 位是 Kp 的最高 4 位。
  // [解析]   规则是"先 &0xF 切出需要的 4 位，再用 << 或 >> 挪到目标位置，最后用 | 合并"。
  data[4] = kp_int & 0xFF;                                 // Kp[7:0]
  data[5] = (kd_int >> 4) & 0xFF;                          // Kd[11:4]
  // [解析] ↑ 同样：Kd 的高 8 位给 data[5]，最低 4 位留给 data[6]。
  data[6] = ((kd_int & 0xF) << 4) | ((t_int >> 8) & 0xF);  // Kd[3:0] | t_ff[11:8]
  // [解析] ↑ Kd 的低 4 位放高半字节，力矩的高 4 位放低半字节。
  data[7] = t_int & 0xFF;                                  // t_ff[7:0]
  // [解析] 反复出现 0xF 是因为"半个字节(nibble)= 4 位 = 最大值 15 = 0xF"，
  // [解析] 用它做掩码就能精确切出 4 位；0xFF 则切出 8 位。这就是拼包时的通用套路。
}

}  // namespace sp