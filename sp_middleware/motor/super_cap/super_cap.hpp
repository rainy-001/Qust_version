// [解析] ==========================================================================
// [解析] 【文件】sp_middleware/motor/super_cap/super_cap.hpp
// [解析] 【作用】把"超级电容/缓冲电容管理模块"封装成一个类。
// [解析]         这个模块是 RoboMaster 步兵/英雄上用来"短时间爆发大功率"的储能装置：
// [解析]         平时用底盘多余功率给电容组充电，需要爆发时再把能量放出来补功率，
// [解析]         这样瞬时功率就不会超过裁判系统的功率上限而被罚。
// [解析] 【被谁用】applications/vision_task.cpp：
// [解析]           sp::SuperCap supercap;  创建对象（用默认参数：自动模式、4.4F）
// [解析]           然后在其 CAN 接收回调里 supercap.read(...) 取功率/电压/能量，
// [解析]           在控制逻辑里 supercap.write(...) 填好要发的 8 字节再 send 到 0x300。
// [解析] 【阅读顺序】先看本文件了解"有哪些量、单位是什么"，再看 super_cap.cpp 看字节怎么拼。
// [解析] 【关键点】最容易踩的坑有三个：
// [解析]   1) 这个模块的字节序和"大疆电机"是【相反】的：它是低字节在前（小端）；
// [解析]   2) 反馈量的单位是靠"除以 10"和"除以 100"得到的，很容易少除或多除一个 0；
// [解析]   3) 写入时 data[3]/data[4] 是保留字节，不能拿来做别的用途。
// [解析] ==========================================================================

#ifndef SP__SUPERCAP_HPP
#define SP__SUPERCAP_HPP

#include <cstdint>

namespace sp
{
// [解析] 工作模式枚举。写入控制帧的第 0 个字节，告诉模块"充放电怎么走"。
// [解析] 这四个值就是模块协议规定的枚举值，注意 AUTOMODE 显式写了 = 0x00，
// [解析] 后面三个没写，会顺着自动排成 0x01、0x02、0x03 —— 这一点很容易看漏。
// [解析] enum class 是强类型枚举，必须写 SuperCapMode::AUTOMODE 才能用。
enum class SuperCapMode
{
  AUTOMODE = 0x00,     // 自动模式（默认）
  // [解析] ↑ 0x00：模块自己根据电压/功率决定充还是放，最省心，比赛常用。
  DISOUTPUT,           // 只充不放
  // [解析] ↑ 0x01：禁止放电，只允许充电（比如赛前先把电容充满）。
  DISCHARGE,           // 只放不充
  // [解析] ↑ 0x02：禁止充电，只允许放电（比如裁判系统限制输入功率时）。
  DISCHARGE_DISOUTPUT  // 不充不放
  // [解析] ↑ 0x03：充放电都禁止，等于把电容"闲置"，一般只在调试或故障保护时用。
};

// [解析] SuperCap：一个对象 = 车上那一台超级电容管理模块（挂在 CAN 总线上）。
// [解析] 用法三步（和 RM_Motor 完全对称，看得懂电机就会用超电）：
// [解析]   1) sp::SuperCap supercap;                       // 创建
// [解析]   2) CAN 中断里：if (can1.rx_id == supercap.super_cap_rx_id) supercap.read(...);
// [解析]      读完后 power_in / power_out / voltage / cap_energy 就是物理量。
// [解析]   3) 任务里：supercap.write(can1.tx_data, 功率上限, 缓冲能量, 底盘输出);
// [解析]              can1.send(supercap.tx_id);
class SuperCap
{
public:
  // [解析] ↓ 三个 ID 也是"协议规定死的"，所以直接在这里赋初值，构造函数不用管。
  // [解析]   0x300 是我们发控制帧用的 ID，0x301/0x302 是两种模块各自的反馈 ID。
  const uint16_t tx_id = 0x300;             // 超级电容和缓冲电容控制帧ID
  // [解析] ↑ 发送用：can1.send(supercap.tx_id)。
  const uint16_t super_cap_rx_id = 0x301;   // 超级电容反馈帧ID
  // [解析] ↑ 超级电容模块的反馈 ID，用来判断"这帧是不是超电发来的"。
  const uint16_t buffer_cap_rx_id = 0x302;  // 缓冲电容反馈帧ID
  // [解析] ↑ 缓冲电容模块的反馈 ID。本项目只用 0x301（超级电容），
  // [解析]   0x302 是给另一种/另一路缓冲电容预留的，收到也要用同一个 read() 解析。

  float power_in;   // 电管输出
  // [解析] ↑ 只读、由 read() 更新。这是"功率控制模块给底盘/电管的输出功率"，
  // [解析]   单位 W（瓦），已在 read() 里除以 10 还原。正负号含义见 super_cap.cpp 的说明。
  float power_out;  // 电容组充电功率
  // [解析] ↑ 只读。电网给电容组充电的功率，单位 W（同样除以 10 还原）。
  float voltage;    // 电容电压（放到3V，然后3v->6V后可再次放电）
  // [解析] ↑ 只读。电容组当前电压，单位 V，read() 里除以 100 还原。
  // [解析]   原文意思是：电压放到约 3V 时就不要再放了，等充回 6V 以上才能再次放电，
  // [解析]   这是模块自身的保护逻辑；你只需要读 voltage 来做上层的能量判断。

  uint8_t temperature;  // 温度(现阶段恒定为25)
  // [解析] ↑ 只读，摄氏度。原注释说当前固件恒定为 25，所以不要指望它真的能反映温度。
  uint8_t status;       // 状态标志位（现阶段保留为0x10）
  // [解析] ↑ 只读。原注释说现阶段恒为 0x10（= 16），基本可以当"没有额外信息"看，
  // [解析]   除非你拿到新版模块的协议文档，否则别拿它做判断。

  float cap_energy;
  // [解析] ↑ 只读。电容组当前储能，单位 J（焦耳）。注意它不是从 CAN 帧里读来的，
  // [解析]   而是 read() 里用 E = 0.5 * C * U^2 现算出来的（C = capacitance_，U = voltage）。

  // [解析] 构造函数。
  // [解析]   mode_       —— 工作模式，默认 AUTOMODE（模块自己决定充放电），一般不用改。
  // [解析]   capacitance —— 电容容量，单位法拉(F)，默认 4.4f。
  // [解析]     ★ 这个 4.4 是"本车所用超电模块的标称容量"，不同批次/不同厂家的模块不一样，
  // [解析]       它直接决定 cap_energy 算得准不准。换模块时务必改成实际容量。
  // [解析] 副作用：只保存参数，不发任何 CAN 帧，也不读硬件。
  SuperCap(SuperCapMode mode_ = SuperCapMode::AUTOMODE, float capacitance = 4.4f);

  // [解析] is_alive(now_ms)：判断超电模块此刻是否在线。
  // [解析]   now_ms —— 当前系统毫秒数（如 osKernelSysTick()）。
  // [解析]   返回 true = 距离最后一次 read() 不到 20ms。
  // [解析]   为什么是 20ms 而电机是 100ms：超电是"安全相关"的功率器件，
  // [解析]   掉线后必须尽快发现并降功率，所以判据更严格（原协议里反馈周期约 10ms 量级）。
  bool is_alive(uint32_t now_ms) const;

  // [解析] read()：解析一帧超电反馈（8 字节），更新上面那些只读属性。
  // [解析]   data     —— 指向收到的 8 字节原始数据（通常是 can1.rx_data）。
  // [解析]   stamp_ms —— 本次收到数据的时刻（毫秒），存下来给 is_alive() 用。
  // [解析] ★ 本函数里所有 16 位量都是"低字节在前"（小端），和大疆电机相反，详见 cpp。
  void read(uint8_t * data, uint32_t stamp_ms);
  // [解析] write()：把要发的 8 字节填进 data（通常是 can1.tx_data，填完再 can1.send(tx_id)）。
  // [解析]   power_limit        —— 底盘功率上限，单位 W，uint16 可表示 0~65535。
  // [解析]                           含义是"允许底盘消耗的最大功率"，模块会据此控制输出。
  // [解析]   buffer_energy      —— 缓冲能量，单位 J（焦耳），表示允许多少能量先由电容垫付；
  // [解析]                           比赛里常用"剩余缓冲能量"来动态限制底盘行为。
  // [解析]   chassis_output  —— 底盘输出/使能标志（就是协议里的"底盘输出"字节），
  // [解析]                           0 一般表示切断输出。注意 cpp 里这个参数改名叫 pm02_chassis_output。
  // [解析] 副作用：只改 data 指向的缓冲区，不发帧、不改对象内部状态（所以它不是 const，
  // [解析]   但只是没加 const 而已，实际没有修改成员）。
  void write(uint8_t * data, uint16_t power_limit, uint16_t buffer_energy, uint8_t chassis_output);

private:
  // [解析] ↓ 两个私有成员在构造时定好，之后不再变。
  SuperCapMode mode_;
  // [解析] ↑ 记住工作模式。write() 时把它写进 data[0]，
  // [解析]   所以"想换模式"得重建对象（或者改这里的封装），不能在运行中改。
  float capacitance_;  // 电容容量
  // [解析] ↑ 电容容量 F，只用于算 cap_energy，不参与任何通信。

  uint32_t last_read_ms_;  // 上一次读取数据的时间戳
  // [解析] ↑ is_alive() 用它做超时判断。注意构造函数里【没有】初始化它，
  // [解析]   而且 is_alive() 里也没有像 RM_Motor 那样先用 is_open() 保护，
  // [解析]   所以上电后还没收到第一帧就调用 is_alive() 时，用的是未初始化的值。
  // [解析]   （见 cpp 里的相关注释。）
};

}  // namespace sp

#endif  // IO_SUPERCAP_HPP