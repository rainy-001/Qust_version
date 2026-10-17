// [解析] 超级电容（缓冲电容）管理模块的类封装，负责短时爆发功率、避免超过裁判系统的功率上限。
// [解析] 由 vision_task.cpp 里的 sp::SuperCap 对象使用：中断里 read() 取功率/电压/能量，任务里 write() 打包再发送。
// [解析] 三个坑：反馈是小端（低字节在前，和电机相反）、功率要 /10 电压要 /100、data[3]/data[4] 是保留字节。

#ifndef SP__SUPERCAP_HPP
#define SP__SUPERCAP_HPP

#include <cstdint>

namespace sp
{
enum class SuperCapMode
{
  AUTOMODE = 0x00,     // 自动模式（默认）
  DISOUTPUT,           // 只充不放
  DISCHARGE,           // 只放不充
  DISCHARGE_DISOUTPUT  // 不充不放
};

// [解析] 一个对象 = 车上那一台超电模块；三个 ID 由协议定死，收 0x301/0x302、发 0x300。
class SuperCap
{
public:
  const uint16_t tx_id = 0x300;             // 超级电容和缓冲电容控制帧ID
  const uint16_t super_cap_rx_id = 0x301;   // 超级电容反馈帧ID
  const uint16_t buffer_cap_rx_id = 0x302;  // 缓冲电容反馈帧ID

  float power_in;   // 电管输出
  float power_out;  // 电容组充电功率
  float voltage;    // 电容电压（放到3V，然后3v->6V后可再次放电）

  uint8_t temperature;  // 温度(现阶段恒定为25)
  uint8_t status;       // 状态标志位（现阶段保留为0x10）

  float cap_energy;

  // [解析] capacitance 默认 4.4f 是本车模块的标称容量，换模块必须改，否则 cap_energy 算不准。
  SuperCap(SuperCapMode mode_ = SuperCapMode::AUTOMODE, float capacitance = 4.4f);

  // [解析] 距最后一次 read() 不到 20ms 算在线；比电机的 100ms 严格，因为超电掉线要马上限功率。
  bool is_alive(uint32_t now_ms) const;

  // [解析] 反馈里所有 16 位量都是小端（低字节在前），和大疆电机相反；拆字节前先翻协议表。
  void read(uint8_t * data, uint32_t stamp_ms);
  // [解析] 填 8 字节控制帧：功率上限与缓冲能量都按大端写入，填完再 can1.send(tx_id)。
  void write(uint8_t * data, uint16_t power_limit, uint16_t buffer_energy, uint8_t chassis_output);

private:
  SuperCapMode mode_;
  float capacitance_;  // 电容容量

  uint32_t last_read_ms_;  // 上一次读取数据的时间戳
};

}  // namespace sp

#endif  // IO_SUPERCAP_HPP