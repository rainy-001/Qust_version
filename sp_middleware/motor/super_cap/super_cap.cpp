// [解析] SuperCap 的实现：把 8 字节反馈解析成功率/电压/能量，把功率上限等参数打包成 8 字节控制帧。
// [解析] 收的是小端（低字节在前），发的却是大端（data[1] 是高字节）——收发两帧由不同文档规定，别按对称想当然。

#include "super_cap.hpp"

namespace sp
{
// [解析] 构造函数只存参数；这里没有初始化 last_read_ms_，所以收到第一帧之前调 is_alive() 结果不可靠。
SuperCap::SuperCap(SuperCapMode mode, float capacitance) : mode_(mode), capacitance_(capacitance) {}

bool SuperCap::is_alive(uint32_t now_ms) const { return (now_ms - last_read_ms_ < 20); }

// [解析] 小端拼 16 位：data[0] 是低字节，data[1] 才左移 8 位；拼错不会报错，只会让数值莫名其妙。
// [解析] 必须转 int16_t 才能表示负功率，且 /10.0f 这种写法带小数点是浮点除法，写 /10 会被截断成 0。
void SuperCap::read(uint8_t * data, uint32_t stamp_ms)
{
  // 数据解析
  this->power_in = ((int16_t)(((data)[1] << 8) | (data)[0])) / 10.0f;
  this->power_out = ((int16_t)(((data)[3] << 8) | (data)[2])) / 10.0f;
  this->voltage = ((int16_t)(((data)[5] << 8) | (data)[4])) / 100.0f;
  this->temperature = (data)[6];
  this->status = (data)[7];

  // 计算能量
  // [解析] E = 0.5*C*U^2 现算（CAN 帧里没有能量）；能量随电压平方下降，所以放到 3V 就基本没劲了。
  this->cap_energy = 0.5f * this->capacitance_ * this->voltage * this->voltage;

  // 更新时间戳
  last_read_ms_ = stamp_ms;
}

// [解析] 功率上限、缓冲能量都按大端拆两字节；data[3]/data[4] 是保留位故不赋值，发前最好先 memset 清零。
void SuperCap::write(
  uint8_t * data, uint16_t chassis_power_limit, uint16_t buffer_energy, uint8_t pm02_chassis_output)
{
  data[0] = static_cast<uint8_t>(mode_);

  data[1] = chassis_power_limit >> 8;
  data[2] = chassis_power_limit;

  // data[3] = 保留
  // data[4] = 保留

  data[5] = buffer_energy >> 8;
  data[6] = buffer_energy;

  data[7] = pm02_chassis_output;
}

}  // namespace sp
