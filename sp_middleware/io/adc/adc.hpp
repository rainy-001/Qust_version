// [解析] ADC 电压采集封装：update() 之后直接从 voltage 成员读走算好的电压（单位 V）。
// [解析] applications/voltage_detect.cpp 的 adc_task() 使用；C 板测的是 ADC3 IN8（PF10）上的电池电压。
// [解析] 只有轮询一种方式，update() 会阻塞等转换完成，不要在中断或高优先级任务里频繁调用。
// [解析] 电压 = 原始值 × (vref × 分压比 ÷ 2^位数) + 偏移；分压比是因为板子先用电阻降压再送进引脚。
#ifndef SP__ADC_HPP
#define SP__ADC_HPP

#include "adc.h"

namespace sp
{
// [解析] 一个对象只管一路通道，要多路采集就多建几个对象。
class Adc
{
public:
  // adc: CubeMX 初始化的 ADC 外设句柄；通道/引脚配置在 adc.c 中
  // channels: ADC 转换位数，如 12（不是 ADC_CHANNEL_14 中的通道号）
  // vref: adc电路图参考电压 C板为3.3V
  // voltage_divider: 分压比，C板为222.0f / 22.0f
  // voltage_offset: 测量值与实际值的偏差
  Adc(
    ADC_HandleTypeDef * adc, uint32_t channels, float vref, float voltage_divider,
    float voltage_offset);
  // [解析] 只做两件事：把 ADC 时钟改成 4 分频、算好换算系数；整个程序生命周期调一次，且必须在 update() 之前。
  void init();
  // 启动、等待并读取一次转换；失败返回 false，保留上次成功测量值
  bool update();
  // 测量得到的电压值；须在 update() 成功后使用
  float voltage;

private:
  //adc口
  ADC_HandleTypeDef * adc_;
  //2的ADC转换位数幂，如12位则2^12(4096)
  uint32_t channel_pow_;
  //ADC参考电压
  float vref_;
  //电压分压比
  float voltage_divider_;
  //ADC转换值到电压的系数
  float adc12_to_v_coef_;
  //电压偏移值
  float voltage_offset_;
};
}  // namespace sp

#endif
