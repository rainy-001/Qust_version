// [解析] ==========================================================================
// [解析] 【文件】sp_middleware/io/adc/adc.hpp
// [解析] 【作用】把『读一次 ADC 电压』这件事封装成一个 Adc 对象：调用 update() 之后，
// [解析]         直接从成员变量 voltage 里读走算好的电压值（单位：伏特 V）。
// [解析] 【被谁用】applications/voltage_detect.cpp 的 adc_task()：
// [解析]         sp::Adc battery(&hadc3, 12, 3.3f, (222.0f / 22.0f), 1.0f);
// [解析]         本工程（C 板）用的是 ADC3 的 IN8 通道，即 PF10 引脚测电池电压。
// [解析] 【使用三步】1) 在 main.c/CubeMX 里配好 hadc1 与对应引脚；2) 构造对象；3) 调用 init() 一次，
// [解析]         之后周期调用 update()，读 voltage。init() 必须在 update() 之前，否则换算系数是 0。
// [解析] 【关键点】本类只有『轮询(Polling)』一种工作方式：update() 里会阻塞等待本次转换结束，
// [解析]         所以不要在中断或高优先级任务里频繁调用；它背后没有 DMA、也没有中断回调。
// [解析] 【关键点】voltage 里的物理量是这么来的：
// [解析]         电压 = ADC原始值 × (vref × 分压比 ÷ 2^位数) + 电压偏移。
// [解析]         用『分压比』是因为板子上先用两个电阻把高电压分压后才送进 ADC 引脚，
// [解析]         所以要把量到的值按比例放大回去，才能得到真实电压。
// [解析] ==========================================================================
#ifndef SP__ADC_HPP
#define SP__ADC_HPP

#include "adc.h"

namespace sp
{
// [解析] Adc 类：一个对象对应一路 ADC 通道（比如 C 板上的电池电压检测）。
// [解析] 典型用法（本工程的实际写法）：Adc adc(&hadc3, 12, 3.3f, 222.0f / 22.0f, 1.0f); adc.init(); 之后循环 adc.update();
// [解析] 注意：一个对象只对应『一路』通道，要多路采集就多建几个对象。
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
  // [解析] init() 只做两件事：1) 把 ADC 时钟分频改成 4 分频（见 adc.cpp）；2) 算好换算系数。
  // [解析] 必须在第一次 update() 之前调用，而且整个程序生命周期调用一次就够了。
  // [解析] 为什么还要在这里做 init：因为分频系数是驱动作者调好的经验值，写在代码里比依赖 CubeMX 更保险。
  void init();
  // 启动、等待并读取一次转换；失败返回 false，保留上次成功测量值
  bool update();
  // [解析] update()：启动一次转换 -> 等它结束 -> 读原始值 -> 换算成电压 -> 写进 voltage。
  // [解析] 返回值：true 表示这次测量有效，voltage 已被刷新；
  // [解析]           false 表示 HAL 失败（比如超时），此时 voltage 会保留上一次成功测到的旧值，
  // [解析]           调用者可以用它做『连续失败计数』，判断传感器是不是掉线了。
  // 测量得到的电压值；须在 update() 成功后使用
  // [解析] ↑ 测量结果（单位：V）。必须等 update() 返回 true 之后读才有效。
  // [解析]   它由 update() 写、由调用者读，所以是 public 的『输出变量』，不靠函数返回值传递。
  float voltage;

private:
  //adc口
  // [解析] ↑ 指向 CubeMX 生成的 ADC 句柄（如 &hadc1），本对象要操作的就是它。
  ADC_HandleTypeDef * adc_;
  //2的ADC转换位数幂，如12位则2^12(4096)
  // [解析] ↑ 1 << 12 = 4096，即『满量程原始值』（12 位 ADC 的最大值 4095 再加 1）。
  uint32_t channel_pow_;
  //ADC参考电压
  // [解析] ↑ 参考电压（V）。C 板一般是 3.3V。
  float vref_;
  //电压分压比
  // [解析] ↑ 分压比 = (上电阻 + 下电阻) / 下电阻。C 板用 222kΩ 与 22kΩ，即 222/22 ≈ 10.09。
  float voltage_divider_;
  //ADC转换值到电压的系数
  // [解析] ↑ 换算系数，在 init() 里算出来：vref × 分压比 ÷ 4096，单位是 V/原始值。
  float adc12_to_v_coef_;
  //电压偏移值
  // [解析] ↑ 标定用的加性偏差（V）：万用表量出来差多少就在这里补多少，用来抵消分压电阻的误差。
  float voltage_offset_;
};
}  // namespace sp

#endif
