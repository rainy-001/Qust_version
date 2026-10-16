// [解析] ==========================================================================
// [解析] 【文件】sp_middleware/io/buzzer/buzzer.hpp
// [解析] 【作用】把『蜂鸣器』封装成一个 Buzzer 对象：set(频率, 占空比) 就能发出指定音调。
// [解析] 【原理】蜂鸣器接在定时器(TIM)的 PWM 输出脚上，PWM 的『频率』决定音调高低，
// [解析]         『占空比』决定推动蜂鸣器的能量大小（0.5 时通常最响）。
// [解析] 【被谁用】applications/ 下的蜂鸣器/提示音任务，例如 sp::Buzzer buzzer(&htim4, TIM_CHANNEL_3, 84000000);
// [解析] 【使用三步】1) CubeMX 把某个 TIM 通道配成 PWM 输出；2) 构造对象并 start()；3) set(音调, 占空比) 发声，
// [解析]         不响就 stop()。
// [解析] 【关键点】clock_hz 必须填『该定时器的输入时钟频率』（不是 PWM 输出频率！），
// [解析]         填错了音调就会整体偏高或偏低。C 板 TIM4 挂 APB1，常见值是 84MHz。
// [解析] ==========================================================================
#ifndef SP__BUZZER_HPP
#define SP__BUZZER_HPP

#include "tim.h"

namespace sp
{
// [解析] Buzzer 类：一个对象管一路 PWM 蜂鸣器（想同时响两个音就得建两个对象）。
// [解析] 注意：set() 里会直接改定时器的分频(PSC)和重装值(ARR)，
// [解析] 所以这个定时器不要再被别的功能共用，否则两边会互相抢参数。
class Buzzer
{
public:
  // clock_hz: 定时器的时钟频率, 单位: Hz
  // [解析]   htim   ：CubeMX 生成的定时器句柄（如 &htim4）；
  // [解析]   channel：该定时器的通道号，如 TIM_CHANNEL_3（注意是宏，不是数字 3）；
  // [解析]   clock_hz：定时器输入时钟频率（Hz），例如 84e6 表示 84MHz。
  Buzzer(TIM_HandleTypeDef * htim, uint16_t channel, float clock_hz);
  // [解析] ↑ 构造函数只把参数记下来，不动硬件；之后还要调用 start() 才会真正有波形输出。

  // [解析] ↑ 打开 PWM 输出（内部就是 HAL_TIM_PWM_Start）。只调用 set() 而不 start() 是没有声音的。
  void start();
  // [解析] ↑ 关闭 PWM 输出，蜂鸣器停。和 start() 成对使用，不需要重新构造对象。
  void stop();

  // hz: 发声频率, 单位: Hz
  // duty: 占空比, 取值范围: [0, 1], 0.5时声音最响
  // [解析] ↑ set() 的用法与单位：
  // [解析]   hz  ：想要发出的声音频率（Hz），人耳可听范围约 20~20000Hz，常用 1000~4000Hz；
  // [解析]   duty：占空比，取值 [0, 1]，默认 0.5；蜂鸣器一般 0.5 最响，太大太小都会变轻。
  // [解析]   副作用：它会改写定时器的 PSC / ARR / CCR 三个寄存器并把计数器清零，
  // [解析]   所以同一时刻只能保持一个音调（不能像唱歌那样同时发多个音）。
  // [解析]   注意：只 set() 不 start() 是没有输出的；调用前请确认已经 start()。
  void set(float hz, float duty = 0.5);

private:
  // [解析] ↑ 本对象控制的定时器句柄。
  TIM_HandleTypeDef * htim_;
  // [解析] ↑ 本对象使用的 PWM 通道（TIM_CHANNEL_x）。
  uint16_t channel_;
  // [解析] ↑ 定时器输入时钟频率（Hz），只读常量，构造之后不会再变；set() 靠它算分频和 ARR。
  const float clock_hz_;
};

}  // namespace sp

#endif  // SP__BUZZER_HPP