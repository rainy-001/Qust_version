// [解析] 蜂鸣器封装：把 TIM 通道配成 PWM，频率决定音调、占空比决定响度。
// [解析] applications/ 的提示音任务建对象：start() 打开输出，set(hz, duty) 发声，stop() 停。
// [解析] clock_hz 要填定时器输入时钟（如 84e6），不是想要的音调；它只用来算分频，填错音调整体偏。
#ifndef SP__BUZZER_HPP
#define SP__BUZZER_HPP

#include "tim.h"

namespace sp
{
class Buzzer
{
public:
  // clock_hz: 定时器的时钟频率, 单位: Hz
  Buzzer(TIM_HandleTypeDef * htim, uint16_t channel, float clock_hz);

  void start();
  void stop();

  // hz: 发声频率, 单位: Hz
  // duty: 占空比, 取值范围: [0, 1], 0.5时声音最响
  // [解析] 只 set() 不 start() 是没有声音的；它每次改写 PSC/ARR/CCR 并把计数器清零，同时只能保持一个音调。
  void set(float hz, float duty = 0.5);

private:
  TIM_HandleTypeDef * htim_;
  uint16_t channel_;
  const float clock_hz_;
};

}  // namespace sp

#endif  // SP__BUZZER_HPP