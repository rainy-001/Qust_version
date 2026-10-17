// [解析] 蜂鸣器驱动：把 TIM 通道当 PWM 用，频率决定音调、占空比决定响度。
// [解析] applications/ 的提示音任务通过 Buzzer 对象调用 start() / set(hz, duty) / stop()。
// [解析] set() 直接改写 TIM 的 PSC/ARR/CCR 三个寄存器，所以这个定时器不能再分给别的功能共用。
#include "buzzer.hpp"

namespace sp
{
Buzzer::Buzzer(TIM_HandleTypeDef * htim, uint16_t channel, float clock_hz)
: htim_(htim), channel_(channel), clock_hz_(clock_hz)
{
}

void Buzzer::start()
{
  HAL_TIM_PWM_Start(htim_, channel_);  // dismiss return
}

void Buzzer::stop()
{
  HAL_TIM_PWM_Stop(htim_, channel_);  // dismiss return
}

// [解析] 先把计数时钟压到 1MHz（1 计数 = 1us），ARR 和 CCR 的数字就等于微秒数，直接心算。
void Buzzer::set(float hz, float duty)
{
  // ref: https://www.bilibili.com/video/BV1th411z7sn?t=1393.4&p=15
  __HAL_TIM_SET_PRESCALER(htim_, clock_hz_ / 1e6 - 1);  // 预分频到1MHz
  __HAL_TIM_SET_AUTORELOAD(htim_, clock_hz_ / (htim_->Instance->PSC + 1) / hz - 1);
  __HAL_TIM_SET_COMPARE(htim_, channel_, (htim_->Instance->ARR + 1) * duty);
  __HAL_TIM_SET_COUNTER(htim_, 0);
}

}  // namespace sp
