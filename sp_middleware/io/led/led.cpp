// [解析] 板载 RGB 灯驱动：R/G/B 三个脚挂在同一个定时器的 CH1/CH2/CH3 上，亮度靠 PWM 占空比调。
// [解析] applications/ 的状态指示任务调用 start() 打开三个通道，之后 set(r, g, b) 改颜色。
// [解析] 通道顺序是 CH1=B、CH2=G、CH3=R；ARR 固定 65535，亮度填 1.0 会溢出截断成灭灯，最大用 0.99。
#include "led.hpp"

namespace sp
{
LED::LED(TIM_HandleTypeDef * htim) : htim_(htim) {}

void LED::start()
{
  HAL_TIM_PWM_Start(htim_, TIM_CHANNEL_1);  // dismiss return
  HAL_TIM_PWM_Start(htim_, TIM_CHANNEL_2);  // dismiss return
  HAL_TIM_PWM_Start(htim_, TIM_CHANNEL_3);  // dismiss return
}

void LED::set(float r, float g, float b)
{
  __HAL_TIM_SET_AUTORELOAD(htim_, 65535);
  __HAL_TIM_SET_COMPARE(htim_, TIM_CHANNEL_1, (htim_->Instance->ARR + 1) * b);
  __HAL_TIM_SET_COMPARE(htim_, TIM_CHANNEL_2, (htim_->Instance->ARR + 1) * g);
  __HAL_TIM_SET_COMPARE(htim_, TIM_CHANNEL_3, (htim_->Instance->ARR + 1) * r);
}

}  // namespace sp