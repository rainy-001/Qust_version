// [解析] 舵机驱动：把角度换算成 50Hz PWM 的脉宽，走的是 TIM 通道。
// [解析] applications/ 的舵机机构使用，先 start() 打开 PWM，之后 set(角度) 到位。
// [解析] 行尾三个魔数（1MHz 基准、20000 计数=50Hz、500~2500us 脉宽）就是全部量程假设；angle 超范围也不裁剪。
#include "servo.hpp"

namespace sp
{
Servo::Servo(TIM_HandleTypeDef * htim, uint16_t channel, float clock_hz, float max_angle)
: htim_(htim), channel_(channel), clock_hz_(clock_hz), max_angle_(max_angle)
{
}

void Servo::start()
{
  HAL_TIM_PWM_Start(htim_, channel_);  // dismiss return
}

void Servo::set(float angle)
{
  __HAL_TIM_SET_PRESCALER(htim_, clock_hz_ / 1e6f - 1);                       // 预分频后为1MHz
  __HAL_TIM_SET_AUTORELOAD(htim_, 20000 - 1);                                 // 输出频率为50Hz
  __HAL_TIM_SET_COMPARE(htim_, channel_, (angle / max_angle_) * 2000 + 500);  // [500, 2500]
}

}  // namespace sp