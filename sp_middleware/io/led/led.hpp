// [解析] 板载 RGB 灯的类封装：set(r, g, b) 就能改颜色和亮度。
// [解析] applications/ 的状态指示任务使用，典型顺序是 LED led(&htim5); led.start(); led.set(1, 0, 0);
// [解析] 亮度是 [0, 1] 的浮点，不是 0~255；通道对应又是 CH1=B、CH2=G、CH3=R，最容易看错。
#ifndef SP__LED_HPP
#define SP__LED_HPP

#include "tim.h"

namespace sp
{
// [解析] 一个对象管一组三色灯，占用该定时器的 CH1/CH2/CH3；set() 会重写 ARR，别把这个定时器挪作他用。
class LED
{
public:
  LED(TIM_HandleTypeDef * htim);

  void start();

  // 取值范围: [0, 1]
  void set(float r, float g, float b);

private:
  TIM_HandleTypeDef * htim_;
};

}  // namespace sp

#endif  // SP__LED_HPP