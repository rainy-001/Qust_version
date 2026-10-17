// [解析] 舵机封装：set(角度) 就让舵机转到对应角度，本质是调 50Hz PWM 的脉宽。
// [解析] applications/ 的舵机机构（拨弹、云台辅助等）使用，先 start() 再 set()。
// [解析] clock_hz 要填定时器输入时钟（如 84e6），不是 50Hz；max_angle 要和舵机实际行程一致。
// [解析] 量程是写死的：0.5ms~2.5ms 脉宽对应 0~max_angle 度，换不同行程的舵机要回来改这段映射。
#ifndef SP__SERVO_HPP
#define SP__SERVO_HPP

#include "tim.h"

namespace sp
{
class Servo
{
public:
  // clock_hz: 定时器的时钟频率, 单位: Hz
  // max_angle: 舵机角度范围, 例如90度, 180度等等
  Servo(TIM_HandleTypeDef * htim, uint16_t channel, float clock_hz, float max_angle);

  // 开启对应的PWM端口
  void start();

  // 设置旋转角度
  // angle: 取值[0, max_angle], 单位同max_angle
  // [解析] angle 超出 [0, max_angle] 不会被裁剪，会输出超范围脉宽让舵机堵转，调用前自己限幅。
  void set(float angle);

private:
  TIM_HandleTypeDef * htim_;
  uint16_t channel_;
  const float clock_hz_;
  const float max_angle_;
};

}  // namespace sp

#endif  // SP__SERVO_HPP