// [解析] ==========================================================================
// [解析] 【文件】sp_middleware/io/servo/servo.hpp
// [解析] 【作用】把『舵机』封装成一个 Servo 对象：set(角度) 就让舵机转到对应角度。
// [解析] 【原理】舵机要的是 50Hz 的 PWM 信号（周期 20ms），
// [解析]         代码按『高电平 0.5ms = 0 度、2.5ms = 最大角度』线性映射，所以本质就是调脉宽。
// [解析] 【被谁用】applications/ 下需要舵机的机构（比如拨弹、云台辅助），
// [解析]         例如 sp::Servo servo(&htim1, TIM_CHANNEL_1, 84000000, 180.0f);
// [解析] 【使用三步】1) CubeMX 把某个 TIM 通道配成 PWM；2) 构造对象并 start()；3) set(角度)。
// [解析] 【关键点】clock_hz 要填『定时器的输入时钟频率』（不是 50Hz！）；
// [解析]         max_angle 要和你舵机的实际行程一致，否则转不到位或撞限位。
// [解析] ==========================================================================
#ifndef SP__SERVO_HPP
#define SP__SERVO_HPP

#include "tim.h"

namespace sp
{
// [解析] Servo 类：一个对象管一路舵机信号线（要控制多个舵机就建多个对象）。
// [解析] 注意：set() 会直接改这个定时器的 PSC 和 ARR，所以它不能和别的功能共用同一个定时器。
class Servo
{
public:
  // clock_hz: 定时器的时钟频率, 单位: Hz
  // max_angle: 舵机角度范围, 例如90度, 180度等等
  // [解析] ↑ 构造函数只保存参数，不动硬件；之后还要调用 start() 才有 PWM 输出。
  // [解析]   htim     ：CubeMX 生成的定时器句柄（如 &htim1）；
  // [解析]   channel  ：通道号宏，如 TIM_CHANNEL_1；
  // [解析]   clock_hz ：定时器输入时钟频率（Hz），例如 84e6 表示 84MHz；
  // [解析]   max_angle：舵机的最大角度（度），例如 180.0f 或 270.0f，要和舵机型号一致。
  Servo(TIM_HandleTypeDef * htim, uint16_t channel, float clock_hz, float max_angle);

  // 开启对应的PWM端口
  // [解析] ↑ 打开该通道的 PWM 输出（内部调用 HAL_TIM_PWM_Start）。
  // [解析]   注意：只 start() 还没定角度，真正决定角度的是 set()。
  void start();

  // 设置旋转角度
  // angle: 取值[0, max_angle], 单位同max_angle
  // [解析] ↑ 让舵机转到 angle 度：取值 [0, max_angle]，单位与 max_angle 相同（度）。
  // [解析]   传入超范围的值不会被裁剪，而是会输出超出 [500, 2500] 微秒的脉宽，
  // [解析]   可能导致舵机堵转/异响，所以调用前请自己把角度限制在范围内。
  // [解析]   副作用：每一帧都重设 PSC、ARR、CCR 三个寄存器。
  void set(float angle);

private:
  // [解析] ↑ 本对象控制的定时器句柄。
  TIM_HandleTypeDef * htim_;
  // [解析] ↑ 本对象使用的 PWM 通道（TIM_CHANNEL_x）。
  uint16_t channel_;
  // [解析] ↑ 定时器输入时钟频率（Hz），只读：构造后不再变，set() 靠它算分频。
  const float clock_hz_;
  // [解析] ↑ 舵机最大角度（度），只读：set() 用它把角度归一化成 0~1 的比例。
  const float max_angle_;
};

}  // namespace sp

#endif  // SP__SERVO_HPP