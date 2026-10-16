// [解析] ==========================================================================
// [解析] 【文件】sp_middleware/io/led/led.cpp
// [解析] 【作用】led.hpp 的实现：start() 打开三个 PWM 通道，set() 按亮度算三个通道的比较值。
// [解析] 【被谁用】applications/ 下的状态指示灯任务，通过 LED 对象调用。
// [解析] 【关键点】1) 为什么三个通道写的是 b、g、r 而不是 r、g、b；
// [解析]           2) ARR 为什么固定写 65535；3) 比较值为什么用 (ARR + 1) 乘亮度。
// [解析] ==========================================================================
#include "led.hpp"

namespace sp
{
// [解析] 构造函数：只把定时器句柄存进成员变量，函数体为空（此时不点灯、也不启动 PWM）。
LED::LED(TIM_HandleTypeDef * htim) : htim_(htim) {}

void LED::start()
{
  // [解析] ↑ 以下三行分别打开 CH1、CH2、CH3 的 PWM 输出（它们对应灯的 B、G、R 三个脚，见下面 set()）。
  // [解析]   为什么要三个都开：一个定时器的每个通道要单独使能，少开一个就会少一种颜色。
  // [解析]   每行后面的 dismiss return 是本项目约定：故意不检查 HAL 返回值，不是漏写判断。
  HAL_TIM_PWM_Start(htim_, TIM_CHANNEL_1);  // dismiss return
  HAL_TIM_PWM_Start(htim_, TIM_CHANNEL_2);  // dismiss return
  HAL_TIM_PWM_Start(htim_, TIM_CHANNEL_3);  // dismiss return
}

void LED::set(float r, float g, float b)
{
  // [解析] ↑ 把 ARR 固定成 65535，也就是让 PWM 的计数范围是 0~65535（一轮 65536 个数）。
  // [解析]   为什么是 65535：这是 16 位定时器 ARR 能装的最大值，等于把亮度分辨率拉到最高，
  // [解析]   浮点亮度乘 65536 之后仍然够精细，不会因为分辨率太低而看到明显的亮度跳变。
  // [解析]   代价：PWM 频率会很低（例如定时器 84MHz 时约 84e6/65536 ≈ 1.28kHz），
  // [解析]   对 LED 来说看不出闪烁，但如果这个定时器还要驱动别的外设就要留意。
  __HAL_TIM_SET_AUTORELOAD(htim_, 65535);
  // [解析] ↑ 通道 1 接的是灯的『蓝』脚，所以这里填的是 b（Blue）。
  // [解析]   比较值 CCR 的含义：计数器值小于 CCR 时输出一种电平、大于时输出另一种，
  // [解析]   于是 占空比 = CCR / (ARR + 1)；这里 CCR = (ARR+1) 乘 b，正好让占空比等于亮度 b。
  // [解析]   例：ARR=65535（一轮 65536），b=1.0 时算出 65536，超过 16 位 CCR 上限会被截断，
  // [解析]   实际可能表现为灯灭而不是最亮，所以亮度建议不要长期顶到 1.0。
  __HAL_TIM_SET_COMPARE(htim_, TIM_CHANNEL_1, (htim_->Instance->ARR + 1) * b);
  // [解析] ↑ 通道 2 接的是『绿』脚，填 g（Green）；计算方式与通道 1 完全相同。
  __HAL_TIM_SET_COMPARE(htim_, TIM_CHANNEL_2, (htim_->Instance->ARR + 1) * g);
  // [解析] ↑ 通道 3 接的是『红』脚，填 r（Red）。
  // [解析]   注意：三个通道的顺序是 B、G、R（不是直觉上的 R、G、B），
  // [解析]   因为板子上这三个引脚正好按 B->CH1、G->CH2、R->CH3 接到定时器，改硬件之前别随意对调。
  // [解析]   另：本函数每调用一次都会重写一遍 ARR，属于重复设置，但不影响功能。
  __HAL_TIM_SET_COMPARE(htim_, TIM_CHANNEL_3, (htim_->Instance->ARR + 1) * r);
}

// [解析] 注意：亮度填 1.0 时 (65535 + 1) 乘 1.0 = 65536，超过了 16 位 CCR 的上限 65535，
// [解析]       会被截断（低 16 位为 0），效果可能是灯灭而不是最亮；想要最亮请用接近 1 的值（如 0.99）。
}  // namespace sp