// [解析] 蜂鸣器任务：PWM 驱动蜂鸣器，上电后"嘀"三声提示程序已启动（TIM4_CH3，C 板定时器时钟 84MHz）。
// [解析] 音调由 PWM 频率决定、音量由占空比决定（0.5 最响）；set() 只备参数，start() 才真的响。
// [解析] 换板子只需改 Buzzer 构造的三项：定时器、通道、定时器时钟频率（C 板 84e6 / 达妙 240e6）。

#include "cmsis_os.h"
#include "io/buzzer/buzzer.hpp"

// C板
sp::Buzzer buzzer(&htim4, TIM_CHANNEL_3, 84e6);

// 达妙
// sp::Buzzer buzzer(&htim12, TIM_CHANNEL_2, 240e6);

// [解析] 5000Hz、占空比 0.1，响 100ms 停 100ms 共三次；之后必须死循环挂住 —— FreeRTOS 任务函数 return 会出错。
extern "C" void buzzer_task()
{
  buzzer.set(5000, 0.1);

  for (int i = 0; i < 3; i++) {
    buzzer.start();
    osDelay(100);
    buzzer.stop();
    osDelay(100);
  }

  while (true) {
    osDelay(100);
  }
}
