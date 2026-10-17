// [解析] LED 任务：板载 RGB 灯做蓝色呼吸，兼作"程序还活着"的心跳指示（TIM5 输出 PWM，CH1/2/3 对应 R/G/B）。
// [解析] freertos.c 以 osPriorityLow、栈 128 字创建 —— 纯装饰，不该跟遥控器、电机这些控制任务抢 CPU。
// [解析] 亮度靠占空比，set(r,g,b) 三分量取值 [0,1]；颜色和预期不符多半是 CH1/2/3 与 R/G/B 顺序反了，
// [解析]   把 set() 的三个参数换个位置即可（详见 docs/C板引脚配置说明.md 第 3 节）。

#include "cmsis_os.h"
#include "io/led/led.hpp"

sp::LED led(&htim5);

extern "C" void led_task(void const * argument)
{
  led.start();

  while (true) {
    // [解析] 下面几段被注释掉的红/绿渐变是备选写法，和蓝色呼吸完全同构，只换了 set() 的参数位置。
    // for (uint8_t b = 0; b < 10; b++) {
    //   led.set(b * 0.01f, 0, 0);
    //   osDelay(100);
    // }

    // for (uint8_t b = 10; b > 0; b--) {
    //   led.set(b * 0.01f, 0, 0);
    //   osDelay(100);
    // }
    // for (uint8_t g = 0; g < 10; g++) {
    //   led.set(0, g * 0.01f, 0);
    //   osDelay(100);
    // }

    // for (uint8_t g = 10; g > 0; g--) {
    //   led.set(0, g * 0.01f, 0);
    //   osDelay(100);
    // }

    // [解析] 呼吸灯两段：b 从 0→9 渐亮、再从 10→1 渐暗，每步 100ms，一个来回约 2 秒。
    for (uint8_t b = 0; b < 10; b++) {
      led.set(0, 0, b * 0.01f);
      osDelay(100);
    }

    for (uint8_t b = 10; b > 0; b--) {
      led.set(0, 0, b * 0.01f);
      osDelay(100);
    }
  }
}
