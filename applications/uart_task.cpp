// [解析] 遥控器接收任务：读 DBus 遥控器的摇杆/拨杆/鼠标/键盘数据供其他任务使用；C 板接 USART3，100000 波特。
// [解析] freertos.c 以 osPriorityRealtime（最高）、栈 256 字创建 —— 遥控数据晚了就危险。
// [解析] 机制是"请求-回调"接力：request() 发起接收 -> 收满一帧进中断回调解析后再次 request()，故只需请求一次。

#include "cmsis_os.h"
#include "io/dbus/dbus.hpp"

// C板
sp::DBus remote(&huart3);

// 达妙
// sp::DBus remote(&huart5, false);

extern "C" void uart_task()
{
  remote.request();

  while (true) {
    // 使用调试(f5)查看remote内部变量的变化
    osDelay(10);
  }
}

// [解析] 串口收满一帧/空闲的中断回调，Size 是本次实际字节数；必须在里面再 request() 一次接力，否则只收一次就停。
extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef * huart, uint16_t Size)
{
  auto stamp_ms = osKernelSysTick();

  if (huart == &huart3) {
    remote.update(Size, stamp_ms);
    remote.request();
  }
}

// [解析] 串口出错回调：HAL 出错后会停止当前接收，不在这里重新 request() 遥控器就永久失联。
extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef * huart)
{
  if (huart == &huart3) {
    remote.request();
  }
}
