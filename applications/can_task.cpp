// [解析] CAN 通信任务：把 GM6020 的电压指令发出去，并接收电机回传的状态（CAN1，接电机/超电总线）。
// [解析] freertos.c 的 MX_FREERTOS_Init() 以 osPriorityHigh、栈 128 字创建，1ms 发一次（1000Hz）。
// [解析] 接收在下面的 FIFO0 中断回调里完成，那里只取帧与解析，不做耗时操作。

#include "cmsis_os.h"
#include "io/can/can.hpp"
#include "motor/rm_motor/rm_motor.hpp"

sp::CAN can1(&hcan1);
sp::RM_Motor motor6020(1, sp::RM_Motors::GM6020_V);  // 一个电机ID为1, 电压控制模式的6020

// [解析] extern "C" 是必需的（它会被 FreeRTOS 的 C 代码调用）；循环里 write() 只打包进 tx_data，真正发出靠 send(tx_id)。
extern "C" void can_task(void const * argument)
{
  can1.config();
  can1.start();

  while (true) {
    motor6020.cmd(0.0f);  // 电压指令, 单位V; 先用较小电压测试, 注意安全

    motor6020.write(can1.tx_data);
    can1.send(motor6020.tx_id);

    osDelay(1);
  }
}

// [解析] FIFO0 中断回调，CAN1/CAN2 共用：用 while 把 FIFO 取空防丢帧，只做取帧 + 调 read() 解析。
extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef * hcan)
{
  auto stamp_ms = osKernelSysTick();

  while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0) {
    if (hcan == &hcan1) {
      can1.recv();

      if (can1.rx_id == motor6020.rx_id) motor6020.read(can1.rx_data, stamp_ms);
    }
  }
}
