#include "cmsis_os.h"
#include "io/can/can.hpp"
#include "motor/rm_motor/rm_motor.hpp"

sp::CAN can1(&hcan1);
sp::RM_Motor motor6020(1, sp::RM_Motors::GM6020_V);  // 一个电机ID为1, 电压控制模式的6020

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
