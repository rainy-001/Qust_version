// [解析] ==========================================================================
// [解析] 【文件】applications/can_task.cpp
// [解析] 【作用】CAN 通信任务：把 GM6020 电机的控制指令发出去，并接收电机回传的状态。
// [解析] 【谁创建】Core/Src/freertos.c 的 MX_FREERTOS_Init() 里：
// [解析]           osThreadDef(CAN, can_task, osPriorityHigh, 0, 128);
// [解析]           优先级 osPriorityHigh，栈 128 字，任务函数就是本文件下面的 can_task()。
// [解析] 【运行节奏】while(true) + osDelay(1)，即每 1ms 发一次指令（1000Hz）。
// [解析] 【硬件】CAN1（hcan1）。C 板的 CAN1 接电机/超电总线。
// [解析] ==========================================================================

#include "cmsis_os.h"
#include "io/can/can.hpp"
#include "motor/rm_motor/rm_motor.hpp"

// [解析] can1：管理 CAN1 这一路总线的对象，第 5 行下面的任务里用它收发数据。
sp::CAN can1(&hcan1);
// [解析] motor6020：一个 RoboMaster GM6020 电机对象。
// [解析]   参数 1 = 电机 ID 为 1（拨码开关设定的那个 ID）
// [解析]   参数 GM6020_V = "电压控制模式"（另一种 GM6020 是电流控制模式）
// [解析]   不用写减速比，因为 6020 是直驱云台电机，减速比按 1 处理。
sp::RM_Motor motor6020(1, sp::RM_Motors::GM6020_V);  // 一个电机ID为1, 电压控制模式的6020

// [解析] extern "C" 是必需的：这个函数会被 FreeRTOS 的 C 代码调用，
// [解析] 而 C++ 默认会对函数名做"名字修饰(mangling)"，加了 extern "C" 才不会找不到符号。
extern "C" void can_task(void const * argument)
{
  // [解析] 只做一次初始化：配置 CAN 过滤器 + 启动外设并打开接收中断。
  can1.config();
  can1.start();

  while (true) {
    // [解析] 第 1 步：给出控制量。GM6020_V 模式下单位是"伏特"。
    // [解析]   这里写 0.0f 表示"先不发扭矩"，是安全的调试起点；
    // [解析]   想让它转就把它改大（比如 1.0f），但注意电机上电前要固定好。
    motor6020.cmd(0.0f);  // 电压指令, 单位V; 先用较小电压测试, 注意安全

    // [解析] 第 2 步：write() 把刚设好的指令"打包"进 can1.tx_data 这 8 个字节里。
    // [解析]   注意 write() 只是准备数据，还没有真的发出去。参数写成 const，不会改缓冲区。
    motor6020.write(can1.tx_data);
    // [解析] 第 3 步：真正发送。tx_id 是电机规定的"控制帧 ID"，
    // [解析]   不同 ID 的电机对应不同的控制帧 ID，由 rm_motor.hpp 里的逻辑自动算好。
    can1.send(motor6020.tx_id);

    // [解析] 让出 CPU 1ms。这 1ms 里其他任务（比如 IMU、DBUS）会得到运行机会。
    osDelay(1);
  }
}

// [解析] ==========================================================================
// [解析] 下面是"接收中断回调"：CAN1 收到任意一帧时，STM32 HAL 库会自动调用它。
// [解析] 之所以会被调用，是因为前面 can1.start() 里打开了
// [解析] CAN_IT_RX_FIFO0_MSG_PENDING 这个中断通知。
// [解析] ★ 重要：这是中断服务程序(ISR)，必须尽量短！所以这里只做"取数据 + 存起来"，
// [解析]   不要在这里做耗时运算、打印、或调用会阻塞的 API。
// [解析] ==========================================================================
extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef * hcan)
{
  // [解析] 取当前系统毫秒数。后面传给 motor6020.read()，
  // [解析] 用来判断"电机是否还在线"（is_alive 会拿它和 last_read_ms_ 比较）。
  auto stamp_ms = osKernelSysTick();

  // [解析] 硬件 FIFO 里可能一次积压了好几帧，用 while 一次全部取空，避免丢帧。
  while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0) {
    // [解析] 这个回调是 CAN1 和 CAN2 共用的，所以要判断一下是不是我们关心的 CAN1。
    if (hcan == &hcan1) {
      // [解析] 从 FIFO 取出一帧，结果写进 can1.rx_id / can1.rx_data。
      can1.recv();

      // [解析] 再判断"这一帧是不是 motor6020 发来的"（比较 ID），
      // [解析] 是的话才解析：read() 会把 8 字节原始数据换算成角度/转速/扭矩/温度。
      if (can1.rx_id == motor6020.rx_id) motor6020.read(can1.rx_data, stamp_ms);
    }
  }
}
