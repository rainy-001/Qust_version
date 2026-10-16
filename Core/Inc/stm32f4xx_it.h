// [解析] ==========================================================================
// [解析] 【文件】Core/Inc/stm32f4xx_it.h
// [解析] 【作用】中断服务函数(ISR)的"声明总表"。本文件里没有任何实现，
// [解析]         只是把 12 个中断/异常处理函数的原型列出来，实现全在 Core/Src/stm32f4xx_it.c。
// [解析] 【谁生成】CubeMX 自动生成。本文件里【没有任何 USER CODE BEGIN/END 保护区】，
// [解析]           也就是说在这里写的东西下次 GENERATE CODE 会被整个覆盖，别把代码写进来。
// [解析] 【关键点】① 这 12 个函数名一个字都不能改 —— 它们必须和启动文件
// [解析]             （startup_stm32f407xx.s）里的中断向量表完全一致，芯片靠"中断号 -> 向量表项"
// [解析]             找到函数地址。名字写错不会报编译错，而是中断永远进不来或跳进死循环；
// [解析]           ② 为什么没有 SysTick_Handler？因为 FreeRTOS 把调度心跳也放进了 SysTick，
// [解析]             详见文件末尾的说明；
// [解析]           ③ 每个中断最终"调用到你写的哪个业务函数"，下面逐条标注了。
// [解析] ==========================================================================

/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.h
  * @brief   This file contains the headers of the interrupt handlers.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
// [解析] ↑ 上面这段是 CubeMX 生成的固定版权注释，对编译没有作用。
// [解析] 提醒：本文件【通篇没有 USER CODE 保护区】。也就是说，直接改本文件的内容，
// [解析]   下次在 CubeMX 里 GENERATE CODE 时会全部被重新生成覆盖。要加自己的中断，
// [解析]   正确做法是在对应的 _IRQHandler 里用 USER CODE 保护区（那种写法在 .c 文件里）。

/* Define to prevent recursive inclusion -------------------------------------*/
// [解析] ↓ 条件编译"包含卫士"：防止本头文件被重复包含导致函数原型重复声明（本文件里其实
// [解析]   重复声明不会报错，但保持这个习惯是通用做法）。
#ifndef __STM32F4xx_IT_H
#define __STM32F4xx_IT_H

// [解析] ↓ extern "C"：中断服务函数是由硬件向量表按 C 符号名调用的，必须是 C 链接。
// [解析]   C++ 会对函数名做改编(name mangling)，一旦被改编，向量表里记的名字就对不上了，
// [解析]   分分钟跑飞。本工程上层是 C++，所以这个包裹尤其重要。
#ifdef __cplusplus
extern "C" {
#endif

/* Private includes ----------------------------------------------------------*/
// [解析] ↓ 这一块（以及下面的 ET / EC / EM）是 CubeMX 的模板分区，本工程一块都没用到：
// [解析]   Includes -> 本文件不需要任何头文件（下面全是 void xxx(void) 这种无参声明）；
// [解析]   ET = Exported Types     （要对外公开的结构体/枚举类型）；
// [解析]   EC = Exported Constants （要对外公开的常量）；
// [解析]   EM = Exported Macro     （要对外公开的宏）。
// [解析]   由于空着，从本文件看不出各中断用到哪些句柄；那些 extern 句柄声明写在
// [解析]   Core/Src/stm32f4xx_it.c 里（例如 hcan1、huart3、hdma_usart3_rx、htim1）。
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
// [解析] 从下面开始就是本工程的"中断总表"，一共 12 个函数（6 个内核异常 + 6 个外设中断）。
// [解析] ★命名规律先记住，看名字就知道它是什么：
// [解析]   · 内核异常以 _Handler 结尾：NMI / HardFault / MemManage / BusFault / UsageFault / DebugMon；
// [解析]   · 外设中断以 _IRQHandler 结尾：EXTI4 / DMA1_Stream1 / CAN1_RX0 / EXTI9_5 /
// [解析]     TIM1_UP_TIM10 / USART3；
// [解析]   · 名字里带下划线数字的表示"多个中断共用同一个向量"，例如 EXTI9_5 = EXTI5~9 共享，
// [解析]     TIM1_UP_TIM10 = TIM1 更新中断与 TIM10 共享。
// [解析] ★这些名字必须与启动文件 startup_stm32f407xx.s 的向量表严格一致，改一个字就失效。
// [解析]   而且要写在 extern "C" 里面（见上面），否则 C++ 的改名机制会让名字对不上。
void NMI_Handler(void);
// [解析] ↑ NMI = Non-Maskable Interrupt，不可屏蔽中断：优先级最高且关不掉，正常运行永远不进。
// [解析]   一旦进来通常意味着硬件级严重问题（例如时钟安全系统 CSS 检测到外部晶振失效）。
// [解析]   stm32f4xx_it.c 里的实现是 while(1) 卡死 —— 这是有意的保护策略，不是忘写代码：
// [解析]   与其带着未知故障继续跑（可能乱转电机），不如停下来让调试器抓住现场。
void HardFault_Handler(void);
// [解析] ↑ HardFault：调试阶段最常见的"崩溃"入口。空指针解引用、数组越界写、除零、栈溢出、
// [解析]   往只读区写数据、中断里执行了非法操作等都会掉进来。
// [解析]   排查方法：在 while(1) 处打断点，用调试器看 Call Stack 找到崩溃前最后一个函数；
// [解析]   也可以进一步读 CFSR/HFSR 寄存器和压栈的 PC/LR 精确定位。
void MemManage_Handler(void);
// [解析] ↑ MemManage：内存管理错误，访问了受 MPU 保护的区域时触发。
// [解析]   本工程没有启用 MPU，所以实际不会进来，属于 Cortex-M4 的标准模板。
void BusFault_Handler(void);
// [解析] ↑ BusFault：总线错误。典型诱因是访问了"没有外设应答"的地址，例如外设时钟没使能
// [解析]   就去读它的寄存器。本工程未单独使能该异常，一般会合并到 HardFault 处理。
void UsageFault_Handler(void);
// [解析] ↑ UsageFault：用法错误，例如执行未定义指令、除零（需开 DIV_0_TRP）、
// [解析]   非对齐访问（需开 UNALIGN_TRP）。用 packed 结构体直接取地址时值得留意。
void DebugMon_Handler(void);
// [解析] ↑ DebugMon：调试监视器异常，只有在开启调试相关功能时才可能触发。函数体是空的，正常。
void EXTI4_IRQHandler(void);
// [解析] ↑ 外部中断线 4 —— 对应引脚 **PC4 = INT1_ACCEL_Pin**（BMI088 加速度计的中断输出）。
// [解析]   注意是 PC4 不是 PA4：EXTI 的线号只看"引脚号"，端口由外部中断配置寄存器选择，
// [解析]   main.h 里写得明明白白：INT1_ACCEL_GPIO_Port = GPIOC、INT1_ACCEL_Pin = GPIO_PIN_4。
// [解析]   （Core/Src/stm32f4xx_it.c 里把它注释成 "PA4" 是不准确的，以 main.h 为准。）
// [解析]   stm32f4xx_it.c 里调用 HAL_GPIO_EXTI_IRQHandler(INT1_ACCEL_Pin) 清标志并回调
// [解析]   HAL_GPIO_EXTI_Callback()。本工程没有实现那个回调，所以这个中断目前进来了也没事做
// [解析]   —— IMU 数据是 imu_task 定时用 SPI 主动去读的，不靠中断。
void DMA1_Stream1_IRQHandler(void);
// [解析] ↑ DMA1 数据流 1 的中断 —— 在本工程里它就是"USART3 收满/收完一段数据"的通知，
// [解析]   也是 DBus 遥控器数据的入口。完整数据流：
// [解析]     遥控器 -> USART3 的 RX 引脚 -> DMA 自动搬进内存缓冲 -> 触发本中断
// [解析]   stm32f4xx_it.c 里调用 HAL_DMA_IRQHandler(&hdma_usart3_rx)，
// [解析]   最终回调到 applications/uart_task.cpp 的 HAL_UARTEx_RxEventCallback()。
// [解析]   注意传入的句柄必须是 USART3 专用的 hdma_usart3_rx，传错会导致 DMA 状态机错乱。
void CAN1_RX0_IRQHandler(void);
// [解析] ↑ CAN1 接收 FIFO0 有新的报文 —— 电机等设备回包的入口。
// [解析]   stm32f4xx_it.c 里调用 HAL_CAN_IRQHandler(&hcan1)，
// [解析]   最终回调到 applications/can_task.cpp 的 HAL_CAN_RxFifo0MsgPendingCallback()，
// [解析]   在那里用 while (HAL_CAN_GetRxFifoFillLevel(...)) + can1.recv() 把积压的帧一次取完。
void EXTI9_5_IRQHandler(void);
// [解析] ↑ 外部中断线 5~9 共用的向量（这就是名字里 "9_5" 的来历）。
// [解析]   本工程用到的是 **PC5 = INT1_GYRO_Pin**（BMI088 陀螺仪的中断输出脚）。
// [解析]   共用向量意味着函数里必须能区分是哪条线来的 —— HAL_GPIO_EXTI_IRQHandler(引脚号) 会
// [解析]   按传入的引脚号检查并清除对应的挂起标志，所以 stm32f4xx_it.c 传的是 INT1_GYRO_Pin。
// [解析]   和 EXTI4 一样，本工程没有实现 HAL_GPIO_EXTI_Callback()，它目前不起实际作用。
void TIM1_UP_TIM10_IRQHandler(void);
// [解析] ↑ ★本工程最特殊、也最重要的一个中断：TIM1 的更新中断被借用来当 HAL 的 1 ms 时基。
// [解析]   【为什么必须这样】HAL 库默认用 SysTick 数超时（HAL_GetTick / HAL_Delay），
// [解析]   但 FreeRTOS 也要用 SysTick 做任务调度心跳（configTICK_RATE_HZ = 1000），
// [解析]   一个中断没法伺候两家，于是把 HAL 的时基挪到 TIM1 —— 这就是 .ioc 里
// [解析]   "Timebase Source = TIM1" 的含义，也是 Core/Src/stm32f4xx_hal_timebase_tim.c 存在的原因。
// [解析]   【命名】TIM1_UP（更新）与 TIM10 共用一个向量，所以名字里出现两个定时器。
// [解析]   【调用链】本函数 -> HAL_TIM_IRQHandler(&htim1) -> HAL_TIM_PeriodElapsedCallback()
// [解析]     -> Core/Src/main.c 里的 if (htim->Instance == TIM1) HAL_IncTick();
// [解析]   HAL_IncTick() 让全局变量 uwTick 每毫秒加 1，所有 HAL 延时/超时判断都靠它。
// [解析]   如果这个中断被关掉或时基配错，现象是"程序能跑但所有延时都失灵"。
// [解析]   注意 TIM1 的句柄定义不在 tim.c，而在 Core/Src/stm32f4xx_hal_timebase_tim.c。
void USART3_IRQHandler(void);
// [解析] ↑ USART3 全局中断：里面既有"串口空闲(IDLE)检测"也有出错事件，和上面的 DMA1_Stream1
// [解析]   分工配合：DMA 负责搬数据，串口的 IDLE 事件负责判断"这一帧到此结束"。
// [解析]   stm32f4xx_it.c 里调用 HAL_UART_IRQHandler(&huart3)，最终回调到
// [解析]   applications/uart_task.cpp 里的 HAL_UARTEx_RxEventCallback()（收到一帧 -> 解析）
// [解析]   和 HAL_UART_ErrorCallback()（出错 -> 重新 request() 恢复接收）。
// [解析]   ▼ 为什么这里没有 SysTick_Handler：
// [解析] 注意：本文件里【没有】SysTick_Handler 的原型，这不是 CubeMX 漏生成，而是 FreeRTOS 工程
// [解析]   的正常现象：Core/Inc/FreeRTOSConfig.h 里有一行
// [解析]     #define xPortSysTickHandler SysTick_Handler
// [解析]   意思是"FreeRTOS 的调度心跳函数就叫 SysTick_Handler"，它由 FreeRTOS 的移植层
// [解析]   （port.c）提供实现并声明。如果这里再声明或实现一次，就会与它重复定义、链接报错。
/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

#ifdef __cplusplus
}
#endif

// [解析] ↑ 这对 #ifdef __cplusplus / #endif 结束了开头的 extern "C" 块。
// [解析] ↓ 下面这行结束开头的 #ifndef __STM32F4xx_IT_H 包含卫士。
// [解析]   注意这两行是【紧挨着】的，中间没有空行，也没有 /* 注释 */ —— 这是 CubeMX 的输出格式。
#endif /* __STM32F4xx_IT_H */
