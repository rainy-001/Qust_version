// [解析] ==========================================================================
// [解析] 【文件】Core/Src/stm32f4xx_it.c
// [解析] 【作用】中断服务程序（ISR）集中营：CPU 的异常、以及所有外设中断的入口函数都写在这里。
// [解析]          "it" 就是 interrupt 的缩写。
// [解析] 【谁生成】CubeMX 自动生成骨架，作者没有改动逻辑，只在必要处加了说明。
// [解析] 【为什么大部分函数体是空的】因为 HAL 库采用了"分层回调"设计：
// [解析]   中断真正发生后，CPU 先跳到这里（比如 CAN1_RX0_IRQHandler），
// [解析]   这里只调用一句 HAL_xxx_IRQHandler()，由 HAL 去判断"是哪个标志位触发的"，
// [解析]   最后再回调到你在 applications/ 里写的业务代码（例如
// [解析]   HAL_CAN_RxFifo0MsgPendingCallback）。所以要找"中断里到底干了什么"，
// [解析]   得顺着这条链往下追，而不是停在这个文件里。
// [解析] --------------------------------------------------------------------------
// [解析] ★★ 本文件里到处都是 /* USER CODE BEGIN xxx */ ... /* USER CODE END xxx */ ★★
// [解析]   这是 CubeMX 的"用户代码保护区"，非常重要，务必理解：
// [解析]   在 CubeMX 里点 GENERATE CODE 重新生成代码时，
// [解析]   只有夹在这对标记【之间】的内容会被保留下来，
// [解析]   标记【之外】的一切都会被新生成的代码覆盖掉！
// [解析]   所以你自己写的代码一定要写在 BEGIN/END 之间，否则下次生成就没了。
// [解析]   （本工程里 NMI_Handler 的 while(1)、TIM1 的 HAL_IncTick 之类就是靠这个机制保住的。）
// [解析] ==========================================================================

/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
// [解析] 下面这 4 个是"外部变量"，也就是定义在别处的外设句柄，这里只是借用一下。
// [解析] 注意到它们全部由 CubeMX 放在保护区外面 —— 因为它们是自动生成的，不需要保护。
// [解析] 每个句柄的含义：
extern CAN_HandleTypeDef hcan1;
// [解析] ↑ CAN1 外设句柄。定义在 Core/Src/can.c 的 MX_CAN1_Init() 里。
extern DMA_HandleTypeDef hdma_usart3_rx;
// [解析] ↑ 专门给"USART3 接收"配的 DMA 通道。定义在 Core/Src/usart.c 里。
// [解析]   遥控器数据就是靠它自动搬进内存的，不需要 CPU 一个个字节去读。
extern UART_HandleTypeDef huart3;
// [解析] ↑ USART3 句柄（接 DBus 遥控器接收机）。
extern TIM_HandleTypeDef htim1;
// [解析] ↑ TIM1 句柄。这里把它当时基用（见文件末尾的 TIM1_UP_TIM10_IRQHandler）。

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
// [解析] 上面这段分隔线以下，是 CPU 内核自身（Cortex-M4）的异常处理函数，
// [解析] 它们和外设无关，任何一个 STM32 工程都有这几个。名字必须与启动文件
// [解析] （startup_stm32f407xx.s）里定义的向量表一致，写错了就跳不进来。
/**
  * @brief This function handles Non maskable interrupt.
  */
// [解析] NMI = Non-Maskable Interrupt，不可屏蔽中断，优先级最高、无法被关闭。
// [解析] 正常运行时永远不会进这里。一旦进来通常意味着硬件出了严重问题。
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  // [解析] 死循环"卡死"在这里。这是一种故意的保护策略：
  // [解析] 与其带着未知错误继续跑（可能乱转电机），不如停下来。
  // [解析] 用调试器连上就能看到程序停在这一行。
   while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
// [解析] ★ HardFault 是调试阶段最常见的"崩溃"入口，务必认识它。
// [解析] 什么情况会进来：访问了非法地址、空指针解引用、除零、栈溢出、
// [解析]   往只读区写数据、中断里调用了不允许的函数等等。
// [解析] 排查方法：用调试器暂停，看调用栈(Call Stack)找到崩溃前最后执行的函数；
// [解析]   也可以在下面 while(1) 处打断点，再看 LR/PC 寄存器定位。
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
// [解析] MemManage：内存管理错误（访问了 MPU 保护的区域）。
// [解析] 本工程没有启用 MPU，所以实际上不会进来，属于 CubeMX 的默认模板。
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
// [解析] BusFault：总线错误，比如访问了没有外设响应的地址。
// [解析] 常见诱因：外设时钟忘记使能、指针指向了错误的外设基地址。
// [解析] 同样本工程未单独启用，一般会合并到 HardFault 处理。
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
// [解析] UsageFault：用法错误，例如执行了未定义的指令、
// [解析] 除零（需开启 DIV_0_TRP）、非对齐访问（需开启 UNALIGN_TRP）。
// [解析] 对使用 __attribute__((packed)) 结构体的代码，这个异常值得留意。
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Debug monitor.
  */
// [解析] 调试监视器异常，只有在打开调试功能时才可能触发。函数体是空的，正常。
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/
// [解析] ★★ 下面才是"本工程真正用到的外设中断"，每个都对应你应用代码里的某段逻辑。
// [解析] 请重点理解这条"中断调用链"，它是整个工程数据流动的关键：
// [解析]   硬件事件 -> 本文件的 xxx_IRQHandler() -> HAL_xxx_IRQHandler()
// [解析]            -> HAL 内部判断标志位 -> 回调到 applications/ 里的业务函数
// [解析] 每个函数上面的注释都标出了"最终会调用到你写的哪个函数"。

/**
  * @brief This function handles EXTI line4 interrupt.
  */
// [解析] 外部中断线 4，对应引脚【PC4】。依据：main.h 里 INT1_ACCEL_Pin = GPIO_PIN_4，
// [解析] 而 INT1_ACCEL_GPIO_Port = GPIOC，所以是 PC4 而不是 PA4。
// [解析] ★ 这里极易看错，务必分清这一对"同号不同端口"的引脚：
// [解析]   PA4 = IMU_CS1_ACCEL —— 加速度计的【片选】输出脚，由 SPI 驱动控制，
// [解析]                             它是"主机告诉传感器我要跟你说话了"。
// [解析]   PC4 = INT1_ACCEL   —— 加速度计的【中断】输出脚，由传感器主动拉电平，
// [解析]                             它是"传感器告诉主机我有新数据了"。
// [解析]   两者都是 GPIO_PIN_4，但一个在 GPIOA、一个在 GPIOC，功能完全相反。
// [解析] 所以本函数是被 PC4 上的电平跳变触发的，和 PA4 的片选没有任何关系。
// [解析] 最终调用链：EXTI4_IRQHandler -> HAL_GPIO_EXTI_IRQHandler
// [解析]            -> HAL_GPIO_EXTI_Callback(INT1_ACCEL_Pin)
// [解析] 说明：本工程目前并没有实现 HAL_GPIO_EXTI_Callback，
// [解析]       所以其实没有用到这个中断（IMU 是靠 imu_task 定时轮询读的）。
// [解析]       它是 CubeMX 根据引脚配置自动生成的。
void EXTI4_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI4_IRQn 0 */

  /* USER CODE END EXTI4_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(INT1_ACCEL_Pin);
  /* USER CODE BEGIN EXTI4_IRQn 1 */

  /* USER CODE END EXTI4_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream1 global interrupt.
  */
// [解析] DMA1 的 Stream1 中断 —— 这是【DBus 遥控器接收】的关键一环。
// [解析] 数据流：遥控器 -> USART3 引脚 -> DMA 自动把字节搬进内存缓冲区
// [解析]        -> 一帧搬完或串口空闲 -> DMA/串口触发中断 -> 进入本函数
// [解析] 最终调用链：HAL_DMA_IRQHandler(&hdma_usart3_rx)
// [解析]            -> HAL_UARTEx_RxEventCallback(huart, Size)
// [解析]            -> applications/uart_task.cpp 里的 remote.update(Size, stamp_ms)
// [解析] 注意 hdma_usart3_rx 是"USART3 接收专用"的，
// [解析] 传错句柄会导致 DMA 状态机错乱、接收直接卡死。
void DMA1_Stream1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream1_IRQn 0 */

  /* USER CODE END DMA1_Stream1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart3_rx);
  /* USER CODE BEGIN DMA1_Stream1_IRQn 1 */

  /* USER CODE END DMA1_Stream1_IRQn 1 */
}

/**
  * @brief This function handles CAN1 RX0 interrupts.
  */
// [解析] CAN1 的"接收 FIFO0 非空"中断 —— 这是【电机数据回传】的入口。
// [解析] 最终调用链：CAN1_RX0_IRQHandler -> HAL_CAN_IRQHandler(&hcan1)
// [解析]            -> HAL_CAN_RxFifo0MsgPendingCallback(hcan)
// [解析]            -> applications/can_task.cpp 里你自己写的那个回调
// [解析]               （在里头循环 can1.recv() 取帧，再交给 motor6020.read() 解析）
// [解析] 这也解释了为什么 can_task.cpp 里必须写 HAL_CAN_RxFifo0MsgPendingCallback：
// [解析] 那个函数就是 HAL 从本文件这里回调过去的。
void CAN1_RX0_IRQHandler(void)
{
  /* USER CODE BEGIN CAN1_RX0_IRQn 0 */

  /* USER CODE END CAN1_RX0_IRQn 0 */
  HAL_CAN_IRQHandler(&hcan1);
  /* USER CODE BEGIN CAN1_RX0_IRQn 1 */

  /* USER CODE END CAN1_RX0_IRQn 1 */
}

/**
  * @brief This function handles EXTI line[9:5] interrupts.
  */
// [解析] 外部中断线 5~9 共用一个中断向量，所以这里处理的是这个范围内的引脚。
// [解析] 本工程用到的是 INT1_GYRO_Pin，即【PC5】引脚（陀螺仪的中断输出脚）。
// [解析] 和上面的 EXTI4 一样，本工程并没有实现对应的回调，
// [解析] 陀螺仪数据是靠 imu_task 定时用 SPI 读的，所以此中断实际未启用。
void EXTI9_5_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI9_5_IRQn 0 */

  /* USER CODE END EXTI9_5_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(INT1_GYRO_Pin);
  /* USER CODE BEGIN EXTI9_5_IRQn 1 */

  /* USER CODE END EXTI9_5_IRQn 1 */
}

/**
  * @brief This function handles TIM1 update interrupt and TIM10 global interrupt.
  */
// [解析] ★★ 这个中断是【整个 FreeRTOS 系统的"心跳"】，非常重要。
// [解析] 正常写法下 HAL 库用 SysTick 定时器做 1ms 时基，
// [解析] 但 FreeRTOS 已经独占了 SysTick（要用它做任务调度的时间基准），
// [解析] 所以本工程改用一个普通定时器 TIM1 来当 HAL 的时基。
// [解析] 最终调用链：TIM1_UP_TIM10_IRQHandler -> HAL_TIM_IRQHandler(&htim1)
// [解析]            -> HAL_TIM_PeriodElapsedCallback(htim)
// [解析]            -> Core/Src/main.c 里那个回调：if (htim->Instance == TIM1) HAL_IncTick();
// [解析] HAL_IncTick() 让全局变量 uwTick 每 1ms 加一，
// [解析] 于是 HAL_Delay()、各种 osDelay/超时判断才有时间概念。
// [解析] 如果这个中断被关掉或配置错了，现象是：程序能跑，但所有延时都失灵。
void TIM1_UP_TIM10_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_UP_TIM10_IRQn 0 */

  /* USER CODE END TIM1_UP_TIM10_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_UP_TIM10_IRQn 1 */

  /* USER CODE END TIM1_UP_TIM10_IRQn 1 */
}

/**
  * @brief This function handles USART3 global interrupt.
  */
// [解析] USART3 的全局中断（集成了"串口空闲检测 IDLE"等事件）。
// [解析] 它专门服务于 DBus 遥控器接收，和上面的 DMA1_Stream1 事件是配合关系：
// [解析]   DMA 负责"搬数据"，串口 IDLE 中断负责判断"这一帧什么时候结束"。
// [解析] 最终调用链：USART3_IRQHandler -> HAL_UART_IRQHandler(&huart3)
// [解析]            -> HAL_UARTEx_RxEventCallback(huart, Size)   [收到一帧]
// [解析]            -> HAL_UART_ErrorCallback(huart)             [出错时]
// [解析]            -> 都在 applications/uart_task.cpp 里实现
void USART3_IRQHandler(void)
{
  /* USER CODE BEGIN USART3_IRQn 0 */

  /* USER CODE END USART3_IRQn 0 */
  HAL_UART_IRQHandler(&huart3);
  /* USER CODE BEGIN USART3_IRQn 1 */

  /* USER CODE END USART3_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
