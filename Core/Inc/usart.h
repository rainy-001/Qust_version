// [解析] ==========================================================================
// [解析] 【文件】Core/Inc/usart.h
// [解析] 【作用】USART3（串口 3）外设的"对外声明"头文件，里面只有两样东西：
// [解析]         1) 全局句柄 extern UART_HandleTypeDef huart3;（变量本体在 usart.c）
// [解析]         2) 初始化函数原型 void MX_USART3_UART_Init(void);（函数本体也在 usart.c）
// [解析] 【谁生成】CubeMX 自动生成。作者没有手改过本文件。
// [解析] 【本工程用途】USART3 = PB10(TX) / PB11(RX)，接 DBus 遥控器接收机，
// [解析]   波特率是 100000（注意：不是常见的 115200！这是 DBus 协议规定的专用速率），
// [解析]   数据格式 8N1。接收用 DMA1_Stream1 + 空闲中断，上层代码在
// [解析]   applications/uart_task.cpp，协议解析在 sp_middleware/io/dbus/dbus.cpp。
// [解析] 【关键点】① 100000 这个波特率写错（比如写成 115200）会表现为"一个字节都收不到"，
// [解析]             而不是收到乱七八糟的数据，排查时优先看这里；
// [解析]           ② huart3 的 DMA 句柄 hdma_usart3_rx 定义在 usart.c，本头文件没有声明它，
// [解析]             但 Core/Src/stm32f4xx_it.c 里用 extern 直接引用了它。
// [解析] ==========================================================================

/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.h
  * @brief   This file contains all the function prototypes for
  *          the usart.c file
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
// [解析]   ▼ 下面是必须先搞清楚的机制：
// [解析] 【CubeMX 用户代码保护区】凡是成对出现的
// [解析]     /* USER CODE BEGIN xxx */  ...  /* USER CODE END xxx */
// [解析]   中间就是"用户代码保护区"。CubeMX 重新 GENERATE CODE 时保护区内的内容被保留，
// [解析]   保护区外的一切按 .ioc 配置重新生成（手改会丢）。本文件的保护区目前都是空的。
/* Define to prevent recursive inclusion -------------------------------------*/
// [解析] ↓ 条件编译"包含卫士"，防止本头文件被重复展开导致重复定义。
#ifndef __USART_H__
#define __USART_H__

// [解析] ↓ extern "C"：工程上层是 C++（applications/ 下是 .cpp），这里是 C 代码。
// [解析]   没有这个包裹，C++ 会按改编后的函数名去找 MX_USART3_UART_Init，链接时报未定义。
// [解析]   配对 #endif 在文件末尾。
#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
// [解析] main.h 里包含 stm32f4xx_hal.h（HAL 总头），UART_HandleTypeDef 由它提供。
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */
// [解析] ↑ 上面三对 BEGIN/END（Includes / Private defines / Prototypes）是 CubeMX 留的保护区，
// [解析]   分别放"自己 include 的头文件 / 自己的宏 / 自己的函数原型"。本工程都是空的。

extern UART_HandleTypeDef huart3;
// [解析] ↑ extern = 只声明不定义；变量本体在 Core/Src/usart.c：UART_HandleTypeDef huart3;
// [解析]   UART_HandleTypeDef（串口在 HAL 里统一叫 UART 句柄，USART 只是硬件外设名）主要成员：
// [解析]     Instance  —— 指向 USART3 的寄存器组；
// [解析]     Init      —— 波特率、数据位、停止位、校验、过采样等（在 MX_USART3_UART_Init 里填）；
// [解析]     hdmarx    —— 绑定的接收 DMA 句柄（由 __HAL_LINKDMA 挂钩，见 usart.c）；
// [解析]     pRxBuffPtr / RxXferSize —— 当前接收缓冲的位置和长度；
// [解析]     State / ErrorCode —— HAL 运行时状态（比如 HAL_UART_STATE_BUSY_RX）。
// [解析]   上层用法：sp::DBus remote(&huart3); 在 uart_task.cpp 里创建遥控器对象。
// [解析]   注意：本工程没有 huart1（调试串口 PA9/PA10）、也没有 huart6（裁判系统），
// [解析]   因为 applications/ 里没有用到它们的代码。

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

// [解析] ★ MX_USART3_UART_Init：USART3 初始化函数，实现见 Core/Src/usart.c。参数逐条解释：
// [解析]   BaudRate = 100000  ★这是本函数最需要注意的一个数：DBus 遥控器接收机的专用波特率，
// [解析]     不是通用的 115200。写错的话表现是"完全收不到数据"，不会报编译错误。
// [解析]     HAL 会用它算出波特率寄存器 BRR：本工程 USART3 挂 APB1 = 42 MHz，
// [解析]     过采样 16 倍时 BRR = 42 MHz / (16 × 100000) = 26.25
// [解析]       -> 整数部分 26(0x1A)、小数部分 0.25 × 16 = 4 -> BRR = 0x1A4。
// [解析]     实际波特率 = 42 MHz / (16 × 26.25) = 正好 100000，误差 0，所以通信很稳。
// [解析]   WordLength = 8B、StopBits = 1、Parity = NONE  -> 合起来就是常说的 "8N1"；
// [解析]     ★注意 DBus 的 18 字节帧里其实自带校验，硬件校验位关掉是协议要求；
// [解析]   Mode = TX_RX        收发都开（遥控器实际只用 RX，但保留 TX 方便调试）；
// [解析]   HwFlowCtl = NONE   不用硬件流控（RTS/CTS），只有 2 根线；
// [解析]   OverSampling = 16  16 倍过采样（标准做法；改成 8 倍能提高上限速率，但抗噪差）。
// [解析] HAL_UART_Init() 内部会自动回调 HAL_UART_MspInit()，那里还做了三件重要的事：
// [解析]   1) 使能 USART3 时钟，把 PB10/PB11 配成 AF7_USART3；
// [解析]   2) 配置并初始化接收 DMA（hdma_usart3_rx = DMA1_Stream1 / Channel4 / 循环模式），
// [解析]      再用 __HAL_LINKDMA(uartHandle, hdmarx, hdma_usart3_rx) 把它挂到串口句柄上
// [解析]      —— 挂钩之后，上层调用 HAL_UARTEx_ReceiveToIdle_DMA(&huart3, ...) 时，
// [解析]        HAL 才知道该用哪个 DMA 去搬数据；
// [解析]   3) 打开 USART3 全局中断（USART3_IRQn，抢占优先级 5）—— 空闲中断(IDLE)靠它触发，
// [解析]      没有这个中断，"一帧收完"的事件就永远不会通知到程序。
void MX_USART3_UART_Init(void);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

// [解析] ↑ 这对 #ifdef __cplusplus / #endif 结束了开头的 extern "C" 块。
// [解析] ↓ 下面这行结束开头的 #ifndef __USART_H__ 包含卫士；两对标记必须完整成对。
#endif /* __USART_H__ */

