// [解析] ==========================================================================
// [解析] 【文件】Core/Inc/dma.h
// [解析] 【作用】DMA（Direct Memory Access，直接内存访问）总开关的声明文件。
// [解析]         它只声明了一个函数：void MX_DMA_Init(void);
// [解析] 【谁生成】CubeMX 自动生成。作者没有手改过本文件。
// [解析] 【DMA 是什么】一种"硬件搬运工"：配置好之后，外设和内存之间的数据搬运
// [解析]               由 DMA 控制器自己完成，CPU 不用一个字节一个字节去读寄存器。
// [解析]               本工程用它来收 DBus 遥控器数据（USART3 + DMA1_Stream1），
// [解析]               这样 100 kHz 的串口数据流不会把 CPU 拖住。
// [解析] 【关键点】① 本文件里的 MX_DMA_Init 只做两件事：打开 DMA1 的时钟、
// [解析]             打开 DMA1_Stream1 的中断（见 Core/Src/dma.c）；
// [解析]           ② 具体的"搬到哪、搬多少、循环还是单次"这些参数不在 dma.c 里，
// [解析]             而是在 Core/Src/usart.c 的 HAL_UART_MspInit() 里配置 hdma_usart3_rx。
// [解析]             这是新手最容易找错地方的一个点：DMA 通道是"跟着外设走"的。
// [解析] ==========================================================================

/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    dma.h
  * @brief   This file contains all the function prototypes for
  *          the dma.c file
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
// [解析] ↑ 上面这段是 CubeMX 生成的固定版权注释，对编译没有任何作用。
// [解析]   ▼ 下面是必须先搞清楚的机制：
// [解析] 【CubeMX 用户代码保护区】凡是成对出现的
// [解析]     /* USER CODE BEGIN xxx */  ...  /* USER CODE END xxx */
// [解析]   中间的内容就是"用户代码保护区"。在 CubeMX 里重新 GENERATE CODE 时：
// [解析]   保护区内的内容会被保留，保护区外的一切都会被重新生成覆盖。
// [解析]   所以自己写的代码必须写在保护区里面。本文件的保护区目前都是空的。
/* Define to prevent recursive inclusion -------------------------------------*/
// [解析] ↓ 条件编译"包含卫士"，防止本头文件被重复包含导致的重复定义。__DMA_H__ 只在本文件用。
#ifndef __DMA_H__
#define __DMA_H__

// [解析] ↓ extern "C"：本工程上层是 C++，这里导出的是 C 函数。见 can.h 里的同类说明。
// [解析]   配对的 #endif 在文件末尾。
#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
// [解析] main.h 里包含了 stm32f4xx_hal.h（HAL 总头），DMA_HandleTypeDef 之类的类型都由它提供。
#include "main.h"

/* DMA memory to memory transfer handles -------------------------------------*/
// [解析] ↑ 这一行是 CubeMX 的模板固定标题，意思是"下面会放内存到内存的 DMA 句柄"。
// [解析]   本工程没有任何"内存到内存"的 DMA（那种用途常见于刷屏、大批量拷贝），
// [解析]   唯一的 DMA 是"外设到内存"（USART3 接收），所以这里下面是空的，属于正常情况。

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

// [解析] ★ MX_DMA_Init：DMA 控制器初始化函数，实现见 Core/Src/dma.c。它只做两件事：
// [解析]   1) __HAL_RCC_DMA1_CLK_ENABLE() —— 打开 DMA1 控制器的时钟
// [解析]      （注意：STM32F4 的 DMA1/DMA2 时钟挂在 AHB1 上，不使能时钟，DMA 完全不工作）；
// [解析]   2) HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 5, 0) + HAL_NVIC_EnableIRQ(...)
// [解析]      —— 设置并打开 DMA1 数据流 1 的中断，抢占优先级 5、子优先级 0。
// [解析]   ▼ 为什么偏偏是 1 号数据流：
// [解析] 【为什么是 Stream1】STM32F4 的 DMA 是"数据流(Stream) + 通道(Channel)"两级结构：
// [解析]   每个数据流可以接 8 个通道之一，而"哪个外设的请求只能接哪个通道"是芯片固定的。
// [解析]   USART3_RX 只能走 DMA1_Stream1 的 Channel 4，所以这里和第 1 号数据流绑死了。
// [解析]   ▼ 调用时机（顺序错了会卡死）：
// [解析] 【调用时机】main.c 的 main() 里，MX_DMA_Init() 必须在 MX_USART3_UART_Init() 之前调用。
// [解析]   因为 USART3 的 MspInit 里要初始化并绑定 hdma_usart3_rx，那时 DMA1 时钟必须已经开好，
// [解析]   顺序反了会卡在 HAL_DMA_Init 里（HAL 会一直等 DMA 状态就绪）。
// [解析]   CubeMX 生成的 main() 里已经把这个顺序排好了。
// [解析]   ▼ 具体搬运参数在哪个文件里配：
// [解析] 【DMA1_Stream1 的搬运参数在哪】在 Core/Src/usart.c 的 HAL_UART_MspInit() 里：
// [解析]   Channel = DMA_CHANNEL_4，Direction = 外设->内存，PeriphInc 关，
// [解析]   MemInc 开（内存地址递增，这样 18 个字节能连续落进数组），
// [解析]   数据宽度都是字节(Byte)，Mode = DMA_CIRCULAR（循环模式，收满自动从头再来），
// [解析]   Priority = HIGH，FIFO 模式关闭。这些是 DBus 接收能正常工作的前提。
void MX_DMA_Init(void);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

// [解析] ↑ 这对 #ifdef __cplusplus / #endif 结束了开头的 extern "C" 块；
// [解析] ↓ 下面这行结束开头的 #ifndef __DMA_H__ 包含卫士。两对标记都必须完整成对。
#endif /* __DMA_H__ */

