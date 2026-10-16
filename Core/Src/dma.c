// [解析] ==========================================================================
// [解析] 【文件】Core/Src/dma.c
// [解析] 【作用】给 DMA 控制器"开总开关"：使能 DMA1 的时钟，并打开本工程用到的那路 DMA 中断。
// [解析] 【谁生成】CubeMX 自动生成，作者没有改动逻辑。
// [解析] 【关键点】
// [解析]   1) 本文件没有配置任何"搬运任务"，具体哪条通道搬什么数据是在 usart.c 里配的；
// [解析]   2) 它必须排在 MX_USART3_UART_Init() 之前被调用（main.c 里已经这么排了），
// [解析]      否则 DMA 时钟还没开就去配通道，配置会静默失效；
// [解析]   3) 文件开头 @brief 写的是 "memory to memory transfers"（内存到内存），
// [解析]      那只是 CubeMX 模板的固定文案，本工程并没有内存到内存的搬运，别被它误导。
// [解析] ==========================================================================

/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    dma.c
  * @brief   This file provides code for the configuration
  *          of all the requested memory to memory DMA transfers.
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
#include "dma.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure DMA                                                              */
/*----------------------------------------------------------------------------*/
// [解析] 上面这个方框是 CubeMX 生成的"代码分区标题"，它下面的 USER CODE BEGIN 1 是配套的保护区。
// [解析] 保护区机制：在 CubeMX 里重新 GENERATE CODE 时，只有夹在 BEGIN/END 之间的内容会被保留，
// [解析] 标记之外的一切都会被重新生成的代码覆盖掉 —— 所以自己写的代码一定要写在里面。

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/**
  * Enable DMA controller clock
  */
// [解析] 使能 DMA 控制器的时钟。DMA 虽然是独立于 CPU 的"搬运工"，但它同样是一个外设，
// [解析] 必须先开时钟才能配置和使用。
// [解析] ★ 忘了这一句的典型症状：串口看起来在收数据，但 DMA 缓冲区里的内容永远不变，
// [解析]   而且不报错，非常难查 —— 所以 main.c 里的顺序是 MX_DMA_Init() 在前、
// [解析]   MX_USART3_UART_Init() 在后。本函数由 main.c 调用，在 FreeRTOS 启动之前完成。
void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  // [解析] DMA1 只有"一个总时钟开关"（不是每个 Stream 一个），开一次即可。
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream1_IRQn interrupt configuration */
  // [解析] 打开 DMA1_Stream1 的中断。Stream1 就是"USART3 接收"用的那条通道
  // [解析]   （在 Core/Src/usart.c 的 HAL_UART_MspInit() 里绑定到 huart3.hdmarx）。
  // [解析]   DMA 中断用来通知"搬到一半/搬满一圈"，或者报告传输错误。
  // [解析] 参数 (抢占优先级 5, 子优先级 0)：本工程 NVIC 分组是 NVIC_PRIORITYGROUP_4
  // [解析]   （HAL_Init 里设置，4 位抢占 / 0 位子优先级），所以子优先级不起作用；
  // [解析]   抢占优先级 5 与 USART3、CAN1 相同，是"允许调用 FreeRTOS API"的级别。
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  // [解析] 注意：本工程只用了 DMA1_Stream1 这一路，所以这里也只开了一个中断。
  // [解析]   以后如果新增别的 DMA 通道（比如串口发送 DMA、ADC 的 DMA），
  // [解析]   CubeMX 会在这个位置自动补上对应的几行。

}

/* USER CODE BEGIN 2 */
// [解析] 用户代码保护区（同上）：重新生成代码时只有这里面的内容会被保留。本工程保持空白。
/* USER CODE END 2 */

