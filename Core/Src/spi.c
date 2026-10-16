// [解析] ==========================================================================
// [解析] 【文件】Core/Src/spi.c
// [解析] 【作用】配置 SPI1，用于和 BMI088 六轴 IMU（加速度计 + 陀螺仪）通信。
// [解析] 【谁生成】STM32CubeMX 自动生成，作者没有改动过这里的逻辑。
// [解析]         上层的使用代码：applications/imu_task.cpp → sp_middleware 里的 BMI088 驱动。
// [解析] 【关键点】
// [解析]   1) CPOL/CPHA 组成 SPI 的 4 种"模式"，BMI088 只认其中两种，配错就通信失败；
// [解析]   2) 波特率分频：APB2 84 MHz / 8 = 10.5 Mbit/s，已经略高于 BMI088 的 10 MHz 上限；
// [解析]   3) 片选 NSS 用的是"软件模式"，也就是 CS 引脚由 GPIO 手动控制（PA4 加速度计 / PB0 陀螺仪）；
// [解析]   4) 引脚 PB3/PB4 复位后默认属于 JTAG 调试口，这个坑在 MspInit 那段里有说明。
// [解析] ==========================================================================

/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    spi.c
  * @brief   This file provides code for the configuration
  *          of the SPI instances.
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
#include "spi.h"

// [解析] ↑ spi.h 里声明了句柄 hspi1 和 MX_SPI1_Init() 的原型。

// [解析] ↓ /* USER CODE BEGIN 0 */ ... /* USER CODE END 0 */ 是 CubeMX 的"用户代码保护区"。
// [解析]   机制：在 CubeMX 里重新 GENERATE CODE 时，只有夹在这对标记之间的内容会被保留，
// [解析]   标记之外的一切都会被按 .ioc 的配置重新生成、覆盖掉。自己写的代码一定要放进保护区。
// [解析]   编号（0/1/2...）只用于区分同一文件里的多块保护区，必须成对使用。
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

// [解析] SPI1 的 HAL 句柄（全局变量）。记录寄存器基地址、初始化参数与运行时状态；
// [解析] 所有 HAL_SPI_xxx(&hspi1, ...) 都靠它定位硬件，BMI088 驱动拿到的就是 &hspi1。
SPI_HandleTypeDef hspi1;

/* SPI1 init function */
// [解析] MX_SPI1_Init()：把 SPI1 配置成"主机 + 全双工 + 8 位 + Mode 3"的通信模式。
// [解析] 参数：无。返回值：无。失败会进 Error_Handler() 死循环（实现在 main.c）。
// [解析] 调用位置：main.c 的 main() 里（CubeMX 生成的调用顺序）。
// [解析] 注意：本函数只做配置，不产生任何通信；收发由 HAL_SPI_TransmitReceive() 等函数发起。
void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  // [解析] ↑ SPI1 的 0 号保护区，默认留空，位置在"填参数之前"。
  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  // [解析] ↓ 下面是把 SPI1 的参数填进句柄，填完由 HAL_SPI_Init() 一次性写进寄存器。
  // [解析]   对照 BMI088 数据手册来理解每一项（IMU 是 SPI 总线上唯一的从机）：
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;               // [解析] 主机模式：时钟(SCK)由 STM32 产生，BMI088 是被动的从机
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;     // [解析] 全双工：MOSI 发、MISO 收，两根线同时工作（读寄存器就是这种用法）
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;         // [解析] 每次收发 8 位（1 字节）。BMI088 的寄存器都是 8 位宽
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;      // [解析] CPOL=1：空闲时 SCK 保持高电平
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;           // [解析] CPHA=1：在 SCK 的第 2 个边沿采样数据
  // [解析] ↑ CPOL=1 + CPHA=1 合起来叫 SPI 模式 3（Mode 3）。BMI088 支持 Mode 0 和 Mode 3，
  // [解析]   如果通信读不到数据，第一件要核对的就是这里和从机要求是否一致。
  hspi1.Init.NSS = SPI_NSS_SOFT;                   // [解析] 片选用"软件模式"：不用硬件 NSS 引脚，改由 GPIO 手动拉低/拉高
  // [解析] ↑ 这就是为什么工程里还要额外配 PA4 / PB0 两个 GPIO 作片选：
  // [解析]   PA4 = 加速度计的 CS，PB0 = 陀螺仪的 CS，它们都在 Core/Src/gpio.c 里初始化，
  // [解析]   由 BMI088 驱动在每次读写前拉低、读完拉高。硬件 NSS 无法同时管两个从机。
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;  // [解析] 波特率分频
  // [解析] ↑ 换算：SPI1 挂在 APB2 上，APB2 = 84 MHz（SPI 不像定时器那样有 ×2 规则，
  // [解析]   直接就是总线频率）。所以 SCK = 84 MHz / 8 = 10.5 Mbit/s。
  // [解析]   注意：BMI088 的 SPI 时钟上限是 10 MHz，这里 10.5 Mbit/s 已经略微超了。
  // [解析]        如果出现读寄存器偶发错值，把分频改成 SPI_BAUDRATEPRESCALER_16（= 5.25 Mbit/s）更保险。
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;          // [解析] 高位先发（MSB First），BMI088 规定如此，不能改 LSB
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;          // [解析] 关闭 TI 的"同步串行协议"兼容模式，普通 SPI 用不到
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;  // [解析] 关闭硬件 CRC 校验（BMI088 通信不需要）
  hspi1.Init.CRCPolynomial = 10;                   // [解析] CRC 多项式，因为上面 CRC 已关闭，这个值填什么都不生效
  // [解析] ↓ 把参数写进 SPI1 寄存器，并回调 HAL_SPI_MspInit() 打开 SPI1 与 GPIO 时钟、配好引脚复用。
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  // [解析] ↑ 2 号保护区：SPI1 已经配置完成，适合放"上电自检、读 WHO_AM_I 芯片 ID"之类的代码。
  /* USER CODE END SPI1_Init 2 */

}

// [解析] HAL_SPI_MspInit()：SPI 的底层初始化回调，由 HAL_SPI_Init() 自动调用
// [解析] （MSP = MCU Support Package，MCU 支持包）。
// [解析] HAL 库只负责外设自身寄存器，而"开时钟、配引脚复用、配 NVIC、绑 DMA"这些
// [解析] 与芯片/板子强相关的动作，都由 HAL 通过 MSP 回调交给用户代码实现。
// [解析] 参数 spiHandle 用来区分是哪一路 SPI；本工程只有 SPI1。
void HAL_SPI_MspInit(SPI_HandleTypeDef* spiHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(spiHandle->Instance==SPI1)
  {
  /* USER CODE BEGIN SPI1_MspInit 0 */

  /* USER CODE END SPI1_MspInit 0 */
    /* SPI1 clock enable */
    __HAL_RCC_SPI1_CLK_ENABLE();  // [解析] 打开 SPI1 外设时钟（APB2）。时钟不开，后面配得再对也不通

    __HAL_RCC_GPIOB_CLK_ENABLE();  // [解析] PB3/PB4（SCK/MISO）属于 GPIOB，先开 GPIOB 时钟
    __HAL_RCC_GPIOA_CLK_ENABLE();  // [解析] PA7（MOSI）属于 GPIOA，再开 GPIOA 时钟
    /**SPI1 GPIO Configuration
    PB4     ------> SPI1_MISO
    PB3     ------> SPI1_SCK
    PA7     ------> SPI1_MOSI
    */
    // [解析] ↑ 引脚分配是硬件（C 板原理图）定死的，不能随意改：
    // [解析]   PB3 = SCK（主机给从机的时钟）、PB4 = MISO（从机发给主机）、PA7 = MOSI（主机发给从机）。
    // [解析]   注意：docs/C板引脚配置说明.md 第 27 行把 MISO 写成了 PA6，与本文件、
    // [解析]   以及 Qust_version.ioc 里的 PB4.Signal=SPI1_MISO 不一致，以代码/原理图为准。
    // [解析] 另一个新手必踩的坑：PB3/PB4 复位后的默认身份是 JTAG 调试口（PB3=JTDO、PB4=NJTRST）。
    // [解析]   若要把它们当 SPI 用，一般需要在 HAL_MspInit() 里调用 __HAL_AFIO_REMAP_SWJ_NOJTAG()
    // [解析]   关掉 JTAG（保留 SWD 下载）。本工程 Core/ 目录里没有找到这句调用，
    // [解析]   如果 IMU 一直读不到数据，可以优先怀疑这里。
    // [解析] ↓ 各字段含义：
    // [解析]   GPIO_MODE_AF_PP        —— 复用推挽输出，把引脚交给 SPI 外设驱动
    // [解析]   GPIO_NOPULL            —— 不用内部上/下拉（SPI 是推挽驱动，不需要）
    // [解析]   GPIO_SPEED_FREQ_VERY_HIGH —— 引脚翻转速率选"非常高"。SPI 有 10.5 MHz，
    // [解析]                              用低速档会导致边沿变缓、波形失真、通信出错，
    // [解析]                              所以这里必须用高速档（和 PWM 那种慢信号正好相反）。
    // [解析]   Alternate = GPIO_AF5_SPI1 —— 复用功能号 AF5，对应 SPI1。
    GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    // [解析] PA7（MOSI）单独再配一次，因为它在另一个 GPIO 端口上。
    // [解析] HAL_GPIO_Init 是按"端口"批量配置的，同一端口可以一次配多个引脚，不同端口要分开调。
    GPIO_InitStruct.Pin = GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN SPI1_MspInit 1 */

  /* USER CODE END SPI1_MspInit 1 */
  }
}

// [解析] HAL_SPI_MspDeInit()：MspInit 的逆操作 —— 关 SPI1 时钟、把三个引脚恢复成复位默认状态。
// [解析] 只有调用 HAL_SPI_DeInit() 时才会执行。本工程不会关闭 SPI，所以它实际不会被跑到。
void HAL_SPI_MspDeInit(SPI_HandleTypeDef* spiHandle)
{

  if(spiHandle->Instance==SPI1)
  {
  /* USER CODE BEGIN SPI1_MspDeInit 0 */

  /* USER CODE END SPI1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_SPI1_CLK_DISABLE();  // [解析] 关闭 SPI1 时钟

    /**SPI1 GPIO Configuration
    PB4     ------> SPI1_MISO
    PB3     ------> SPI1_SCK
    PA7     ------> SPI1_MOSI
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_4|GPIO_PIN_3);  // [解析] 释放 PB3/PB4

    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_7);  // [解析] 释放 PA7

  /* USER CODE BEGIN SPI1_MspDeInit 1 */

  /* USER CODE END SPI1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

// [解析] ↑ 文件末尾的保护区，可放本文件自己的代码，当前为空。
/* USER CODE END 1 */

