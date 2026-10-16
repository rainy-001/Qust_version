// [解析] ==========================================================================
// [解析] 【文件】Core/Inc/spi.h
// [解析] 【作用】SPI1 外设的"对外声明"头文件，里面只有两样东西：
// [解析]         1) 全局句柄 extern SPI_HandleTypeDef hspi1;（变量本体在 spi.c）
// [解析]         2) 初始化函数原型 void MX_SPI1_Init(void);（函数本体也在 spi.c）
// [解析] 【谁生成】CubeMX 自动生成。作者没有手改过本文件。
// [解析] 【本工程用途】SPI1 用来读 IMU（BMI088 六轴传感器），接线：
// [解析]   PB3 = SCK、PB4 = MISO、PA7 = MOSI，另有 PA4/PB0 两个软件片选（在 gpio.c 里配）。
// [解析]   上层代码：sp_middleware/io/bmi088/bmi088.cpp 用 HAL_SPI_TransmitReceive() 读写寄存器。
// [解析] 【关键点】① BMI088 是"双芯片"结构（加速度计 + 陀螺仪各一个片选），
// [解析]             所以 SPI 的 NSS 必须设成"软件管理"，靠 GPIO 手动拉低哪个片选；
// [解析]           ② SPI 时钟 10.5 MHz 略微超过 BMI088 手册的 10 MHz 上限，见文件末尾提示。
// [解析] ==========================================================================

/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    spi.h
  * @brief   This file contains all the function prototypes for
  *          the spi.c file
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
// [解析]   中间就是"用户代码保护区"。CubeMX 重新 GENERATE CODE 时保护区内的内容保留，
// [解析]   保护区外的一切按 .ioc 配置重新生成（手改会丢）。本文件的保护区目前都是空的。
/* Define to prevent recursive inclusion -------------------------------------*/
// [解析] ↓ 条件编译"包含卫士"，保证本头文件只被展开一次。
#ifndef __SPI_H__
#define __SPI_H__

// [解析] ↓ extern "C"：工程上层是 C++（applications/ 下是 .cpp），这里是 C 代码，
// [解析]   不加这个包裹，链接时会因为 C++ 的函数名改编而找不到 MX_SPI1_Init。
// [解析]   配对 #endif 在文件末尾。
#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
// [解析] main.h 里包含 stm32f4xx_hal.h（HAL 总头），SPI_HandleTypeDef 类型由它提供。
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */
// [解析] ↑ 上面三对 BEGIN/END（Includes / Private defines / Prototypes）是 CubeMX 留的保护区，
// [解析]   分别放"自己 include 的头文件 / 自己的宏 / 自己的函数原型"。本工程都是空的。

extern SPI_HandleTypeDef hspi1;
// [解析] ↑ extern = 只声明不定义，变量本体在 Core/Src/spi.c：SPI_HandleTypeDef hspi1;
// [解析]   SPI_HandleTypeDef 是 HAL 的 SPI 句柄结构体：
// [解析]     Instance —— 指向 SPI1 的寄存器组；
// [解析]     Init     —— 主从模式、时钟极性/相位、分频、数据位宽等（在 MX_SPI1_Init 里填）；
// [解析]     State / ErrorCode —— HAL 运行时状态。
// [解析]   上层用法：bmi088 驱动里 sp::BMI088 imu(&hspi1, ...) 把这个句柄存起来，
// [解析]   之后每次读写寄存器都用 HAL_SPI_TransmitReceive(hspi_, ...)。
// [解析]   注意本工程没有 hspi2（用户 SPI 排针在本次配置里没有使用）。

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

// [解析] ★ MX_SPI1_Init：SPI1 初始化函数，实现见 Core/Src/spi.c。逐条参数的含义：
// [解析]   Mode = SPI_MODE_MASTER          主机模式（MCU 产生时钟，BMI088 是从机）；
// [解析]   Direction = SPI_DIRECTION_2LINES 全双工，MOSI/MISO 各一根线；
// [解析]   DataSize = SPI_DATASIZE_8BIT    每次收发 8 位（BMI088 寄存器就是 8 位）；
// [解析]   CLKPolarity = SPI_POLARITY_HIGH + CLKPhase = SPI_PHASE_2EDGE
// [解析]     -> 空闲时 SCK 为高、第二个边沿采样，即 SPI 模式 3（CPOL=1, CPHA=1）。
// [解析]     BMI088 支持模式 0 和模式 3，本工程用模式 3，两边配错就完全读不出数据。
// [解析]   NSS = SPI_NSS_SOFT           片选由软件 GPIO 控制（PA4 加速度计 / PB0 陀螺仪），
// [解析]                               所以硬件 NSS 引脚不用，也解释了为什么有两个片选脚。
// [解析]   BaudRatePrescaler = 8        SPI1 挂在 APB2 上，84 MHz / 8 = 10.5 MHz 的 SCK；
// [解析]   FirstBit = SPI_FIRSTBIT_MSB  先发最高位（BMI088 要求 MSB first）；
// [解析]   TIMode / CRCCalculation 都关闭（TI 同步串行模式和 CRC 校验本工程都不需要）。
// [解析] HAL_SPI_Init() 内部会自动回调 HAL_SPI_MspInit()，在那里做三件事：
// [解析]   使能 SPI1 时钟；把 PB3/PB4 配成 AF5_SPI1；把 PA7 配成 AF5_SPI1。
// [解析]   本工程 SPI 采用阻塞式收发（HAL_SPI_TransmitReceive），所以没有开 SPI 中断。
// [解析]   ▼ 下面两条是"看代码时值得留意的疑点"：
// [解析] 注意 1（引脚与文档不一致）：docs/C板引脚配置说明.md 第 27 行把 SPI1 写成
// [解析]   "PB3 / PA6 / PA7 = SCK / MISO / MOSI"，但 Core/Src/spi.c 里 MISO 实际配的是 **PB4**
// [解析]   （spi.c 的 MspInit 里 GPIO_PIN_4|GPIO_PIN_3 配在 GPIOB，PA7 单独配在 GPIOA）。
// [解析]   请以代码和实物原理图为准；若按文档接线到 PA6，读 IMU 会得到全 0 或全 0xFF。
// [解析]   ▼ 第二条疑点是时钟余量：
// [解析] 注意 2（时钟略微超规格）：BMI088 手册规定 SPI 时钟最高 10 MHz，而 84/8 = 10.5 MHz
// [解析]   略高于上限。多数板子实测能正常跑，但若出现"IMU 偶尔读出脏数据/初始化失败"，
// [解析]   可以把 BaudRatePrescaler 改成 16（84/16 = 5.25 MHz）来排除这一因素
// [解析]   —— 这属于风险提示，本文件不修改任何配置。
void MX_SPI1_Init(void);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

// [解析] ↑ 这对 #ifdef __cplusplus / #endif 结束了开头的 extern "C" 块。
// [解析] ↓ 下面这行结束开头的 #ifndef __SPI_H__ 包含卫士；两对标记都必须成对完整。
#endif /* __SPI_H__ */

