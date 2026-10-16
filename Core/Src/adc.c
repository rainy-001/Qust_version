// [解析] ==========================================================================
// [解析] 【文件】Core/Src/adc.c
// [解析] 【作用】配置 ADC3，本工程只采一路：ADC3_IN8，对应引脚 PF10 —— 电池电压检测。
// [解析] 【谁生成】STM32CubeMX 自动生成，作者没有改动过这里的逻辑。
// [解析]         上层的使用代码在 applications/voltage_detect.cpp：
// [解析]         sp::Adc battery(&hadc3, 12, 3.3f, (222.0f/22.0f), 1.0f);
// [解析] 【关键点】
// [解析]   1) 12 位分辨率 + 3.3V 参考电压，怎么把 0~4095 的读数换算成电压；
// [解析]   2) "采样时间 3 个 ADC 周期"是什么，为什么它对高阻抗分压电路可能偏短；
// [解析]   3) MSP 回调里为什么必须把引脚配成 GPIO_MODE_ANALOG。
// [解析] ==========================================================================

/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    adc.c
  * @brief   This file provides code for the configuration
  *          of the ADC instances.
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
#include "adc.h"

// [解析] ↑ adc.h 里声明了句柄 hadc3 和 MX_ADC3_Init() 的原型。

// [解析] ↓ /* USER CODE BEGIN 0 */ ... /* USER CODE END 0 */ 是 CubeMX 的"用户代码保护区"。
// [解析]   机制：在 CubeMX 里重新 GENERATE CODE 时，只有夹在这对标记之间的内容会被保留，
// [解析]   标记之外的一切都会按 .ioc 的配置重新生成、覆盖掉。自己写的代码务必放进保护区里。
// [解析]   编号只用来区分同一文件中的多块保护区，要成对使用。
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

// [解析] ADC3 的 HAL 句柄（全局变量）。里面记录寄存器基地址、初始化参数和运行时状态；
// [解析] 所有 HAL_ADC_xxx(&hadc3, ...) 都靠它定位硬件，上层 sp::Adc 拿到的就是 &hadc3。
ADC_HandleTypeDef hadc3;

/* ADC3 init function */
// [解析] MX_ADC3_Init()：把 ADC3 配置成"单通道、单次、软件触发"的采集模式。
// [解析] 参数：无。返回值：无。任何一步失败都会进 Error_Handler() 死循环（实现在 main.c）。
// [解析] 调用位置：main.c 的 main() 里，在 MX_GPIO_Init() 之后（顺序由 CubeMX 生成）。
// [解析] 注意：本函数只是"配置"，不会开始转换；真正采集由 sp::Adc::update() 里的
// [解析]       HAL_ADC_Start() + HAL_ADC_PollForConversion() 触发和读取。
void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  // [解析] ↑ 0 号保护区（默认留空），位置在"填参数之前"。
  /* USER CODE END ADC3_Init 0 */

  // [解析] 通道配置结构体：一个通道（通道号 + 在序列中的排位 + 采样时间）用一份。
  // [解析] {0} = 成员全部清零，防止残留随机值。本项目只有 1 个通道，所以只需要 1 份。
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  // [解析] ↓ 下面这段是"整体参数"，影响 ADC3 全部通道，先看两个换算：
  // [解析]   ① ADC 时钟：ADC3 挂在 APB2 上，APB2 = 84 MHz，这里 4 分频 → 84/4 = 21 MHz。
  // [解析]      F407 的 ADC 时钟上限是 36 MHz，21 MHz 既合规又够快（这行在
  // [解析]      sp_middleware/io/adc/adc.cpp 的 init() 里还会被重复设置一次，保持一致）。
  // [解析]   ② 电压换算：12 位分辨率下读数范围 0~4095，
  // [解析]      1 个 LSB 对应 3.3 V / 4096 ≈ 0.806 mV。
  // [解析]      上层 sp::Adc 用的就是 raw × (3.3 × 分压比 / 4096) + 偏置 这个公式。
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;  // [解析] ADC 时钟 = APB2(84MHz)/4 = 21 MHz（上限 36MHz）
  hadc3.Init.Resolution = ADC_RESOLUTION_12B;            // [解析] 12 位分辨率，读数 0~4095，分辨率约 0.806 mV
  hadc3.Init.ScanConvMode = DISABLE;                     // [解析] 关闭扫描模式：只采一个通道，不需要轮流扫多个通道
  hadc3.Init.ContinuousConvMode = DISABLE;               // [解析] 关闭连续转换：转一次就停，下次还要重新 HAL_ADC_Start()
  hadc3.Init.DiscontinuousConvMode = DISABLE;            // [解析] 关闭间断模式（把序列拆成小段转换，一个通道时无意义）
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;  // [解析] 不用外部触发边沿
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;      // [解析] 采用软件触发：调 HAL_ADC_Start() 才开始转换
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;            // [解析] 12 位结果右对齐，直接读寄存器就是 0~4095 的原始值
  hadc3.Init.NbrOfConversion = 1;                        // [解析] 规则组序列长度为 1，即只转换 1 个通道
  hadc3.Init.DMAContinuousRequests = DISABLE;            // [解析] 不请求 DMA：结果由程序主动读取，不用 DMA 搬运
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;         // [解析] 每完成"一次转换"就置 EOC 标志（而非整个序列结束才置）
  // [解析] ↓ 把上面填好的参数写进 ADC3 寄存器，并回调 HAL_ADC_MspInit() 打开 ADC 时钟、配好引脚。
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  // [解析] ↓ 配置"规则组"里的具体通道，三个字段逐一看：
  // [解析]   Channel = ADC_CHANNEL_8 —— 选第 8 号通道，对应引脚 PF10（电池电压经分压后接在这里）。
  // [解析]   Rank = 1              —— 它在转换序列里排第 1 位。只有 1 个通道，所以必然是 1。
  // [解析]   SamplingTime = 3 个 ADC 周期 —— 采样保持电容给输入充电的时间。
  // [解析]     换算成时间：3 / 21 MHz ≈ 143 ns；再加上 12 位转换本身的 12 个周期，
  // [解析]     一次完整转换约 (3+12)/21 MHz ≈ 0.71 µs。
  // [解析]   采样时间越短，对信号源阻抗要求越高（源阻抗大，电容没充够，读数会偏低）。
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;  // [解析] 选 3 个 ADC 周期，换算见上面那几行说明
  // [解析] 注意：docs/C板引脚配置说明.md 第 42 行写的是"采样时间 56 cycles"，
  // [解析]   而本文件与 Qust_version.ioc 里都是 3 cycles，两处并不一致（以代码 / .ioc 为准）。
  // [解析]   电池分压网络阻抗一般在 10 kΩ 量级，3 个周期(≈143 ns)可能来不及把采样电容充满，
  // [解析]   表现就是读数偏低。若发现测得的电压明显偏小，可改为 56 / 84 / 144 / 480 cycles。
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */

  // [解析] ↑ 2 号保护区：此时 ADC3 已配置完成，可以在这里校准或做偏移补偿。
  /* USER CODE END ADC3_Init 2 */

}

// [解析] HAL_ADC_MspInit()：ADC 的底层初始化回调，由 HAL_ADC_Init() 自动调用（MSP = MCU Support Package）。
// [解析] HAL 库只关心外设寄存器，而"开时钟、配引脚、配 NVIC、绑 DMA"这些与芯片/板子强相关的活，
// [解析] 都通过 MSP 回调交给用户代码完成。参数 adcHandle 用来区分是哪一路 ADC。
// [解析] 这里要做两件事：开 ADC3 的时钟 + 把 PF10 配成模拟输入。
void HAL_ADC_MspInit(ADC_HandleTypeDef* adcHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(adcHandle->Instance==ADC3)
  {
  /* USER CODE BEGIN ADC3_MspInit 0 */

  /* USER CODE END ADC3_MspInit 0 */
    /* ADC3 clock enable */
    __HAL_RCC_ADC3_CLK_ENABLE();  // [解析] 打开 ADC3 的时钟（APB2 外设）。时钟不开，ADC 读出来永远是 0

    __HAL_RCC_GPIOF_CLK_ENABLE();  // [解析] PF10 属于 GPIOF，用之前先开 GPIOF 端口时钟
    /**ADC3 GPIO Configuration
    PF10     ------> ADC3_IN8
    */
    // [解析] 模拟输入的引脚配置只有两个字段：
    // [解析]   GPIO_MODE_ANALOG —— 必须配成模拟模式。若留着默认的"数字输入"，引脚内部的
    // [解析]     施密特触发器会一直工作，既增加功耗又会给测量引入误差。
    // [解析]   GPIO_NOPULL      —— 不能用内部上/下拉！否则会和外部的分压电阻并联，
    // [解析]     把分压比改掉，测出来的电压就不准了。
    // [解析] 模拟模式不需要配 Speed 和 Alternate（这两个字段只对数字/复用输出有意义）。
    GPIO_InitStruct.Pin = GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /* USER CODE BEGIN ADC3_MspInit 1 */

  /* USER CODE END ADC3_MspInit 1 */
  }
}

// [解析] HAL_ADC_MspDeInit()：MspInit 的逆操作 —— 关 ADC3 时钟、把 PF10 恢复成复位默认状态。
// [解析] 只有在调用 HAL_ADC_DeInit() 时才会执行。本工程不会关闭 ADC，所以它实际不会跑到。
void HAL_ADC_MspDeInit(ADC_HandleTypeDef* adcHandle)
{

  if(adcHandle->Instance==ADC3)
  {
  /* USER CODE BEGIN ADC3_MspDeInit 0 */

  /* USER CODE END ADC3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_ADC3_CLK_DISABLE();  // [解析] 关闭 ADC3 时钟

    /**ADC3 GPIO Configuration
    PF10     ------> ADC3_IN8
    */
    HAL_GPIO_DeInit(GPIOF, GPIO_PIN_10);  // [解析] 把 PF10 还原成复位后的默认状态，不再当模拟输入

  /* USER CODE BEGIN ADC3_MspDeInit 1 */

  /* USER CODE END ADC3_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

// [解析] ↑ 文件末尾的保护区，可放本文件自己的代码，当前为空。
/* USER CODE END 1 */

