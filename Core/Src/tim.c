// [解析] ==========================================================================
// [解析] 【文件】Core/Src/tim.c
// [解析] 【作用】配置本工程用到的两个 PWM 定时器：
// [解析]         TIM4_CH3 -> PD14 蜂鸣器（buzzer_task）；
// [解析]         TIM5_CH1/2/3 -> PH10/PH11/PH12 RGB LED（led_task）。
// [解析] 【谁生成】STM32CubeMX 自动生成，作者没有改动过里面的逻辑。
// [解析]         真正操作 PWM 的代码在 sp_middleware 里（led / buzzer 驱动）。
// [解析] 【关键点】三个最容易看不懂的地方：
// [解析]   1) Prescaler=0、Period=65535 这两个数字是怎么决定 PWM 频率的（84MHz 除以 65536）；
// [解析]   2) MSP 回调是什么：MspInit 负责开外设时钟，MspPostInit 负责配引脚复用；
// [解析]   3) /* USER CODE BEGIN/END */ 保护区：重新 GENERATE CODE 时只有区内的代码会被保留。
// [解析] 【补充】本文件不涉及 HAL 时基。因为 FreeRTOS 占用了 SysTick，本工程的 HAL 时基
// [解析]         改由 TIM1 提供，实现放在 Core/Src/stm32f4xx_hal_timebase_tim.c。
// [解析] ==========================================================================

/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    tim.c
  * @brief   This file provides code for the configuration
  *          of the TIM instances.
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
#include "tim.h"

// [解析] ↑ tim.h 里声明了 htim4 / htim5 两个句柄变量和 MX_TIM4_Init / MX_TIM5_Init 函数原型。
// [解析]   注意：tim.h 里还声明了 HAL_TIM_PWM_MspInit / HAL_TIM_MspPostInit / MspDeInit，
// [解析]   这三个是给 HAL 库回调用的，不是给你手动调用的。

// [解析] ↓ 下面这一对 /* USER CODE BEGIN 0 */ ... /* USER CODE END 0 */ 是 CubeMX 的
// [解析]   "用户代码保护区"。它的作用机制是：
// [解析]   在 CubeMX 里点 GENERATE CODE 重新生成代码时，只有夹在 BEGIN / END 之间的内容
// [解析]   会被原样保留下来，标记之外的一切都会被重新生成覆盖掉。
// [解析]   所以"自己写的代码"一定要写在保护区里面，否则下次一生成就没了！
// [解析]   编号（0/1/2...）只是用来区分同一文件里的多块保护区，必须成对使用、不要写错号。
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

// [解析] 两个定时器的"HAL 句柄"全局变量。
// [解析] 句柄就是一个结构体变量，里面记录该外设的寄存器基地址（Instance）、初始化参数、
// [解析] 以及 DMA / 互斥锁等运行时状态。所有 HAL_TIM_xxx(&htim4, ...) 都靠它定位到硬件。
// [解析] 上层代码（led.cpp / buzzer.cpp）拿到的就是 &htim4 / &htim5 这两个地址。
TIM_HandleTypeDef htim4;  // [解析] 对应 TIM4，输出 PD14 上的蜂鸣器 PWM（只用了 CH3）
TIM_HandleTypeDef htim5;  // [解析] 对应 TIM5，输出 PH10/PH11/PH12 上的 RGB LED PWM（CH1/2/3）

/* TIM4 init function */
// [解析] MX_TIM4_Init()：把 TIM4 初始化成 PWM 输出模式，只启用了 CH3 一路（PD14 蜂鸣器）。
// [解析] 参数：无。返回值：无。中途失败会调用 Error_Handler() 进入死循环（实现在 main.c）。
// [解析] 注意：本函数只负责"配置"，不会让 PWM 开始输出；要输出还得调 HAL_TIM_PWM_Start()。
void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  // [解析] ↑ 这就是 TIM4 初始化函数里的 0 号保护区，CubeMX 默认留空，
  // [解析]   它是给你"在填参数之前"插入自己代码用的位置。
  /* USER CODE END TIM4_Init 0 */

  // [解析] 两个局部结构体。{0} 的写法表示"所有成员清零"，防止结构体里残留随机垃圾值。
  // [解析] HAL 不会替你填默认值，所以下面的赋值必须一个不少，否则行为不确定。
  TIM_MasterConfigTypeDef sMasterConfig = {0};  // [解析] 主/从模式与触发输出配置（本项目不用级联，填默认值即可）
  TIM_OC_InitTypeDef sConfigOC = {0};           // [解析] OC = Output Compare（输出比较），PWM 就是它的一种用法

  /* USER CODE BEGIN TIM4_Init 1 */

  // [解析] ↑ 1 号保护区：位置在"填参数之前、调用 HAL 初始化之前"。
  /* USER CODE END TIM4_Init 1 */
  // [解析] ↓ 下面这段是把初始化参数填进句柄，填完后由 HAL_TIM_PWM_Init() 一次性写进寄存器。
  // [解析]   先说时钟：TIM4 挂在 APB1 上，APB1 总线是 42 MHz，但"定时器时钟"是它的 2 倍 = 84 MHz
  // [解析]   （F4 的规则：APB 分频系数不等于 1 时，送到定时器的时钟自动 ×2）。
  // [解析]   于是：计数器频率 = 84 MHz / (Prescaler+1) = 84 MHz / 1 = 84 MHz，
  // [解析]         PWM 频率 = 84 MHz / (Period+1) = 84 MHz / 65536 ≈ 1281.7 Hz ≈ 1.28 kHz。
  // [解析]   另外 Period=65535 还有个好处：占空比分辨率是 1/65536，
  // [解析]   上层就能很方便地用 0~65535 表示 0~100%（LED 调光、蜂鸣器音量都用这个）。
  htim4.Instance = TIM4;  // [解析] 告诉 HAL"我要操作的是 TIM4"，后面的寄存器操作都基于它
  htim4.Init.Prescaler = 0;                                       // [解析] 预分频，0 表示分频比 = 0+1 = 1，不降频
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;                    // [解析] 向上计数：0→1→…→Period，然后重装回 0
  htim4.Init.Period = 65535;                                      // [解析] 自动重装载值 ARR，也就是计数上限，决定 PWM 周期
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;              // [解析] 只影响输入滤波的采样时钟，不影响 PWM 频率
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;  // [解析] 关闭 ARR 预装载：改 Period 会立即生效
  // [解析] ↓ 这一步才真正把事情做完：写 TIM4 寄存器，并回调 HAL_TIM_PWM_MspInit() 打开 TIM4 时钟。
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();  // [解析] 初始化失败就死循环（并点亮错误灯），让问题在调试时立刻暴露
  }
  // [解析] 主/从同步配置。本项目没有做定时器级联，所以触发源填 RESET（等于不往外输出触发信号）、
  // [解析] 从模式关闭。这两行保持默认值就行，不用理解太深。
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  // [解析] 下面 4 行配置 PWM 波形的"形状"，它们只对后面 ConfigChannel 指定的那一路通道生效。
  // [解析]   PWM1 模式：计数值 < Pulse 时输出高电平，计数值 >= Pulse 时输出低电平。
  // [解析]   所以 占空比 = Pulse / (Period+1)。Pulse=0 → 占空比 0%，蜂鸣器初始不响。
  // [解析]   OCPolarity_HIGH 表示"有效电平是高电平"；OCFastMode 关闭（普通 PWM 用不到）。
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;  // [解析] 比较值 CCR3 的初值，运行时会由 buzzer 驱动改写来发出不同音调
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  // [解析] 把上面这套配置只应用到 CH3 一路（即 PD14）。TIM4 的其它通道没有配置，也不会输出。
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  // [解析] ↑ 2 号保护区：此时外设已经初始化完毕，适合放"启动 PWM / 使能中断"之类的代码。
  /* USER CODE END TIM4_Init 2 */
  // [解析] HAL_TIM_MspPostInit()：配置 PWM 输出引脚的"复用功能"，由 HAL 在初始化末尾自动调用。
  // [解析] 顺便解释 MSP = MCU Support Package（MCU 支持包）：HAL 库只负责外设本身寄存器的读写，
  // [解析] 而"开外设时钟、把引脚切成复用模式、配 NVIC、绑定 DMA"这些和具体芯片/板子强相关的事，
  // [解析] 由 HAL 通过回调 MSP 函数交给用户实现。本文件下面那两个
  // [解析] HAL_TIM_PWM_MspInit() / HAL_TIM_MspPostInit() 就是这些回调，不用你手动调。
  // [解析] 为什么拆成两个？因为"配 SPI/PWM 引脚"必须在通道配置(ConfigChannel)之后才做，
  // [解析] 不然配置过程中引脚会先输出一段莫名其妙的电平。
  HAL_TIM_MspPostInit(&htim4);

}
/* TIM5 init function */
// [解析] MX_TIM5_Init()：把 TIM5 初始化成 PWM 输出，启用了 CH1/CH2/CH3 三路
// [解析] （对应 PH10 / PH11 / PH12 三个 RGB LED 通道）。
// [解析] 它和 MX_TIM4_Init() 几乎一模一样，区别只有两点：
// [解析]   1) Instance 换成 TIM5，MSP 回调里针对的引脚换成 PH10~PH12；
// [解析]   2) 多配置了 CH1、CH2 两路通道（TIM4 只用了一路）。
void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  // [解析] ↑ TIM5 的 0 号保护区，同样默认留空。
  /* USER CODE END TIM5_Init 0 */

  // [解析] 含义同 TIM4，不再重复：sMasterConfig 管主从模式，sConfigOC 管 PWM 波形。
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  // [解析] TIM5 也挂在 APB1 上，所以定时器时钟同样是 84 MHz。
  // [解析] Prescaler=0、Period=65535 → PWM 频率 = 84 MHz / 65536 ≈ 1.28 kHz，与 TIM4 一致。
  // [解析] LED 调光用这个频率完全够（人眼只要 >100 Hz 就看不到闪烁），而且 1/65536 的分辨率很细腻。
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 0;                                       // [解析] 不分频，计数时钟 = 84 MHz
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;                    // [解析] 向上计数
  htim5.Init.Period = 65535;                                      // [解析] 计数上限 ARR，对应 1.28 kHz
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;              // [解析] 滤波采样分频，与 PWM 频率无关
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;  // [解析] 关闭 ARR 预装载
  if (HAL_TIM_PWM_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  // [解析] 波形配置：PWM1 模式 + 高电平有效 + Pulse 初值 0（上电时三路 LED 全灭）。
  // [解析] 三个通道共用同一份 sConfigOC，所以下面连着调用 3 次 ConfigChannel，
  // [解析] 只是把"同一套波形参数"复制到 CH1、CH2、CH3 各自的比较寄存器（CCR1/CCR2/CCR3）。
  // [解析] 上层的 sp::LED 就是通过写这 3 个 CCR 来调 R/G/B 三色亮度的。
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  // [解析] ↓ CH1 对应 PH10，CH2 对应 PH11，CH3 对应 PH12（具体引脚在 MspPostInit 里配）。
  // [解析] 注意：这里通道与颜色的对应关系由硬件走线决定；如果实际点灯颜色不对，
  // [解析] 交换上层 led.set(r,g,b) 的参数即可，不用改本文件。
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */
  HAL_TIM_MspPostInit(&htim5);

}

// [解析] HAL_TIM_PWM_MspInit()：PWM 外设的底层初始化回调，由 HAL_TIM_PWM_Init() 内部自动调用。
// [解析] 参数 tim_pwmHandle 就是调用方传进来的句柄（&htim4 或 &htim5），用它区分是哪一路定时器。
// [解析] 里面要做的事很单一：打开对应 TIM 的外设时钟。时钟不开，后面写再多寄存器都无效，
// [解析] 这也是新手最常见的"配置全对但没输出"的原因之一。
// [解析] 函数名前缀 HAL_..._MspInit 是 HAL 规定的回调命名，HAL 库里是 __weak 弱定义，
// [解析] CubeMX 在本文件里给出强定义把它覆盖掉（__weak 的含义见 freertos.c 的说明）。
void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef* tim_pwmHandle)
{

  if(tim_pwmHandle->Instance==TIM4)
  {
  /* USER CODE BEGIN TIM4_MspInit 0 */

  /* USER CODE END TIM4_MspInit 0 */
    /* TIM4 clock enable */
    __HAL_RCC_TIM4_CLK_ENABLE();  // [解析] 打开 TIM4 的时钟门控（APB1 外设，寄存器位 TIM4EN）
  /* USER CODE BEGIN TIM4_MspInit 1 */

  /* USER CODE END TIM4_MspInit 1 */
  }
  else if(tim_pwmHandle->Instance==TIM5)
  {
  /* USER CODE BEGIN TIM5_MspInit 0 */

  /* USER CODE END TIM5_MspInit 0 */
    /* TIM5 clock enable */
    __HAL_RCC_TIM5_CLK_ENABLE();  // [解析] 打开 TIM5 的时钟门控
  /* USER CODE BEGIN TIM5_MspInit 1 */

  /* USER CODE END TIM5_MspInit 1 */
  }
}
// [解析] HAL_TIM_MspPostInit()：配置 PWM 输出引脚的"复用功能(AF)"。
// [解析] 一个引脚既能当普通 IO，也能当定时器/串口/USB 的引脚，靠"复用功能号"来选择。
// [解析] STM32F407 每个引脚的 AF 是固定的，查数据手册的"复用功能映射表"可知：
// [解析]   PD14 的 AF2 = TIM4_CH3，PH10/PH11/PH12 的 AF2 = TIM5_CH1/CH2/CH3。
// [解析] 参数 timHandle 用来区分是哪一路定时器，从而只配它自己那几个引脚。
void HAL_TIM_MspPostInit(TIM_HandleTypeDef* timHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(timHandle->Instance==TIM4)
  {
  /* USER CODE BEGIN TIM4_MspPostInit 0 */

  /* USER CODE END TIM4_MspPostInit 0 */
    __HAL_RCC_GPIOD_CLK_ENABLE();  // [解析] 用哪个 GPIO 端口就要先开哪个端口的时钟（PD14 属于 GPIOD）
    /**TIM4 GPIO Configuration
    PD14     ------> TIM4_CH3
    */
    // [解析] 只配 PD14 一个引脚，各字段含义：
    // [解析]   GPIO_MODE_AF_PP —— 复用推挽输出：交给外设(TIM)控制，且能输出高低电平并有驱动能力
    // [解析]   GPIO_NOPULL      —— 不使用芯片内部上/下拉电阻（外部电路自己会处理）
    // [解析]   GPIO_SPEED_FREQ_LOW —— 引脚电平翻转速率选"低"。PWM 才 1.28 kHz，用低速足够，
    // [解析]                        还能减小电磁干扰；高速档只在高频信号时才需要。
    // [解析]   Alternate = GPIO_AF2_TIM4 —— 复用功能号选 AF2，也就是连到 TIM4
    GPIO_InitStruct.Pin = GPIO_PIN_14;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* USER CODE BEGIN TIM4_MspPostInit 1 */

  /* USER CODE END TIM4_MspPostInit 1 */
  }
  else if(timHandle->Instance==TIM5)
  {
  /* USER CODE BEGIN TIM5_MspPostInit 0 */

  /* USER CODE END TIM5_MspPostInit 0 */

    __HAL_RCC_GPIOH_CLK_ENABLE();  // [解析] PH10/PH11/PH12 属于 GPIOH，先开 GPIOH 的时钟
    /**TIM5 GPIO Configuration
    PH12     ------> TIM5_CH3
    PH11     ------> TIM5_CH2
    PH10     ------> TIM5_CH1
    */
    // [解析] 三路 LED 引脚一次配完：用 | 把三个引脚号拼在一起，调用一次 HAL_GPIO_Init 即可。
    // [解析] 其余字段含义与 TIM4 那段完全相同，只是复用功能号换成 AF2_TIM5。
    GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_11|GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM5;
    HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /* USER CODE BEGIN TIM5_MspPostInit 1 */

  /* USER CODE END TIM5_MspPostInit 1 */
  }

}

// [解析] HAL_TIM_PWM_MspDeInit()：MspInit 的逆操作 —— 关外设时钟、把引脚恢复成复位默认状态。
// [解析] 只有在调用 HAL_TIM_PWM_DeInit() 时才会被触发。本工程从头到尾不会关定时器，
// [解析] 所以这个函数实际上永远不会被执行，了解它的用途即可。
void HAL_TIM_PWM_MspDeInit(TIM_HandleTypeDef* tim_pwmHandle)
{

  if(tim_pwmHandle->Instance==TIM4)
  {
  /* USER CODE BEGIN TIM4_MspDeInit 0 */

  /* USER CODE END TIM4_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM4_CLK_DISABLE();  // [解析] 关闭 TIM4 时钟，省电
  /* USER CODE BEGIN TIM4_MspDeInit 1 */

  /* USER CODE END TIM4_MspDeInit 1 */
  }
  else if(tim_pwmHandle->Instance==TIM5)
  {
  /* USER CODE BEGIN TIM5_MspDeInit 0 */

  /* USER CODE END TIM5_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM5_CLK_DISABLE();  // [解析] 关闭 TIM5 时钟
  /* USER CODE BEGIN TIM5_MspDeInit 1 */

  /* USER CODE END TIM5_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

// [解析] ↑ 文件末尾的保护区，可以放本文件自己的辅助函数/变量。当前为空。
/* USER CODE END 1 */

