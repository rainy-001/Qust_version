// [解析] ==========================================================================
// [解析] 【文件】Core/Inc/tim.h
// [解析] 【作用】定时器（TIM）外设的"对外声明"头文件，本工程用到两路：
// [解析]         TIM4 -> 蜂鸣器（PD14 = TIM4_CH3）；TIM5 -> RGB 灯（PH10/11/12 = CH1/2/3）。
// [解析]         文件里只有：2 个句柄 + 2 个初始化函数原型 + 1 个 MspPostInit 原型。
// [解析] 【谁生成】CubeMX 自动生成。作者没有手改过本文件（保护区都是空的）。
// [解析] 【本工程用途】
// [解析]   · TIM4_CH3 输出 PWM 驱动蜂鸣器：applications/buzzer_task.cpp + sp_middleware/io/buzzer/
// [解析]     蜂鸣器要按乐曲改音调，所以 buzzer.cpp 的 set() 会在运行时改写 PSC 和 ARR；
// [解析]   · TIM5_CH1/CH2/CH3 输出 PWM 驱动 RGB 三色灯：applications/led_task.cpp +
// [解析]     sp_middleware/io/led/led.cpp，set(r,g,b) 靠改占空比来调颜色。
// [解析] 【关键点】① 本文件里【没有】htim1 —— TIM1 在本工程被用作 HAL 时基
// [解析]             （因为 SysTick 被 FreeRTOS 占用了），它的句柄和初始化代码在
// [解析]             Core/Src/stm32f4xx_hal_timebase_tim.c 里，中断入口在 stm32f4xx_it.c；
// [解析]           ② HAL_TIM_MspPostInit() 不是 HAL 自动调用的，而是 CubeMX 在
// [解析]             MX_TIMx_Init() 末尾手工调用的，原因见下面该函数的说明。
// [解析] ==========================================================================

/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    tim.h
  * @brief   This file contains all the function prototypes for
  *          the tim.c file
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
// [解析]   中间就是"用户代码保护区"。CubeMX 重新 GENERATE CODE 时保护区内的内容保留、
// [解析]   保护区外的一切被重新生成覆盖，所以自己的代码必须写在里面。本文件的保护区是空的。
/* Define to prevent recursive inclusion -------------------------------------*/
// [解析] ↓ 条件编译"包含卫士"，保证本头文件只被展开一次，避免重复定义。
#ifndef __TIM_H__
#define __TIM_H__

// [解析] ↓ extern "C"：工程上层是 C++（applications/ 下是 .cpp），这里是 C 代码；
// [解析]   没有这个包裹，链接时会因为 C++ 的函数名改编而找不到 MX_TIM4_Init / MX_TIM5_Init。
// [解析]   配对 #endif 在文件末尾。
#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
// [解析] main.h 里包含 stm32f4xx_hal.h（HAL 总头），所以下面能直接用 TIM_HandleTypeDef。
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern TIM_HandleTypeDef htim4;
// [解析] ↑ TIM4 句柄（本体在 Core/Src/tim.c）。TIM4 挂在 APB1 总线上，是通用定时器，
// [解析]   本工程只用了它的第 3 通道（CH3）输出 PWM 去驱动蜂鸣器，引脚 PD14。
// [解析]   上层用法：sp::Buzzer buzzer(&htim4, TIM_CHANNEL_3, clock_hz) 把句柄和通道号存起来。

extern TIM_HandleTypeDef htim5;
// [解析] ↑ TIM5 句柄（本体在 Core/Src/tim.c）。TIM5 同样挂在 APB1，是 32 位通用定时器
// [解析]   （F4 上只有 TIM2 和 TIM5 是 32 位的，本工程只是拿它当普通 16 位 PWM 用）。
// [解析]   本工程用了 CH1/CH2/CH3 三个通道驱动 RGB 灯，引脚 PH10/PH11/PH12。
// [解析]   上层用法：sp::LED led(&htim5); 然后在 led.set(r,g,b) 里改三个通道的占空比。
// [解析]   注意：led.cpp 内部把 CH1 当 B、CH2 当 G、CH3 当 R 使用（顺序与引脚名 R/G/B 相反），
// [解析]   这与 docs/C板引脚配置说明.md 第 3 节末尾的提醒一致：颜色不对就交换参数。

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

// [解析] ★ MX_TIM4_Init：TIM4 初始化函数（实现见 Core/Src/tim.c），给蜂鸣器提供 PWM。
// [解析]   参数：Prescaler = 0，Period(ARR) = 65535，向上计数，PWM1 模式，初始占空比 0（先不响）。
// [解析]   【频率换算】TIM4 挂 APB1。本工程 APB1 = 42 MHz，而 STM32 的规则是
// [解析]     "APB 预分频 ≠ 1 时，定时器时钟 = 2 × PCLK" -> TIM4 计数时钟 = 84 MHz。
// [解析]     PWM 频率 = 计数时钟 / ((PSC+1) × (ARR+1)) = 84 MHz / (1 × 65536) ≈ 1281.7 Hz
// [解析]     与 docs 里写的"约 1.28 kHz"一致。占空比 = Pulse / (ARR+1)。
// [解析]   注意：这个 1.28 kHz 只是初始值。蜂鸣器要发出不同音高，所以
// [解析]     sp_middleware/io/buzzer/buzzer.cpp 的 set() 会在运行时用
// [解析]     __HAL_TIM_SET_PRESCALER / __HAL_TIM_SET_AUTORELOAD 改写 PSC 和 ARR
// [解析]     （先分频到 1 MHz，再按目标频率设 ARR），这里填的值会被覆盖，不用纠结。
// [解析]   最后一行 HAL_TIM_MspPostInit(&htim4) 是 CubeMX 手工加的，见下方说明。
void MX_TIM4_Init(void);
// [解析] ★ MX_TIM5_Init：TIM5 初始化函数（实现见 Core/Src/tim.c），给 RGB 灯提供 PWM。
// [解析]   参数与 TIM4 完全相同：PSC = 0、ARR = 65535、向上计数、PWM1、初始占空比 0（灯全灭）。
// [解析]   它一次配置了 3 个通道（CH1/CH2/CH3），对应 PH10/PH11/PH12 三个颜色引脚。
// [解析]   频率同样是 84 MHz / 65536 ≈ 1.28 kHz —— 对 LED 调光来说足够高，
// [解析]   人眼看不到闪烁；ARR 取最大 65535 是为了让 8 位颜色值有足够的调光分辨率。
// [解析]   led.cpp 的 set() 里也会先 __HAL_TIM_SET_AUTORELOAD(htim_, 65535) 再改比较值，
// [解析]   目的就是防止别处改过 ARR 之后占空比换算错位。
void MX_TIM5_Init(void);

// [解析] ★ HAL_TIM_MspPostInit：MSP = MCU Support Package（MCU 支持包）。
// [解析]   它专门负责"引脚复用、时钟使能、NVIC、DMA 绑定"这类 HAL 初始化之外的底层杂活。
// [解析]   定时器有两个 MSP 函数，分工不一样，这是最容易混淆的地方：
// [解析]     1) HAL_TIM_PWM_MspInit()  —— 由 HAL_TIM_PWM_Init() 内部【自动回调】，
// [解析]        只做"使能 TIM4/TIM5 时钟"；它不需要在头文件里声明（HAL 用弱符号调用）。
// [解析]     2) HAL_TIM_MspPostInit()  —— 【HAL 不会自动调用】，是 CubeMX 生成的
// [解析]        MX_TIM4_Init() / MX_TIM5_Init() 在返回前【手工调用】的，做的是
// [解析]        "把 PD14 配成 GPIO_AF2_TIM4、把 PH10/11/12 配成 GPIO_AF2_TIM5"。
// [解析]   为什么要单独拆出 Post 版本？因为 PWM 输出引脚的复用配置必须在
// [解析]   HAL_TIM_PWM_ConfigChannel() 把通道配置好之后再设置，顺序反了引脚可能输出异常电平。
// [解析]   因为它是普通函数（不是弱符号），调用点（tim.c）需要先看到原型声明，
// [解析]   所以 CubeMX 把它的原型放在了这里。
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

// [解析] ↑ 这对 #ifdef __cplusplus / #endif 结束了开头的 extern "C" 块。
// [解析] ↓ 下面这行结束开头的 #ifndef __TIM_H__ 包含卫士；两对标记都必须完整成对。
#endif /* __TIM_H__ */

