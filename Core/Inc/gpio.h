// [解析] ==========================================================================
// [解析] 【文件】Core/Inc/gpio.h
// [解析] 【作用】GPIO（普通输入输出引脚）初始化的声明文件，只声明了一个函数：
// [解析]         void MX_GPIO_Init(void);
// [解析] 【谁生成】CubeMX 自动生成。作者没有手改过本文件。
// [解析] 【本工程用到的 GPIO】只有两组，都是给 IMU(BMI088) 服务的：
// [解析]   1) 两个片选脚：PA4 = IMU_CS1_ACCEL（加速度计 CS）、PB0 = IMU_CS2_GYRO（陀螺仪 CS）
// [解析]      —— 都是推挽输出，SPI 靠拉低片选来"点名"跟哪个芯片说话；
// [解析]   2) 两个中断脚：PC4 = INT1_ACCEL、PC5 = INT1_GYRO
// [解析]      —— 都是上升沿触发的外部中断，IMU 有新数据时会拉一下，通知 MCU 去读。
// [解析] 【关键点】① 引脚编号的宏定义不在本文件，而在 Core/Inc/main.h 的"Private defines"里
// [解析]             （IMU_CS1_ACCEL_Pin / IMU_CS1_ACCEL_GPIO_Port 等等），别找错地方；
// [解析]           ② 本文件只声明函数，具体配置见 Core/Src/gpio.c。
// [解析] ==========================================================================

/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.h
  * @brief   This file contains all the function prototypes for
  *          the gpio.c file
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
// [解析]   中间的内容就是"用户代码保护区"。CubeMX 重新 GENERATE CODE 时：
// [解析]   保护区内保留、保护区外全部重新生成覆盖，所以自己的代码要写在里面。
// [解析]   本文件的保护区目前都是空的。
/* Define to prevent recursive inclusion -------------------------------------*/
// [解析] ↓ 条件编译"包含卫士"，保证本头文件只被展开一次（重复展开会导致重复定义）。
#ifndef __GPIO_H__
#define __GPIO_H__

// [解析] ↓ extern "C"：工程上层是 C++，这里导出的是 C 函数名（否则链接时找不到 MX_GPIO_Init）。
// [解析]   配对 #endif 在文件末尾。
#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
// [解析] main.h 提供了 stm32f4xx_hal.h，以及本工程用到的 GPIO 引脚宏定义。
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

// [解析] ★ MX_GPIO_Init：GPIO 初始化函数，实现见 Core/Src/gpio.c。它做了 4 件事：
// [解析]   1) 使能 GPIOB / GPIOA / GPIOD / GPIOH / GPIOF / GPIOC 六个端口的时钟
// [解析]      （这是 CubeMX 按"哪些端口有引脚被分配"自动列的；不使能时钟，写引脚无效）；
// [解析]   2) 先把两个片选脚 PA4、PB0 输出高电平：HAL_GPIO_WritePin(..., GPIO_PIN_SET)
// [解析]      —— 片选是低电平有效，开机先拉高表示"暂时不和任何芯片通信"，避免误触发；
// [解析]   3) 配置引脚模式：
// [解析]      · PA4 / PB0：GPIO_MODE_OUTPUT_PP（推挽输出），Speed = LOW（片选变化慢，够用且干扰小）
// [解析]      · PC4 / PC5：GPIO_MODE_IT_RISING（上升沿触发的外部中断输入），无上下拉；
// [解析]   4) 打开外部中断：EXTI4（对应 PC4）和 EXTI9_5（对应 PC5）
// [解析]      —— 抢占优先级都是 5、子优先级 0。
// [解析]   ▼ 为什么 PC5 的中断叫 EXTI9_5：
// [解析] 【为什么 PC5 的中断叫 EXTI9_5】STM32F4 的 EXTI 中断线是多个引脚共用一个入口的：
// [解析]   EXTI0~EXTI4 各自独立，EXTI5~EXTI9 共用一个中断向量（EXTI9_5_IRQHandler），
// [解析]   EXTI10~EXTI15 共用另一个。所以 PC5 触发的是 EXTI9_5_IRQHandler，
// [解析]   在中断服务函数里必须用 __HAL_GPIO_EXTI_GET_IT() 区分到底是哪个引脚来的。
// [解析]   ▼ 关于 NVIC 优先级的说明：
// [解析] 【优先级说明】NVIC 用的是 (抢占优先级, 子优先级) 两个数字。本工程把抢占优先级都设为 5，
// [解析]   意思是"这些中断彼此之间不能互相打断"。FreeRTOS 的配置里
// [解析]   configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY 决定了"哪些中断里可以调用
// [解析]   FreeRTOS 的 FromISR 接口"，数值比它大的（优先级更低）才允许调用，改动时要注意。
void MX_GPIO_Init(void);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
// [解析] ↑ 这对 #ifdef __cplusplus / #endif 结束了开头的 extern "C" 块（注意本文件这两行
// [解析]   和下面前后紧挨着，没有空行，属于 CubeMX 的输出格式差异，不是错误）。
// [解析] ↓ 这一行结束开头的 #ifndef __GPIO_H__ 包含卫士。注意末尾注释写的是 /*__ GPIO_H__ */，
// [解析]   与真正的宏名 __GPIO_H__ 在下划线分组和空格上并不一致，这是 CubeMX 输出的小瑕疵；
// [解析]   它只是注释文字，不影响编译，也不影响 #ifndef / #define 的配对关系。
#endif /*__ GPIO_H__ */

