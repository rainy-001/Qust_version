// [解析] ==========================================================================
// [解析] 【文件】Core/Src/gpio.c
// [解析] 【作用】配置所有"当普通 GPIO 用"的引脚（复用给外设的引脚不在这里配）：
// [解析]         BMI088 的两个片选脚（输出）和两个中断脚（外部中断输入）。
// [解析] 【谁生成】CubeMX 自动生成，作者没有改动逻辑。
// [解析] 【用在哪】片选脚由 sp_middleware/io/bmi088/bmi088.cpp 拉高/拉低；
// [解析]           中断脚 PC4/PC5 接 BMI088 的 INT1，本意是"数据就绪时通知 CPU"。
// [解析] 【关键点】
// [解析]   1) 第一步必须开 GPIO 端口时钟，不开时钟后面配引脚全部无效；
// [解析]   2) 片选脚是"先写输出电平、再配成输出"，顺序不能反（见下面的详细解释）；
// [解析]   3) STM32 的外部中断线按"引脚号"共用：PC4 -> EXTI4；PC5 -> EXTI9_5（5~9 号共用一个中断向量）。
// [解析] ==========================================================================

/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
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
#include "gpio.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
// [解析] ↑ 这段英文是 CubeMX 的固定说明：本函数会把引脚按这几种模式之一来配置。
// [解析] MX_GPIO_Init() 分三步：① 开各 GPIO 端口的时钟；② 预设片选脚的初始电平；
// [解析]   ③ 逐个引脚配置模式、上下拉、速度。它由 main.c 调用，是所有外设初始化里最先执行的。
void MX_GPIO_Init(void)
{

  // [解析] 初始化结构体清零，避免残留随机值导致某个选项被意外打开。
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  // [解析] 使能本工程会用到的所有 GPIO 端口时钟。为什么这些端口都要开（括号里是本工程用到的引脚）：
  // [解析]   GPIOB —— PB0（陀螺仪片选）、PB10/PB11（USART3 遥控器）
  // [解析]   GPIOA —— PA4（加速度计片选）、PA6/PA7（SPI1）
  // [解析]   GPIOD —— PD0/PD1（CAN1）、PD14（蜂鸣器 PWM）
  // [解析]   GPIOH —— PH0/PH1（12MHz 晶振）、PH10~PH12（RGB LED，TIM5）
  // [解析]   GPIOF —— PF10（电池电压 ADC3_IN8）
  // [解析]   GPIOC —— PC4/PC5（BMI088 中断脚）
  // [解析] 注意：GPIOH 和 GPIOF 在本文件里其实没有配置任何引脚（PH10~12 在 tim.c、
  // [解析]   PF10 在 adc.c 里配，那两个文件也会各自再开一次时钟），这里属于重复使能，
  // [解析]   不会出错，只是多一点点功耗，属于 CubeMX 的生成习惯。
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  // [解析] ★ 先写电平、再配成输出，这个顺序很重要，不要调换：
  // [解析]   SPI 从机的片选(CS)是"低电平有效"——CS 拉低代表"选中我，开始传输"。
  // [解析]   如果先配成输出再写电平，配置完成的一瞬间引脚是低电平，
  // [解析]   传感器会误以为被选中，可能造成读写错乱。CubeMX 特意把这两句排在最前面。
  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(IMU_CS1_ACCEL_GPIO_Port, IMU_CS1_ACCEL_Pin, GPIO_PIN_SET);
  // [解析] ↑ 把加速度计片选拉高（GPIO_PIN_SET = 高电平 = 不选中）。
  // [解析]   宏定义在 Core/Inc/main.h：IMU_CS1_ACCEL_Pin = PA4，对应 bmi088.cpp 的 csb1 参数。

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(IMU_CS2_GYRO_GPIO_Port, IMU_CS2_GYRO_Pin, GPIO_PIN_SET);
  // [解析] ↑ 把陀螺仪片选拉高（不选中），同样是上电后必须保持的默认状态。
  // [解析]   注意：宏名写的是 CS2，而 docs/C板引脚配置说明.md 里叫 CS1_Gyro，
  // [解析]   两者指的是同一个 PB0，只是 CubeMX 与原理图的命名习惯不同，以引脚号为准
  // [解析]   （bmi088.cpp 里 PB0 对应构造参数 csb2，两边是对得上的）。

  /*Configure GPIO pin : IMU_CS1_ACCEL_Pin */
  // [解析] 把 PA4 配成普通推挽输出，用来控制加速度计片选。
  GPIO_InitStruct.Pin = IMU_CS1_ACCEL_Pin;      // [解析] 本次只操作 PA4 这一个引脚
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;   // [解析] 推挽输出(PP = Push-Pull)：能主动输出高、低电平，驱动片选正合适
  GPIO_InitStruct.Pull = GPIO_PULLUP;           // [解析] 打开内部上拉：即使单片机还没初始化完，引脚也保持高电平（不选中）
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;  // [解析] 翻转速度选最低档：片选信号不需要快，慢一点抗干扰更好
  HAL_GPIO_Init(IMU_CS1_ACCEL_GPIO_Port, &GPIO_InitStruct);
  // [解析] ↑ 以上设置在这一句才真正写进 GPIOA 的寄存器；之后 bmi088.cpp 里用
  // [解析]   HAL_GPIO_WritePin 拉低/拉高这个脚来"选中/释放"传感器。

  /*Configure GPIO pins : INT1_ACCEL_Pin INT1_GYRO_Pin */
  // [解析] 把 PC4、PC5 配成"上升沿触发的外部中断输入"。
  // [解析] 这两个脚接 BMI088 的 INT1：传感器内部把 INT1 配成"数据就绪(DRDY)"输出，
  // [解析] 平时为低，新数据准备好时变高 —— 所以选上升沿触发（见 bmi088.cpp 的 INT1 配置表）。
  GPIO_InitStruct.Pin = INT1_ACCEL_Pin|INT1_GYRO_Pin;  // [解析] 一次配两个脚：PC4(加速度计中断) 与 PC5(陀螺仪中断)
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;          // [解析] IT = Interrupt 中断模式，RISING = 上升沿触发
  GPIO_InitStruct.Pull = GPIO_NOPULL;                  // [解析] 不上下拉：BMI088 的中断脚是推挽输出，能自己驱动电平，不需要内部上拉
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
  // [解析] 注意：这一段没有重新设置 GPIO_InitStruct.Speed，沿用了上一段留下的
  // [解析]   GPIO_SPEED_FREQ_LOW。输入/中断模式下 Speed 字段本来就不起作用，所以不影响功能。

  /*Configure GPIO pin : IMU_CS2_GYRO_Pin */
  // [解析] 把 PB0 配成普通推挽输出，用来控制陀螺仪片选；参数含义与上面的加速度计片选完全一样。
  GPIO_InitStruct.Pin = IMU_CS2_GYRO_Pin;       // [解析] 本次只操作 PB0
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;   // [解析] 推挽输出
  GPIO_InitStruct.Pull = GPIO_PULLUP;           // [解析] 内部上拉，保证默认是高电平（不选中）
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;  // [解析] 低速档足够
  HAL_GPIO_Init(IMU_CS2_GYRO_GPIO_Port, &GPIO_InitStruct);
  // [解析] ↑ 设置写入 GPIOB 的寄存器，之后 bmi088.cpp 就是用这个脚来选陀螺仪的。

  /* EXTI interrupt init*/
  // [解析] EXTI = External Interrupt（外部中断），下面打开两个中断向量并设置优先级：
  // [解析]   EXTI4   —— 只管 4 号引脚线，也就是 PC4（加速度计 INT1）；
  // [解析]   EXTI9_5 —— 管 5~9 号引脚线，共用一个中断向量，本工程用到其中的 PC5（陀螺仪 INT1）。
  // [解析]   中断向量之所以共享，是因为 STM32 的 EXTI 硬件就是这么连的，
  // [解析]   所以进 EXTI9_5 的处理函数后要靠"读引脚状态"来区分到底是 5~9 里的哪一个触发的。
  // [解析] 参数 (抢占优先级 5, 子优先级 0)：本工程 NVIC 分组是 NVIC_PRIORITYGROUP_4
  // [解析]   （HAL_Init 里设置，4 位抢占 / 0 位子优先级），子优先级不起作用；
  // [解析]   抢占优先级 5 与 CAN1/USART3 相同，属于"允许调用 FreeRTOS API"的级别。
  // [解析] 注意：本工程并没有实现 HAL_GPIO_EXTI_Callback()，所以这两路中断进来后
  // [解析]   只是把标志位清掉就返回，不会做任何处理；IMU 实际是靠 imu_task 定时轮询读数据的
  // [解析]   （详见 Core/Src/stm32f4xx_it.c 里对 EXTI4_IRQHandler 的说明）。
  HAL_NVIC_SetPriority(EXTI4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 2 */
// [解析] 用户代码保护区：CubeMX 重新 GENERATE CODE 时，只有夹在 BEGIN/END 之间的内容会被保留，
// [解析]   标记之外的一切都会被重新生成覆盖掉。本工程这里保持空白（业务代码都在 applications/ 里）。
/* USER CODE END 2 */
