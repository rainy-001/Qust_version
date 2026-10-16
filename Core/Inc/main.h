// [解析] ==========================================================================
// [解析] 【文件】Core/Inc/main.h
// [解析] 【作用】工程的"公共头文件"：声明全局错误处理函数 Error_Handler()，
// [解析]         并用 #define 给每个自己命名的 GPIO 引脚起一个见名知意的别名
// [解析]         （例如 IMU_CS1_ACCEL_Pin 实际就是 PA4）。它被 main.c、freertos.c、
// [解析]         gpio.c、stm32f4xx_it.c 等多个文件共同包含。
// [解析] 【谁生成】STM32CubeMX 自动生成。引脚宏来自 .ioc 里各个引脚的 User Label 设置；
// [解析]           作者没有手改过本文件（下面几个 USER CODE 保护区都是空的）。
// [解析] 【关键点】
// [解析]   1) 引脚宏只是"把 PA4 这种原始信息换个好记的名字"，并不配置引脚。
// [解析]      真正决定引脚是输入/输出/复用/中断的代码在 Core/Src/gpio.c 的 MX_GPIO_Init() 里。
// [解析]   2) 每个引脚都有两个宏：xxx_Pin（第几号引脚）和 xxx_GPIO_Port（哪个端口），
// [解析]      HAL 的 GPIO 函数必须成对传这两个参数，只传一个就会操作到错误的引脚。
// [解析]   3) 引脚分配对不对，以 docs/C板引脚配置说明.md 为权威依据。
// [解析] ==========================================================================
/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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
// [解析] ↑ 上面是 CubeMX 自动生成的"文件说明注释块"（@file / @brief / 版权），属于模板文字，
// [解析]   可以原样保留不去管它。它也是夹在 USER CODE Header 保护区里的，重新生成不会丢。
/* USER CODE END Header */

// [解析] "防重复包含"保护：__MAIN_H 这个宏第一次进来时还没定义，于是执行 #define 并展开内容；
// [解析]   如果同一个 .c 文件间接包含了 main.h 两次，第二次 #ifndef 就为假，内容不会再展开，
// [解析]   从而避免出现"同一个变量/结构体被定义两次"的编译错误。
// [解析]   宏名习惯上跟文件名对应，所以叫 __MAIN_H（前后双下划线是保留给编译器用的命名风格，
// [解析]   这里只是历史习惯，无实际影响）。
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

// [解析] ★ 这段是"C 与 C++ 混编"的固定写法，本工程必须要它 ★
// [解析] 背景：本工程的业务代码是 C++（applications/*.cpp、sp_middleware/*.hpp），
// [解析]   而 CubeMX 生成的 HAL 和 main.c 是 C。C++ 编译器会对函数名做"名字改编"(name mangling)，
// [解析]   同一个函数在 C 和 C++ 里的符号名不一样，直接混用会出现"未定义引用"的链接错误。
// [解析] extern "C" { ... } 的作用就是告诉 C++ 编译器："这中间声明的函数按 C 的规则命名"。
// [解析] #ifdef __cplusplus 则保证用 C 编译器编译时看不到 extern "C"（C 语言里没这个关键字）。
// [解析] 文件末尾有一对配对的 } 和 #endif。
#ifdef __cplusplus
extern "C" {
#endif

// [解析] stm32f4xx_hal.h 是 HAL 库的"总头文件"，它会根据 Core/Inc/stm32f4xx_hal_conf.h
// [解析]   里开启的模块（HAL_ADC_MODULE_ENABLED 等），把对应的 HAL 子头文件全部包含进来。
// [解析] 所以只要有这一行，各外设的句柄类型（UART_HandleTypeDef 等）和 HAL_xxx() 函数声明
// [解析]   就都能用了，不用一个一个手动 include。
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
// [解析] ★★ CubeMX"用户代码保护区"机制（本文件有 6 处，规则完全一样）★★
// [解析]   在 CubeMX 里重新 GENERATE CODE 时，只有夹在
// [解析]     /* USER CODE BEGIN 名字 */ 与 /* USER CODE END 名字 */  之间的内容会被保留，
// [解析]   标记【外面】的代码全部按 .ioc 的配置重新生成覆盖。
// [解析]   ⇒ 结论：自己写的代码一定要写在保护区里面！写在外面，下次生成就没了。
// [解析]   这里这一处叫 Includes，用途是"补充自己要 include 的头文件"（本工程为空）。
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

// [解析] 下面连着 4 个空保护区，是 CubeMX 固定留出的"分类抽屉"，本工程都还没往里放东西：
// [解析]   ET  = Exported Types        —— 放自己定义的 struct / enum
// [解析]   EC  = Exported Constants    —— 放自己定义的常量
// [解析]   EM  = Exported Macro        —— 放自己写的宏
// [解析]   EFP = Exported Function Prototypes —— 放自己写的函数声明
// [解析] 想加代码就放进对应的抽屉；只要在抽屉里，重新生成代码就不会丢。
/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
// [解析] 全局错误处理函数。这里只有"声明"（告诉别的文件有这么一个函数），
// [解析]   函数体在 Core/Src/main.c 里：关掉所有中断然后死循环。
// [解析] 谁会调用它：所有 MX_xxx_Init() 里，只要 HAL 返回不是 HAL_OK（参数非法、硬件没就绪等）
// [解析]   就会调用它。这是"安全停机"策略 —— 宁可停住，也不要带着错误配置去驱动电机。
// [解析] 参数：无。返回值：void（实际永远不会返回）。
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

// [解析] --------- 引脚别名（CubeMX 按 .ioc 里每个引脚的 User Label 自动生成） ---------
// [解析] 命名规律：<标签>_Pin = 引脚号（GPIO_PIN_x），<标签>_GPIO_Port = 端口（GPIOA/GPIOB/…）。
// [解析] 本工程定义了 4 个引脚 + 2 个外部中断号，全部属于 BMI088 六轴传感器：
// [解析]   IMU_CS1_ACCEL : PA4 —— 加速度计片选（原理图网络名 CS1_Accel），拉低才选中该芯片
// [解析]   IMU_CS2_GYRO  : PB0 —— 陀螺仪片选（原理图网络名 CS1_Gyro）
// [解析]   INT1_ACCEL    : PC4 —— 加速度计中断输出，走 EXTI4
// [解析]   INT1_GYRO     : PC5 —— 陀螺仪中断输出，走 EXTI9_5
// [解析] 这些宏被谁用：Core/Src/gpio.c（配置引脚）、Core/Src/stm32f4xx_it.c（中断里判断来源）。
// [解析] 注意：applications/imu_task.cpp 构造 bmi088 对象时是【直接写 GPIOA, GPIO_PIN_4】
// [解析]   而没有用这些宏 —— 也就是说改引脚要改两处，别只改这里就以为完事了。
// [解析] 注意：两个片选宏的编号(CS1/CS2)与原理图网络名(CS1_Accel/CS1_Gyro)并不一致
// [解析]   （陀螺仪在原理图上叫 CS1_Gyro，宏里却写成 CS2_GYRO），属于历史遗留的命名不一致，
// [解析]   功能上没有问题（以实际引脚号为准），但看代码时不要被名字带偏。
/* Private defines -----------------------------------------------------------*/
#define IMU_CS1_ACCEL_Pin GPIO_PIN_4
#define IMU_CS1_ACCEL_GPIO_Port GPIOA
#define INT1_ACCEL_Pin GPIO_PIN_4
#define INT1_ACCEL_GPIO_Port GPIOC
// [解析] ↓ 这两个宏是"外部中断的中断号(IRQn)"，传给 HAL_NVIC_EnableIRQ() 用来开中断，
// [解析]   同时也是中断服务函数名的来源（EXTI4_IRQHandler / EXTI9_5_IRQHandler）。
// [解析] 引脚与中断号的对应关系：PC4 → EXTI 线 4，独享 EXTI4_IRQn；
// [解析]   PC5 → EXTI 线 5，而 EXTI 线 5~9 共用同一个中断向量 EXTI9_5_IRQn，
// [解析]   所以陀螺仪的中断进来时，还要在服务函数里判断到底是不是 PC5 引起的。
#define INT1_ACCEL_EXTI_IRQn EXTI4_IRQn
#define INT1_GYRO_Pin GPIO_PIN_5
#define INT1_GYRO_GPIO_Port GPIOC
#define INT1_GYRO_EXTI_IRQn EXTI9_5_IRQn
#define IMU_CS2_GYRO_Pin GPIO_PIN_0
#define IMU_CS2_GYRO_GPIO_Port GPIOB

// [解析] 又一处保护区（Private defines）：可以在这里补自己的宏定义，本工程为空。
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

// [解析] 与文件开头 #ifdef __cplusplus / extern "C" { 配对：这里关掉 extern "C" 的作用域，
// [解析]   让本文件后面（如果有）的内容恢复 C++ 的默认命名规则。配对写错会引发链接错误。
#ifdef __cplusplus
}
#endif

// [解析] 与文件开头的 #ifndef __MAIN_H 配对，结束"防重复包含"保护。
#endif /* __MAIN_H */
