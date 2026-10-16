// [解析] ==========================================================================
// [解析] 【文件】Core/Inc/adc.h
// [解析] 【作用】ADC3（模数转换器）外设的"对外声明"头文件，里面只有两样东西：
// [解析]         1) 全局句柄 extern ADC_HandleTypeDef hadc3;（真正的变量定义在 adc.c）
// [解析]         2) 初始化函数原型 void MX_ADC3_Init(void);（函数实现也在 adc.c）
// [解析] 【谁生成】CubeMX 自动生成。作者没有手改过本文件（本文件里的用户保护区全是空的），
// [解析]           作者写的相关代码在 applications/voltage_detect.cpp 里。
// [解析] 【本工程用途】ADC3 只用了 1 个通道：PF10 = ADC3_IN8，接电池分压检测电路。
// [解析]               applications/voltage_detect.cpp 里用 sp::Adc battery(&hadc3, 12, 3.3f, 222/22, 1.0f)
// [解析]               拿到这个句柄读电池电压，用来做低电压保护（电压太低会伤电池）。
// [解析] 【关键点】① 头文件里只有"声明"，看不到任何配置参数；想看懂配置必须去读 Core/Src/adc.c。
// [解析]           ② USER CODE BEGIN/END 是 CubeMX 的"用户代码保护区"，机制见下面的详细说明。
// [解析] ==========================================================================

/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    adc.h
  * @brief   This file contains all the function prototypes for
  *          the adc.c file
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
// [解析] ↑ 上面这一大段是 CubeMX 生成的固定版权注释，@file / @brief 是 Doxygen 文档风格，
// [解析]   对编译没有任何影响，只是给文档工具看的。下面开始才是真正的 C 代码。
// [解析]   ▼ 下面是必须先搞清楚的机制：
// [解析] 【CubeMX 用户代码保护区 · 本工程最重要的规矩，务必看懂】
// [解析]   本文件（以及所有 CubeMX 生成的文件）里会出现很多成对的标记：
// [解析]       /* USER CODE BEGIN xxx */   ...   /* USER CODE END xxx */
// [解析]   夹在这一对标记之间的内容叫"用户代码保护区"。在 CubeMX 里点 GENERATE CODE
// [解析]   重新生成代码时的规则是：
// [解析]     · 保护区【之内】的内容：原样保留，你自己写的代码不会丢；
// [解析]     · 保护区【之外】的内容：全部按 .ioc 里的配置重新生成，手改的部分会被覆盖掉。
// [解析]   结论：自己写的代码一定要写在 BEGIN/END 之间！写在外面等于白写。
// [解析]   本文件里的几个保护区目前都是空的，因为外设配置全部交给 CubeMX 管理了。
/* Define to prevent recursive inclusion -------------------------------------*/
// [解析] ↓ 条件编译"包含卫士"(include guard)：防止同一个头文件被 #include 两次导致重复定义。
// [解析]   第一次包含时 __ADC_H__ 还没定义 -> 条件成立 -> 中间的内容被编译；
// [解析]   第二次包含时 __ADC_H__ 已经有定义 -> 整段被跳过。
// [解析]   注意 __ADC_H__ 这种"前后双下划线"的名字是编译器保留命名，自己起宏名不要模仿。
#ifndef __ADC_H__
#define __ADC_H__

// [解析] ↓ extern "C"：本工程主体是 C++（applications/ 下都是 .cpp），而这里是 C 代码。
// [解析]   C++ 为了支持函数重载会对函数名做"改名"(name mangling)，如果 C++ 文件直接引用
// [解析]   这个头文件，就会按 C++ 的名字去找函数，链接时报 undefined reference。
// [解析]   用 extern "C" 包起来等于告诉编译器"下面这些按 C 的规则导出符号名"。
// [解析]   配对的 #endif 在文件末尾，注意别把它们拆散。
#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
// [解析] main.h 是所有外设头文件的公共入口：它内部 include 了 stm32f4xx_hal.h（HAL 库总头文件），
// [解析] 并声明了 Error_Handler()、定义了各种引脚宏。正因为包含了它，
// [解析] 下面才能直接使用 ADC_HandleTypeDef 这个类型名。
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */
// [解析] ↑ 上面这三对 USER CODE BEGIN/END Includes / Private defines / Prototypes 块都是
// [解析]   CubeMX 留的保护区（目前里面全是空的）。它们的约定用途是：
// [解析]     Includes        -> 自己额外要 include 的头文件；
// [解析]     Private defines -> 自己额外的 #define 宏；
// [解析]     Prototypes      -> 自己额外的函数原型声明。
// [解析]   本工程习惯把真正的代码放到 applications/ 里，所以这些块一直空着。

extern ADC_HandleTypeDef hadc3;
// [解析] ↑ extern 表示"这个变量在别的文件里定义"，此处只是声明，不分配内存。
// [解析]   它的本体在 Core/Src/adc.c：ADC_HandleTypeDef hadc3;
// [解析]   ADC_HandleTypeDef 是 HAL 的 ADC 句柄结构体，主要成员：
// [解析]     Instance  —— 指向哪个外设（这里 = ADC3，即 ADC3 的寄存器基地址）；
// [解析]     Init      —— 分辨率/时钟分频/采样时间等配置（在 MX_ADC3_Init 里填）；
// [解析]     State / ErrorCode —— HAL 运行时维护的状态和错误码。
// [解析]   所有 HAL_ADC_xxx() 都要把这个句柄当第一个参数传进去。

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

// [解析] ★ MX_ADC3_Init：ADC3 初始化函数，实现见 Core/Src/adc.c。它一共做了这几件事：
// [解析]   1) 使能 ADC3 的时钟和 GPIOF 的时钟（不使能时钟，外设/引脚就是不工作的）；
// [解析]   2) 把 PF10 配成 GPIO_MODE_ANALOG（模拟输入模式，绕开数字输入电路，ADC 专用）；
// [解析]   3) 分辨率设成 12 位 -> 读数范围 0~4095，对应 0~3.3V，1 个 LSB ≈ 0.806 mV；
// [解析]   4) 时钟分频 ADC_CLOCK_SYNC_PCLK_DIV4：ADC 时钟 = APB2(84MHz)/4 = 21 MHz
// [解析]      （F4 的 ADC 时钟上限是 36 MHz，21 MHz 安全，且分频越大转换越慢、越准）；
// [解析]   5) 扫描模式关、非连续模式关、外部触发关（用软件启动）、数据右对齐；
// [解析]   6) 规则组只放 1 个转换：通道 ADC_CHANNEL_8、排序 Rank=1、采样时间 3 个 ADC 周期。
// [解析] "MX_" 是 CubeMX 给外设初始化函数加的固定前缀，main() 里会按 .ioc 的顺序调用它。
void MX_ADC3_Init(void);
// [解析] 注意：docs/C板引脚配置说明.md 第 42 行写的是"采样时间 56 cycles"，
// [解析]   但 Core/Src/adc.c 里实际生成的是 ADC_SAMPLETIME_3CYCLES（3 个 ADC 时钟周期）。
// [解析]   两处不一致，以代码为准。采样时间 = ADC 内部采样电容充电的时间：
// [解析]   电池分压电路等效内阻较大，3 个周期偏短，可能让电压读数跳动或偏低；
// [解析]   若发现电压读数不稳，建议在 CubeMX 里把 Sampling Time 改大（如 56 / 84 cycles）。
// [解析]   （此处只作提示，未改动任何代码。）

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

// [解析] ↑ 上面这对 #ifdef __cplusplus / #endif 结束了文件开头开始的 extern "C" 块。
// [解析] ↓ 下面这行结束了文件开头的 #ifndef __ADC_H__ 包含卫士。两者都必须成对出现，
// [解析]   少一个就会编译报错（预处理器不匹配）。
#endif /* __ADC_H__ */

