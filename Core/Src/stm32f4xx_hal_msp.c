// [解析] ==========================================================================
// [解析] 【文件】Core/Src/stm32f4xx_hal_msp.c
// [解析] 【作用】MSP = MCU Support Package（单片机支持包）。HAL 库把"初始化一个外设"拆成两层：
// [解析]         第一层：这个外设自己怎么工作（波特率、分频、工作模式）→ 各外设文件里的 MX_xxx_Init()；
// [解析]         第二层：它要占用哪些"芯片底层资源"（引脚复用、外设时钟、NVIC 中断、DMA 通道）→ 就是 MSP。
// [解析]         本文件只放"全局 MSP"（所有外设共同的那部分），也就是 HAL_MspInit() 一个函数。
// [解析]         每个外设专属的 HAL_xxx_MspInit() 被 CubeMX 生成到了 adc.c / can.c / spi.c / tim.c / usart.c 里，
// [解析]         所以在本文件里看不到 ADC/CAN/SPI 的引脚配置，别以为代码丢了。
// [解析] 【谁生成】CubeMX 自动生成。全文没有作者手改的痕迹；两处 USER CODE BEGIN 保护区目前都是空的。
// [解析] 【关键点】
// [解析]   1) HAL_MspInit() 不是谁手动调用的，是 HAL 库在 HAL_Init() 内部回调的（main.c 第 82 行）；
// [解析]   2) 它执行得比 SystemClock_Config() 还早，此时主频还是复位默认的 HSI 16 MHz；
// [解析]   3) 想加自己的代码，必须写进 USER CODE BEGIN/END 保护区里，否则下次 GENERATE CODE 会被覆盖删除。
// [解析] ==========================================================================

/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file         stm32f4xx_hal_msp.c
  * @brief        This file provides code for the MSP Initialization
  *               and de-Initialization codes.
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
// [解析] 本文件只 include 了 main.h：main.h 里已经包含了 stm32f4xx_hal.h（HAL 库总头文件）
// [解析] 和各引脚的宏定义（引脚号 + GPIO 端口），所以这里不用再一个一个 include。
#include "main.h"
/* USER CODE BEGIN Includes */

// [解析] 保护区：需要额外 include 的自定义头文件写在这两行之间，不要写到保护区外面。
/* USER CODE END Includes */

// [解析] 下面这一串是 CubeMX 的"分区占位符"：
// [解析]   Private typedef(TD) / define / macro / variables(PV) / function prototypes(PFP) /
// [解析]   External functions，以及文件末尾的 USER CODE BEGIN 0、1。
// [解析]   作用是把代码位置固定下来，方便 CubeMX 每次生成时找到"该往哪儿插"。
// [解析] 每一对 USER CODE BEGIN xxx / USER CODE END xxx 之间称为"用户代码保护区"：
// [解析]   用 CubeMX 重新 GENERATE CODE 时，只有夹在这对标记之间的内容会被原样保留，
// [解析]   标记之外的一切（包括下面 HAL_MspInit 的函数体）都会被重新生成、覆盖掉。
// [解析] 所以结论是：自己写的代码一定要写在保护区里面！本文件这几处保护区目前都是空的。
/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN Define */

/* USER CODE END Define */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN Macro */

/* USER CODE END Macro */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* External functions --------------------------------------------------------*/
/* USER CODE BEGIN ExternalFunctions */

/* USER CODE END ExternalFunctions */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */
// [解析] 全局 MSP 初始化：只做"所有外设都需要的"底层初始化，不针对某一个外设。
// [解析] 调用时机 —— HAL 库自动回调，不用你手动调：
// [解析]   main() -> HAL_Init() -> 设中断优先级分组 -> HAL_InitTick() -> HAL_MspInit()（就是本函数）
// [解析]   -> 回到 main() -> SystemClock_Config()。所以它比时钟配置执行得更早。
// [解析] 无参数、无返回值，属于"开机必经之路"；HAL 库里它是 __weak 空函数，本文件提供强定义覆盖它。
/**
  * Initializes the Global MSP.
  */
void HAL_MspInit(void)
{

  /* USER CODE BEGIN MspInit 0 */

  // [解析] 保护区（函数开头）。想在开外设时钟之前做点什么，就写在这两行之间。
  /* USER CODE END MspInit 0 */

  // [解析] SYSCFG = System Configuration Controller（系统配置控制器）。
  // [解析] 它管的是"外部中断线 EXTI 与 GPIO 端口的映射"（寄存器 SYSCFG_EXTICR）。
  // [解析] 本工程 PC4 / PC5 两个 IMU 中断脚要走 EXTI4 / EXTI9_5，
  // [解析] 所以必须先给 SYSCFG 开时钟，否则中断线映射不生效、中断永远不来。
  __HAL_RCC_SYSCFG_CLK_ENABLE();
  // [解析] PWR = 电源控制。给"调压器电压等级（Scale1）"和备份域访问用；
  // [解析] 168 MHz 高频运行必须配 Scale1，见 main.c 的 SystemClock_Config()。
  // [解析] 注意：SystemClock_Config() 里会再开一次，但重复开同一时钟是无害的（幂等）。
  __HAL_RCC_PWR_CLK_ENABLE();

  /* System interrupt init*/
  /* PendSV_IRQn interrupt configuration */
  // [解析] 给 PendSV 异常设优先级：抢占优先级 = 15，子优先级 = 0。
  // [解析] 为什么给最低的 15？因为 FreeRTOS 靠 PendSV 完成"任务切换"，
  // [解析]   它必须能被任何外设中断打断/推迟，否则中断响应会被拖慢，所以优先级最低。
  // [解析] 15 要和 FreeRTOSConfig.h 里的 configKERNEL_INTERRUPT_PRIORITY
  // [解析]   （= configLIBRARY_LOWEST_INTERRUPT_PRIORITY 15 << (8-4)）对应，否则任务切换会出错。
  // [解析] 补充：HAL_Init() 里已经设过"优先级分组 = NVIC_PRIORITYGROUP_4"，
  // [解析]   即 4 位全给抢占优先级、0 位给子优先级，所以第二个参数 0 在分组 4 下没有实际作用。
  HAL_NVIC_SetPriority(PendSV_IRQn, 15, 0);

  /* USER CODE BEGIN MspInit 1 */

  // [解析] 保护区（函数结尾）。要加自己的全局初始化代码，写在这两行之间最合适。
  /* USER CODE END MspInit 1 */
}

/* USER CODE BEGIN 1 */

// [解析] 文件最末尾的保护区：可以在这里追加自己写的函数（比如自定义的 MSP 初始化辅助函数）。
/* USER CODE END 1 */
