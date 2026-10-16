// [解析] ==========================================================================
// [解析] 【文件】Core/Src/main.c
// [解析] 【作用】整个程序的入口。负责"把芯片和外设初始化好，然后启动 FreeRTOS 调度器"。
// [解析] 【谁生成】CubeMX 自动生成骨架，用户可以在 USER CODE 保护区里加自己的代码。
// [解析] 【阅读顺序建议】这是理解整个工程的最佳起点，因为下面 main() 里的调用顺序
// [解析]                 就是"程序从上电到跑起来"的真实顺序。
// [解析] --------------------------------------------------------------------------
// [解析] ★ 满屏的 /* USER CODE BEGIN xxx */ 和 /* USER CODE END xxx */ 是什么？
// [解析]   这是 CubeMX 的"用户代码保护区"。在 CubeMX 里重新 GENERATE CODE 时，
// [解析]   只有夹在这对标记【之间】的东西会被保留，标记【外面】的全会被覆盖重写。
// [解析]   所以：你自己写的代码必须放进保护区，否则下次生成就丢了。
// [解析]   本文件里 main() 中的大部分内容都在保护区外 —— 那是 CubeMX 自己维护的，
// [解析]   不要手改；要加东西就加到 BEGIN/END 之间。
// [解析] ==========================================================================

/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
// [解析] main.h 里包含 stm32f4xx_hal.h 以及各引脚的宏定义（如 INT1_ACCEL_Pin）。
#include "main.h"
// [解析] cmsis_os.h 是 CMSIS-RTOS 的封装层，提供 osKernelStart()、osDelay() 这些接口。
#include "cmsis_os.h"
// [解析] 下面这几个头文件各自声明了对应外设的初始化函数和句柄（hcan1、huart3 ...）。
#include "adc.h"
#include "can.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
// [解析] 这里原本是放全局变量的地方，但本工程把变量都放在了各自的任务文件里，
// [解析] 所以保护区是空的。

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
// [解析] MX_FREERTOS_Init() 定义在 Core/Src/freertos.c，负责创建那 6 个任务。
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
// [解析] ★★ 程序真正的入口。上电/复位后，启动文件(startup_stm32f407xx.s)会先做
// [解析]   最小化的初始化（设置栈指针、拷贝 .data、清零 .bss），然后跳到 main()。
// [解析]   返回值其实毫无意义 —— 因为下面 osKernelStart() 永远不会返回。
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  // [解析] 【第 1 步】HAL_Init()：复位所有外设、初始化 Flash 接口和 SysTick 中断。
  // [解析]   它还会把中断优先级分组设为 NVIC_PRIORITYGROUP_4（4 位全部给抢占优先级）。
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  // [解析] 【第 2 步】配置系统时钟。必须在任何外设初始化之前做完，
  // [解析]   因为后面所有外设的波特率、分频都依赖最终的主频（168MHz）来算。
  // [解析]   具体怎么配的在文件下半部分的 SystemClock_Config() 里，那里有详细说明。
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  // [解析] 【第 3 步】初始化各个外设。注意顺序有讲究：DMA 要排在使用它的外设之前，
  // [解析]   所以 MX_GPIO_Init 之后紧跟着 MX_DMA_Init，然后才是 CAN/SPI/UART。
  MX_GPIO_Init();
  // [解析] ↑ 把用到的 GPIO 配成输入/输出/复用/外部中断。
  MX_DMA_Init();
  // [解析] ↑ 使能 DMA 控制器时钟，并配置 DMA 中断优先级。USART3 的接收就靠它。
  MX_CAN1_Init();
  // [解析] ↑ CAN1：接电机总线，1Mbps。
  MX_SPI1_Init();
  // [解析] ↑ SPI1：和 BMI088 惯性传感器通信。
  MX_TIM4_Init();
  // [解析] ↑ TIM4：用它的 CH3 输出 PWM 驱动蜂鸣器（PD14）。
  MX_TIM5_Init();
  // [解析] ↑ TIM5：用它的 CH1/2/3 输出 PWM 驱动 RGB 灯（PH10/11/12）。
  MX_USART3_UART_Init();
  // [解析] ↑ USART3：接 DBus 遥控器接收机，波特率 100000。
  MX_ADC3_Init();
  // [解析] ↑ ADC3：读电池电压（PF10 / ADC3_IN8）。
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Call init function for freertos objects (in cmsis_os2.c) */
  // [解析] 【第 4 步】创建 FreeRTOS 的任务、队列等对象。
  // [解析]   注意：这里只是"创建"，任务还不会跑起来，因为调度器还没启动。
  // [解析]   本工程会创建 6 个任务：LED / IMU / CAN / DBUS / BUZZER / ADC。
  MX_FREERTOS_Init();

  /* Start scheduler */
  // [解析] 【第 5 步】★ 启动 FreeRTOS 调度器。从这一行开始，
  // [解析]   程序的执行权就完全交给 FreeRTOS 了：它会在各个任务之间按优先级轮流切换。
  // [解析]   这个函数正常情况下【永远不会返回】—— 它会挑一个优先级最高的就绪任务跳进去。
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  // [解析] 如果执行真的跑到了下面这个 while(1)，说明调度器启动失败了（异常情况）。
  // [解析] 正常运行时程序会永远停在上面那一行内部的某个任务里。

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
// [解析] ★★ 系统时钟配置：把外部晶振的 12MHz 变成 CPU 要跑的 168MHz。
// [解析]   这是整个工程"时间"的源头，看懂了它，就能明白为什么蜂鸣器要传 84e6、
// [解析]   以及为什么 ADC/串口的那些分频值长成那样。下面是完整的推算过程：
// [解析]
// [解析]   ① 时钟来源：开发板焊的是 12MHz 外部晶振(HSE)。先把它打开。
// [解析]   ② PLL 把低频"倍频"成高频。STM32F4 的 PLL 由 4 个参数决定：
// [解析]        PLLM —— 先分频，把 HSE 降到一个固定的小频率（VCO 输入）
// [解析]                VCO输入 = HSE / PLLM = 12MHz / 6 = 2MHz
// [解析]                （手册要求 VCO 输入必须在 1~2MHz，取 2MHz 最理想）
// [解析]        PLLN —— 再倍频，得到 VCO 输出
// [解析]                VCO输出 = VCO输入 × PLLN = 2MHz × 168 = 336MHz
// [解析]                （手册要求 VCO 输出在 100~432MHz）
// [解析]        PLLP —— 最后分频，得到系统时钟 SYSCLK
// [解析]                SYSCLK = VCO输出 / PLLP = 336MHz / 2 = 168MHz  ← F407 的最高主频
// [解析]        PLLQ —— 专门给 USB / SDIO 用的分频，必须配成 48MHz
// [解析]                PLLQ输出 = VCO输出 / PLLQ = 336MHz / 4 = 84MHz
// [解析]                （注意：本工程没启用 USB；若以后要用视觉通信的 USB CDC，
// [解析]                  这个值需要重新核对，USB 要求精确的 48MHz。）
// [解析]
// [解析]   ③ 各条总线的分频（下面 RCC_ClkInitStruct 那几行）：
// [解析]        AHB  = SYSCLK / 1 = 168MHz   ← 也就是 HCLK，CPU 核心跑的频率
// [解析]        APB1 = HCLK   / 4 =  42MHz   ← 低速外设总线（最大 42MHz）
// [解析]        APB2 = HCLK   / 2 =  84MHz   ← 高速外设总线（最大 84MHz）
// [解析]        ★ 有个重要特例：当 APB 分频系数不为 1 时，接到 APB 上的【定时器】
// [解析]          时钟会自动×2。所以：
// [解析]            APB1 上的定时器时钟 = 42MHz × 2 = 84MHz
// [解析]            APB2 上的定时器时钟 = 84MHz × 2 = 168MHz
// [解析]          这就解释了 applications/buzzer_task.cpp 里为什么写 84e6
// [解析]          （TIM4 挂在 APB1 上），而 TIM5 挂在 APB2 上则按 168MHz 算。
// [解析]
// [解析]   ④ FLASH_LATENCY_5：Flash 读取速度跟不上 168MHz，
// [解析]      必须插入 5 个等待周期（F4 手册规定 150~168MHz 对应 5WS）。
// [解析]      这个值填错会导致程序跑飞或直接 HardFault。
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  // [解析] ↑ 两个配置结构体，先全部清零（{0}），避免里面有随机值导致意外配置。

  /** Configure the main internal regulator output voltage
  */
  // [解析] 内部稳压器调压：168MHz 属于"高性能档"，必须把内核电压调到 Scale 1
  // [解析] （对应大约 1.2V）。电压没跟上就跑不了高频，这是超频类问题的常见坑。
  // [解析] 要先开 PWR 时钟才能写电压相关寄存器，所以先 __HAL_RCC_PWR_CLK_ENABLE()。
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  // [解析] 下面开始填"振荡器/PLL 配置"：
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  // [解析] ↑ 本次要配置的是 HSE（外部高速晶振），不碰内部的 HSI。
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  // [解析] ↑ 打开外部晶振。注意：如果 .ioc 里 PH0/PH1 引脚配置丢了，HSE 实际起不来，
  // [解析]   现象是 PLL 没有输入、程序卡在下面 HAL_RCC_OscConfig 的超时判断里。
  // [解析]   docs/C板引脚配置说明.md 第 8 节记录过这个坑。
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  // [解析] ↑ 打开 PLL（锁相环，就是上面说的倍频器）。
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  // [解析] ↑ PLL 的输入源选 HSE（12MHz 晶振），而不是内部 HSI（16MHz）。
  RCC_OscInitStruct.PLL.PLLM = 6;
  // [解析] ↑ 12MHz / 6 = 2MHz，得到合格的 VCO 输入频率。
  RCC_OscInitStruct.PLL.PLLN = 168;
  // [解析] ↑ 2MHz × 168 = 336MHz，得到 VCO 输出频率。
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  // [解析] ↑ 336MHz / 2 = 168MHz，得到最终系统时钟 SYSCLK。
  RCC_OscInitStruct.PLL.PLLQ = 4;
  // [解析] ↑ 336MHz / 4 = 84MHz，这是给 USB/SDIO 用的分频输出。
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    // [解析] HAL_OK 表示配置成功。失败就进 Error_Handler() 死循环卡住。
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  // [解析] 接着填"总线分频配置"：
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  // [解析] ↑ 这里声明"我要一次性配置这 4 个时钟"，没被点名的不会被改动。
  // [解析]   SYSCLK=系统时钟, HCLK=AHB总线, PCLK1=APB1, PCLK2=APB2。
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  // [解析] ↑ 系统时钟的来源选 PLL（也就是上面 168MHz 那个结果），不用 HSE 直连。
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  // [解析] ↑ AHB 不分频：HCLK = 168MHz / 1 = 168MHz。
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  // [解析] ↑ APB1 = 168MHz / 4 = 42MHz（APB1 最高只能 42MHz，这是它的上限）。
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  // [解析] ↑ APB2 = 168MHz / 2 = 84MHz（APB2 最高 84MHz）。

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    // [解析] HAL_RCC_ClockConfig 同时把 Flash 等待周期设为 5WS。
    // [解析] 第二个参数就是 FLASH_LATENCY_5：168MHz 必须配 5 个等待周期。
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
// [解析] ★ 定时器"周期到"回调 —— 这就是整个工程的 1ms 心跳。
// [解析]   完整来龙去脉：TIM1 每 1ms 溢出一次 -> 触发中断 ->
// [解析]   Core/Src/stm32f4xx_it.c 的 TIM1_UP_TIM10_IRQHandler() ->
// [解析]   HAL_TIM_IRQHandler(&htim1) -> 回调到本函数 -> HAL_IncTick()。
// [解析]   HAL_IncTick() 让全局变量 uwTick 加一，于是 HAL 就有了"毫秒"的概念，
// [解析]   HAL_Delay() 和各种超时判断才能正常工作。
// [解析]   为什么不直接用 SysTick？因为 FreeRTOS 已经把 SysTick 拿去做任务调度了，
// [解析]   二者不能共用，所以 HAL 的时基改用 TIM1。相关配置在
// [解析]   Core/Src/stm32f4xx_hal_timebase_tim.c 里。
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  // [解析] 系统里可能有多个定时器都会进这个回调（比如 TIM4 蜂鸣器、TIM5 灯），
  // [解析] 所以必须先判断"是不是 TIM1"，避免别的定时器把系统时间加错。
  if (htim->Instance == TIM1)
  {
    HAL_IncTick();
    // [解析] ↑ 就这一句：uwTick++，表示"又过了 1 毫秒"。
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
// [解析] 出错处理函数。上面所有的 if (... != HAL_OK) 失败时都会走到这里。
// [解析] 实际现象就是"程序停住不动了"。调试时可以在 __disable_irq() 这一行打断点，
// [解析] 一旦命中，再往回看调用栈，就知道是哪个外设初始化失败了。
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  // [解析] __disable_irq() 关掉所有中断（除了 NMI 和 HardFault），
  // [解析] 然后死循环。这是一种"安全停机"：宁可停住，也不要带着错误继续控制电机。
  // [解析] 常见触发原因：晶振没起振、引脚被别的外设占用、时钟树配置非法。
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
// [解析] 断言失败回调。只有在 stm32f4xx_hal_conf.h 里定义了 USE_FULL_ASSERT
// [解析] 时才会编译进来（本工程默认没开）。HAL 库内部用它检查参数合法性，
// [解析] 比如给 HAL_GPIO_WritePin 传了一个不存在的引脚。调试期建议打开，
// [解析] 能让"参数传错"这种 bug 立刻暴露，而不是变成莫名其妙的现象。
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
