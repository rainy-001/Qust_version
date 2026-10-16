// [解析] ==========================================================================
// [解析] 【文件】Core/Src/stm32f4xx_hal_timebase_tim.c
// [解析] 【作用】把 HAL 库的"1 毫秒时基"从 SysTick 换成硬件定时器 TIM1。
// [解析]         它提供的 HAL_InitTick / HAL_SuspendTick / HAL_ResumeTick 三个函数，
// [解析]         在 HAL 库里原本是 __weak（弱符号）实现，本文件用强定义把它们"顶替"掉。
// [解析] 【为什么要换时基】FreeRTOS 的调度心跳（configTICK_RATE_HZ = 1000，即 1 ms 一次）
// [解析]         必须独占 SysTick 中断；而 HAL 库默认也用 SysTick 来数超时
// [解析]         （HAL_GetTick() / HAL_Delay() 全靠它）。两个都想用同一个中断就会互相打架，
// [解析]         所以把 HAL 的时基挪到 TIM1 —— 这就是 .ioc 里 "Timebase Source = TIM1" 的含义，
// [解析]         也是本工程必须这么配的原因。docs/C板引脚配置说明.md 第 2 节也记了这一点。
// [解析] 【谁生成】CubeMX 根据 .ioc 的时基源选择自动生成。
// [解析]         特别注意：本文件里【没有】任何 USER CODE BEGIN/END 保护区，
// [解析]         也就是说在这里写的任何东西，下次 GENERATE CODE 都会被整个覆盖掉，
// [解析]         所以不要把业务代码写进这个文件。
// [解析] 【关键点】
// [解析]   1) 计数时钟先分频到 1 MHz（1 个计数 = 1 微秒），再用 ARR=999 让它每 1 ms 溢出一次；
// [解析]   2) TIM1 挂在 APB2 上，而本工程 APB2 分频 = 2（不是 1），
// [解析]      按 STM32 的规则此时"定时器时钟 = 2 × PCLK2"，即 2 × 84 = 168 MHz；
// [解析]   3) 本函数会被调用两次：HAL_Init() 里一次（当时还是复位后的 HSI 16 MHz），
// [解析]      时钟切到 PLL 后 HAL_RCC_ClockConfig() 又调一次（必须重算分频，否则时基会不准）；
// [解析]   4) 中断入口在 Core/Src/stm32f4xx_it.c 的 TIM1_UP_TIM10_IRQHandler()，
// [解析]      它最终调到 main.c 里的 HAL_TIM_PeriodElapsedCallback() -> HAL_IncTick()。
// [解析]   5) 注意：TIM1 已经被时基独占，PSC/ARR 固定为 167/999。以后若想用 TIM1_CH1~CH4 输出舵机 PWM
// [解析]      （PE9/PE11/PE13/PE14，见 docs/C板引脚配置说明.md 第 7 节），PWM 频率会被时基锁死在约 1 kHz，
// [解析]      而且不能再随意改 ARR/PSC —— 否则 HAL 的 1 ms 时基（HAL_GetTick/HAL_Delay）就跟着不准了。
// [解析] ==========================================================================

/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_hal_timebase_tim.c
  * @brief   HAL time base based on the hardware TIM.
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
// [解析] stm32f4xx_hal.h 是 HAL 库总头文件（里面声明了 HAL_InitTick 的原型、
// [解析] uwTickPrio 等全局变量）；stm32f4xx_hal_tim.h 提供 TIM_HandleTypeDef 和 TIM1 相关定义。
// [解析] 这两个模块能编译进来的前提是 stm32f4xx_hal_conf.h 里打开了 HAL_TIM_MODULE_ENABLED（本工程已打开）。
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_tim.h"

// [解析] 下面几行是 CubeMX 保留的分区标题（Private typedef / define / macro / variables /
// [解析] function prototypes / functions）。因为本文件里没有 USER CODE 保护区，
// [解析] 所以这些分区现在全是空的、也不需要你往里面加东西。
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
// [解析] 定义（不是声明）TIM1 的句柄。时基本身就是一个普通定时器，所以要有一个句柄结构体。
// [解析] 注意它是全局变量而不是 static：因为 Core/Src/stm32f4xx_it.c 里的
// [解析]   TIM1_UP_TIM10_IRQHandler() 要用 extern TIM_HandleTypeDef htim1; 引用它。
// [解析] 结构体成员在下面 HAL_InitTick() 里被逐个赋值：Instance / Period / Prescaler / ...
TIM_HandleTypeDef        htim1;
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  This function configures the TIM1 as a time base source.
  *         The time source is configured  to have 1ms time base with a dedicated
  *         Tick interrupt priority.
  * @note   This function is called  automatically at the beginning of program after
  *         reset by HAL_Init() or at any time when clock is configured, by HAL_RCC_ClockConfig().
  * @param  TickPriority: Tick interrupt priority.
  * @retval HAL status
  */
// [解析] 时基初始化：把 TIM1 配成"每 1 ms 溢出一次"的定时器，并打开它的更新中断。
// [解析] 参数 TickPriority：时基中断（TIM1 更新中断）要占用的 NVIC 抢占优先级，取值 0~15，
// [解析]   数字越小优先级越高。HAL_Init() 传进来的其实是 stm32f4xx_hal_conf.h 里的 TICK_INT_PRIORITY，
// [解析]   本工程定义为 15U，也就是最低优先级 —— "数时间"不着急，让它排在所有外设中断后面。
// [解析]   还有一层原因：优先级数值必须 ≥ configMAX_SYSCALL_INTERRUPT_PRIORITY（本工程为 5），
// [解析]   这样才允许在这个中断里调用 FreeRTOS 的 FromISR 接口；15 满足条件。
// [解析] 返回值：HAL_OK = 成功；HAL_ERROR = TickPriority 超出 4 位优先级的范围（≥ 16）。
HAL_StatusTypeDef HAL_InitTick(uint32_t TickPriority)
{
  // [解析] clkconfig / pFLatency 用来接收 HAL_RCC_GetClockConfig() 读回的时钟树配置。
  // [解析] 注意：下面虽然把它们读出来了，但函数体里并没有再用 clkconfig 的内容，
  // [解析]   真正算频率用的是 HAL_RCC_GetPCLK2Freq()，所以这个变量属于"取了没用"（CubeMX 模板遗留写法）。
  RCC_ClkInitTypeDef    clkconfig;
  uint32_t              uwTimclock = 0U;

  // [解析] uwPrescalerValue 最终要写进 TIM1 的预分频寄存器 PSC。
  // [解析] pFLatency 用来接收 Flash 等待周期数（本函数中同样没有被使用）。
  uint32_t              uwPrescalerValue = 0U;
  uint32_t              pFLatency;

  // [解析] 接住各 HAL 函数的返回值，最后原样返回给调用者（HAL_Init 或 HAL_RCC_ClockConfig）。
  HAL_StatusTypeDef     status;

  /* Enable TIM1 clock */
  // [解析] 先开 TIM1 的时钟。外设不开时钟，写它的寄存器不会有任何反应（这是新手最常见的"配了没反应"）。
  // [解析] TIM1 是"高级定时器"，挂在 APB2 总线上，对应 RCC_APB2ENR 里的 TIM1EN 位。
  __HAL_RCC_TIM1_CLK_ENABLE();

  /* Get clock configuration */
  // [解析] 读回当前时钟配置（HCLK/PCLK1/PCLK2 各自的分频系数）。如上所述，读回来的值后面没被用到。
  HAL_RCC_GetClockConfig(&clkconfig, &pFLatency);

  /* Compute TIM1 clock */
  // [解析] ↓ 本文件最关键的一行：算出 TIM1 的计数时钟频率，单位 Hz。
  // [解析] STM32F4 的规则：APB 预分频系数 = 1 时，定时器时钟 = PCLK；
  // [解析]   APB 预分频系数 ≠ 1 时，定时器时钟 = 2 × PCLK（硬件自动倍频）。
  // [解析] 本工程 PCLK2 = 84 MHz（APB2 分频 = 2 ≠ 1），所以 TIM1CLK = 2 × 84 = 168 MHz。
  // [解析] 注意：这个写死的 ×2 只在"APB2 分频 ≠ 1"时成立。若把 APB2 分频改成 1，
  // [解析]   这里就必须去掉 ×2，否则算出的 PSC 偏大一倍，时基会慢成 2 ms（HAL_Delay 全部翻倍）。
      uwTimclock = 2*HAL_RCC_GetPCLK2Freq();

  /* Compute the prescaler value to have TIM1 counter clock equal to 1MHz */
  // [解析] 预分频值：PSC = 定时器时钟 / 想要的计数频率 - 1 = 168000000 / 1000000 - 1 = 167。
  // [解析] 即把 168 MHz 分频成 1 MHz —— 此时"计数器加 1"恰好等于 1 微秒。
  // [解析] 为什么要凑成 1 MHz？因为这样 ARR 的值就能直接当微秒用，换算直观：
  // [解析]   想要 1 ms 中断，让 ARR = 1000 - 1 即可。
  // [解析] 记住寄存器里的实际分频是 PSC + 1，所以公式末尾要减 1（最容易记错的地方）。
  uwPrescalerValue = (uint32_t) ((uwTimclock / 1000000U) - 1U);

  /* Initialize TIM1 */
  // [解析] Instance 指明操作哪一路定时器（TIM1）。F4 每个定时器都有独立的一套寄存器。
  htim1.Instance = TIM1;

  /* Initialize TIMx peripheral as follow:
   * Period = [(TIM1CLK/1000) - 1]. to have a (1/1000) s time base.
   * Prescaler = (uwTimclock/1000000 - 1) to have a 1MHz counter clock.
   * ClockDivision = 0
   * Counter direction = Up
   */
  // [解析] Period 就是自动重装载值 ARR，决定"数多少个计数溢出一次"。
  // [解析] (1000000U / 1000U) - 1U = 1000 - 1 = 999；配合上面 1 MHz 的计数时钟：
  // [解析]   溢出周期 = (ARR + 1) / 1 MHz = 1000 / 1 MHz = 1 ms —— 这正是 HAL 想要的 1 ms 时基。
  // [解析]   所以 PSC 和 ARR 必须配套看：计数时钟一旦不是 1 MHz，这个 999 就不再代表 1 ms。
  htim1.Init.Period = (1000000U / 1000U) - 1U;
  // [解析] 把上面算好的 PSC 写进去（本工程为 167）。时钟一改就要重算，这就是本函数被调用两次的原因。
  htim1.Init.Prescaler = uwPrescalerValue;
  // [解析] ClockDivision = 0：这是给输入捕获/数字滤波用的额外分频，纯做时基时用不到，填 0 即可。
  htim1.Init.ClockDivision = 0;
  // [解析] 向上计数模式：0 -> ARR 循环计数，数到 ARR 就产生一次更新事件。
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  // [解析] 关闭"自动重装载预装载"：写 ARR 立即生效，不缓冲到下一个更新事件。
  // [解析] 时基的 ARR 只设一次，关掉更省事、也避免前后两次调用之间出现不期望的周期。
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

  // [解析] 把上面填好的参数真正写进 TIM1 的寄存器，并把状态设为"就绪"。
  // [解析] 它内部会回调 HAL_TIM_Base_MspInit()（一个 __weak 空函数）。本工程没为 TIM1 写这个回调，
  // [解析]   因为 TIM1 只做内部时基、不占用任何 GPIO 引脚，而时钟已经在上面手动开过了。
  status = HAL_TIM_Base_Init(&htim1);
  if (status == HAL_OK)
  {

    /* Start the TIM time Base generation in interrupt mode */
    // [解析] "Start_IT" = 启动定时器并允许它产生中断（这里是更新中断 UIE）。
    // [解析] 如果换成 HAL_TIM_Base_Start()，就只能轮询，不会进中断，uwTick 也就不会自增了。
    status = HAL_TIM_Base_Start_IT(&htim1);
    if (status == HAL_OK)
    {
    /* Enable the TIM1 global Interrupt */
        // [解析] 打开 NVIC 中该中断号的"总开关"。
        // [解析] TIM1_UP_TIM10_IRQn = TIM1 的更新中断向量。F4 上 TIM1_UP 与 TIM10 共用一个入口，
        // [解析]   所以中断服务函数叫 TIM1_UP_TIM10_IRQHandler（见 Core/Src/stm32f4xx_it.c）。
        // [解析] 经验：只置了外设自己的中断允许位、忘了 HAL_NVIC_EnableIRQ，是"中断死活不进"的经典原因。
        HAL_NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);
      /* Configure the SysTick IRQ priority */
      // [解析] ↑ 上面这句原注释说 "SysTick IRQ"，其实是 CubeMX 从 SysTick 版模板改过来时留下的笔误：
      // [解析]   下面配置的是 TIM1 更新中断的优先级，和 SysTick 已经没有关系了（SysTick 归 FreeRTOS）。
      // [解析] 先做范围检查：__NVIC_PRIO_BITS = 4，优先级只有 4 位、范围 0~15，
      // [解析]   所以 TickPriority 必须 < (1 << 4) = 16，否则视为非法参数，直接返回 HAL_ERROR。
      if (TickPriority < (1UL << __NVIC_PRIO_BITS))
      {
        /* Configure the TIM IRQ priority */
        // [解析] 抢占优先级 = TickPriority（本工程 15），子优先级 = 0。
        // [解析] 因为优先级分组是 NVIC_PRIORITYGROUP_4（4 位抢占 + 0 位子优先级），第二个参数不起作用。
        HAL_NVIC_SetPriority(TIM1_UP_TIM10_IRQn, TickPriority, 0U);
        // [解析] 记到 HAL 的全局变量 uwTickPrio 里：HAL_GetTickPrio() 会返回它。
        // [解析] 更实际的作用是——时钟切到 PLL 后 HAL_RCC_ClockConfig() 会拿 uwTickPrio 再调一次
        // [解析]   HAL_InitTick()，这样重算分频时优先级不会丢。
        uwTickPrio = TickPriority;
      }
      else
      {
        status = HAL_ERROR;
      }
    }
  }

 /* Return function status */
  return status;
}

// [解析] 暂停时基：只清掉 TIM1 的"更新中断允许位"，定时器本身仍在计数，
// [解析] 效果是 uwTick 停止自增，也就是 HAL_GetTick() / HAL_Delay() 的时间"冻住"。
// [解析] 典型用途：进低功耗前、或调试时想冻结超时判断。恢复要用下面的 HAL_ResumeTick()。
// [解析] 注意：本工程（FreeRTOS 常驻调度）并没有调用这两个函数；
// [解析]   在任务里想"等一会儿"应该用 osDelay()/vTaskDelay()，别拿 SuspendTick 当延时手段。
/**
  * @brief  Suspend Tick increment.
  * @note   Disable the tick increment by disabling TIM1 update interrupt.
  * @param  None
  * @retval None
  */
void HAL_SuspendTick(void)
{
  /* Disable TIM1 update Interrupt */
  // [解析] __HAL_TIM_DISABLE_IT 是宏，直接改 DIER 寄存器的 UIE 位，比调函数快，适合放在临界区里。
  __HAL_TIM_DISABLE_IT(&htim1, TIM_IT_UPDATE);
}

// [解析] 恢复时基：和 HAL_SuspendTick() 相反，把更新中断允许位重新置上，uwTick 继续自增。
// [解析] 两个函数必须成对使用，否则 HAL_GetTick() 会一直停在暂停时的值，
// [解析]   而依赖超时的 HAL 函数（如 HAL_CAN_AddTxMessage 内部等邮箱）就会永远等不到超时而卡死。
/**
  * @brief  Resume Tick increment.
  * @note   Enable the tick increment by Enabling TIM1 update interrupt.
  * @param  None
  * @retval None
  */
void HAL_ResumeTick(void)
{
  /* Enable TIM1 Update interrupt */
  __HAL_TIM_ENABLE_IT(&htim1, TIM_IT_UPDATE);
}

