// [解析] ==========================================================================
// [解析] 【文件】Core/Src/system_stm32f4xx.c
// [解析] 【作用】芯片"上电后的第一段软件"：在 main() 之前被启动文件调用，负责打开 FPU、
// [解析]         （可选）搬移中断向量表、（可选）初始化外扩存储器；同时对外提供全局变量
// [解析]         SystemCoreClock（CPU 内核频率 HCLK），HAL 延时、波特率计算、FreeRTOS
// [解析]         调度都靠它知道"CPU 现在跑多快"。
// [解析] 【谁生成】CubeMX 自动生成的 ST 官方模板文件，作者没有改动过任何一行。
// [解析] 【执行顺序】复位 -> startup_stm32f407xx.s 的 Reset_Handler
// [解析]             -> SystemInit()（本文件）-> 初始化 C 运行环境 -> main()
// [解析]             -> main() 里的 SystemClock_Config() 才真正把主频配到 168 MHz。
// [解析] 【本文件真正"活着"的代码只有两个函数】
// [解析]   1) SystemInit()            ：上电时打开硬件浮点单元（FPU）。
// [解析]   2) SystemCoreClockUpdate()：反过来读 RCC 寄存器，推算并更新 SystemCoreClock。
// [解析] 【关键点】1. 文件里大半篇幅是"外扩 SDRAM/SRAM"的初始化，本工程两个开关宏都处于
// [解析]             注释状态，所以 SystemInit_ExtMemCtl() 根本不会被编译，属于死代码，
// [解析]             新手可以整段跳过（文中已标出范围）。
// [解析]           2. 本文件里那个 25 MHz 的 HSE_VALUE 并不是本工程用的值！本工程是 12 MHz，
// [解析]             真正生效的定义在 Core/Inc/stm32f4xx_hal_conf.h，详见文件内说明。
// [解析]           3. SystemCoreClock 初值 16 MHz 只是复位默认值（HSI），main() 配好 PLL 后
// [解析]             HAL 会把它更新成 168 MHz；调试时直接 Watch 这个变量即可确认主频。
// [解析] 【和谁配合】时钟树的具体配置在 Core/Src/main.c 的 SystemClock_Config()；
// [解析]             HAL 的 1 ms 时基改用了 TIM1，代码在 Core/Src/stm32f4xx_hal_timebase_tim.c
// [解析]             （因为 SysTick 被 FreeRTOS 占用了）。本文件只负责打好底层地基。
// [解析] ==========================================================================

/**
  ******************************************************************************
  * @file    system_stm32f4xx.c
  * @author  MCD Application Team
  * @brief   CMSIS Cortex-M4 Device Peripheral Access Layer System Source File.
  *
  *   This file provides two functions and one global variable to be called from 
  *   user application:
  *      - SystemInit(): This function is called at startup just after reset and 
  *                      before branch to main program. This call is made inside
  *                      the "startup_stm32f4xx.s" file.
  *
  *      - SystemCoreClock variable: Contains the core clock (HCLK), it can be used
  *                                  by the user application to setup the SysTick 
  *                                  timer or configure other parameters.
  *                                     
  *      - SystemCoreClockUpdate(): Updates the variable SystemCoreClock and must
  *                                 be called whenever the core clock is changed
  *                                 during program execution.
  *
  *
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2017 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/** @addtogroup CMSIS
  * @{
  */

/** @addtogroup stm32f4xx_system
  * @{
  */  
  
/** @addtogroup STM32F4xx_System_Private_Includes
  * @{
  */


// [解析] stm32f4xx.h 是 CMSIS 的"器件头文件"，本文件后面用到的 RCC、SCB、FMC_Bank5_6、
// [解析] SRAM_BASE/FLASH_BASE 这些名字全部来自它（它内部定义了一整套与寄存器一一对应的结构体）。
// [解析] 因为工程编译时带了 -DUSE_HAL_DRIVER，它还会顺手把 stm32f4xx_hal.h -> stm32f4xx_hal_conf.h
// [解析] 一起包含进来；下面 HSE_VALUE / HSI_VALUE 的真正取值就是从那两个文件传过来的。
#include "stm32f4xx.h"

#if !defined  (HSE_VALUE) 
// [解析] ★ 一个极易踩的坑：这里写的 25 MHz 并不是本工程用的晶振频率！
// [解析] 本工程（开发板 C 型）用的是 12 MHz 晶振，见 Core/Src/main.c 的 SystemClock_Config()。
// [解析] 25 MHz 是 ST 给所有 F4 板子留的"兜底默认值"，只有当别处都没有定义过 HSE_VALUE 时
// [解析] 才轮到它生效。而 Core/Inc/stm32f4xx_hal_conf.h 里已经明确定义了：
// [解析]     #define HSE_VALUE    12000000U
// [解析] 加上本文件在上一行已经先 #include 了 stm32f4xx.h（进而包含 hal_conf.h），
// [解析] 所以这个 #if !defined(HSE_VALUE) 判断为假，25 MHz 那一行被"跳过"，实际不生效。
// [解析] 为什么必须搞清楚它：SystemCoreClockUpdate() 用 HSE_VALUE 反推主频，HAL 库算串口
// [解析] 波特率、CAN 位时序时也依赖它。晶振与这个值对不上时程序不报错、能跑，但所有
// [解析] 波特率、延时、定时全部按错误比例偏掉，是很难查的一类 bug。
  #define HSE_VALUE    ((uint32_t)25000000) /*!< Default value of the External oscillator in Hz */
#endif /* HSE_VALUE */

#if !defined  (HSI_VALUE)
// [解析] HSI = High Speed Internal，芯片【内部】的 RC 振荡器，出厂标称 16 MHz。
// [解析] 它不需要外部晶振就能工作，所以复位后 CPU 默认就跑在它上面（这就是 SystemCoreClock
// [解析] 初值为 16000000 的原因）。缺点是精度差（随温度和电压漂移，通常 ±1% 以上），
// [解析] 拿它当串口/CAN 的时钟源容易通信出错，所以本工程配好时钟后就切到外部的 HSE + PLL 了。
// [解析] HSI_VALUE 在这里同样只是兜底默认值，hal_conf.h 里也定义了 16000000U，两者一致。
  #define HSI_VALUE    ((uint32_t)16000000) /*!< Value of the Internal oscillator in Hz*/
#endif /* HSI_VALUE */

/**
  * @}
  */

/** @addtogroup STM32F4xx_System_Private_TypesDefinitions
  * @{
  */

/**
  * @}
  */

/** @addtogroup STM32F4xx_System_Private_Defines
  * @{
  */

/************************* Miscellaneous Configuration ************************/
// [解析] 从这里开始是"杂项配置"，全部靠宏开关控制，没有可执行代码。
// [解析] ★ DATA_IN_ExtSRAM / DATA_IN_ExtSDRAM 这两个开关的含义是：
// [解析]   把板子上【外扩】的 SRAM / SDRAM 芯片当作主数据内存使用 —— 也就是堆(heap)、
// [解析]   栈(stack)、全局变量全部放到外部芯片里（片内 192 KB 不够用时才这么做）。
// [解析]   代价是必须配套改链接脚本(linker script)里 RAM 的地址和长度，否则程序跑飞。
// [解析] 本工程这两个宏【都被注释掉了】：C 板没有外扩存储器，全部变量都放在片内 SRAM 里。
// [解析] 因此下面那一大段 SystemInit_ExtMemCtl() 不会被编译，属于死代码。
// [解析] 外围这一圈 #if defined(STM32F4xx) 是 ST 为了让同一个文件适配所有 F4 型号而写的
// [解析] 型号判断，本工程编译时定义了 STM32F407xx（属于第一行的 F405/F415/F407/F417 一档），
// [解析] 条件成立；但里面真正的那句 #define 是注释，所以开关依然是"关"。
/*!< Uncomment the following line if you need to use external SRAM or SDRAM as data memory  */
#if defined(STM32F405xx) || defined(STM32F415xx) || defined(STM32F407xx) || defined(STM32F417xx)\
 || defined(STM32F427xx) || defined(STM32F437xx) || defined(STM32F429xx) || defined(STM32F439xx)\
 || defined(STM32F469xx) || defined(STM32F479xx) || defined(STM32F412Zx) || defined(STM32F412Vx)
/* #define DATA_IN_ExtSRAM */
#endif /* STM32F40xxx || STM32F41xxx || STM32F42xxx || STM32F43xxx || STM32F469xx || STM32F479xx ||\
          STM32F412Zx || STM32F412Vx */
 
#if defined(STM32F427xx) || defined(STM32F437xx) || defined(STM32F429xx) || defined(STM32F439xx)\
 || defined(STM32F446xx) || defined(STM32F469xx) || defined(STM32F479xx)
// [解析] 同上，这是"外接 SDRAM"的开关。注意它只对带 FMC（支持 SDRAM）的型号有效：
// [解析] F427/F437/F429/F439/F446/F469/F479。本工程是 STM32F407，不在这个列表里，
// [解析] 连外面的 #if 都不成立，所以这一段对本工程完全不参与编译（写了也没用）。
/* #define DATA_IN_ExtSDRAM */
#endif /* STM32F427xx || STM32F437xx || STM32F429xx || STM32F439xx || STM32F446xx || STM32F469xx ||\
          STM32F479xx */

/* Note: Following vector table addresses must be defined in line with linker
         configuration. */
// [解析] ★ USER_VECT_TAB_ADDRESS：是否要把"中断向量表"搬到别的位置去。
// [解析] 向量表就是"中断编号 -> 处理函数入口地址"的那张表（每个表项 4 字节），
// [解析] 复位后默认放在 Flash 起始地址 0x08000000。CPU 靠 SCB->VTOR 寄存器知道它在哪。
// [解析] 什么情况下需要搬：写 Bootloader 的时候。Bootloader 占住 0x08000000，
// [解析] 用户程序被烧到后面某个地址，这时用户程序的中断向量表也整体后移了，
// [解析] 如果不改 VTOR，一旦发生中断 CPU 会跳回 Bootloader 的向量表去，程序立刻跑飞。
// [解析] 本工程没有 Bootloader，所以这个宏保持注释状态，SCB->VTOR 保持默认值不动。
/*!< Uncomment the following line if you need to relocate the vector table
     anywhere in Flash or Sram, else the vector table is kept at the automatic
     remap of boot address selected */
/* #define USER_VECT_TAB_ADDRESS */

// [解析] 以下三行是"搬到哪里"的具体参数，只有上面那个 USER_VECT_TAB_ADDRESS 打开时
// [解析] 才会被编译进去（本工程不编译，看个意思即可）：
// [解析]   VECT_TAB_SRAM          ：打开则把表放 SRAM，否则放 Flash。
// [解析]   VECT_TAB_BASE_ADDRESS  ：表的基地址。
// [解析]   VECT_TAB_OFFSET        ：相对基地址的偏移。
// [解析] 为什么反复强调"必须是 0x200 的倍数"：向量表的最小对齐要求（前 16 个异常入口
// [解析] 组成一组，硬件要求基址低 9 位为 0），写错对齐会导致中断跳转到错误地址。
#if defined(USER_VECT_TAB_ADDRESS)
/*!< Uncomment the following line if you need to relocate your vector Table
     in Sram else user remap will be done in Flash. */
/* #define VECT_TAB_SRAM */
#if defined(VECT_TAB_SRAM)
#define VECT_TAB_BASE_ADDRESS   SRAM_BASE       /*!< Vector Table base address field.
                                                     This value must be a multiple of 0x200. */
#else
#define VECT_TAB_BASE_ADDRESS   FLASH_BASE      /*!< Vector Table base address field.
                                                     This value must be a multiple of 0x200. */
#endif /* VECT_TAB_SRAM */
#if !defined(VECT_TAB_OFFSET)
#define VECT_TAB_OFFSET         0x00000000U     /*!< Vector Table offset field.
                                                     This value must be a multiple of 0x200. */
#endif /* VECT_TAB_OFFSET */
#endif /* USER_VECT_TAB_ADDRESS */
/******************************************************************************/

/**
  * @}
  */

/** @addtogroup STM32F4xx_System_Private_Macros
  * @{
  */

/**
  * @}
  */

/** @addtogroup STM32F4xx_System_Private_Variables
  * @{
  */
  /* This variable is updated in three ways:
      1) by calling CMSIS function SystemCoreClockUpdate()
      2) by calling HAL API function HAL_RCC_GetHCLKFreq()
      3) each time HAL_RCC_ClockConfig() is called to configure the system clock frequency 
         Note: If you use this function to configure the system clock; then there
               is no need to call the 2 first functions listed above, since SystemCoreClock
               variable is updated automatically.
  */
// [解析] ★ SystemCoreClock：全局的"CPU 内核频率（HCLK）"，单位 Hz。
// [解析] 初值 16000000 对应芯片刚复位、还没配置 PLL 的状态 —— 此时 SYSCLK 来自内部 HSI(16 MHz)。
// [解析] main() -> SystemClock_Config() 调用 HAL_RCC_ClockConfig() 配好 PLL 之后，
// [解析] HAL 库内部会自动把这里改成 168000000（本工程的主频），所以不要手动去改初值。
// [解析] 谁在用它：HAL_Delay()/HAL_GetTick() 的计时、串口波特率分频计算、
// [解析]           FreeRTOS 的 configCPU_CLOCK_HZ、DWT 微秒级延时等，几乎全靠它。
// [解析] 调试技巧：怀疑主频不对时，不用算 PLL，直接在调试器 Watch 窗口里看这个变量最快。
uint32_t SystemCoreClock = 16000000;
// [解析] ★ AHBPrescTable：AHB 分频器的"换算表"。别被名字骗了 —— 表里存的不是分频比，
// [解析] 而是【右移位数】：HCLK = SYSCLK >> AHBPrescTable[HPRE]。
// [解析] 下标 = RCC->CFGR 里 HPRE 字段的值(0~15)；前 8 项都是 0 表示不分频(÷1)，
// [解析] 后面依次对应 ÷2、÷4、÷8、÷16、÷64、÷128、÷256、÷512。
// [解析] 注意中间"跳"了一档：下标 11 是 ÷16，下标 12 直接跳到 ÷64，硬件里没有 ÷32 这一档。
// [解析] 本工程 AHB 不分频（AHBCLKDivider = RCC_SYSCLK_DIV1），取到的就是 0，等于不右移。
// [解析] APBPrescTable：APB1/APB2 的分频表，下标 = 相应 PRE 字段的值(0~7)，
// [解析] 前 4 项 0 表示不分频，之后依次是 ÷2、÷4、÷8、÷16。
// [解析] 本工程 APB1 = HCLK/4 = 42 MHz、APB2 = HCLK/2 = 84 MHz（见 docs/C板引脚配置说明.md）。
// [解析] 提醒：挂在 APB 上的定时器(TIM)在"分频比不为 1"时，其计数时钟会再额外 ×2，
// [解析] 所以算 PWM 频率时 TIM 的时钟源常常是 84 MHz 而不是 42 MHz，这点极易算错。
const uint8_t AHBPrescTable[16] = {0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 6, 7, 8, 9};
const uint8_t APBPrescTable[8]  = {0, 0, 0, 0, 1, 2, 3, 4};
// [解析] ↑ 单独说明这一行：表里只用到下标 0~7，其中 0~3 全是 0（不分频），
// [解析] 4→÷2、5→÷4、6→÷8、7→÷16。本工程 APB1 用 RCC_HCLK_DIV4（下标 5）、
// [解析] APB2 用 RCC_HCLK_DIV2（下标 4），一查表就能算出 42 MHz / 84 MHz。
/**
  * @}
  */

/** @addtogroup STM32F4xx_System_Private_FunctionPrototypes
  * @{
  */

// [解析] 函数原型声明：只有打开了外扩存储器开关，才会声明、也才会编译这个函数。
// [解析] static 表示它"只在本文件内可见"，外部文件用不到、也不该用（属于内部辅助函数）。
// [解析] 它的定义在本文件很靠后的位置，中间隔了几百行，所以这里必须先声明一下。
#if defined (DATA_IN_ExtSRAM) || defined (DATA_IN_ExtSDRAM)
  static void SystemInit_ExtMemCtl(void); 
#endif /* DATA_IN_ExtSRAM || DATA_IN_ExtSDRAM */

/**
  * @}
  */

/** @addtogroup STM32F4xx_System_Private_Functions
  * @{
  */

/**
  * @brief  Setup the microcontroller system
  *         Initialize the FPU setting, vector table location and External memory 
  *         configuration.
  * @param  None
  * @retval None
  */
// [解析] ★ SystemInit()：芯片复位后、进入 main() 之前，由启动文件
// [解析] （startup_stm32f407xx.s 里的 Reset_Handler）自动调用，我们不需要手动调它。
// [解析] 参数无、返回值无 —— 它只做三件事：开 FPU、配外部存储器（本工程没开）、
// [解析] 搬中断向量表（本工程没开）。所以在本工程里它实际只干了"开 FPU"这一件事。
// [解析] ⚠ 注意：它【不配置系统时钟】！168 MHz 是 main() -> SystemClock_Config() 里配的。
// [解析]    也就是说执行到这里时 CPU 还跑在 16 MHz 的内部 HSI 上，此时改串口波特率、
// [解析]    算定时器分频都会得到不一样的结果，别在这之前做时序相关的事。
void SystemInit(void)
{
  /* FPU settings ------------------------------------------------------------*/
  // [解析] ★ 本函数最核心的一步：打开 Cortex-M4 内核的硬件浮点单元（FPU）。
  // [解析]   SCB->CPACR = Coprocessor Access Control Register（协处理器访问控制寄存器）。
  // [解析]   CP10、CP11 这两个协处理器编号对应的就是单精度浮点单元。每个协处理器占 2 个
  // [解析]   比特位，写 0b11 表示"完全访问权限"，所以表达式是 (3 << 10*2) | (3 << 11*2)，
  // [解析]   即把第 20~23 位全部置 1（10*2=20、11*2=22）。|= 是"或等"，只置位不清除别的位。
  // [解析] 为什么必须开：本工程编译选项是 -mfpu=fpv4-sp-d16 -mfloat-abi=hard
  // [解析]   （见 cmake/stm32cubemx/CMakeLists.txt），浮点参数直接用 FPU 寄存器传递、
  // [解析]   浮点指令直接下发给 FPU。如果这里忘了开，一执行浮点指令 CPU 就会进 HardFault。
  // [解析] 外层 #if 是【编译期】判断（不是运行期）：芯片确实带 FPU(__FPU_PRESENT==1)
  // [解析]   且工程确实要用(__FPU_USED==1) 时才生成这条语句，否则这行代码根本不存在。
  #if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
    SCB->CPACR |= ((3UL << 10*2)|(3UL << 11*2));  /* set CP10 and CP11 Full Access */
  #endif

#if defined (DATA_IN_ExtSRAM) || defined (DATA_IN_ExtSDRAM)
  SystemInit_ExtMemCtl(); 
#endif /* DATA_IN_ExtSRAM || DATA_IN_ExtSDRAM */

  /* Configure the Vector Table location -------------------------------------*/
  // [解析] 把"中断向量表基地址"写进 SCB->VTOR 寄存器。只有定义了 USER_VECT_TAB_ADDRESS
  // [解析] 时这句才会被编译；本工程该宏是注释状态，所以这句不存在，向量表就老老实实留在
  // [解析] Flash 起始处 0x08000000，CPU 复位后自动使用它。
  // [解析] 以后若要加 Bootloader（引导程序），就是靠这一句把表搬到新位置去。
#if defined(USER_VECT_TAB_ADDRESS)
  SCB->VTOR = VECT_TAB_BASE_ADDRESS | VECT_TAB_OFFSET; /* Vector Table Relocation in Internal SRAM */
#endif /* USER_VECT_TAB_ADDRESS */
}

/**
   * @brief  Update SystemCoreClock variable according to Clock Register Values.
  *         The SystemCoreClock variable contains the core clock (HCLK), it can
  *         be used by the user application to setup the SysTick timer or configure
  *         other parameters.
  *           
  * @note   Each time the core clock (HCLK) changes, this function must be called
  *         to update SystemCoreClock variable value. Otherwise, any configuration
  *         based on this variable will be incorrect.         
  *     
  * @note   - The system frequency computed by this function is not the real 
  *           frequency in the chip. It is calculated based on the predefined 
  *           constant and the selected clock source:
  *             
  *           - If SYSCLK source is HSI, SystemCoreClock will contain the HSI_VALUE(*)
  *                                              
  *           - If SYSCLK source is HSE, SystemCoreClock will contain the HSE_VALUE(**)
  *                          
  *           - If SYSCLK source is PLL, SystemCoreClock will contain the HSE_VALUE(**) 
  *             or HSI_VALUE(*) multiplied/divided by the PLL factors.
  *         
  *         (*) HSI_VALUE is a constant defined in stm32f4xx_hal_conf.h file (default value
  *             16 MHz) but the real value may vary depending on the variations
  *             in voltage and temperature.   
  *    
  *         (**) HSE_VALUE is a constant defined in stm32f4xx_hal_conf.h file (its value
  *              depends on the application requirements), user has to ensure that HSE_VALUE
  *              is same as the real frequency of the crystal used. Otherwise, this function
  *              may have wrong result.
  *                
  *         - The result of this function could be not correct when using fractional
  *           value for HSE crystal.
  *     
  * @param  None
  * @retval None
  */
// [解析] ★ SystemCoreClockUpdate()：不去读时钟树配置，而是【反过来】读 RCC 里的寄存器，
// [解析] 推算出现在的 SYSCLK 是多少、再折算成 HCLK，最后更新全局变量 SystemCoreClock。
// [解析] 什么时候用：别人（例如 Bootloader）已经把时钟配好了，你不想/不能重新配置一遍，
// [解析] 只想让系统知道"现在跑多快" —— 就调它。本工程里没有调用它，因为
// [解析] SystemClock_Config() 走的是 HAL_RCC_ClockConfig()，HAL 内部已经同步过了
// [解析] （这一点在本文件上方那段英文注释里也写明了）。
// [解析] 局限性（原注释同样提到了）：
// [解析]   1) 算出来的是"依据宏定义的标称频率"，不是实测频率；
// [解析]   2) HSE_VALUE 必须与真实晶振一致，否则结果整体按比例偏掉。
// [解析] 另外，本工程 HSE 是整数 12 MHz、PLLM=6 整除，所以不存在原注释说的"小数晶振"误差。
void SystemCoreClockUpdate(void)
{
  uint32_t tmp, pllvco, pllp, pllsource, pllm;
  
  /* Get SYSCLK source -------------------------------------------------------*/
  // [解析] RCC->CFGR 的 SWS 字段（bit3:2）= System clock switch status，是硬件告诉我们
  // [解析] "现在 SYSCLK 究竟由谁提供"的【只读状态位】。掩码取出来只会是三个值之一：
  // [解析]   0x00 = HSI（内部 16 MHz RC 振荡器）
  // [解析]   0x04 = HSE（外部晶振）
  // [解析]   0x08 = PLL（本工程的实际选择）
  // [解析] 注意它读的是 SWS（状态）而不是 SW（我们写进去的请求），也就是说"以硬件实际
  // [解析] 切换结果为准"—— 万一晶振没起振导致切换失败，这里能反映出真实情况。
  tmp = RCC->CFGR & RCC_CFGR_SWS;

  switch (tmp)
  {
    case 0x00:  /* HSI used as system clock source */
      // [解析] 复位后的默认状态，也是最"保险"的状态：主频就是 16 MHz。
      SystemCoreClock = HSI_VALUE;
      break;
    case 0x04:  /* HSE used as system clock source */
      // [解析] 直接把外部晶振当 SYSCLK，不经过 PLL（此时 SYSCLK 只能是 4~26 MHz 这一档）。
      // [解析] 本工程没有采用这种方式；若走了这支，SystemCoreClock 就是 HSE_VALUE。
      SystemCoreClock = HSE_VALUE;
      break;
    case 0x08:  /* PLL used as system clock source */

      // [解析] ★ 本工程走的就是这个分支。三个核心公式（原注释里也写了）：
      // [解析]   VCO 输入 = 时钟源频率 / PLLM      （F4 要求 VCO 输入 1~2 MHz，最好 2 MHz）
      // [解析]   VCO 输出 = VCO 输入 × PLLN        （F4 要求 VCO 输出 100~432 MHz）
      // [解析]   SYSCLK   = VCO 输出 / PLLP        （PLLP 只能是 2 / 4 / 6 / 8）
      // [解析] 代入本工程参数（HSE 12 MHz, PLLM=6, PLLN=168, PLLP=2）：
      // [解析]   12/6 = 2 MHz  ->  2 × 168 = 336 MHz  ->  336/2 = 168 MHz
      // [解析] 另外还有一个 PLLQ 分频器（本工程 = 4），它只影响 USB/SDIO 等外设的 48 MHz
      // [解析] 时钟，不参与 SYSCLK 的计算，所以下面这段代码里没出现它。
      /* PLL_VCO = (HSE_VALUE or HSI_VALUE / PLL_M) * PLL_N
         SYSCLK = PLL_VCO / PLL_P
         */    
      pllsource = (RCC->PLLCFGR & RCC_PLLCFGR_PLLSRC) >> 22;
      // [解析] ↑ PLLSRC 是 RCC->PLLCFGR 的 bit22，用来选 PLL 的输入源：0 = HSI，1 = HSE。
      // [解析]   右移 22 位就是把它挪到最低位，方便后面直接判断 !=0。
      // [解析] ↑ 注意 pllsource 是"位域掩码与运算"的结果，不是直接等于 1，所以只能判断
      // [解析]   "等不等于 0"，不能写成 == 1。
      pllm = RCC->PLLCFGR & RCC_PLLCFGR_PLLM;
      // [解析] ↑ PLLM 位于 PLLCFGR 的 bit5:0，本身就是最低位，所以不用移位，直接与掩码相与。
      // [解析]   本工程 PLLM = 6；12 MHz / 6 = 2 MHz，正好是手册要求的最佳 VCO 输入频率。
      
      if (pllsource != 0)
      {
        /* HSE used as PLL clock source */
        // [解析] 本工程 PLLSource = RCC_PLLSOURCE_HSE，走的就是这一支。
        // [解析] 计算过程：(12000000 / 6) × (PLLN=168) = 336000000，即 VCO 输出 336 MHz。
        // [解析] 先除后乘是有意为之：C 语言的整数除法会截断小数，若先乘 12e6×168 虽不溢出，
        // [解析]   但换成高频晶振就可能溢出 32 位；先除后乘更安全。
        // [解析] 代价是要求"HSE 能被 PLLM 整除"，否则结果会有零点几 MHz 的误差
        // [解析]   （原注释说的"小数晶振不准确"就是指这个）。本工程 12/6=2 恰好整除，无误差。
        // [解析] (RCC->PLLCFGR & RCC_PLLCFGR_PLLN) >> 6：PLLN 在 bit14:6，所以要右移 6 位。
        pllvco = (HSE_VALUE / pllm) * ((RCC->PLLCFGR & RCC_PLLCFGR_PLLN) >> 6);
      }
      else
      {
        /* HSI used as PLL clock source */
        // [解析] 用内部 16 MHz HSI 倍频的情况，本工程未使用（HSI 精度差，倍频到 168 MHz
        // [解析] 后误差会被放大 168 倍，串口/CAN 基本没法可靠通信）。
        pllvco = (HSI_VALUE / pllm) * ((RCC->PLLCFGR & RCC_PLLCFGR_PLLN) >> 6);
      }

      pllp = (((RCC->PLLCFGR & RCC_PLLCFGR_PLLP) >>16) + 1 ) *2;
      // [解析] ↑ PLLP 字段在 bit17:16，所以右移 16 位取出来。硬件里这个字段是"编码"而不是
      // [解析]   直接的除数，编码规则为：00->÷2、01->÷4、10->÷6、11->÷8。
      // [解析]   写成 (字段值 + 1) * 2，正好把 0/1/2/3 映射成 2/4/6/8，省掉一张表。
      // [解析]   本工程 PLLP = RCC_PLLP_DIV2，字段值 0 → 除数 2：336 MHz / 2 = 168 MHz。
      SystemCoreClock = pllvco/pllp;
      // [解析] 整数除法：336000000 / 2 = 168000000，SYSCLK 就确定了。
      break;
    default:
      // [解析] 理论上不会走到这里（SWS 只有上面三种取值）。写 default 属于防御性编程：
      // [解析] 万一寄存器读数异常，也给 SystemCoreClock 一个安全的缺省值，而不是随机数。
      SystemCoreClock = HSI_VALUE;
      break;
  }
  /* Compute HCLK frequency --------------------------------------------------*/
  /* Get HCLK prescaler */
  // [解析] ↓ 到这里 SystemCoreClock 里装的是 SYSCLK，但变量名的含义是 HCLK，所以还得再过
  // [解析]   一遍 AHB 分频器：HCLK = SYSCLK >> AHBPrescTable[HPRE]，表里存的是右移位数。
  // [解析]   HPRE 字段在 CFGR 的 bit7:4，右移 4 位取出 0~15 的下标。
  // [解析]   本工程 AHB 不分频，取到 0，右移 0 位，因此 HCLK = SYSCLK = 168 MHz。
  // [解析]   注意 FreeRTOS 的 configCPU_CLOCK_HZ 用的也是 HCLK，所以这里算错会直接影响
  // [解析]   任务调度的时间片长度。
  tmp = AHBPrescTable[((RCC->CFGR & RCC_CFGR_HPRE) >> 4)];
  /* HCLK frequency */
  // [解析] >>= 是"右移赋值"，右移 n 位等价于除以 2 的 n 次方，比除法指令快。
  SystemCoreClock >>= tmp;
}

#if defined (DATA_IN_ExtSRAM) && defined (DATA_IN_ExtSDRAM)
// [解析] ★ 从这里到文件末尾的几百行，是"外接 SDRAM / SRAM 初始化"代码，作者没有改动过。
// [解析] 在本工程里它的命运是：DATA_IN_ExtSRAM 和 DATA_IN_ExtSDRAM 两个宏都被注释掉了
// [解析] （见文件上半部分的"杂项配置"），所以下面所有 #if 判断全为假，
// [解析] SystemInit_ExtMemCtl() 这个函数【根本不会被编译】，属于死代码。
// [解析] 那为什么还留着？因为 CubeMX 生成代码时是照抄 ST 官方模板，删了下次生成又会回来。
// [解析] 阅读建议：新手可以整段跳过。只要知道"它是给板子上那颗外扩存储芯片做上电时序
// [解析] 初始化的，本工程用不到"就够了。下面是它涉及的概念：
// [解析]   FMC = Flexible Memory Controller（灵活存储控制器），F429 等型号用它驱动
// [解析]         SDRAM/NOR/SRAM，自动产生地址、数据、片选、行列选通等时序信号，
// [解析]         CPU 像访问普通内存一样访问外扩芯片。F407 只有不支持 SDRAM 的 FSMC。
// [解析]   下面这个 #if / #elif / #endif 三层结构的用意：
// [解析]     两个宏都定义   -> 用"SRAM 和 SDRAM 都要初始化"的完整版本（即下面这一段）；
// [解析]     只定义了其中一个 -> 用后面 #elif 分支里的精简版本；
// [解析]     都没定义        -> 两个版本都不编译（本工程就是这种情况）。
// [解析] 两个版本里的函数同名，但同一时刻只会编译其中一个，所以不会重定义冲突。
#if defined(STM32F427xx) || defined(STM32F437xx) || defined(STM32F429xx) || defined(STM32F439xx)\
 || defined(STM32F469xx) || defined(STM32F479xx)
/**
  * @brief  Setup the external memory controller.
  *         Called in startup_stm32f4xx.s before jump to main.
  *         This function configures the external memories (SRAM/SDRAM)
  *         This SRAM/SDRAM will be used as program data memory (including heap and stack).
  * @param  None
  * @retval None
  */
void SystemInit_ExtMemCtl(void)
{
  // [解析] __IO 就是 volatile 的宏（CMSIS 里 #define __IO volatile），意思是"这个变量
  // [解析] 可能被硬件/中断改掉，编译器不要自作聪明优化掉对它的读写"。
  // [解析] 在这段代码里，tmp 的真正用途是"读一下刚使能的那个寄存器"，借此产生 STM32
  // [解析] 要求的"使能外设时钟后至少等一小段时间"的延时效果（读操作会占用总线周期）。
  // [解析] 也正因为它的值根本没被使用，函数末尾才有一段 (void)(tmp); 来压掉编译警告。
  __IO uint32_t tmp = 0x00;

  // [解析] register 是"建议编译器把这个变量放在 CPU 寄存器里"（现代编译器基本会忽略它，
  // [解析] 写不写效果一样，是老代码的遗留风格）。
  // [解析] timeout 初值 0xFFFF 是后面 while 循环里的"防死等"计数：硬件万一没响应，
  // [解析] 循环也会自己退出，不会把整个程序卡死在上电阶段。
  register uint32_t tmpreg = 0, timeout = 0xFFFF;
  register __IO uint32_t index;

  /* Enable GPIOC, GPIOD, GPIOE, GPIOF, GPIOG, GPIOH and GPIOI interface clock */
  // [解析] 0x1F8 = 二进制 1_1111_1000，对应 AHB1 外设时钟使能寄存器 RCC->AHB1ENR 里的
  // [解析] bit3~bit8，也就是 GPIOC/GPIOD/GPIOE/GPIOF/GPIOG/GPIOH/GPIOI 这 7 个端口的时钟。
  // [解析] 外扩存储器的数据/地址线分布在这些端口上，不先给端口开时钟，
  // [解析] 后面写它们的 AFR/MODER 等寄存器不会生效（这是新手最常犯的错误之一）。
  RCC->AHB1ENR |= 0x000001F8;

  /* Delay after an RCC peripheral clock enabling */
  // [解析] 见上面 tmp 的说明：这一句的唯一目的就是"读一下刚写的寄存器"来消耗一点时间。
  tmp = READ_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOCEN);
  
  // [解析] 下面这一大片（一直到 PIx 结束）是把 GPIO 引脚改造成"复用功能模式"，让它们交给
  // [解析] FMC 外设去驱动，而不是当普通 IO 用。每个端口都是同一套五连招，理解一组即可：
  // [解析]   AFR[0]/AFR[1] = Alternate Function Register，每个引脚占 4 位，用来选择复用编号
  // [解析]                     （FMC 的复用编号是 12 = 0xC，所以满屏都是 C）；
  // [解析]   MODER       = 模式寄存器，每个引脚 2 位，0b10 表示"复用功能"（0b01 才是普通输出）；
  // [解析]   OSPEEDR     = 输出速度，外扩总线跑得快，所以要设成 50 MHz 或 100 MHz；
  // [解析]   OTYPER      = 输出类型，0 = 推挽输出；
  // [解析]   PUPDR       = 上下拉，0 = 既不上拉也不下拉。
  // [解析] 每个值都是"一整排引脚的位图拼装"，盯着十六进制数看不出所以然 —— 这类寄存器配置
  // [解析] 在 CubeMX 里是图形化勾选的，实际开发不要手算，让工具生成就好。
  /* Connect PDx pins to FMC Alternate function */
  GPIOD->AFR[0]  = 0x00CCC0CC;
  GPIOD->AFR[1]  = 0xCCCCCCCC;
  /* Configure PDx pins in Alternate function mode */  
  GPIOD->MODER   = 0xAAAA0A8A;
  /* Configure PDx pins speed to 100 MHz */  
  GPIOD->OSPEEDR = 0xFFFF0FCF;
  /* Configure PDx pins Output type to push-pull */  
  GPIOD->OTYPER  = 0x00000000;
  /* No pull-up, pull-down for PDx pins */ 
  GPIOD->PUPDR   = 0x00000000;

  /* Connect PEx pins to FMC Alternate function */
  GPIOE->AFR[0]  = 0xC00CC0CC;
  GPIOE->AFR[1]  = 0xCCCCCCCC;
  /* Configure PEx pins in Alternate function mode */ 
  GPIOE->MODER   = 0xAAAA828A;
  /* Configure PEx pins speed to 100 MHz */ 
  GPIOE->OSPEEDR = 0xFFFFC3CF;
  /* Configure PEx pins Output type to push-pull */  
  GPIOE->OTYPER  = 0x00000000;
  /* No pull-up, pull-down for PEx pins */ 
  GPIOE->PUPDR   = 0x00000000;
  
  /* Connect PFx pins to FMC Alternate function */
  GPIOF->AFR[0]  = 0xCCCCCCCC;
  GPIOF->AFR[1]  = 0xCCCCCCCC;
  /* Configure PFx pins in Alternate function mode */   
  GPIOF->MODER   = 0xAA800AAA;
  /* Configure PFx pins speed to 50 MHz */ 
  GPIOF->OSPEEDR = 0xAA800AAA;
  /* Configure PFx pins Output type to push-pull */  
  GPIOF->OTYPER  = 0x00000000;
  /* No pull-up, pull-down for PFx pins */ 
  GPIOF->PUPDR   = 0x00000000;

  /* Connect PGx pins to FMC Alternate function */
  GPIOG->AFR[0]  = 0xCCCCCCCC;
  GPIOG->AFR[1]  = 0xCCCCCCCC;
  /* Configure PGx pins in Alternate function mode */ 
  GPIOG->MODER   = 0xAAAAAAAA;
  /* Configure PGx pins speed to 50 MHz */ 
  GPIOG->OSPEEDR = 0xAAAAAAAA;
  /* Configure PGx pins Output type to push-pull */  
  GPIOG->OTYPER  = 0x00000000;
  /* No pull-up, pull-down for PGx pins */ 
  GPIOG->PUPDR   = 0x00000000;
  
  /* Connect PHx pins to FMC Alternate function */
  GPIOH->AFR[0]  = 0x00C0CC00;
  GPIOH->AFR[1]  = 0xCCCCCCCC;
  /* Configure PHx pins in Alternate function mode */ 
  GPIOH->MODER   = 0xAAAA08A0;
  /* Configure PHx pins speed to 50 MHz */ 
  GPIOH->OSPEEDR = 0xAAAA08A0;
  /* Configure PHx pins Output type to push-pull */  
  GPIOH->OTYPER  = 0x00000000;
  /* No pull-up, pull-down for PHx pins */ 
  GPIOH->PUPDR   = 0x00000000;
  
  /* Connect PIx pins to FMC Alternate function */
  GPIOI->AFR[0]  = 0xCCCCCCCC;
  GPIOI->AFR[1]  = 0x00000CC0;
  /* Configure PIx pins in Alternate function mode */ 
  GPIOI->MODER   = 0x0028AAAA;
  /* Configure PIx pins speed to 50 MHz */ 
  GPIOI->OSPEEDR = 0x0028AAAA;
  /* Configure PIx pins Output type to push-pull */  
  GPIOI->OTYPER  = 0x00000000;
  /* No pull-up, pull-down for PIx pins */ 
  GPIOI->PUPDR   = 0x00000000;
  
/*-- FMC Configuration -------------------------------------------------------*/
  /* Enable the FMC interface clock */
  // [解析] FMC 挂在 AHB3 总线上，bit0 就是它的时钟开关。同样要在配置之前先开时钟。
  RCC->AHB3ENR |= 0x00000001;
  /* Delay after an RCC peripheral clock enabling */
  tmp = READ_BIT(RCC->AHB3ENR, RCC_AHB3ENR_FMCEN);

  // [解析] SDCR[0] = SDRAM Control Register（SDRAM 控制寄存器），决定这颗 SDRAM 的
  // [解析] 行列地址位数、数据总线宽度、内部 Bank 数量、CAS 延迟、时钟分频等基本参数；
  // [解析] SDTR[0] = SDRAM Timing Register（时序寄存器），填 tRCD/tRP/tWR 等时间参数。
  // [解析] 这两个寄存器里全是"裸数值"，是照板上那颗 SDRAM 芯片的数据手册和 ST 官方例程
  // [解析] 配出来的：换一颗芯片就必须重算，所以实际项目里都交给 CubeMX 的 FMC 界面生成，
  // [解析] 千万别凭感觉改这里的十六进制数，写错的表现是"内存读写随机出错"。
  FMC_Bank5_6->SDCR[0] = 0x000019E4;
  FMC_Bank5_6->SDTR[0] = 0x01115351;      
  
  /* SDRAM initialization sequence */
  // [解析] ★ 下面按 JEDEC 标准规定的顺序连发 4 条命令，SDRAM 才能正常工作，顺序不能乱：
  // [解析]   时钟使能 -> 预充电所有 Bank(PALL) -> 自动刷新若干次 -> 加载模式寄存器(MRD)。
  // [解析] 每条命令都是往 SDCMR 寄存器写一个值，低 3 位 MODE 决定这是哪条命令(001/010/011/100)。
  // [解析] 发完一条就轮询 SDSR 的 BUSY 位（掩码 0x20 = bit5）等硬件执行完；while 里的
  // [解析] timeout-- 是"防死等"保护，硬件万一没响应循环也会退出，不会永久卡在上电阶段。
  /* Clock enable command */
  FMC_Bank5_6->SDCMR = 0x00000011; 
  tmpreg = FMC_Bank5_6->SDSR & 0x00000020; 
  while((tmpreg != 0) && (timeout-- > 0))
  {
    tmpreg = FMC_Bank5_6->SDSR & 0x00000020; 
  }

  /* Delay */
  // [解析] 空循环延时：此刻 HAL 的 1 ms 时基可能还没建立，不能用 HAL_Delay()，
  // [解析] 只能靠 CPU 空转消耗时间。循环体是空的，末尾那个分号千万别删。
  for (index = 0; index<1000; index++);
  
  /* PALL command */
  FMC_Bank5_6->SDCMR = 0x00000012;           
  tmpreg = FMC_Bank5_6->SDSR & 0x00000020;
  timeout = 0xFFFF;
  while((tmpreg != 0) && (timeout-- > 0))
  {
    tmpreg = FMC_Bank5_6->SDSR & 0x00000020; 
  }
  
  /* Auto refresh command */
  FMC_Bank5_6->SDCMR = 0x00000073;
  tmpreg = FMC_Bank5_6->SDSR & 0x00000020;
  timeout = 0xFFFF;
  while((tmpreg != 0) && (timeout-- > 0))
  {
    tmpreg = FMC_Bank5_6->SDSR & 0x00000020; 
  }
 
  /* MRD register program */
  FMC_Bank5_6->SDCMR = 0x00046014;
  tmpreg = FMC_Bank5_6->SDSR & 0x00000020;
  timeout = 0xFFFF;
  while((tmpreg != 0) && (timeout-- > 0))
  {
    tmpreg = FMC_Bank5_6->SDSR & 0x00000020; 
  } 
  
  /* Set refresh count */
  // [解析] SDRTR = SDRAM Refresh Timer Register。SDRAM 靠电容存数据，必须周期性刷新
  // [解析] （典型要求 64 ms 内把所有行刷一遍），否则数据会慢慢丢失。
  // [解析] 计数值 ≈ 刷新周期 ÷ 行数 × SDRAM 时钟频率，0x27C = 636 是为当前这颗芯片和
  // [解析] 当前时钟算出来的固定值；<< 1 是因为该字段从 bit1 开始、bit0 是保留位。
  // [解析] 换 SDRAM 型号或改了时钟分频就必须重算，否则表现为"内存用一会儿就开始出错"。
  tmpreg = FMC_Bank5_6->SDRTR;
  FMC_Bank5_6->SDRTR = (tmpreg | (0x0000027C<<1));
  
  /* Disable write protection */
  // [解析] tmpreg 在这里又被借来暂存寄存器旧值（它就是个通用临时变量）。
  // [解析] 0xFFFFFDFF 的作用是把 bit9 清零（= 与上"0x200 取反"的结果），按上面的原注释，
  // [解析] 这一步是解除 SDRAM 的写保护，让这块外扩内存可以被正常读写。
  tmpreg = FMC_Bank5_6->SDCR[0]; 
  FMC_Bank5_6->SDCR[0] = (tmpreg & 0xFFFFFDFF);

#if defined(STM32F427xx) || defined(STM32F437xx) || defined(STM32F429xx) || defined(STM32F439xx)
  /* Configure and enable Bank1_SRAM2 */
  // [解析] 这一段（以及下面 F469/F479 的版本）是给"外扩的 NOR/SRAM"用的：
  // [解析] BTCR[2]/BTCR[3] 配置 Bank1 第 2 个片选的读/写时序，BWTR[2] 是写时序寄存器。
  // [解析] 0x0fffffff 表示把各种等待时间全部拉到最长 —— 最保守、最慢，但最不容易出错。
  // [解析] 本工程没有外扩 SRAM，所以这段不会参与编译。
  FMC_Bank1->BTCR[2]  = 0x00001011;
  FMC_Bank1->BTCR[3]  = 0x00000201;
  FMC_Bank1E->BWTR[2] = 0x0fffffff;
#endif /* STM32F427xx || STM32F437xx || STM32F429xx || STM32F439xx */ 
#if defined(STM32F469xx) || defined(STM32F479xx)
  /* Configure and enable Bank1_SRAM2 */
  FMC_Bank1->BTCR[2]  = 0x00001091;
  FMC_Bank1->BTCR[3]  = 0x00110212;
  FMC_Bank1E->BWTR[2] = 0x0fffffff;
#endif /* STM32F469xx || STM32F479xx */

  (void)(tmp); 
}
#endif /* STM32F427xx || STM32F437xx || STM32F429xx || STM32F439xx || STM32F469xx || STM32F479xx */
// [解析] 上面就是"两个宏都定义"时的完整版本。而本工程两个宏都没定义，所以整个函数
// [解析] 连同函数体都不会出现在编译产物里 —— 在反汇编/Map 文件里找不到 SystemInit_ExtMemCtl。
// [解析] (void)(tmp); 这一句的作用也很实用：tmp 前面只被赋值、从未被真正使用，
// [解析] 加一句"把它转成 void 丢掉"就能压掉编译器的 -Wunused-but-set-variable 警告。
// [解析] 下面这个 #elif 分支处理"只定义了其中一个宏"的情况：里面的 SystemInit_ExtMemCtl()
// [解析] 与上面同名，但同一时刻只会编译其中一个，所以不会产生重定义冲突。
#elif defined (DATA_IN_ExtSRAM) || defined (DATA_IN_ExtSDRAM)
/**
  * @brief  Setup the external memory controller.
  *         Called in startup_stm32f4xx.s before jump to main.
  *         This function configures the external memories (SRAM/SDRAM)
  *         This SRAM/SDRAM will be used as program data memory (including heap and stack).
  * @param  None
  * @retval None
  */
void SystemInit_ExtMemCtl(void)
{
  __IO uint32_t tmp = 0x00;
  // [解析] 下面是"只定义了 DATA_IN_ExtSRAM 或 DATA_IN_ExtSDRAM 其中之一"时用的精简版本。
  // [解析] 先按型号分档：只有带 FMC 的型号（F427/F437/F429/F439/F446/F469/F479）才支持
  // [解析] SDRAM，所以先套一层型号判断；F407 不满足，这里的内容对本工程同样不参与编译。
#if defined(STM32F427xx) || defined(STM32F437xx) || defined(STM32F429xx) || defined(STM32F439xx)\
 || defined(STM32F446xx) || defined(STM32F469xx) || defined(STM32F479xx)
#if defined (DATA_IN_ExtSDRAM)
  // [解析] ↓ 这两个局部变量只有 SDRAM 分支才用得上，所以声明也放在这个 #if 里面，
  // [解析]   免得在"只用 SRAM"时产生"变量定义未使用"的编译警告。
  register uint32_t tmpreg = 0, timeout = 0xFFFF;
  register __IO uint32_t index;

#if defined(STM32F446xx)
  /* Enable GPIOA, GPIOC, GPIOD, GPIOE, GPIOF, GPIOG interface
      clock */
  RCC->AHB1ENR |= 0x0000007D;
#else
  /* Enable GPIOC, GPIOD, GPIOE, GPIOF, GPIOG, GPIOH and GPIOI interface 
      clock */
  RCC->AHB1ENR |= 0x000001F8;
#endif /* STM32F446xx */  
  /* Delay after an RCC peripheral clock enabling */
  tmp = READ_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOCEN);
  
#if defined(STM32F446xx)
  /* Connect PAx pins to FMC Alternate function */
  GPIOA->AFR[0]  |= 0xC0000000;
  GPIOA->AFR[1]  |= 0x00000000;
  /* Configure PDx pins in Alternate function mode */
  GPIOA->MODER   |= 0x00008000;
  /* Configure PDx pins speed to 50 MHz */
  GPIOA->OSPEEDR |= 0x00008000;
  /* Configure PDx pins Output type to push-pull */
  GPIOA->OTYPER  |= 0x00000000;
  /* No pull-up, pull-down for PDx pins */
  GPIOA->PUPDR   |= 0x00000000;

  /* Connect PCx pins to FMC Alternate function */
  GPIOC->AFR[0]  |= 0x00CC0000;
  GPIOC->AFR[1]  |= 0x00000000;
  /* Configure PDx pins in Alternate function mode */
  GPIOC->MODER   |= 0x00000A00;
  /* Configure PDx pins speed to 50 MHz */
  GPIOC->OSPEEDR |= 0x00000A00;
  /* Configure PDx pins Output type to push-pull */
  GPIOC->OTYPER  |= 0x00000000;
  /* No pull-up, pull-down for PDx pins */
  GPIOC->PUPDR   |= 0x00000000;
#endif /* STM32F446xx */

  /* Connect PDx pins to FMC Alternate function */
  GPIOD->AFR[0]  = 0x000000CC;
  GPIOD->AFR[1]  = 0xCC000CCC;
  /* Configure PDx pins in Alternate function mode */  
  GPIOD->MODER   = 0xA02A000A;
  /* Configure PDx pins speed to 50 MHz */  
  GPIOD->OSPEEDR = 0xA02A000A;
  /* Configure PDx pins Output type to push-pull */  
  GPIOD->OTYPER  = 0x00000000;
  /* No pull-up, pull-down for PDx pins */ 
  GPIOD->PUPDR   = 0x00000000;

  /* Connect PEx pins to FMC Alternate function */
  GPIOE->AFR[0]  = 0xC00000CC;
  GPIOE->AFR[1]  = 0xCCCCCCCC;
  /* Configure PEx pins in Alternate function mode */ 
  GPIOE->MODER   = 0xAAAA800A;
  /* Configure PEx pins speed to 50 MHz */ 
  GPIOE->OSPEEDR = 0xAAAA800A;
  /* Configure PEx pins Output type to push-pull */  
  GPIOE->OTYPER  = 0x00000000;
  /* No pull-up, pull-down for PEx pins */ 
  GPIOE->PUPDR   = 0x00000000;

  /* Connect PFx pins to FMC Alternate function */
  GPIOF->AFR[0]  = 0xCCCCCCCC;
  GPIOF->AFR[1]  = 0xCCCCCCCC;
  /* Configure PFx pins in Alternate function mode */   
  GPIOF->MODER   = 0xAA800AAA;
  /* Configure PFx pins speed to 50 MHz */ 
  GPIOF->OSPEEDR = 0xAA800AAA;
  /* Configure PFx pins Output type to push-pull */  
  GPIOF->OTYPER  = 0x00000000;
  /* No pull-up, pull-down for PFx pins */ 
  GPIOF->PUPDR   = 0x00000000;

  /* Connect PGx pins to FMC Alternate function */
  GPIOG->AFR[0]  = 0xCCCCCCCC;
  GPIOG->AFR[1]  = 0xCCCCCCCC;
  /* Configure PGx pins in Alternate function mode */ 
  GPIOG->MODER   = 0xAAAAAAAA;
  /* Configure PGx pins speed to 50 MHz */ 
  GPIOG->OSPEEDR = 0xAAAAAAAA;
  /* Configure PGx pins Output type to push-pull */  
  GPIOG->OTYPER  = 0x00000000;
  /* No pull-up, pull-down for PGx pins */ 
  GPIOG->PUPDR   = 0x00000000;

#if defined(STM32F427xx) || defined(STM32F437xx) || defined(STM32F429xx) || defined(STM32F439xx)\
 || defined(STM32F469xx) || defined(STM32F479xx)  
  /* Connect PHx pins to FMC Alternate function */
  GPIOH->AFR[0]  = 0x00C0CC00;
  GPIOH->AFR[1]  = 0xCCCCCCCC;
  /* Configure PHx pins in Alternate function mode */ 
  GPIOH->MODER   = 0xAAAA08A0;
  /* Configure PHx pins speed to 50 MHz */ 
  GPIOH->OSPEEDR = 0xAAAA08A0;
  /* Configure PHx pins Output type to push-pull */  
  GPIOH->OTYPER  = 0x00000000;
  /* No pull-up, pull-down for PHx pins */ 
  GPIOH->PUPDR   = 0x00000000;
  
  /* Connect PIx pins to FMC Alternate function */
  GPIOI->AFR[0]  = 0xCCCCCCCC;
  GPIOI->AFR[1]  = 0x00000CC0;
  /* Configure PIx pins in Alternate function mode */ 
  GPIOI->MODER   = 0x0028AAAA;
  /* Configure PIx pins speed to 50 MHz */ 
  GPIOI->OSPEEDR = 0x0028AAAA;
  /* Configure PIx pins Output type to push-pull */  
  GPIOI->OTYPER  = 0x00000000;
  /* No pull-up, pull-down for PIx pins */ 
  GPIOI->PUPDR   = 0x00000000;
#endif /* STM32F427xx || STM32F437xx || STM32F429xx || STM32F439xx || STM32F469xx || STM32F479xx */
  
/*-- FMC Configuration -------------------------------------------------------*/
  /* Enable the FMC interface clock */
  RCC->AHB3ENR |= 0x00000001;
  /* Delay after an RCC peripheral clock enabling */
  tmp = READ_BIT(RCC->AHB3ENR, RCC_AHB3ENR_FMCEN);

  /* Configure and enable SDRAM bank1 */
  // [解析] 与第一版一样，SDCR[0]/SDTR[0] 里的位段完全由 SDRAM 芯片型号决定；
  // [解析] 这里只是 F446 芯片的引脚/时序略有不同，所以用 #if 单独给了一组值。
  // [解析] 下面那句 SDTR[0] = 0x01115351 对两种型号是共用的，所以放在 #endif 外面。
#if defined(STM32F446xx)
  FMC_Bank5_6->SDCR[0] = 0x00001954;
#else  
  FMC_Bank5_6->SDCR[0] = 0x000019E4;
#endif /* STM32F446xx */
  FMC_Bank5_6->SDTR[0] = 0x01115351;      
  
  /* SDRAM initialization sequence */
  // [解析] ★ 下面按 JEDEC 标准规定的顺序连发 4 条命令，SDRAM 才能正常工作，顺序不能乱：
  // [解析]   时钟使能 -> 预充电所有 Bank(PALL) -> 自动刷新若干次 -> 加载模式寄存器(MRD)。
  // [解析] 每条命令都是往 SDCMR 寄存器写一个值，低 3 位 MODE 决定这是哪条命令(001/010/011/100)。
  // [解析] 发完一条就轮询 SDSR 的 BUSY 位（掩码 0x20 = bit5）等硬件执行完；while 里的
  // [解析] timeout-- 是"防死等"保护，硬件万一没响应循环也会退出，不会永久卡在上电阶段。
  /* Clock enable command */
  FMC_Bank5_6->SDCMR = 0x00000011; 
  tmpreg = FMC_Bank5_6->SDSR & 0x00000020; 
  while((tmpreg != 0) && (timeout-- > 0))
  {
    tmpreg = FMC_Bank5_6->SDSR & 0x00000020; 
  }

  /* Delay */
  // [解析] 空循环延时：此刻 HAL 的 1 ms 时基可能还没建立，不能用 HAL_Delay()，
  // [解析] 只能靠 CPU 空转消耗时间。循环体是空的，末尾那个分号千万别删。
  for (index = 0; index<1000; index++);
  
  /* PALL command */
  FMC_Bank5_6->SDCMR = 0x00000012;           
  tmpreg = FMC_Bank5_6->SDSR & 0x00000020;
  timeout = 0xFFFF;
  while((tmpreg != 0) && (timeout-- > 0))
  {
    tmpreg = FMC_Bank5_6->SDSR & 0x00000020; 
  }
  
  /* Auto refresh command */
#if defined(STM32F446xx)
  FMC_Bank5_6->SDCMR = 0x000000F3;
#else  
  FMC_Bank5_6->SDCMR = 0x00000073;
#endif /* STM32F446xx */
  tmpreg = FMC_Bank5_6->SDSR & 0x00000020;
  timeout = 0xFFFF;
  while((tmpreg != 0) && (timeout-- > 0))
  {
    tmpreg = FMC_Bank5_6->SDSR & 0x00000020; 
  }
 
  /* MRD register program */
#if defined(STM32F446xx)
  FMC_Bank5_6->SDCMR = 0x00044014;
#else  
  FMC_Bank5_6->SDCMR = 0x00046014;
#endif /* STM32F446xx */
  tmpreg = FMC_Bank5_6->SDSR & 0x00000020;
  timeout = 0xFFFF;
  while((tmpreg != 0) && (timeout-- > 0))
  {
    tmpreg = FMC_Bank5_6->SDSR & 0x00000020; 
  } 
  
  /* Set refresh count */
  tmpreg = FMC_Bank5_6->SDRTR;
#if defined(STM32F446xx)
  FMC_Bank5_6->SDRTR = (tmpreg | (0x0000050C<<1));
#else    
  FMC_Bank5_6->SDRTR = (tmpreg | (0x0000027C<<1));
#endif /* STM32F446xx */
  
  /* Disable write protection */
  // [解析] tmpreg 在这里又被借来暂存寄存器旧值（它就是个通用临时变量）。
  // [解析] 0xFFFFFDFF 的作用是把 bit9 清零（= 与上"0x200 取反"的结果），按上面的原注释，
  // [解析] 这一步是解除 SDRAM 的写保护，让这块外扩内存可以被正常读写。
  tmpreg = FMC_Bank5_6->SDCR[0]; 
  FMC_Bank5_6->SDCR[0] = (tmpreg & 0xFFFFFDFF);
#endif /* DATA_IN_ExtSDRAM */
#endif /* STM32F427xx || STM32F437xx || STM32F429xx || STM32F439xx || STM32F446xx || STM32F469xx || STM32F479xx */

#if defined(STM32F405xx) || defined(STM32F415xx) || defined(STM32F407xx) || defined(STM32F417xx)\
 || defined(STM32F427xx) || defined(STM32F437xx) || defined(STM32F429xx) || defined(STM32F439xx)\
 || defined(STM32F469xx) || defined(STM32F479xx) || defined(STM32F412Zx) || defined(STM32F412Vx)

#if defined(DATA_IN_ExtSRAM)
/*-- GPIOs Configuration -----------------------------------------------------*/
// [解析] 这一段是"只用外扩 SRAM（不用 SDRAM）"时的引脚配置，和前面 SDRAM 那段的套路一样：
// [解析] 先开 GPIO 端口时钟，再把 PD/PE/PF/PG 上的引脚设成 FMC(FSMC) 复用功能。
// [解析] 注意这里的 OSPEEDR 都写成了 100 MHz（前面 SDRAM 那段是 50 MHz）—— 外扩 SRAM
// [解析] 是异步接口，引脚翻转速度给足一点更稳妥，属于 ST 官方例程里的经验值。
   /* Enable GPIOD, GPIOE, GPIOF and GPIOG interface clock */
  RCC->AHB1ENR   |= 0x00000078;
  /* Delay after an RCC peripheral clock enabling */
  tmp = READ_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIODEN);
  
  /* Connect PDx pins to FMC Alternate function */
  GPIOD->AFR[0]  = 0x00CCC0CC;
  GPIOD->AFR[1]  = 0xCCCCCCCC;
  /* Configure PDx pins in Alternate function mode */  
  GPIOD->MODER   = 0xAAAA0A8A;
  /* Configure PDx pins speed to 100 MHz */  
  GPIOD->OSPEEDR = 0xFFFF0FCF;
  /* Configure PDx pins Output type to push-pull */  
  GPIOD->OTYPER  = 0x00000000;
  /* No pull-up, pull-down for PDx pins */ 
  GPIOD->PUPDR   = 0x00000000;

  /* Connect PEx pins to FMC Alternate function */
  GPIOE->AFR[0]  = 0xC00CC0CC;
  GPIOE->AFR[1]  = 0xCCCCCCCC;
  /* Configure PEx pins in Alternate function mode */ 
  GPIOE->MODER   = 0xAAAA828A;
  /* Configure PEx pins speed to 100 MHz */ 
  GPIOE->OSPEEDR = 0xFFFFC3CF;
  /* Configure PEx pins Output type to push-pull */  
  GPIOE->OTYPER  = 0x00000000;
  /* No pull-up, pull-down for PEx pins */ 
  GPIOE->PUPDR   = 0x00000000;

  /* Connect PFx pins to FMC Alternate function */
  GPIOF->AFR[0]  = 0x00CCCCCC;
  GPIOF->AFR[1]  = 0xCCCC0000;
  /* Configure PFx pins in Alternate function mode */   
  GPIOF->MODER   = 0xAA000AAA;
  /* Configure PFx pins speed to 100 MHz */ 
  GPIOF->OSPEEDR = 0xFF000FFF;
  /* Configure PFx pins Output type to push-pull */  
  GPIOF->OTYPER  = 0x00000000;
  /* No pull-up, pull-down for PFx pins */ 
  GPIOF->PUPDR   = 0x00000000;

  /* Connect PGx pins to FMC Alternate function */
  GPIOG->AFR[0]  = 0x00CCCCCC;
  GPIOG->AFR[1]  = 0x000000C0;
  /* Configure PGx pins in Alternate function mode */ 
  GPIOG->MODER   = 0x00085AAA;
  /* Configure PGx pins speed to 100 MHz */ 
  GPIOG->OSPEEDR = 0x000CAFFF;
  /* Configure PGx pins Output type to push-pull */  
  GPIOG->OTYPER  = 0x00000000;
  /* No pull-up, pull-down for PGx pins */ 
  GPIOG->PUPDR   = 0x00000000;
  
/*-- FMC/FSMC Configuration --------------------------------------------------*/
  /* Enable the FMC/FSMC interface clock */
  // [解析] 这里注释写成 "FMC/FSMC" 是有原因的：F4 系列里这两个名字指的是同一个控制器，
  // [解析] 只是早期型号(F405/F407/F415/F417)内核里叫 FSMC，只能驱动 NOR/SRAM；
  // [解析] 后期型号(F427/F429/F446/F469…)升级为 FMC，多了 SDRAM 支持。
  // [解析] 所以本工程（STM32F407）虽然用的是 FSMC_Bank1，但 AHB3ENR 的 bit0 是同一个
  // [解析] 使能位，只是头文件里给出的宏名字不同（FMCEN / FSMCEN）。
  RCC->AHB3ENR         |= 0x00000001;

#if defined(STM32F427xx) || defined(STM32F437xx) || defined(STM32F429xx) || defined(STM32F439xx)
  /* Delay after an RCC peripheral clock enabling */
  tmp = READ_BIT(RCC->AHB3ENR, RCC_AHB3ENR_FMCEN);
  /* Configure and enable Bank1_SRAM2 */
  FMC_Bank1->BTCR[2]  = 0x00001011;
  FMC_Bank1->BTCR[3]  = 0x00000201;
  FMC_Bank1E->BWTR[2] = 0x0fffffff;
#endif /* STM32F427xx || STM32F437xx || STM32F429xx || STM32F439xx */ 
#if defined(STM32F469xx) || defined(STM32F479xx)
  /* Delay after an RCC peripheral clock enabling */
  tmp = READ_BIT(RCC->AHB3ENR, RCC_AHB3ENR_FMCEN);
  /* Configure and enable Bank1_SRAM2 */
  FMC_Bank1->BTCR[2]  = 0x00001091;
  FMC_Bank1->BTCR[3]  = 0x00110212;
  FMC_Bank1E->BWTR[2] = 0x0fffffff;
#endif /* STM32F469xx || STM32F479xx */
#if defined(STM32F405xx) || defined(STM32F415xx) || defined(STM32F407xx)|| defined(STM32F417xx)\
   || defined(STM32F412Zx) || defined(STM32F412Vx)
  // [解析] ↑ 这一组就是本工程（STM32F407xx）会命中的分支：用的是 FSMC_Bank1 那套寄存器。
  // [解析]   注意上面两组 FMC 的代码在 F407 上根本不会编译 —— 头文件里没有 FMC_Bank1 这个
  // [解析]   定义（F407 只有 FSMC），所以要靠这个 #if 把两组代码彻底隔开，否则会编译报错。
  /* Delay after an RCC peripheral clock enabling */
  tmp = READ_BIT(RCC->AHB3ENR, RCC_AHB3ENR_FSMCEN);
  /* Configure and enable Bank1_SRAM2 */
  FSMC_Bank1->BTCR[2]  = 0x00001011;
  FSMC_Bank1->BTCR[3]  = 0x00000201;
  FSMC_Bank1E->BWTR[2] = 0x0FFFFFFF;
#endif /* STM32F405xx || STM32F415xx || STM32F407xx || STM32F417xx || STM32F412Zx || STM32F412Vx */

#endif /* DATA_IN_ExtSRAM */
#endif /* STM32F405xx || STM32F415xx || STM32F407xx || STM32F417xx || STM32F427xx || STM32F437xx ||\
          STM32F429xx || STM32F439xx || STM32F469xx || STM32F479xx || STM32F412Zx || STM32F412Vx  */ 
  (void)(tmp); 
}
#endif /* DATA_IN_ExtSRAM && DATA_IN_ExtSDRAM */
// [解析] ==========================================================================
// [解析] 【本文件阅读小结】
// [解析]   1) 真正会执行到的只有上面两个函数：SystemInit()（上电开 FPU）
// [解析]      和 SystemCoreClockUpdate()（反推主频，本工程实际并未调用）。
// [解析]   2) 中间那几百行 SystemInit_ExtMemCtl() 是外扩 SDRAM/SRAM 的初始化死代码，
// [解析]      因为文件开头的 DATA_IN_ExtSRAM / DATA_IN_ExtSDRAM 两个宏都是注释状态。
// [解析]   3) 本工程的主频链路：12 MHz 晶振 -> PLLM=6 -> VCO 输入 2 MHz
// [解析]      -> PLLN=168 -> VCO 输出 336 MHz -> PLLP=2 -> SYSCLK 168 MHz
// [解析]      -> AHB 不分频 -> HCLK 168 MHz；APB1 42 MHz、APB2 84 MHz。
// [解析]   4) 配置时钟的代码不在这里，而在 Core/Src/main.c 的 SystemClock_Config()；
// [解析]      HAL 的 1 ms 时基在 Core/Src/stm32f4xx_hal_timebase_tim.c（用 TIM1）。
// [解析] ==========================================================================
/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */
