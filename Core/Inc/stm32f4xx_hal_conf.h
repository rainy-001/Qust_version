// [解析] ==========================================================================
// [解析] 【文件】Core/Inc/stm32f4xx_hal_conf.h
// [解析] 【作用】HAL 库的"总开关 + 参数表"，决定三件事：
// [解析]   ① 哪些 HAL 模块参与编译（Module Selection 那一大段 #define）；
// [解析]   ② 板子的外部硬件参数（晶振频率、供电电压等），HAL 靠它们换算真实的时钟与超时；
// [解析]   ③ 若干功能开关（ART 缓存、SPI 的 CRC、参数断言、以太网 PHY 等）。
// [解析] 【谁生成】STM32CubeMX 自动生成，内容来自 .ioc 里 Project Manager / 各外设的配置。
// [解析]           小提示：文件内部的 @file 还写着 stm32f4xx_hal_conf_template.h，
// [解析]           那是 CubeMX 从模板复制过来时没改的残留说明，不影响使用。
// [解析] 【关键点】
// [解析]   1) 用到了某个外设，却忘了在这里打开对应的 HAL_xxx_MODULE_ENABLED，
// [解析]      编译时会报 "undefined reference to HAL_xxx_Init" 之类的链接错误。
// [解析]   2) HSE_VALUE 必须等于板上晶振的真实频率（本工程 12MHz）。填错不会报错，
// [解析]      但串口波特率、定时器周期、各种超时全都会偏掉。
// [解析]   3) 文件下半部分那一大堆 #ifdef ... #include ... 是"按开关引头文件"的固定套路，
// [解析]      不用逐行看，理解机制即可（已经在对应位置说明）。
// [解析] ==========================================================================
/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_hal_conf_template.h
  * @author  MCD Application Team
  * @brief   HAL configuration template file.
  *          This file should be copied to the application folder and renamed
  *          to stm32f4xx_hal_conf.h.
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
// [解析] 上面是 CubeMX 自动生成的文件说明注释块（@file / @brief / 版权），模板文字，原样保留即可。
/* USER CODE END Header */

// [解析] "防重复包含"保护：同一个 .c 文件里如果间接包含了本文件两次，
// [解析]   第二次 #ifndef 判断为假，内容不再重复展开，避免宏/类型被重复定义。
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F4xx_HAL_CONF_H
#define __STM32F4xx_HAL_CONF_H

// [解析] 与 main.h 里同样的"C/C++ 混编"写法：让 C++ 代码能正确调用本文件里声明的 C 函数。
// [解析] 本工程业务代码是 C++（applications/*.cpp），而 HAL 是 C，所以必须有这对标记。
#ifdef __cplusplus
 extern "C" {
#endif

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

// [解析] ============== ① 模块选择：哪些 HAL 外设驱动参与编译 ==============
// [解析] 每一个 #define HAL_xxx_MODULE_ENABLED 都是一个"编译开关"：
// [解析]   打开 → 这个外设的 HAL 驱动被编进程序，可以使用 HAL_xxx_Init() 等函数；
// [解析]   注释掉 → 该模块完全不编译（省 Flash），但它的任何函数都用不了。
// [解析] ★ 新手最常见的链接错误 "undefined reference to HAL_xxx_xxx" 就是这里忘了开。
// [解析] 本工程开启的模块（正好对应硬件上真正用到的东西）：
// [解析]   ADC  —— 读电池电压（ADC3 / PF10）
// [解析]   CAN  —— 和电机、超电通信（CAN1 / PD0、PD1）
// [解析]   SPI  —— 读 BMI088 六轴传感器（SPI1）
// [解析]   TIM  —— 蜂鸣器(TIM4)、RGB 灯(TIM5)、以及 HAL 时基(TIM1)
// [解析]   UART —— 收 DBUS 遥控器信号（USART3）
// [解析]   GPIO / EXTI / DMA / RCC / FLASH / PWR / CORTEX —— 底层基础设施，几乎必须全开
// [解析] 其余全部处于注释状态，说明本工程没用到：I2C、USB、以太网、RTC、看门狗、SD 卡、摄像头……等。
// [解析] 下面这一行 HAL_MODULE_ENABLED 本身不控制任何东西，是 CubeMX 的固定写法，不用管它。
/* ########################## Module Selection ############################## */
/**
  * @brief This is the list of modules to be used in the HAL driver
  */
#define HAL_MODULE_ENABLED

  /* #define HAL_CRYP_MODULE_ENABLED */
// [解析] ↓ 下面这几行是"真正干活"的外设模块，每一行都对应机器人上的一个功能，
// [解析]   缺哪一行，对应功能的 HAL 函数就会链接失败：
// [解析]   ADC → 电池电压检测(voltage_detect.cpp)；CAN → 电机通信(can_task.cpp)；
// [解析]   SPI → BMI088 姿态传感器(imu_task.cpp)；
// [解析]   TIM → 蜂鸣器(TIM4)、RGB 灯(TIM5)、HAL 时基(TIM1)；UART → DBUS 遥控器(USART3)。
#define HAL_ADC_MODULE_ENABLED
#define HAL_CAN_MODULE_ENABLED
/* #define HAL_CRC_MODULE_ENABLED */
/* #define HAL_CAN_LEGACY_MODULE_ENABLED */
/* #define HAL_DAC_MODULE_ENABLED */
/* #define HAL_DCMI_MODULE_ENABLED */
/* #define HAL_DMA2D_MODULE_ENABLED */
/* #define HAL_ETH_MODULE_ENABLED */
/* #define HAL_ETH_LEGACY_MODULE_ENABLED */
/* #define HAL_NAND_MODULE_ENABLED */
/* #define HAL_NOR_MODULE_ENABLED */
/* #define HAL_PCCARD_MODULE_ENABLED */
/* #define HAL_SRAM_MODULE_ENABLED */
/* #define HAL_SDRAM_MODULE_ENABLED */
/* #define HAL_HASH_MODULE_ENABLED */
/* #define HAL_I2C_MODULE_ENABLED */
/* #define HAL_I2S_MODULE_ENABLED */
/* #define HAL_IWDG_MODULE_ENABLED */
/* #define HAL_LTDC_MODULE_ENABLED */
/* #define HAL_RNG_MODULE_ENABLED */
/* #define HAL_RTC_MODULE_ENABLED */
/* #define HAL_SAI_MODULE_ENABLED */
/* #define HAL_SD_MODULE_ENABLED */
/* #define HAL_MMC_MODULE_ENABLED */
#define HAL_SPI_MODULE_ENABLED
#define HAL_TIM_MODULE_ENABLED
#define HAL_UART_MODULE_ENABLED
// [解析] 注意区分 UART 与 USART：本工程遥控器接的是 USART3 外设，但它工作在"异步串口"模式，
// [解析]   由 HAL_UART 模块负责（上面那行已经开了）。这里注释掉的 HAL_USART 模块是
// [解析]   "同步/智能卡"那套用法，本工程用不到，所以不编译。
/* #define HAL_USART_MODULE_ENABLED */
/* #define HAL_IRDA_MODULE_ENABLED */
/* #define HAL_SMARTCARD_MODULE_ENABLED */
/* #define HAL_SMBUS_MODULE_ENABLED */
/* #define HAL_WWDG_MODULE_ENABLED */
/* #define HAL_PCD_MODULE_ENABLED */
/* #define HAL_HCD_MODULE_ENABLED */
/* #define HAL_DSI_MODULE_ENABLED */
// [解析] 小冗余：上面两行 HAL_QSPI_MODULE_ENABLED 是重复出现的（都是注释状态），
// [解析]   来自 CubeMX 模板，不影响编译。本工程没用到 QSPI，无需处理。
/* #define HAL_QSPI_MODULE_ENABLED */
/* #define HAL_QSPI_MODULE_ENABLED */
/* #define HAL_CEC_MODULE_ENABLED */
/* #define HAL_FMPI2C_MODULE_ENABLED */
/* #define HAL_FMPSMBUS_MODULE_ENABLED */
/* #define HAL_SPDIFRX_MODULE_ENABLED */
/* #define HAL_DFSDM_MODULE_ENABLED */
/* #define HAL_LPTIM_MODULE_ENABLED */
// [解析] ↓ 这 7 个属于"基础设施模块"，几乎所有 STM32 工程都会打开，一般不要动：
// [解析]   GPIO(引脚)、EXTI(外部中断，BMI088 的 INT 脚要用)、DMA(后台搬运数据，串口收包要用)、
// [解析]   RCC(时钟与复位)、FLASH(程序存储/等待周期)、PWR(电源与内核调压)、CORTEX(内核、NVIC)
#define HAL_GPIO_MODULE_ENABLED
#define HAL_EXTI_MODULE_ENABLED
#define HAL_DMA_MODULE_ENABLED
#define HAL_RCC_MODULE_ENABLED
#define HAL_FLASH_MODULE_ENABLED
#define HAL_PWR_MODULE_ENABLED
#define HAL_CORTEX_MODULE_ENABLED

// [解析] ============== ② 时钟源参数（必须与板上硬件一致，填错不报错但一切时间都会偏） ==============
// [解析] 下面这些宏是"HAL 眼里的硬件参数"。HAL 用它们把寄存器里的分频/倍频数值换算成
// [解析]   真实频率，进而算出串口波特率、定时器周期、超时时间等。
// [解析] 本工程实际只用到 HSE（外部 12MHz 晶振），但其余几项 HAL 编译时也必须有定义，
// [解析]   所以 CubeMX 一律给出标准值保留着。
/* ########################## HSE/HSI Values adaptation ##################### */
/**
  * @brief Adjust the value of External High Speed oscillator (HSE) used in your application.
  *        This value is used by the RCC HAL module to compute the system frequency
  *        (when HSE is used as system clock source, directly or through the PLL).
  */
// [解析] ★ 外部高速晶振(HSE)的真实频率，单位 Hz。C 板焊的是 12MHz 晶振，所以这里必须是 12000000U。
// [解析]   它和 .ioc 里的 PLL 参数配合工作：PLLM=6 → 12MHz/6 = 2MHz（PLL 的标准输入频率），
// [解析]   再 ×168 ÷2 得到 168MHz 系统主频。完整过程见 Core/Src/main.c 的 SystemClock_Config()，
// [解析]   以及 docs/C板引脚配置说明.md 第 8 节（换晶振时必须同时改这里和 PLLM！）。
// [解析] 外层 #if !defined (HSE_VALUE) 的含义：允许在编译命令里用 -DHSE_VALUE=xxx 覆盖本值，
// [解析]   属于给高级用法的"后门"，正常运行不需要关心。
#if !defined  (HSE_VALUE)
  #define HSE_VALUE    12000000U /*!< Value of the External oscillator in Hz */
#endif /* HSE_VALUE */

// [解析] 等待晶振起振的最长时间，单位毫秒（100 = 100ms）。
// [解析] 如果晶振虚焊/损坏，HAL 会等满这个时间然后放弃，HAL_RCC_OscConfig() 返回错误，
// [解析]   main.c 里的 Error_Handler() 就把程序停住。所以"上电就卡死"可以先怀疑晶振。
#if !defined  (HSE_STARTUP_TIMEOUT)
  #define HSE_STARTUP_TIMEOUT    100U   /*!< Time out for HSE start up, in ms */
#endif /* HSE_STARTUP_TIMEOUT */

/**
  * @brief Internal High Speed oscillator (HSI) value.
  *        This value is used by the RCC HAL module to compute the system frequency
  *        (when HSI is used as system clock source, directly or through the PLL).
  */
// [解析] HSI = 芯片内部的 RC 振荡器（不用外接晶振就能跑），标称 16MHz，但精度较差。
// [解析] 本工程没有拿它当主时钟源（用的是更准的 HSE 晶振），这个宏只是 HAL 编译时需要。
// [解析] 调试小知识：若 HSE 起不来，程序其实可以退化到 HSI 上跑，但主频变成 16MHz，
// [解析]   所有基于 168MHz 计算的波特率、PWM 频率都会错几倍。
#if !defined  (HSI_VALUE)
  #define HSI_VALUE    ((uint32_t)16000000U) /*!< Value of the Internal oscillator in Hz*/
#endif /* HSI_VALUE */

/**
  * @brief Internal Low Speed oscillator (LSI) value.
  */
// [解析] LSI = 内部低速 RC 振荡器（约 32kHz）、LSE = 外部低速晶振（32.768kHz）。
// [解析]   这两个时钟通常服务 RTC（实时时钟）和独立看门狗 IWDG。本工程没有启用 RTC/IWDG，
// [解析]   所以这两个宏只是"让 HAL 能编译通过"，填的是标准值，不影响运行。
// [解析] 原注释提醒：LSI 的实际频率随电压和温度有偏差，所以不能用来做精确定时。
#if !defined  (LSI_VALUE)
 #define LSI_VALUE  32000U       /*!< LSI Typical Value in Hz*/
#endif /* LSI_VALUE */                      /*!< Value of the Internal Low Speed oscillator in Hz
                                             The real value may vary depending on the variations
                                             in voltage and temperature.*/
/**
  * @brief External Low Speed oscillator (LSE) value.
  */
#if !defined  (LSE_VALUE)
 #define LSE_VALUE  32768U    /*!< Value of the External Low Speed oscillator in Hz */
#endif /* LSE_VALUE */

// [解析] 等待 LSE（外部 32.768kHz 晶振）起振的最长时间，单位毫秒。5000 = 5 秒。
// [解析] 低速晶振起振慢，所以给的时间比 HSE 长得多。本工程没用 LSE，这个值不会被触发。
#if !defined  (LSE_STARTUP_TIMEOUT)
  #define LSE_STARTUP_TIMEOUT    5000U   /*!< Time out for LSE start up, in ms */
#endif /* LSE_STARTUP_TIMEOUT */

/**
  * @brief External clock source for I2S peripheral
  *        This value is used by the I2S HAL module to compute the I2S clock source
  *        frequency, this source is inserted directly through I2S_CKIN pad.
  */
// [解析] I2S 外接时钟(CKIN)的频率。本工程没用 I2S（HAL_I2S_MODULE_ENABLED 也是关的），
// [解析]   这一项属于模板残留，保留不影响任何功能。
#if !defined  (EXTERNAL_CLOCK_VALUE)
  #define EXTERNAL_CLOCK_VALUE    12288000U /*!< Value of the External audio frequency in Hz*/
#endif /* EXTERNAL_CLOCK_VALUE */

/* Tip: To avoid modifying this file each time you need to use different HSE,
   ===  you can define the HSE value in your toolchain compiler preprocessor. */

// [解析] ============== ③ 系统级参数（供电电压、HAL 时基中断优先级、缓存、回调注册） ==============
/* ########################### System Configuration ######################### */
/**
  * @brief This is the HAL system configuration section
  */
// [解析] VDD_VALUE：芯片供电电压，单位【毫伏】，3300 = 3.3V。HAL 内部某些换算要用到它，
// [解析]   板子是 3.3V 供电，所以保持 3300U；填错会影响模拟量换算的准确性。
#define  VDD_VALUE		      3300U /*!< Value of VDD in mv */
// [解析] HAL"1ms 时基中断"要占用的 NVIC 优先级（15 = 最低优先级）。
// [解析] ★ 本工程的时基是 TIM1 而不是 SysTick（SysTick 被 FreeRTOS 占了），所以这个值最终
// [解析]   被写到 TIM1_UP_TIM10 中断上，见 Core/Src/stm32f4xx_hal_timebase_tim.c。
// [解析]   它比 FreeRTOSConfig.h 里那条分界线(configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY = 5)
// [解析]   数值更大、优先级更低，所以理论上是允许在该中断里调用 FreeRTOS 的 FromISR 接口的。
#define  TICK_INT_PRIORITY            15U   /*!< tick interrupt priority */
// [解析] USE_RTOS 是 CubeMX 模板里的历史遗留项，STM32F4 的 HAL 内部并没有真正使用它。
// [解析]   ★ 所以"本工程明明跑着 FreeRTOS，这里却是 0U"是正常现象，不要以为填错了。
// [解析]   FreeRTOS 相关的真正配置在 Core/Inc/FreeRTOSConfig.h 里。
#define  USE_RTOS                     0U
// [解析] STM32F4 的"自适应实时加速器(ART)"相关开关：预取指、指令缓存、数据缓存，1 = 打开。
// [解析]   它们让 CPU 从 Flash 取指令时不用等待，从而在 168MHz 下满速运行（不像老 F1 需要插等待周期）。
// [解析]   ★ 这三个关掉程序功能仍正常，但会明显变慢，一般不修改。
#define  PREFETCH_ENABLE              1U
#define  INSTRUCTION_CACHE_ENABLE     1U
#define  DATA_CACHE_ENABLE            1U

// [解析] ↓↓↓ 下面这 39 行全是同一个含义，统一解释一次，不用逐行看 ↓↓↓
// [解析] 它们叫"回调注册开关"(USE_HAL_xxx_REGISTER_CALLBACKS)，决定回调函数的写法：
// [解析]   1U = 允许运行时用 HAL_xxx_RegisterCallback() 动态注册回调（灵活，但每个外设多占一点 RAM）；
// [解析]   0U = 使用传统的 __weak 弱函数覆盖方式（回调函数名固定，用户重写一个同名强函数即可）。
// [解析] ★ 本工程全部是 0U，也就是走 __weak 方式，例如：
// [解析]   HAL_TIM_PeriodElapsedCallback()（main.c 里的 1ms 心跳）、
// [解析]   HAL_CAN_RxFifo0MsgPendingCallback()（applications/can_task.cpp 里的收帧回调），
// [解析]   都是靠"在别处写同名函数覆盖弱定义"来生效的。
// [解析] 只有本工程实际启用的模块（ADC/CAN/SPI/TIM/UART…）对应的开关才有意义，其余是模板填空。
#define  USE_HAL_ADC_REGISTER_CALLBACKS         0U /* ADC register callback disabled       */
#define  USE_HAL_CAN_REGISTER_CALLBACKS         0U /* CAN register callback disabled       */
#define  USE_HAL_CEC_REGISTER_CALLBACKS         0U /* CEC register callback disabled       */
#define  USE_HAL_CRYP_REGISTER_CALLBACKS        0U /* CRYP register callback disabled      */
#define  USE_HAL_DAC_REGISTER_CALLBACKS         0U /* DAC register callback disabled       */
#define  USE_HAL_DCMI_REGISTER_CALLBACKS        0U /* DCMI register callback disabled      */
#define  USE_HAL_DFSDM_REGISTER_CALLBACKS       0U /* DFSDM register callback disabled     */
#define  USE_HAL_DMA2D_REGISTER_CALLBACKS       0U /* DMA2D register callback disabled     */
#define  USE_HAL_DSI_REGISTER_CALLBACKS         0U /* DSI register callback disabled       */
#define  USE_HAL_ETH_REGISTER_CALLBACKS         0U /* ETH register callback disabled       */
#define  USE_HAL_HASH_REGISTER_CALLBACKS        0U /* HASH register callback disabled      */
#define  USE_HAL_HCD_REGISTER_CALLBACKS         0U /* HCD register callback disabled       */
#define  USE_HAL_I2C_REGISTER_CALLBACKS         0U /* I2C register callback disabled       */
#define  USE_HAL_FMPI2C_REGISTER_CALLBACKS      0U /* FMPI2C register callback disabled    */
#define  USE_HAL_FMPSMBUS_REGISTER_CALLBACKS    0U /* FMPSMBUS register callback disabled  */
#define  USE_HAL_I2S_REGISTER_CALLBACKS         0U /* I2S register callback disabled       */
#define  USE_HAL_IRDA_REGISTER_CALLBACKS        0U /* IRDA register callback disabled      */
#define  USE_HAL_LPTIM_REGISTER_CALLBACKS       0U /* LPTIM register callback disabled     */
#define  USE_HAL_LTDC_REGISTER_CALLBACKS        0U /* LTDC register callback disabled      */
#define  USE_HAL_MMC_REGISTER_CALLBACKS         0U /* MMC register callback disabled       */
#define  USE_HAL_NAND_REGISTER_CALLBACKS        0U /* NAND register callback disabled      */
#define  USE_HAL_NOR_REGISTER_CALLBACKS         0U /* NOR register callback disabled       */
#define  USE_HAL_PCCARD_REGISTER_CALLBACKS      0U /* PCCARD register callback disabled    */
#define  USE_HAL_PCD_REGISTER_CALLBACKS         0U /* PCD register callback disabled       */
#define  USE_HAL_QSPI_REGISTER_CALLBACKS        0U /* QSPI register callback disabled      */
#define  USE_HAL_RNG_REGISTER_CALLBACKS         0U /* RNG register callback disabled       */
#define  USE_HAL_RTC_REGISTER_CALLBACKS         0U /* RTC register callback disabled       */
#define  USE_HAL_SAI_REGISTER_CALLBACKS         0U /* SAI register callback disabled       */
#define  USE_HAL_SD_REGISTER_CALLBACKS          0U /* SD register callback disabled        */
#define  USE_HAL_SMARTCARD_REGISTER_CALLBACKS   0U /* SMARTCARD register callback disabled */
#define  USE_HAL_SDRAM_REGISTER_CALLBACKS       0U /* SDRAM register callback disabled     */
#define  USE_HAL_SRAM_REGISTER_CALLBACKS        0U /* SRAM register callback disabled      */
#define  USE_HAL_SPDIFRX_REGISTER_CALLBACKS     0U /* SPDIFRX register callback disabled   */
#define  USE_HAL_SMBUS_REGISTER_CALLBACKS       0U /* SMBUS register callback disabled     */
#define  USE_HAL_SPI_REGISTER_CALLBACKS         0U /* SPI register callback disabled       */
#define  USE_HAL_TIM_REGISTER_CALLBACKS         0U /* TIM register callback disabled       */
#define  USE_HAL_UART_REGISTER_CALLBACKS        0U /* UART register callback disabled      */
#define  USE_HAL_USART_REGISTER_CALLBACKS       0U /* USART register callback disabled     */
#define  USE_HAL_WWDG_REGISTER_CALLBACKS        0U /* WWDG register callback disabled      */

// [解析] ============== ④ 参数断言开关（调试用的"参数检查"） ==============
// [解析] 下面这行 USE_FULL_ASSERT 目前处于【注释状态】= 不启用。
// [解析] 启用后的效果：HAL 每个函数开头都会检查传入的参数是否合法，
// [解析]   不合法就调用 assert_failed(文件名, 行号)（这个函数需要你自己实现），
// [解析]   能第一时间指出"哪个文件的哪一行传错了参数"，调试时非常有用。
// [解析] 代价：代码体积变大、执行变慢，所以只在调试阶段打开、比赛前关掉。
// [解析] ★ 本工程是关闭的，于是文件末尾的 assert_param() 被展开成"什么都不做"，
// [解析]   也就是 HAL 不会替你检查参数 —— 参数写错了只能从现象去猜（很费时间，务必小心）。
/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */

// [解析] ============== ⑤ 以太网(PHY)配置 —— ★ 本工程【没有】用到，整段可以直接跳过 ==============
// [解析] 为什么说没用？因为上面的模块选择里 HAL_ETH_MODULE_ENABLED 是注释掉的，
// [解析]   所以这里填的所有内容都不会被编译进程序，属于 CubeMX 模板的固定残留。
// [解析] 留着它的唯一意义：将来真要用以太网时这些默认值还在。
// [解析] 各项简要说明（要用以太网时再看）：
// [解析]   MAC_ADDR0~5：网卡的 6 字节 MAC 地址，同一网络内不能重复。
// [解析]   ETH_RX_BUF_SIZE / ETH_TXBUF_SIZE：收/发缓冲区大小，取最大以太网包长 1524 字节。
// [解析]   ETH_RXBUFNB / ETH_TXBUFNB：收、发各准备 4 个缓冲区（避免来不及处理时丢包）。
// [解析]   DP83848_PHY_ADDRESS：PHY 芯片地址；PHY_RESET_DELAY / PHY_CONFIG_DELAY：复位与配置延时。
// [解析]   PHY_BCR/PHY_BSR 及后面一串 PHY_xxx：PHY 芯片寄存器地址与位定义（IEEE 802.3 标准）。
// [解析] ★ 注意：DP83848_PHY_ADDRESS、PHY_SR、PHY_SPEED_STATUS、PHY_DUPLEX_STATUS 这 4 行
// [解析]   都没有真正填值 —— 前者是彻底空的；后三者只写了 ((uint16_t)) 这个类型转换、
// [解析]   括号里没有数值。原因：它们的真实取值要按实际 PHY 芯片型号填写，
// [解析]   CubeMX 只有在启用以太网时才会生成。现在整段不参与编译所以无害；
// [解析]   但将来若打开 HAL_ETH_MODULE_ENABLED，必须先补上这些值，
// [解析]   否则宏一展开就是语法错误（例如 ((uint16_t)) 单独出现是不能编译的）。
/* ################## Ethernet peripheral configuration ##################### */

/* Section 1 : Ethernet peripheral configuration */

/* MAC ADDRESS: MAC_ADDR0:MAC_ADDR1:MAC_ADDR2:MAC_ADDR3:MAC_ADDR4:MAC_ADDR5 */
#define MAC_ADDR0   2U
#define MAC_ADDR1   0U
#define MAC_ADDR2   0U
#define MAC_ADDR3   0U
#define MAC_ADDR4   0U
#define MAC_ADDR5   0U

/* Definition of the Ethernet driver buffers size and count */
#define ETH_RX_BUF_SIZE                ETH_MAX_PACKET_SIZE /* buffer size for receive               */
#define ETH_TX_BUF_SIZE                ETH_MAX_PACKET_SIZE /* buffer size for transmit              */
#define ETH_RXBUFNB                    4U       /* 4 Rx buffers of size ETH_RX_BUF_SIZE  */
#define ETH_TXBUFNB                    4U       /* 4 Tx buffers of size ETH_TX_BUF_SIZE  */

/* Section 2: PHY configuration section */

/* DP83848_PHY_ADDRESS Address*/
#define DP83848_PHY_ADDRESS
/* PHY Reset delay these values are based on a 1 ms Systick interrupt*/
#define PHY_RESET_DELAY                 0x000000FFU
/* PHY Configuration delay */
#define PHY_CONFIG_DELAY                0x00000FFFU

#define PHY_READ_TO                     0x0000FFFFU
#define PHY_WRITE_TO                    0x0000FFFFU

/* Section 3: Common PHY Registers */

#define PHY_BCR                         ((uint16_t)0x0000U)    /*!< Transceiver Basic Control Register   */
#define PHY_BSR                         ((uint16_t)0x0001U)    /*!< Transceiver Basic Status Register    */

#define PHY_RESET                       ((uint16_t)0x8000U)  /*!< PHY Reset */
#define PHY_LOOPBACK                    ((uint16_t)0x4000U)  /*!< Select loop-back mode */
#define PHY_FULLDUPLEX_100M             ((uint16_t)0x2100U)  /*!< Set the full-duplex mode at 100 Mb/s */
#define PHY_HALFDUPLEX_100M             ((uint16_t)0x2000U)  /*!< Set the half-duplex mode at 100 Mb/s */
#define PHY_FULLDUPLEX_10M              ((uint16_t)0x0100U)  /*!< Set the full-duplex mode at 10 Mb/s  */
#define PHY_HALFDUPLEX_10M              ((uint16_t)0x0000U)  /*!< Set the half-duplex mode at 10 Mb/s  */
#define PHY_AUTONEGOTIATION             ((uint16_t)0x1000U)  /*!< Enable auto-negotiation function     */
#define PHY_RESTART_AUTONEGOTIATION     ((uint16_t)0x0200U)  /*!< Restart auto-negotiation function    */
#define PHY_POWERDOWN                   ((uint16_t)0x0800U)  /*!< Select the power down mode           */
#define PHY_ISOLATE                     ((uint16_t)0x0400U)  /*!< Isolate PHY from MII                 */

#define PHY_AUTONEGO_COMPLETE           ((uint16_t)0x0020U)  /*!< Auto-Negotiation process completed   */
#define PHY_LINKED_STATUS               ((uint16_t)0x0004U)  /*!< Valid link established               */
#define PHY_JABBER_DETECTION            ((uint16_t)0x0002U)  /*!< Jabber condition detected            */

/* Section 4: Extended PHY Registers */
#define PHY_SR                          ((uint16_t))    /*!< PHY status register Offset                      */

#define PHY_SPEED_STATUS                ((uint16_t))  /*!< PHY Speed mask                                  */
#define PHY_DUPLEX_STATUS               ((uint16_t))  /*!< PHY Duplex mask                                 */

// [解析] ============== ⑥ SPI 的 CRC 校验开关 ==============
// [解析] SPI 协议可以选择在每帧数据后附带 1 个 CRC 校验字节，用来发现通信错误。
// [解析] USE_SPI_CRC = 0 表示不用：代码里不编译 CRC 相关逻辑，省一点 Flash。
// [解析] ★ 本工程 SPI1 接的是 BMI088 传感器，它的协议里没有 CRC，
// [解析]   所以必须保持 0；若误改成 1，HAL 会往数据流里插入 CRC 字节，传感器就读不出正确数据了。
/* ################## SPI peripheral configuration ########################## */

/* CRC FEATURE: Use to activate CRC feature inside HAL SPI Driver
* Activated: CRC code is present inside driver
* Deactivated: CRC code cleaned from driver
*/

#define USE_SPI_CRC                     0U

// [解析] ============== ⑦ 按上面的开关"按需"引入各 HAL 子模块的头文件 ==============
// [解析] 下面 200 来行的写法完全一样，不需要逐行看，套路是：
// [解析]   #ifdef HAL_xxx_MODULE_ENABLED          ← 如果这个模块被打开了
// [解析]     #include "stm32f4xx_hal_xxx.h"       ← 就把它的头文件包含进来
// [解析]   #endif
// [解析] 也就是说：上面"打开哪些模块"和这里"包含哪些头文件"是严格一一对应的。
// [解析]   打开一个模块 = 它的 .c 源文件参与编译 + 它的头文件被包含。
// [解析] 本工程真正会生效的只有十几行（RCC / GPIO / EXTI / DMA / CORTEX / ADC / CAN / FLASH /
// [解析]   PWR / SPI / TIM / UART）。想确认某外设是否启用，看上面有没有它的 MODULE_ENABLED 即可。
/* Includes ------------------------------------------------------------------*/
/**
  * @brief Include module's header file
  */

#ifdef HAL_RCC_MODULE_ENABLED
  #include "stm32f4xx_hal_rcc.h"
#endif /* HAL_RCC_MODULE_ENABLED */

#ifdef HAL_GPIO_MODULE_ENABLED
  #include "stm32f4xx_hal_gpio.h"
#endif /* HAL_GPIO_MODULE_ENABLED */

#ifdef HAL_EXTI_MODULE_ENABLED
  #include "stm32f4xx_hal_exti.h"
#endif /* HAL_EXTI_MODULE_ENABLED */

#ifdef HAL_DMA_MODULE_ENABLED
  #include "stm32f4xx_hal_dma.h"
#endif /* HAL_DMA_MODULE_ENABLED */

#ifdef HAL_CORTEX_MODULE_ENABLED
  #include "stm32f4xx_hal_cortex.h"
#endif /* HAL_CORTEX_MODULE_ENABLED */

#ifdef HAL_ADC_MODULE_ENABLED
  #include "stm32f4xx_hal_adc.h"
#endif /* HAL_ADC_MODULE_ENABLED */

#ifdef HAL_CAN_MODULE_ENABLED
  #include "stm32f4xx_hal_can.h"
#endif /* HAL_CAN_MODULE_ENABLED */

#ifdef HAL_CAN_LEGACY_MODULE_ENABLED
  #include "stm32f4xx_hal_can_legacy.h"
#endif /* HAL_CAN_LEGACY_MODULE_ENABLED */

#ifdef HAL_CRC_MODULE_ENABLED
  #include "stm32f4xx_hal_crc.h"
#endif /* HAL_CRC_MODULE_ENABLED */

#ifdef HAL_CRYP_MODULE_ENABLED
  #include "stm32f4xx_hal_cryp.h"
#endif /* HAL_CRYP_MODULE_ENABLED */

#ifdef HAL_DMA2D_MODULE_ENABLED
  #include "stm32f4xx_hal_dma2d.h"
#endif /* HAL_DMA2D_MODULE_ENABLED */

#ifdef HAL_DAC_MODULE_ENABLED
  #include "stm32f4xx_hal_dac.h"
#endif /* HAL_DAC_MODULE_ENABLED */

#ifdef HAL_DCMI_MODULE_ENABLED
  #include "stm32f4xx_hal_dcmi.h"
#endif /* HAL_DCMI_MODULE_ENABLED */

#ifdef HAL_ETH_MODULE_ENABLED
  #include "stm32f4xx_hal_eth.h"
#endif /* HAL_ETH_MODULE_ENABLED */

#ifdef HAL_ETH_LEGACY_MODULE_ENABLED
  #include "stm32f4xx_hal_eth_legacy.h"
#endif /* HAL_ETH_LEGACY_MODULE_ENABLED */

#ifdef HAL_FLASH_MODULE_ENABLED
  #include "stm32f4xx_hal_flash.h"
#endif /* HAL_FLASH_MODULE_ENABLED */

#ifdef HAL_SRAM_MODULE_ENABLED
  #include "stm32f4xx_hal_sram.h"
#endif /* HAL_SRAM_MODULE_ENABLED */

#ifdef HAL_NOR_MODULE_ENABLED
  #include "stm32f4xx_hal_nor.h"
#endif /* HAL_NOR_MODULE_ENABLED */

#ifdef HAL_NAND_MODULE_ENABLED
  #include "stm32f4xx_hal_nand.h"
#endif /* HAL_NAND_MODULE_ENABLED */

#ifdef HAL_PCCARD_MODULE_ENABLED
  #include "stm32f4xx_hal_pccard.h"
#endif /* HAL_PCCARD_MODULE_ENABLED */

#ifdef HAL_SDRAM_MODULE_ENABLED
  #include "stm32f4xx_hal_sdram.h"
#endif /* HAL_SDRAM_MODULE_ENABLED */

#ifdef HAL_HASH_MODULE_ENABLED
 #include "stm32f4xx_hal_hash.h"
#endif /* HAL_HASH_MODULE_ENABLED */

#ifdef HAL_I2C_MODULE_ENABLED
 #include "stm32f4xx_hal_i2c.h"
#endif /* HAL_I2C_MODULE_ENABLED */

#ifdef HAL_SMBUS_MODULE_ENABLED
 #include "stm32f4xx_hal_smbus.h"
#endif /* HAL_SMBUS_MODULE_ENABLED */

#ifdef HAL_I2S_MODULE_ENABLED
 #include "stm32f4xx_hal_i2s.h"
#endif /* HAL_I2S_MODULE_ENABLED */

#ifdef HAL_IWDG_MODULE_ENABLED
 #include "stm32f4xx_hal_iwdg.h"
#endif /* HAL_IWDG_MODULE_ENABLED */

#ifdef HAL_LTDC_MODULE_ENABLED
 #include "stm32f4xx_hal_ltdc.h"
#endif /* HAL_LTDC_MODULE_ENABLED */

#ifdef HAL_PWR_MODULE_ENABLED
 #include "stm32f4xx_hal_pwr.h"
#endif /* HAL_PWR_MODULE_ENABLED */

#ifdef HAL_RNG_MODULE_ENABLED
 #include "stm32f4xx_hal_rng.h"
#endif /* HAL_RNG_MODULE_ENABLED */

#ifdef HAL_RTC_MODULE_ENABLED
 #include "stm32f4xx_hal_rtc.h"
#endif /* HAL_RTC_MODULE_ENABLED */

#ifdef HAL_SAI_MODULE_ENABLED
 #include "stm32f4xx_hal_sai.h"
#endif /* HAL_SAI_MODULE_ENABLED */

#ifdef HAL_SD_MODULE_ENABLED
 #include "stm32f4xx_hal_sd.h"
#endif /* HAL_SD_MODULE_ENABLED */

#ifdef HAL_SPI_MODULE_ENABLED
 #include "stm32f4xx_hal_spi.h"
#endif /* HAL_SPI_MODULE_ENABLED */

#ifdef HAL_TIM_MODULE_ENABLED
 #include "stm32f4xx_hal_tim.h"
#endif /* HAL_TIM_MODULE_ENABLED */

#ifdef HAL_UART_MODULE_ENABLED
 #include "stm32f4xx_hal_uart.h"
#endif /* HAL_UART_MODULE_ENABLED */

#ifdef HAL_USART_MODULE_ENABLED
 #include "stm32f4xx_hal_usart.h"
#endif /* HAL_USART_MODULE_ENABLED */

#ifdef HAL_IRDA_MODULE_ENABLED
 #include "stm32f4xx_hal_irda.h"
#endif /* HAL_IRDA_MODULE_ENABLED */

#ifdef HAL_SMARTCARD_MODULE_ENABLED
 #include "stm32f4xx_hal_smartcard.h"
#endif /* HAL_SMARTCARD_MODULE_ENABLED */

#ifdef HAL_WWDG_MODULE_ENABLED
 #include "stm32f4xx_hal_wwdg.h"
#endif /* HAL_WWDG_MODULE_ENABLED */

#ifdef HAL_PCD_MODULE_ENABLED
 #include "stm32f4xx_hal_pcd.h"
#endif /* HAL_PCD_MODULE_ENABLED */

#ifdef HAL_HCD_MODULE_ENABLED
 #include "stm32f4xx_hal_hcd.h"
#endif /* HAL_HCD_MODULE_ENABLED */

#ifdef HAL_DSI_MODULE_ENABLED
 #include "stm32f4xx_hal_dsi.h"
#endif /* HAL_DSI_MODULE_ENABLED */

#ifdef HAL_QSPI_MODULE_ENABLED
 #include "stm32f4xx_hal_qspi.h"
#endif /* HAL_QSPI_MODULE_ENABLED */

#ifdef HAL_CEC_MODULE_ENABLED
 #include "stm32f4xx_hal_cec.h"
#endif /* HAL_CEC_MODULE_ENABLED */

#ifdef HAL_FMPI2C_MODULE_ENABLED
 #include "stm32f4xx_hal_fmpi2c.h"
#endif /* HAL_FMPI2C_MODULE_ENABLED */

#ifdef HAL_FMPSMBUS_MODULE_ENABLED
 #include "stm32f4xx_hal_fmpsmbus.h"
#endif /* HAL_FMPSMBUS_MODULE_ENABLED */

#ifdef HAL_SPDIFRX_MODULE_ENABLED
 #include "stm32f4xx_hal_spdifrx.h"
#endif /* HAL_SPDIFRX_MODULE_ENABLED */

#ifdef HAL_DFSDM_MODULE_ENABLED
 #include "stm32f4xx_hal_dfsdm.h"
#endif /* HAL_DFSDM_MODULE_ENABLED */

#ifdef HAL_LPTIM_MODULE_ENABLED
 #include "stm32f4xx_hal_lptim.h"
#endif /* HAL_LPTIM_MODULE_ENABLED */

#ifdef HAL_MMC_MODULE_ENABLED
 #include "stm32f4xx_hal_mmc.h"
#endif /* HAL_MMC_MODULE_ENABLED */

// [解析] ============== ⑧ assert_param 的最终定义（第 ④ 节那个开关在这里生效） ==============
// [解析] HAL 源码里到处都是 assert_param(条件) 这样的检查，它到底干什么就看这里：
// [解析]   · 若定义了 USE_FULL_ASSERT（第 ④ 节的开关打开）：条件为假时调用
// [解析]     assert_failed(__FILE__, __LINE__)，把出错的文件名和行号报出来（函数要自己实现）。
// [解析]   · 若没定义（★ 本工程的情况）：展开成 ((void)0U)，也就是"什么都不做"，
// [解析]     编译器会把它整个优化掉，因此不影响速度和体积。这就是"关掉断言零开销"的原理。
/* Exported macro ------------------------------------------------------------*/
#ifdef  USE_FULL_ASSERT
/**
  * @brief  The assert_param macro is used for function's parameters check.
  * @param  expr If expr is false, it calls assert_failed function
  *         which reports the name of the source file and the source
  *         line number of the call that failed.
  *         If expr is true, it returns no value.
  * @retval None
  */
// [解析] 启用断言时的写法：条件成立 → (void)0U 什么都不做；条件不成立 → 调 assert_failed()
// [解析]   把 __FILE__（当前文件名）和 __LINE__（当前行号）传进去，方便直接定位问题。
  #define assert_param(expr) ((expr) ? (void)0U : assert_failed((uint8_t *)__FILE__, __LINE__))
/* Exported functions ------------------------------------------------------- */
  void assert_failed(uint8_t* file, uint32_t line);
#else
// [解析] ★ 本工程走的是这一支：不启用断言，于是 assert_param(...) 被替换成空操作，
// [解析]   编译器优化后连函数调用都不剩。副作用：HAL 完全不检查参数。
  #define assert_param(expr) ((void)0U)
#endif /* USE_FULL_ASSERT */

// [解析] 与文件开头的 #ifdef __cplusplus / extern "C" { 配对，结束"C 语言命名规则"的作用域。
#ifdef __cplusplus
}
#endif

// [解析] 与文件开头的 #ifndef __STM32F4xx_HAL_CONF_H 配对，结束"防重复包含"保护。
#endif /* __STM32F4xx_HAL_CONF_H */
