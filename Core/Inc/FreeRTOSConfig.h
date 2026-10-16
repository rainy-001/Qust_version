// [解析] ==========================================================================
// [解析] 【文件】Core/Inc/FreeRTOSConfig.h
// [解析] 【作用】FreeRTOS 操作系统内核的"总配置单"。本文件里没有任何逻辑代码，全部是 #define，
// [解析]         内核源码（tasks.c / queue.c / port.c ...）在编译时会 #include 本文件，
// [解析]         于是这些宏就变成内核里的编译开关：任务数上限、时钟节拍、堆大小、
// [解析]         中断优先级门槛、哪些 API 参与编译……全都由这里决定。
// [解析] 【类比】可以把它当成"操作系统的设置界面"，只不过是用宏来设置的。
// [解析] 【谁生成】STM32CubeMX 自动生成，内容来自 .ioc 里 Middleware → FREERTOS 的各项设置。
// [解析]           作者只动过 /* USER CODE BEGIN xxx */ 保护区里的内容（比如 configASSERT）。
// [解析] 【关键点·新手必读】
// [解析]   1) 本文件里几乎所有"时间"的单位都是节拍(tick)。本工程 configTICK_RATE_HZ = 1000，
// [解析]      所以 1 tick = 1 ms，代码里的 osDelay(5) 就是延时 5 毫秒。
// [解析]   2) 优先级数字越大 = 优先级越高，最大只能用 configMAX_PRIORITIES - 1 = 15。
// [解析]   3) configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY = 5 是最容易踩坑的一条：
// [解析]      优先级数值比 5 更小（即更紧急，0~4）的中断里禁止调用任何 FreeRTOS API，
// [解析]      否则内核数据会被破坏，现象是"偶发死机 / 程序跑飞"，极难排查。
// [解析]   4) 本文件是"编译期配置"，改完必须重新编译整个工程才会生效。
// [解析] ==========================================================================
/* USER CODE BEGIN Header */
/*
 * FreeRTOS Kernel V10.3.1
 * Portion Copyright (C) 2017 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 * Portion Copyright (C) 2019 StMicroelectronics, Inc.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * http://www.FreeRTOS.org
 * http://aws.amazon.com/freertos
 *
 * 1 tab == 4 spaces!
 */
// [解析] ↑ 以上一大段是 FreeRTOS 内核自带的 MIT 开源许可证声明，属于"原样保留"的版权文本，
// [解析]   不需要看懂，也不要改它（删改许可证是违反开源协议的行为）。
// [解析] 下面这行就是 Header 保护区的结束标记。
/* USER CODE END Header */

#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

/*-----------------------------------------------------------
 * Application specific definitions.
 *
 * These definitions should be adjusted for your particular hardware and
 * application requirements.
 *
 * These parameters and more are described within the 'configuration' section of the
 * FreeRTOS API documentation available on the FreeRTOS.org web site.
 *
 * See http://www.freertos.org/a00110.html
 *----------------------------------------------------------*/

// [解析] ★★ 这就是 CubeMX 的"用户代码保护区"，本文件里它出现了好几次，规则都一样 ★★
// [解析]   在 CubeMX 里点 GENERATE CODE 重新生成代码时：
// [解析]     · 夹在 /* USER CODE BEGIN xxx */ 和 /* USER CODE END xxx */ 之间的内容会被【保留】；
// [解析]     · 标记【外面】的一切都会被按 .ioc 的配置重新生成，你写的东西直接消失。
// [解析]   所以自己写的代码一定要写在保护区【里面】！
// [解析]   注意 BEGIN 和 END 后面的名字必须一一对应（这里两边都是 Includes），不能只写一个。
/* USER CODE BEGIN Includes */
/* Section where include file can be added */
// [解析] ↑ 原注释意思：这里可以添加自己要 include 的头文件（比如想在本文件里引用某个宏）。
// [解析]   本工程这里是空的，因为 FreeRTOSConfig.h 不需要依赖别的头文件。
/* USER CODE END Includes */

// [解析] 下面这个 #if 在判断"当前是哪种编译器"：__ICCARM__（IAR）、__CC_ARM（Keil AC5）、
// [解析]   __GNUC__（GCC）。本工程用 CMake + arm-none-eabi-gcc，所以只有第三个条件成立，
// [解析]   但三种工具链的宏都写上是标准做法，方便同一份代码换编译器时不用改。
// [解析] 原注释说的"只给编译器用、不给汇编器用"：因为 .s 汇编文件也可能间接包含本头文件，
// [解析]   而汇编器不认识 #include <stdint.h> 和 extern 这种 C 语法，必须用宏挡在外面。
/* Ensure definitions are only used by the compiler, and not by the assembler. */
#if defined(__ICCARM__) || defined(__CC_ARM) || defined(__GNUC__)
  #include <stdint.h>
  extern uint32_t SystemCoreClock;
// [解析] ↑ 这是"声明"而不是"定义"：只是告诉编译器"有这么个变量、类型是 uint32_t"，
// [解析]   真正的存储空间在 system_stm32f4xx.c 里，开机时 SystemInit() 会把它更新成实际主频
// [解析]   （本工程为 168000000，即 168MHz）。下面 configCPU_CLOCK_HZ 就是取它的值。
#endif
// [解析] FPU = 浮点运算单元，MPU = 内存保护单元。这两个宏只对 ARMv8-M 内核
// [解析]   （Cortex-M33 之类）的移植层生效；本工程用的是 portable/GCC/ARM_CM4F 移植层
// [解析]   （见 cmake/stm32cubemx/CMakeLists.txt 里的 FreeRTOS_Src），它根本不看这两个宏。
// [解析]   所以填 0 完全不影响 float/double 运算 —— 本工程的 Mahony 姿态解算大量用浮点，
// [解析]   它靠的是 Cortex-M4F 自带的硬件 FPU 和编译器开关（-mfpu=fpv4-sp-d16），与这里无关。
#define configENABLE_FPU                         0
#define configENABLE_MPU                         0

// [解析] ---------------------- 调度方式与内存（最核心的几条） ----------------------
// [解析] 抢占式 vs 协作式调度：
// [解析]   1 = 抢占式：只要出现了一个比当前任务优先级更高的"就绪"任务，
// [解析]       就立刻打断（抢占）当前任务，切换到高优先级任务去跑。
// [解析]   0 = 协作式：只能等当前任务主动让出 CPU（例如调用 osDelay、等信号量）才能切换。
// [解析] 机器人工程必须用 1：否则高优先级任务被低优先级任务卡住，
// [解析]   遥控急停、电机指令就来不及发出，车会失控。
#define configUSE_PREEMPTION                     1
// [解析] 允许"静态分配"内存：任务/队列的内存可以由用户提供的静态数组给出，而不从堆里要。
// [解析] ★ 打开它有一个强制要求：必须自己实现 vApplicationGetIdleTaskMemory()，
// [解析]   否则链接时会报错。本工程确实实现了，在 Core/Src/freertos.c 里
// [解析]   （用两个 static 数组 xIdleTaskTCBBuffer / xIdleStack 给空闲任务用内存）。
#define configSUPPORT_STATIC_ALLOCATION          1
// [解析] 允许"动态分配"内存：任务/队列的内存由内核从堆里申请（本工程主要是这种方式）。
// [解析] 两个都打开是 CubeMX 的默认做法：想省堆内存的地方用静态，图方便的地方用动态。
// [解析] 堆的实现是 portable/MemMang/heap_4.c，堆的总大小见下面的 configTOTAL_HEAP_SIZE。
#define configSUPPORT_DYNAMIC_ALLOCATION         1
// [解析] 两个"钩子(hook)函数"开关，本工程都关成 0：
// [解析]   IDLE_HOOK = 1 时，空闲任务每跑一轮都会调用你写的 vApplicationIdleHook()，
// [解析]     常用来做"低功耗休眠"或"统计 CPU 空闲率"；关掉就少一次函数调用，省点开销。
// [解析]   TICK_HOOK = 1 时，每个节拍中断都会调用 vApplicationTickHook()，
// [解析]     ★ 注意它是在【中断】里执行的，绝不能做耗时操作，否则会拖慢整个系统。
// [解析] 本工程不需要这两种功能，所以保持 CubeMX 默认的 0（也避免忘记实现空钩子函数导致编译报错）。
#define configUSE_IDLE_HOOK                      0
#define configUSE_TICK_HOOK                      0
// [解析] CPU 主频（Hz），内核用它把"节拍数"换算成真实时间、并在启动时校准 SysTick。
// [解析] 这里直接引用 SystemCoreClock 变量（168000000 = 168MHz），好处是主频改了不用手工同步；
// [解析] 代价是它必须在 SystemClock_Config() 之后才准确，所以内核要在 main() 里最后才启动。
#define configCPU_CLOCK_HZ                       ( SystemCoreClock )
// [解析] ★ 系统节拍频率，单位 Hz。1000 表示"每秒 1000 个节拍"，即 1 tick = 1 ms。
// [解析]   这是整个工程最常用的换算基准：osDelay(1) = 1ms，osDelay(500) = 0.5s。
// [解析]   不要为了"更精细"随便改成 10000：节拍中断太频繁会明显吃掉 CPU 时间。
#define configTICK_RATE_HZ                       ((TickType_t)1000)
// [解析] 优先级"档位"总数，可用优先级是 0 ~ configMAX_PRIORITIES-1，即 0~15 共 16 档。
// [解析]   其中 0 档固定留给空闲任务(Idle)，用户任务实际用 1~15。
// [解析]   本工程 6 个任务的优先级见 Core/Src/freertos.c 的 MX_FREERTOS_Init()。
#define configMAX_PRIORITIES                     ( 16 )
// [解析] 最小栈大小，单位是【字(word)】而不是字节！1 字 = 4 字节，所以 128 字 = 512 字节。
// [解析]   ★ 这是新手最容易算错的地方：看到 128 千万别以为只有 128 字节。
// [解析]   空闲任务的栈就用这个值（见 Core/Src/freertos.c 里的 xIdleStack 数组）。
#define configMINIMAL_STACK_SIZE                 ((uint16_t)128)
// [解析] FreeRTOS 堆的总大小，单位是【字节】。15360 = 15KB。
// [解析]   本工程所有 osThreadCreate 创建的任务（栈 + 任务控制块 TCB）都从这个堆里分配，
// [解析]   堆的实现是 heap_4.c（带碎片合并）。如果创建任务失败、或运行时莫名跑飞，
// [解析]   优先怀疑这里：把 6 个任务的栈加起来（256+128+128+256+128+128 字 ≈ 4KB）
// [解析]   再加上 TCB 和队列，15KB 是够用的，但新增任务时要重新核算。
#define configTOTAL_HEAP_SIZE                    ((size_t)15360)
// [解析] 任务名的最大长度（含结尾的 '\0'）。16 个字节能放下 "IMU"、"BUZZER" 这类短名字。
// [解析]   只在打印/调试任务列表时用得到，不影响运行。
#define configMAX_TASK_NAME_LEN                  ( 16 )
// [解析] 0 = TickType_t 用 32 位。本工程必须用 0：
// [解析]   16 位节拍在 1ms 节拍下 65 秒就溢出回绕，而 32 位可以撑约 49.7 天，足够比赛时长。
#define configUSE_16_BIT_TICKS                   0
// [解析] 1 = 启用互斥量(Mutex)相关功能，允许用 osMutexCreate 等接口。
// [解析] 互斥量用来保护"多个任务都要用的同一份数据"（比如传感器结构体），
// [解析]   它自带的"优先级继承"能防止优先级反转导致的卡死。本工程允许使用，
// [解析]   虽然当前代码主要靠"一个数据一个任务负责"来避免竞争。
#define configUSE_MUTEXES                        1
// [解析] 队列注册表的容量：最多允许把 8 个队列/信号量"登记名字"，方便调试时打印查看。
// [解析] 只是调试辅助功能，填 0 也能正常工作（本工程实际没登记任何队列）。
#define configQUEUE_REGISTRY_SIZE                8
// [解析] 1 = 使用"移植层优化过的任务选择算法"：用 Cortex-M 的 CLZ 指令（数前导零）
// [解析]   直接从优先级位图里算出最高优先级任务，比普通的逐级查找更快（调度更省时间）。
// [解析] ★ 它的前提是 configMAX_PRIORITIES ≤ 32，本工程 16，满足条件，所以可以开。
#define configUSE_PORT_OPTIMISED_TASK_SELECTION  1
// [解析] 又一处保护区（名字叫 MESSAGE_BUFFER_LENGTH_TYPE）。看下面这行 #define：
// [解析]   它是"消息缓冲区(Stream Buffer / Message Buffer)"里表示长度的类型，
// [解析]   填 size_t 是与旧版本兼容的默认值；若确定长度永远用不了几个字节，也可以改成 uint16_t
// [解析]   来省内存。本工程没用消息缓冲区，保持默认即可。
/* USER CODE BEGIN MESSAGE_BUFFER_LENGTH_TYPE */
/* Defaults to size_t for backward compatibility, but can be changed
   if lengths will always be less than the number of bytes in a size_t. */
#define configMESSAGE_BUFFER_LENGTH_TYPE         size_t
/* USER CODE END MESSAGE_BUFFER_LENGTH_TYPE */

// [解析] 协程(Co-routine)是 FreeRTOS 里一种很古老的"轻量任务"机制，
// [解析]   需要给每个协程手工分配栈、写起来麻烦，现在基本被普通任务取代了。
// [解析] 0 = 不编译协程相关代码（省 Flash 和内存）；2 是协程优先级档位数，只在开启时才有意义。
/* Co-routine definitions. */
#define configUSE_CO_ROUTINES                    0
#define configMAX_CO_ROUTINE_PRIORITIES          ( 2 )

// [解析] --------------- API 裁剪开关（决定哪些函数会被编进程序） ---------------
// [解析] 每个 INCLUDE_xxx = 1 表示"把这个 API 编进程序"，= 0 表示不编（省 Flash 空间）。
// [解析] ★ 新手最常见的报错之一：某函数"未定义/undefined reference"，原因就是这里忘了开。
// [解析] 需要新功能时，先到这个列表里确认对应的 INCLUDE_ 是不是 1。
/* Set the following definitions to 1 to include the API function, or zero
to exclude the API function. */
#define INCLUDE_vTaskPrioritySet             1
#define INCLUDE_uxTaskPriorityGet            1
#define INCLUDE_vTaskDelete                  1
#define INCLUDE_vTaskCleanUpResources        0
#define INCLUDE_vTaskSuspend                 1
// [解析] ↓ 注意这一项是 0（关闭），而其他项大多是 1。
// [解析]   vTaskDelayUntil 是"绝对延时"（保证固定周期），CMSIS 封装里的 osDelayUntil() 依赖它。
// [解析]   所以本工程【不能】用 osDelayUntil()：虽然编译能过，但运行时会直接返回错误码，
// [解析]   周期性的活只能靠 osDelay()（相对延时）来写，例如 applications 里各任务的 while + osDelay。
// [解析]   如果想改成固定周期节拍，需要把这里改成 1（属于改配置，请先和队长确认）。
#define INCLUDE_vTaskDelayUntil              0
#define INCLUDE_vTaskDelay                   1
#define INCLUDE_xTaskGetSchedulerState       1

// [解析] --------------- Cortex-M 中断优先级相关（本文件最容易看晕的一段） ---------------
// [解析] 背景：STM32F407 的 NVIC 用 8 位寄存器存放一个中断的优先级，但芯片只实现了高 4 位，
// [解析]   所以优先级只有 16 档（0~15），而且 0 是最高优先级，15 是最低优先级 ——
// [解析]   ★ 和 FreeRTOS 任务的优先级规则正好相反（任务数字越大越优先），别记混。
// [解析] configPRIO_BITS 表示"实际实现了几个优先级位"。CMSIS 头文件里定义了 __NVIC_PRIO_BITS = 4，
// [解析]   于是走上面的分支取到 4；下面 #else 里的 4 是为了"没包含 CMSIS 时"兜底。
/* Cortex-M specific definitions. */
#ifdef __NVIC_PRIO_BITS
 /* __BVIC_PRIO_BITS will be specified when CMSIS is being used. */
 #define configPRIO_BITS         __NVIC_PRIO_BITS
#else
 #define configPRIO_BITS         4
#endif

// [解析] 允许用户设置的最低中断优先级 = 15（也就是最"不急"的那一档）。
// [解析] 内核自己的中断（PendSV/SysTick，用于任务切换和节拍）就放在这一档，
// [解析]   好处是：任何外设中断都能打断内核中断，任务切换不会延误外设响应。
// [解析] 原注释说的是"可以传给 set priority 函数的最低优先级"。
/* The lowest interrupt priority that can be used in a call to a "set priority"
function. */
#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY   15

// [解析] ★★ 全文件最关键的一条，务必理解 ★★
// [解析] 这条线把中断分成两类（数值小的优先级高）：
// [解析]   · 优先级 0 ~ 4：比这条线"更紧急"，【禁止】调用任何 FreeRTOS API（哪怕是 xxxFromISR 版本），
// [解析]     因为内核在操作链表时会短暂关中断，这些高优先级中断会打断它，导致内核数据被破坏；
// [解析]     现象是"偶发死机/跑飞"，且极难复现和排查。
// [解析]   · 优先级 5 ~ 15：允许调用 xxxFromISR 结尾的 API（如 xQueueSendFromISR）。
// [解析] 本工程的实际取值（见 Core/Src/*.c 里的 HAL_NVIC_SetPriority）：
// [解析]   CAN1_RX0 / USART3 / DMA1_Stream1 / EXTI4 / EXTI9_5 都是 5
// [解析]     => 恰好在线内，可以在这些中断里调用 FreeRTOS 的 FromISR 接口；
// [解析]   HAL 时基 TIM1 的中断优先级是 15（来自 stm32f4xx_hal_conf.h 的 TICK_INT_PRIORITY）。
// [解析] 原注释的警告"绝不能设为 0"：填 0 会让整条线失去意义，等于允许所有中断调用内核 API。
/* The highest interrupt priority that can be used by any interrupt service
routine that makes calls to interrupt safe FreeRTOS API functions.  DO NOT CALL
INTERRUPT SAFE FREERTOS API FUNCTIONS FROM ANY INTERRUPT THAT HAS A HIGHER
PRIORITY THAN THIS! (higher priorities are lower numeric values. */
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY 5

// [解析] 下面两行就是把上面的"0~15 档"翻译成 NVIC 寄存器的真实数值。
// [解析] 换算原因：NVIC 寄存器是 8 位，而芯片只实现了高 4 位，所以 4 位优先级要左移
// [解析]   (8 - 4) = 4 位，放到字节的高半部分。于是：
// [解析]     configKERNEL_INTERRUPT_PRIORITY      = 15 << 4 = 0xF0（内核中断，最低档）
// [解析]     configMAX_SYSCALL_INTERRUPT_PRIORITY =  5 << 4 = 0x50（那条"分界线"）
// [解析] 综合上面两句可以推出：本工程外设中断抢占优先级必须 ≥ 5 才能安全调用 FreeRTOS API。
// [解析] ↑↓ 两句原注释分别说明：这些优先级是内核移植层自己用的；以及那条线绝不能是 0。
/* Interrupt priorities used by the kernel port layer itself.  These are generic
to all Cortex-M ports, and do not rely on any particular library functions. */
#define configKERNEL_INTERRUPT_PRIORITY 		( configLIBRARY_LOWEST_INTERRUPT_PRIORITY << (8 - configPRIO_BITS) )
/* !!!! configMAX_SYSCALL_INTERRUPT_PRIORITY must not be set to zero !!!!
See http://www.FreeRTOS.org/RTOS-Cortex-M3-M4.html. */
#define configMAX_SYSCALL_INTERRUPT_PRIORITY 	( configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY << (8 - configPRIO_BITS) )

/* Normal assert() semantics without relying on the provision of an assert.h
header file. */
// [解析] 又一处保护区（名字叫 1）。里面这一行就是作者改过的内容：
// [解析] 它是 FreeRTOS 的"断言"机制。内核在关键位置会调用 configASSERT(条件)，
// [解析]   条件成立什么都不做；条件不成立（参数非法、链表被破坏等）就执行这里写的动作。
// [解析] 本工程的写法是：关掉所有中断 + for(;;) 死循环，即"出错就停住"。
// [解析] ★ 调试技巧：程序莫名停住不动时，在 for(;;) 那行打个断点，
// [解析]   命中后看调用栈(Call Stack)就知道是哪个 API 传了非法参数。
// [解析] 注意：关中断是故意的 —— 出错后不再让中断打乱现场，方便排查，也避免车继续乱动。
/* USER CODE BEGIN 1 */
#define configASSERT( x ) if ((x) == 0) {taskDISABLE_INTERRUPTS(); for( ;; );}
// [解析] ↑ 逐段读：(x) == 0 判断断言失败 -> taskDISABLE_INTERRUPTS() 关全局中断（PRIMASK）
// [解析]   -> for( ;; ); 死循环卡死。之所以不用 assert.h 的标准断言，
// [解析]   是因为单片机里没有能打印错误信息的终端，"停下来让人来查"更实用。
/* USER CODE END 1 */

// [解析] ---------- 中断服务函数"改名"（把 FreeRTOS 的入口接到启动文件的向量表上） ----------
// [解析] 背景：中断入口的名字是固定的，由启动文件 startup_stm32f407xx.s 里的中断向量表决定，
// [解析]   比如 SVC_Handler / PendSV_Handler / SysTick_Handler 这些名字不能自己乱起。
// [解析] 而 FreeRTOS 移植层（port.c）里的函数叫 xPortPendSVHandler 之类。下面用 #define 在
// [解析]   编译期把后者的名字"替换"成前者，FreeRTOS 的函数就自动变成了中断服务函数，
// [解析]   不需要额外写一层胶水函数去调用它 —— 这是 FreeRTOS 移植的常用手法。
/* Definitions that map the FreeRTOS port interrupt handlers to their CMSIS
standard names. */
// [解析] SVC = Supervisor Call（超级用户调用，属于内核异常）。
// [解析]   FreeRTOS 第一次启动任务时（osKernelStart → vPortStartFirstTask）用它把 CPU
// [解析]   从 main() 的上下文"跳"进第一个任务，之后就不再用了。
#define vPortSVCHandler    SVC_Handler
// [解析] PendSV = Pended Service Call（可挂起的系统调用），★ 任务切换真正发生的地方 ★。
// [解析]   切换时机：SysTick 中断里发现"该换任务了"，就把 PendSV 挂起，等所有中断处理完再切。
// [解析]   它的优先级被特意设成最低，这样任务切换绝不会打断任何外设中断，保证了实时性。
#define xPortPendSVHandler PendSV_Handler

/* IMPORTANT: This define is commented when used with STM32Cube firmware, when the timebase source is SysTick,
              to prevent overwriting SysTick_Handler defined within STM32Cube HAL */

// [解析] ★ 这一行是理解"FreeRTOS 与 HAL 谁在用时基"的关键 ★
// [解析] 它把 FreeRTOS 的节拍处理函数 xPortSysTickHandler 改名为 SysTick_Handler，
// [解析]   于是 SysTick 中断就归 FreeRTOS 了：每 1ms 进来一次给内核节拍 +1，必要时触发任务切换。
// [解析] ↑ 上面那段原注释的意思是：如果 HAL 的时基源也是 SysTick，CubeMX 会把下面这一行
// [解析]   【注释掉】，否则就和 HAL 自己的 SysTick_Handler 重名冲突（两个同名函数无法链接）。
// [解析] 本工程恰好相反：.ioc 里把 HAL 时基改成了 TIM1
// [解析]   （因为 SysTick 已被 FreeRTOS 独占，实现见 Core/Src/stm32f4xx_hal_timebase_tim.c），
// [解析]   HAL 不再需要 SysTick_Handler，所以这一行被保留下来交给 FreeRTOS 用。
// [解析] 由此可以解释一个疑问：Core/Src/stm32f4xx_it.c 里为什么找不到 SysTick_Handler？
// [解析]   因为它就是这里的 FreeRTOS 函数（port.c 里的 xPortSysTickHandler，编译时被改名）。
#define xPortSysTickHandler SysTick_Handler

// [解析] 最后一处保护区（名字叫 Defines）。用途：在这里定义参数以【覆盖】FreeRTOS.h 里的默认值，
// [解析]   例如想改 configUSE_TIME_SLICING、configUSE_TRACE_FACILITY 等，写在这里比改上面更清晰。
// [解析] 本工程没有需要覆盖的项，所以是空的。
/* USER CODE BEGIN Defines */
/* Section where parameter definitions can be added (for instance, to override default ones in FreeRTOS.h) */
/* USER CODE END Defines */

// [解析] 最后这个 #endif 与文件开头的 #ifndef FREERTOS_CONFIG_H 配对，是"防止重复包含"保护：
// [解析]   同一个头文件被多个 .c 文件包含时，保证内容只展开一次，避免宏被重复定义。
#endif /* FREERTOS_CONFIG_H */
