// [解析] ==========================================================================
// [解析] 【文件】Core/Src/freertos.c
// [解析] 【作用】★ 这是整个工程的"任务总调度台"：把 6 个任务（LED/IMU/CAN/DBUS/BUZZER/ADC）
// [解析]         创建出来，并给它们分配优先级和栈空间。
// [解析] 【谁生成】CubeMX 自动生成骨架，任务的名字/优先级/栈大小来自 .ioc 里的配置。
// [解析] 【谁调用】Core/Src/main.c 的 main() 里调用 MX_FREERTOS_Init()，
// [解析]           紧接着调用 osKernelStart() 把调度器跑起来。
// [解析] --------------------------------------------------------------------------
// [解析] ★★ 本文件最需要理解的一件事：__weak（弱符号）机制 ★★
// [解析]   注意文件下半部分那 6 个任务函数，前面都写着 __weak，函数体是空的死循环。
// [解析]   __weak 的含义是："这是个弱定义，如果程序里别处还有一个同名的强定义，
// [解析]   链接器就用那个强定义，把它忽略掉。"
// [解析]   而 applications/*.cpp 里写的正是同名同签名的强定义，例如
// [解析]   applications/led_task.cpp 里的 led_task()。
// [解析]   所以最终运行的是 applications/ 里的实现 —— 这个空壳只是"占位兜底"：
// [解析]   万一某个任务忘了写实现，程序也能编译通过，不会直接报链接错误。
// [解析]   这也解释了为什么修改任务逻辑要去 applications/ 目录，而不是改这个文件。
// [解析]   ⚠ 但要注意：任务的名字、优先级、栈大小是在【本文件】的 MX_FREERTOS_Init()
// [解析]     里决定的，改优先级得改这里（或在 CubeMX 里改）。
// [解析] ==========================================================================

/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
// [解析] 四个头文件的分工：FreeRTOS.h/task.h 是 FreeRTOS 内核原生接口，
// [解析] cmsis_os.h 是 CMSIS-RTOS 封装层（提供 osThreadDef/osDelay 这些），
// [解析] main.h 提供引脚宏定义。

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
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
// [解析] 下面 6 个变量保存"任务句柄"(thread id)。创建任务时由 osThreadCreate() 返回，
// [解析] 后续可以用它来操作该任务，比如挂起(osThreadTerminate)、改优先级、查栈使用情况。
// [解析] 命名规律：<任务名>Handle。
osThreadId LEDHandle;
osThreadId IMUHandle;
osThreadId CANHandle;
osThreadId DBUSHandle;
osThreadId BUZZERHandle;
osThreadId ADCHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

// [解析] 这 6 行是任务函数的"前置声明"。
// [解析] 之所以要声明：MX_FREERTOS_Init() 里用到了它们，但真正的定义在文件更下面，
// [解析] 而且最终会由 applications/*.cpp 里的同名函数覆盖。
void led_task(void const * argument);
void imu_task(void const * argument);
void can_task(void const * argument);
void uart_task(void const * argument);
void buzzer_task(void const * argument);
void adc_task(void const * argument);
// [解析] 注意：任务名和函数名不是一一对应的！比如任务名叫 DBUS，函数却叫 uart_task；
// [解析] 任务名叫 ADC，函数叫 adc_task 定义在 voltage_detect.cpp 里。
// [解析] 因为 .ioc 里配置的任务名和实际函数名可以不同，认准函数名即可。

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
// [解析] ★ 这段是"给空闲任务静态分配内存"。
// [解析]   背景：FreeRTOS 里必然存在一个优先级为 0 的"空闲任务"(Idle Task)，
// [解析]   它的作用是"当所有任务都在延时/等待时，让 CPU 有事可做（并回收被删除任务的内存）"。
// [解析]   本工程在 FreeRTOSConfig.h 里同时打开了"静态分配"和"动态分配"
// [解析]   （configSUPPORT_STATIC_ALLOCATION = 1）。
// [解析]   一旦打开静态分配，FreeRTOS 就要求你【必须】自己提供空闲任务的内存
// [解析]   —— 也就是下面这两个数组；否则编译时会报缺少
// [解析]   vApplicationGetIdleTaskMemory 的链接错误。
// [解析]   好处是：空闲任务的内存不再从 heap 里分配，是编译期就确定的静态数组，
// [解析]   不会因为堆碎片或堆不够而创建失败。
static StaticTask_t xIdleTaskTCBBuffer;
// [解析] ↑ 空闲任务的"任务控制块"(TCB)，存放任务状态、优先级等信息。
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];
// [解析] ↑ 空闲任务的栈。configMINIMAL_STACK_SIZE 在 FreeRTOSConfig.h 里是 128，
// [解析]   单位是"字"(StackType_t = 4 字节)，所以等于 512 字节。

// [解析] 这个函数由 FreeRTOS 内核自动调用（它在创建空闲任务时会来找你要内存）。
// [解析] 你只需要把上面那两个静态数组的地址"填进"输出的指针里就行。
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
// [解析] ★★ 创建所有任务的地方。任务在 main() 里被调用，但此时调度器还没启动，
// [解析]   所以这里只是"登记"，任务要等 osKernelStart() 之后才真正开始跑。
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  // [解析] ★ 下面每个任务都是"两行一组"，理解了这个套路，6 个任务就都通了：
  // [解析]
  // [解析]   第 1 行 osThreadDef(名字, 函数, 优先级, 实例数, 栈大小);
  // [解析]     —— 只是定义一个"任务模板"常量，占静态内存，还不会创建任务。
  // [解析]   第 2 行 osThreadCreate(osThread(名字), NULL);
  // [解析]     —— 按模板真正创建任务。第二个参数是传给任务函数的参数，
  // [解析]        这里都传 NULL（本工程的任务不需要参数）。
  // [解析]        返回的任务句柄存进 <名字>Handle。
  // [解析]
  // [解析]   ★ 参数细节（很容易搞错，务必看清）：
  // [解析]   · 栈大小单位是【字(word)，4 字节】，不是字节！
  // [解析]     所以 128 → 128×4 = 512 字节；256 → 1024 字节。
  // [解析]     如果任务里有大数组或浮点运算，栈要开大一点，否则会栈溢出把别的内存踩坏。
  // [解析]   · 实例数填 0，表示"默认即 1 个实例"。这个参数在本封装里其实没被用到，
  // [解析]     填 0 是 CubeMX 的固定写法。
  // [解析]   · 优先级的换算：CMSIS 的 osPriorityXxx 会先映射成 FreeRTOS 的优先级数字，
  // [解析]     规则是"数字 = CMSIS优先级 + 3"（本工程 cmsis_os.h 里 osPriorityIdle = -3）。
  // [解析]     于是：osPriorityRealtime→6, osPriorityHigh→5, osPriorityAboveNormal→4,
  // [解析]           osPriorityNormal→3, osPriorityBelowNormal→2, osPriorityLow→1。
  // [解析]     数字越大优先级越高；数字 0 被空闲任务占用。
  // [解析]     FreeRTOSConfig.h 里 configMAX_PRIORITIES = 16，所以这些值都在合法范围内。
  // [解析]   · 另外注意 configTICK_RATE_HZ = 1000，也就是 1 个 tick = 1ms，
  // [解析]     所以各任务里的 osDelay(1) 就是 1 毫秒。

  /* Create the thread(s) */
  /* definition and creation of LED */
  // [解析] LED 任务：优先级最低(→1)，栈 512 字节。只是点灯，不能抢占控制类任务。
  osThreadDef(LED, led_task, osPriorityLow, 0, 128);
  LEDHandle = osThreadCreate(osThread(LED), NULL);

  /* definition and creation of IMU */
  // [解析] IMU 任务：优先级次低(→2)，但栈给到 1024 字节。
  // [解析] 栈大是因为 Mahony 姿态解算里有大量浮点运算和中间变量。
  osThreadDef(IMU, imu_task, osPriorityBelowNormal, 0, 256);
  IMUHandle = osThreadCreate(osThread(IMU), NULL);

  /* definition and creation of CAN */
  // [解析] CAN 任务：优先级较高(→5)，栈 512 字节。
  // [解析] 给高优先级是为了保证电机控制指令能准时发出去，不能因为别的任务而延迟。
  osThreadDef(CAN, can_task, osPriorityHigh, 0, 128);
  CANHandle = osThreadCreate(osThread(CAN), NULL);

  /* definition and creation of DBUS */
  // [解析] ★ DBUS（遥控器接收）任务：优先级【最高】(→6)，栈 1024 字节。
  // [解析]   为什么它比电机任务还高？因为遥控器一旦失联超过约 100ms 就必须触发保护动作
  // [解析]   （急停/失能），所以收包必须最及时、绝不能被别的任务拖延。
  // [解析]   这也是安全设计上的常见取舍：通信链路 > 控制输出 > 感知 > 显示。
  osThreadDef(DBUS, uart_task, osPriorityRealtime, 0, 256);
  DBUSHandle = osThreadCreate(osThread(DBUS), NULL);

  /* definition and creation of BUZZER */
  // [解析] 蜂鸣器任务：优先级中上(→4)，栈 512 字节。
  // [解析] 它只在开机时响三声，之后就一直 osDelay 挂着了，所以优先级高低影响不大。
  osThreadDef(BUZZER, buzzer_task, osPriorityAboveNormal, 0, 128);
  BUZZERHandle = osThreadCreate(osThread(BUZZER), NULL);

  /* definition and creation of ADC */
  // [解析] ADC（电池电压检测）任务：和蜂鸣器同级(→4)，栈 512 字节。
  // [解析] 注意这里任务名写的是 ADC，但函数是 adc_task，
  // [解析] 而 adc_task 的真实实现位于 applications/voltage_detect.cpp。
  osThreadDef(ADC, adc_task, osPriorityAboveNormal, 0, 128);
  ADCHandle = osThreadCreate(osThread(ADC), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

// [解析] ==========================================================================
// [解析] ★★ 以下是 6 个任务函数的"弱定义空壳"，方便对照，只看第一个就够 ★★
// [解析]   共同套路：
// [解析]     1) 前面有 __weak 关键字 —— 表示"别处若有强定义就覆盖我"；
// [解析]     2) 函数体是一个空的 for(;;) 死循环，里面只 osDelay(1)；
// [解析]     3) 真实逻辑在 applications/ 目录下的同名文件里。
// [解析]   ★ 为什么必须是死循环而不能直接 return？
// [解析]     因为 FreeRTOS 的任务函数一旦返回，就会走到 taskEXIT 相关处理，
// [解析]     若没有配置删除/清理机制会触发断言或未定义行为。
// [解析]     所以规范写法就是"任务永不返回"，用 while(1)/for(;;) 包住。
// [解析]   ★ 这些空壳对应关系（左=本文件函数名，右=真实实现文件）：
// [解析]     led_task    -> applications/led_task.cpp
// [解析]     imu_task    -> applications/imu_task.cpp
// [解析]     can_task    -> applications/can_task.cpp
// [解析]     uart_task   -> applications/uart_task.cpp      （对应 DBUS 任务）
// [解析]     buzzer_task -> applications/buzzer_task.cpp
// [解析]     adc_task    -> applications/voltage_detect.cpp （对应 ADC 任务）
// [解析] ==========================================================================

/* USER CODE BEGIN Header_led_task */
/**
* @brief Function implementing the LED thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_led_task */
__weak void led_task(void const * argument)
{
  /* USER CODE BEGIN led_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END led_task */
}

/* USER CODE BEGIN Header_imu_task */
/**
* @brief Function implementing the IMU thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_imu_task */
__weak void imu_task(void const * argument)
{
  /* USER CODE BEGIN imu_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END imu_task */
}

/* USER CODE BEGIN Header_can_task */
/**
* @brief Function implementing the CAN thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_can_task */
__weak void can_task(void const * argument)
{
  /* USER CODE BEGIN can_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END can_task */
}

/* USER CODE BEGIN Header_uart_task */
/**
* @brief Function implementing the DBUS thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_uart_task */
__weak void uart_task(void const * argument)
{
  /* USER CODE BEGIN uart_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END uart_task */
}

/* USER CODE BEGIN Header_buzzer_task */
/**
* @brief Function implementing the BUZZER thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_buzzer_task */
__weak void buzzer_task(void const * argument)
{
  /* USER CODE BEGIN buzzer_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END buzzer_task */
}

/* USER CODE BEGIN Header_adc_task */
/**
* @brief Function implementing the ADC thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_adc_task */
__weak void adc_task(void const * argument)
{
  /* USER CODE BEGIN adc_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END adc_task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
