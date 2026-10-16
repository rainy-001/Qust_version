// [解析] ==========================================================================
// [解析] 【文件】Core/Src/can.c
// [解析] 【作用】CAN1 外设的初始化：配好波特率、收发引脚和中断，让 CAN1 能收发电机指令。
// [解析] 【谁生成】CubeMX 自动生成，作者没有改动逻辑（参数就是 CubeMX 界面上填的那些）。
// [解析] 【怎么被调用】main.c 里 MX_CAN1_Init() -> HAL_CAN_Init() -> HAL_CAN_MspInit()。
// [解析] 【关键点】
// [解析]   1) 波特率怎么来的：PCLK1 = 42 MHz，42 MHz ÷ 3 ÷ (1+12+1) = 1 Mbps（推导见下面）；
// [解析]   2) HAL_CAN_MspInit() 是 HAL 库自动回调的"底层初始化"，时钟/引脚/中断都在那里；
// [解析]   3) 本文件只负责"把硬件配好"；真正的收发逻辑在 sp_middleware/io/can/can.cpp。
// [解析] ==========================================================================

/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.c
  * @brief   This file provides code for the configuration
  *          of the CAN instances.
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
#include "can.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

// [解析] hcan1 是 CAN1 的"句柄"：HAL 库用它记录这个外设的全部信息（寄存器基址、配置、错误码…）。
// [解析] 它是全局变量，其他文件只要写 extern CAN_HandleTypeDef hcan1; 就能共用同一个外设。
// [解析] sp_middleware/io/can/can.cpp 里的 sp::CAN can1(&hcan1) 拿的就是它。
CAN_HandleTypeDef hcan1;

/* CAN1 init function */
// [解析] CAN1 初始化函数，做两件事：
// [解析]   ① HAL_CAN_Init() 内部先回调 HAL_CAN_MspInit()：开时钟、配 PD0/PD1 引脚、开中断；
// [解析]   ② 再把下面 hcan1.Init 里填好的参数写进 CAN1 的寄存器。
// [解析] 调用时机：main.c 的 MX_CAN1_Init()，在 FreeRTOS 启动之前完成。
void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  // [解析] 下面一大串是给"初始化结构体" hcan1.Init 赋值，也就是 CAN1 的工作参数。
  // [解析] 参数看着多，其实只有前 6 行决定通信本身（分频、模式、位时间），其余都是"高级功能开关"。
  // [解析] 每一项的含义见行末注释。
  hcan1.Instance = CAN1;                          // [解析] 指明管的是哪组寄存器：CAN1（另一路是 CAN2）
  hcan1.Init.Prescaler = 3;                       // [解析] 波特率预分频：CAN 时钟 42MHz ÷ 3 = 14MHz（一个 TQ 的时间基准）
  hcan1.Init.Mode = CAN_MODE_NORMAL;              // [解析] 正常工作模式（不是回环/静默等自测模式）
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;         // [解析] 重同步跳转宽度 SJW = 1 个 TQ：硬件靠它微调位时间、吸收累积误差
  hcan1.Init.TimeSeg1 = CAN_BS1_12TQ;             // [解析] 相位缓冲段 1（BS1）= 12 个 TQ
  hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;              // [解析] 相位缓冲段 2（BS2）= 1 个 TQ
  // [解析] ★★ 波特率 1 Mbps 是怎么算出来的（面试常问，务必会推）：
  // [解析]   一个 CAN"位"占的时间 = 同步段(固定 1 个 TQ) + BS1 + BS2 = 1 + 12 + 1 = 14 个 TQ；
  // [解析]   一个 TQ 的时钟 = PCLK1(APB1 = 42 MHz) ÷ Prescaler(3) = 14 MHz；
  // [解析]   所以波特率 = 14 MHz ÷ 14 = 1 000 000 bit/s = 1 Mbps，正好是电机/C 板要求的 1 Mbps。
  // [解析]   （注意 HAL 写寄存器时会自动减 1，所以这里填 3 就是除以 3，不用自己换算。）
  // [解析]   采样点 = (1 + BS1) / 14 = 13/14 ≈ 92.9%，落在 CAN 推荐的 87.5% 之后，
  // [解析]   采样越靠后抗干扰越好。改动这三个数必须一起算，只改一个就通信失败。
  hcan1.Init.TimeTriggeredMode = DISABLE;         // [解析] 时间触发通信(TTCAN)：关闭，普通 CAN 用不到
  hcan1.Init.AutoBusOff = DISABLE;                // [解析] 自动离线恢复：关闭。进入 Bus-Off 后不会自动恢复，要软件处理
  hcan1.Init.AutoWakeUp = DISABLE;                // [解析] 自动唤醒（被总线活动从睡眠中唤醒）：关闭，本工程不进睡眠
  hcan1.Init.AutoRetransmission = DISABLE;        // [解析] 自动重传：关闭。出错或仲裁失败时这一帧直接丢弃，不会自动重发
  hcan1.Init.ReceiveFifoLocked = DISABLE;         // [解析] 接收 FIFO 锁定：关闭。FIFO 满时新帧覆盖最旧帧（打开则是丢弃新帧）
  hcan1.Init.TransmitFifoPriority = DISABLE;      // [解析] 发送优先级：关闭 = 帧 ID 小的先发；打开 = 谁先进邮箱谁先发
  // [解析] 上面这些只是"填表"，真正生效靠下面这句。HAL_CAN_Init() 会在这里回调 MspInit()。
  // [解析] 返回值不是 HAL_OK 说明参数非法或硬件没就绪，直接进 Error_Handler()
  // [解析] （里面是关中断 + 死循环，程序会停在那里，方便用调试器看现场，不会带着错误继续跑）。
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */
  // [解析] ↑ 这是"用户代码保护区"（CubeMX 的 USER CODE 机制），务必理解：
  // [解析]   在 CubeMX 里点 GENERATE CODE 重新生成代码时，只有夹在 BEGIN/END 之间的内容会被保留，
  // [解析]   标记【之外】的一切都会被重新生成的代码覆盖掉。自己写的代码一定要写在保护区里面！
  // [解析]   本工程 CAN 的过滤器和启动没有放在这里，而是在 can_task 里通过 sp::CAN 类做的，
  // [解析]   因为必须等 HAL_CAN_Init() 跑完（外设退出复位）之后才能配过滤器，顺序反了不生效。

  /* USER CODE END CAN1_Init 2 */

}

// [解析] ★ MSP = MCU Support Package（MCU 支持包）。它不由你手动调用，
// [解析]   而是 HAL_CAN_Init() 在初始化过程中"回调"进来，专门干 HAL 库不方便干的底层杂活：
// [解析]   使能外设时钟、把 GPIO 配成复用功能、绑定 DMA、配置 NVIC 中断。
// [解析]   这样设计的好处：上层初始化代码可以跨芯片复用，换芯片只改这个 Msp 函数。
// [解析]   参数 canHandle 就是正在初始化的那个句柄（这里等于 &hcan1），所以能同时兼容多路 CAN。
void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{

  // [解析] = {0} 表示"全部清零"，避免结构体里残留随机值导致配置出错。
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  // [解析] 判断当前初始化的是哪一路 CAN：CAN1 和 CAN2 用的引脚不同，要分开配。
  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspInit 0 */

  /* USER CODE END CAN1_MspInit 0 */
    /* CAN1 clock enable */
    // [解析] 使能 CAN1 的外设时钟。STM32 的外设默认是"断电"的，不开时钟后面写它的寄存器全部无效。
    __HAL_RCC_CAN1_CLK_ENABLE();

    // [解析] CAN1 的引脚挂在 GPIOD 上，所以还要开 GPIOD 的时钟（GPIO 端口也是外设，同样要开时钟）。
    __HAL_RCC_GPIOD_CLK_ENABLE();
    /**CAN1 GPIO Configuration
    PD0     ------> CAN1_RX
    PD1     ------> CAN1_TX
    */
    // [解析] ↑ 这是 CubeMX 自动生成的引脚说明：PD0 收、PD1 发，与 docs/C板引脚配置说明.md 一致。
    // [解析] 下面几行把 PD0/PD1 配成"CAN 复用功能"，两个脚一起配，所以用 | 把引脚号拼起来。
    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;        // [解析] 同时选中 PD0 和 PD1
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;             // [解析] 复用功能(AF) + 推挽输出；复用就是"这个脚交给 CAN 用，不归 GPIO 管"
    GPIO_InitStruct.Pull = GPIO_NOPULL;                 // [解析] 不用内部上拉/下拉：板上已有 CAN 收发器，外部电路会把电平定好
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;  // [解析] 引脚翻转速度选最高档：CAN 是 1 Mbps，边沿必须够快
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;          // [解析] ★ 复用编号必须是 AF9，这是 F407 里 CAN1/CAN2 的固定编号（查数据手册复用表）
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);             // [解析] 上面的设置在这一句才真正写进 GPIOD 的寄存器

    /* CAN1 interrupt Init */
    // [解析] 配置并打开 CAN1 的 RX0 中断：接收 FIFO0 里进入新帧时触发中断。
    // [解析] 两个参数分别是 (抢占优先级, 子优先级)，数字越小优先级越高：
    // [解析]   本工程 NVIC 分组是 NVIC_PRIORITYGROUP_4（HAL_Init 里设置，4 位抢占 / 0 位子优先级），
    // [解析]   所以子优先级 0 实际上不起作用，真正决定先后顺序的是抢占优先级 5。
    // [解析]   抢占优先级 = 5 与 FreeRTOS 的 configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY(=5) 相同，
    // [解析]   含义是"这个中断里可以安全调用 FreeRTOS 的 API"（例如往队列里发消息）。
    // [解析]   数值比 5 更小的中断（优先级更高）是禁止调用 FreeRTOS API 的，否则系统会崩。
    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
    // [解析] 注意：这里只开了 RX0 这一路中断。can.cpp 里虽然也给 FIFO1 打开了中断通知，
    // [解析]   但 NVIC 里没有使能 CAN1_RX1_IRQn，所以实际上只有 FIFO0 会进中断。
  /* USER CODE BEGIN CAN1_MspInit 1 */

  /* USER CODE END CAN1_MspInit 1 */
  }
}

// [解析] MspDeInit 是 MspInit 的逆操作：关时钟、把引脚还原成普通 GPIO、关中断。
// [解析] 它由 HAL_CAN_DeInit() 回调。本工程运行期间不会调用它
// [解析] （一般在低功耗休眠、外设重启、或者关闭某路 CAN 时才用），先理解成"收拾现场"即可。
void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle)
{

  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspDeInit 0 */

  /* USER CODE END CAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    // [解析] 关掉 CAN1 的时钟。时钟一关，CAN1 的寄存器就不再响应，之前的配置也随之失效。
    __HAL_RCC_CAN1_CLK_DISABLE();

    /**CAN1 GPIO Configuration
    PD0     ------> CAN1_RX
    PD1     ------> CAN1_TX
    */
    // [解析] 把 PD0/PD1 从"CAN 复用功能"还原成复位后的默认状态（普通引脚），不再给 CAN 用。
    HAL_GPIO_DeInit(GPIOD, GPIO_PIN_0|GPIO_PIN_1);

    /* CAN1 interrupt Deinit */
    // [解析] 关掉 RX0 中断，避免外设都停了中断还往 CPU 里冲。
    HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
  /* USER CODE BEGIN CAN1_MspDeInit 1 */

  /* USER CODE END CAN1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
// [解析] 文件末尾的用户代码保护区，可以放自己写的辅助函数（比如 CAN 的中断回调）。
// [解析] 本工程没往这里写东西：CAN 的业务代码都在 applications/can_task.cpp 和
// [解析] sp_middleware/io/can/can.cpp 里，这样 CubeMX 重新生成代码也不会把它们删掉。
/* USER CODE END 1 */

