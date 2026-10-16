// [解析] ==========================================================================
// [解析] 【文件】Core/Src/usart.c
// [解析] 【作用】USART3 的初始化：波特率、收发引脚，以及"接收用的 DMA 通道"。
// [解析] 【谁生成】CubeMX 自动生成，作者没有改动逻辑。
// [解析] 【用在哪】C 板把 DBus 遥控器接收机接在 USART3 上：PB10=TX / PB11=RX，波特率 100000。
// [解析]           上层代码：sp_middleware/io/dbus/dbus.cpp -> HAL_UARTEx_ReceiveToIdle_DMA()。
// [解析] 【关键点】
// [解析]   1) 波特率是 100000，不是常见的 115200 —— 这是大疆 DBus 接收机的专用值，填错收不到数据；
// [解析]   2) 接收走 DMA1_Stream1 的循环模式，CPU 不用一个字节一个字节去读（见 HAL_UART_MspInit）；
// [解析]   3) 只配了"接收"方向的 DMA，发送仍是普通方式（遥控器只需要收）。
// [解析] ==========================================================================

/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
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
#include "usart.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

// [解析] USART3 的句柄，保存波特率等配置和运行状态；定义在这里，别的文件 extern 之后就能用。
UART_HandleTypeDef huart3;
// [解析] 给"USART3 接收"配套的 DMA 句柄。它和 huart3 是"两个对象配合干活"的关系，
// [解析] 中间靠 __HAL_LINKDMA() 把两者挂在一起（见下面 HAL_UART_MspInit 里的说明）。
DMA_HandleTypeDef hdma_usart3_rx;

/* USART3 init function */

// [解析] USART3 初始化函数。同样分两步：
// [解析]   ① HAL_UART_Init() 内部先回调 HAL_UART_MspInit()：开时钟、配 PB10/PB11、配 DMA、开中断；
// [解析]   ② 再把下面 huart3.Init 里的参数写进 USART3 的寄存器。
// [解析] 调用时机：main.c 的 MX_USART3_UART_Init()，且必须排在 MX_DMA_Init() 之后
// [解析] （DMA1 的时钟要先开好，否则后面配 DMA 是白配）。
void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  // [解析] 下面几行是串口的通信格式参数，也就是串口助手里常说的"8N1"：
  // [解析]   8 = 数据位 8 位，N = 无校验(No parity)，1 = 1 个停止位。
  // [解析] 收发双方必须完全一致，否则收到的全是乱码。
  huart3.Instance = USART3;                              // [解析] 使用 USART3（C 板 DBus 接口就接在 USART3 的 PB10/PB11 上）
  huart3.Init.BaudRate = 100000;                         // [解析] ★ 波特率 100000（10 万），不是常见的 115200！DBus 接收机专用值
  huart3.Init.WordLength = UART_WORDLENGTH_8B;           // [解析] 数据位 8 位
  huart3.Init.StopBits = UART_STOPBITS_1;                // [解析] 停止位 1 位
  huart3.Init.Parity = UART_PARITY_NONE;                 // [解析] 无校验位
  huart3.Init.Mode = UART_MODE_TX_RX;                    // [解析] 收发都使能（遥控器只用到收，发送留着调试/扩展用）
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;           // [解析] 不用 RTS/CTS 硬件流控：DBus 接口只有两根信号线
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;       // [解析] 16 倍过采样（标准做法，抗噪好；波特率寄存器也按 16 倍计算）
  // [解析] 顺手验算一下波特率误差：APB1 = 42 MHz，16 倍过采样时
  // [解析]   USARTDIV = 42 000 000 / (16 × 100 000) = 26.25，能精确表示（BRR = 0x1A4），
  // [解析]   所以实际波特率正好 100000，误差 0%，不会因为累积误差丢包。
  // [解析] 注意：DBus 是"一整包连着一个空闲间隔"发来的，必须用 IDLE/ReceiveToIdle 方式接收
  // [解析]   （见 sp_middleware/io/dbus/dbus.cpp），不能一个字节一个字节阻塞地读，否则会丢包。
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */
  // [解析] ↑ 用户代码保护区：CubeMX 重新 GENERATE CODE 时，只有夹在 BEGIN/END 之间的内容会保留，
  // [解析]   标记之外的一切都会被重新生成覆盖。要加自己的初始化代码就写在中间。

  /* USER CODE END USART3_Init 2 */

}

// [解析] ★ MSP = MCU Support Package（MCU 支持包）。不是给人手动调的，
// [解析]   而是 HAL_UART_Init() 自动回调进来，专门做"HAL 库不方便做的底层事"：
// [解析]   使能时钟、配置引脚复用、初始化并绑定 DMA、设置 NVIC 中断。
// [解析]   参数 uartHandle 就是正在初始化的句柄（这里等于 &huart3）。
void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(uartHandle->Instance==USART3)
  {
  /* USER CODE BEGIN USART3_MspInit 0 */

  /* USER CODE END USART3_MspInit 0 */
    /* USART3 clock enable */
    // [解析] 使能 USART3 外设时钟。不开时钟，后面写它的寄存器全部无效（这是很常见的坑）。
    __HAL_RCC_USART3_CLK_ENABLE();

    // [解析] 串口引脚挂在 GPIOB 上，GPIO 端口本身也是外设，也要开时钟。
    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**USART3 GPIO Configuration
    PB10     ------> USART3_TX
    PB11     ------> USART3_RX
    */
    // [解析] ↑ CubeMX 生成的引脚说明：PB10 发送、PB11 接收（与 docs/C板引脚配置说明.md 一致）。
    // [解析]   注意 TX/RX 不能接反：单片机的 TX 要接接收机的 RX。
    GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11;      // [解析] 两个脚一起配，用 | 把引脚号拼起来
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;             // [解析] 复用功能(AF) + 推挽输出：这个脚交给 USART3 控制，不归 GPIO 管
    GPIO_InitStruct.Pull = GPIO_NOPULL;                 // [解析] 不用内部上下拉：外部接收机电路会把空闲电平定好
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;  // [解析] 引脚翻转速度选最高档：以后提高波特率也够用
    GPIO_InitStruct.Alternate = GPIO_AF7_USART3;        // [解析] ★ 复用编号必须是 AF7，这是 F407 里 USART3 的固定编号（查数据手册复用表）
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);             // [解析] 以上设置在这一句才真正写进 GPIOB 的寄存器

    /* USART3 DMA Init */
    /* USART3_RX Init */
    // [解析] 下面这一段配置"接收用的 DMA 通道"：让数据自动从串口数据寄存器搬进内存，
    // [解析] CPU 只需在整包收完后处理一次，不必被每个字节打断。各项含义见行末注释。
    hdma_usart3_rx.Instance = DMA1_Stream1;                        // [解析] 用 DMA1 的 Stream1（F4 的 DMA 按 Stream 分组，共 8 个）
    hdma_usart3_rx.Init.Channel = DMA_CHANNEL_4;                   // [解析] ★ USART3_RX 在 DMA1 上固定走 Stream1 的"通道 4"，芯片硬件决定，选错就收不到数据
    hdma_usart3_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;          // [解析] 传输方向：外设 -> 内存，也就是"接收"
    hdma_usart3_rx.Init.PeriphInc = DMA_PINC_DISABLE;              // [解析] 外设地址不自增：永远读 USART3->DR 这一个数据寄存器
    hdma_usart3_rx.Init.MemInc = DMA_MINC_ENABLE;                  // [解析] 内存地址自增：每搬 1 字节指针 +1，才能逐个字节填满接收缓冲区
    hdma_usart3_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE; // [解析] 外设侧按"字节"对齐（串口一次给出 1 字节）
    hdma_usart3_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;    // [解析] 内存侧也按"字节"对齐：两边一致才不会数据错位
    hdma_usart3_rx.Init.Mode = DMA_CIRCULAR;                       // [解析] ★ 循环模式：缓冲区搬满一圈自动回到开头，不需要软件反复重启 DMA
    hdma_usart3_rx.Init.Priority = DMA_PRIORITY_HIGH;              // [解析] 多个 Stream 同时抢总线时的仲裁优先级；本工程只有一路在用，填高一点无妨
    hdma_usart3_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;           // [解析] 不使用 DMA 的 4 级 FIFO（直接模式）：来一个字节立刻搬走，延迟最低
    if (HAL_DMA_Init(&hdma_usart3_rx) != HAL_OK)
    {
      Error_Handler();
    }

    // [解析] ★ 把 DMA 句柄"挂"到串口句柄上（宏展开后是 uartHandle->hdmarx = &hdma_usart3_rx 加状态标记）。
    // [解析] 之后调用 HAL_UARTEx_ReceiveToIdle_DMA(&huart3, buf, len) 时，HAL 才能从 huart3.hdmarx
    // [解析] 里找到该用哪路 DMA；少了这一句，接收函数会直接返回错误，遥控器就没数据了。
    __HAL_LINKDMA(uartHandle,hdmarx,hdma_usart3_rx);

    /* USART3 interrupt Init */
    // [解析] 打开 USART3 自身的中断。有了 DMA 为什么还要串口中断？因为
    // [解析]   HAL_UARTEx_ReceiveToIdle_DMA() 是靠串口的"空闲(IDLE)中断"判断一帧结束的：
    // [解析]   DMA 只管搬字节，帧尾（总线安静超过一个字节时间）由串口硬件检测并触发中断，
    // [解析]   两者配合才能交出一包完整数据，所以这两个中断缺一不可。
    // [解析] 参数 (抢占优先级 5, 子优先级 0)：本工程 NVIC 分组为 NVIC_PRIORITYGROUP_4，
    // [解析]   子优先级不起作用；抢占优先级 5 正好等于 FreeRTOS 的
    // [解析]   configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY，因此允许在这个中断里调用 FreeRTOS API。
    HAL_NVIC_SetPriority(USART3_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USART3_IRQn);
    // [解析] 注意：DMA1_Stream1 自己的 NVIC 中断是在 Core/Src/dma.c 的 MX_DMA_Init() 里使能的，
    // [解析]   优先级同样是 5；main.c 里 MX_DMA_Init() 排在 MX_USART3_UART_Init() 之前。
  /* USER CODE BEGIN USART3_MspInit 1 */

  /* USER CODE END USART3_MspInit 1 */
  }
}

// [解析] MspDeInit 是 MspInit 的逆操作，由 HAL_UART_DeInit() 回调：
// [解析] 关外设时钟、把引脚还原成普通 GPIO、释放 DMA、关中断。
// [解析] 本工程运行期间不会调用它（一般用于低功耗休眠或外设重启），先理解成"收拾现场"即可。
void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==USART3)
  {
  /* USER CODE BEGIN USART3_MspDeInit 0 */

  /* USER CODE END USART3_MspDeInit 0 */
    /* Peripheral clock disable */
    // [解析] 关掉 USART3 时钟，串口就停止工作了。
    __HAL_RCC_USART3_CLK_DISABLE();

    /**USART3 GPIO Configuration
    PB10     ------> USART3_TX
    PB11     ------> USART3_RX
    */
    // [解析] 把 PB10/PB11 还原成复位后的默认状态，不再当串口用。
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_10|GPIO_PIN_11);

    /* USART3 DMA DeInit */
    // [解析] 释放接收 DMA。这里传的是 uartHandle->hdmarx，正是上面 __HAL_LINKDMA 挂上去的那个句柄，
    // [解析]   所以能"顺着串口找到它的 DMA"。
    HAL_DMA_DeInit(uartHandle->hdmarx);

    /* USART3 interrupt Deinit */
    // [解析] 关掉串口中断，避免外设停了中断还在往 CPU 里冲。
    HAL_NVIC_DisableIRQ(USART3_IRQn);
  /* USER CODE BEGIN USART3_MspDeInit 1 */

  /* USER CODE END USART3_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
// [解析] 文件末尾的用户代码保护区，可以放自己写的函数（比如串口空闲中断的回调）。
// [解析] 本工程把 DBus 的业务代码放在 sp_middleware/io/dbus/ 和 applications/uart_task.cpp 里，
// [解析]   所以这里保持空白 —— 这样 CubeMX 重新生成代码时不会动到我们的代码。
/* USER CODE END 1 */

