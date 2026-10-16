// [解析] ==========================================================================
// [解析] 【文件】Core/Inc/can.h
// [解析] 【作用】CAN1 外设的"对外声明"头文件，里面只有两样东西：
// [解析]         1) 全局句柄 extern CAN_HandleTypeDef hcan1;（变量本体在 can.c）
// [解析]         2) 初始化函数原型 void MX_CAN1_Init(void);（函数本体也在 can.c）
// [解析] 【谁生成】CubeMX 自动生成。作者没有手改过本文件（保护区全是空的），
// [解析]           作者写的 CAN 相关代码在 sp_middleware/io/can/can.cpp 和
// [解析]           applications/can_task.cpp 里。
// [解析] 【本工程用途】CAN1 = PD0(RX) / PD1(TX)，波特率 1 Mbps，接四个底盘电机 + 超电等。
// [解析]               CAN 是 RoboMaster 车上电机、超级电容、裁判系统互连的总线，
// [解析]               一帧最多 8 字节数据，靠帧 ID 区分"这条命令发给谁/是谁的回包"。
// [解析] 【阅读顺序】本文件 -> Core/Src/can.c（看波特率怎么算出来）-> sp_middleware/io/can/can.cpp
// [解析]               （看上层怎么收发）-> applications/can_task.cpp（看实际怎么用）。
// [解析] ==========================================================================

/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.h
  * @brief   This file contains all the function prototypes for
  *          the can.c file
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
// [解析] ↑ 上面这一大段是 CubeMX 生成的固定版权注释（@file/@brief 是 Doxygen 文档标记），
// [解析]   对编译没影响。下面才是真正的 C 代码。
// [解析]   ▼ 下面是必须先搞清楚的机制：
// [解析] 【CubeMX 用户代码保护区 · 通用规矩，本文件同样适用】
// [解析]   凡是成对出现的
// [解析]       /* USER CODE BEGIN xxx */   ...   /* USER CODE END xxx */
// [解析]   夹在中间的内容就叫"用户代码保护区"。在 CubeMX 里点 GENERATE CODE 重新生成代码时：
// [解析]     · 保护区【之内】的内容原样保留，你写的代码不会丢；
// [解析]     · 保护区【之外】的内容全部按 .ioc 配置重新生成，手改的部分会被覆盖。
// [解析]   所以自己写的代码务必写在 BEGIN/END 之间，写在外面等于白写。
// [解析]   本文件的保护区目前都是空的：外设配置全部由 CubeMX 说了算。
/* Define to prevent recursive inclusion -------------------------------------*/
// [解析] ↓ 条件编译"包含卫士"(include guard)：防止同一头文件被重复 #include 造成重复定义。
// [解析]   第一次包含时 __CAN_H__ 未定义 -> 内容被编译；之后再有包含就整段跳过。
#ifndef __CAN_H__
#define __CAN_H__

// [解析] ↓ extern "C"：本工程上层是 C++（applications/ 下是 .cpp），这里是 C 代码。
// [解析]   不加这个包裹，C++ 会按自己的"改名"规则去找函数名，链接时找不到 MX_CAN1_Init。
// [解析]   配对的 #endif 在文件末尾。
#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
// [解析] main.h 里 include 了 stm32f4xx_hal.h（HAL 库总头），所以下面能直接写
// [解析] CAN_HandleTypeDef 这个类型名。所有外设头文件都这样从 main.h 取 HAL 声明。
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */
// [解析] ↑ 上面三对 BEGIN/END（Includes / Private defines / Prototypes）都是 CubeMX 留的保护区，
// [解析]   分别用来放：自己要 include 的头文件、自己要定义的宏、自己函数的原型声明。
// [解析]   本工程把它们放在 applications/ 里，所以这几块一直是空的。

extern CAN_HandleTypeDef hcan1;
// [解析] ↑ extern = "变量在别的文件里定义"，这里只是声明。
// [解析]   本体在 Core/Src/can.c：CAN_HandleTypeDef hcan1;
// [解析]   CAN_HandleTypeDef 是 HAL 的 CAN 句柄结构体，主要成员：
// [解析]     Instance  = CAN1（指向 CAN1 的寄存器组）；
// [解析]     Init      = 波特率分频等配置（在 MX_CAN1_Init 里填，见 can.c）；
// [解析]     State / ErrorCode —— HAL 维护的运行时状态。
// [解析]   注意本工程没有 CAN2 的句柄，因为 applications/ 里没有任何代码用 hcan2。
// [解析]   上层用法：sp_middleware/io/can/can.cpp 里 sp::CAN can1(&hcan1); 把句柄存进对象。

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

// [解析] ★ MX_CAN1_Init：CAN1 初始化函数，实现见 Core/Src/can.c。它做了三件事：
// [解析]   1) 填 hcan1.Init 里的位时序参数（决定波特率）；
// [解析]   2) 调 HAL_CAN_Init()；
// [解析]   3) HAL 内部自动回调 HAL_CAN_MspInit()，在那里使能 CAN1/GPIO 时钟、
// [解析]      把 PD0/PD1 配成复用功能 GPIO_AF9_CAN1、并打开 CAN1_RX0 中断。
// [解析]   ▼ 波特率的推导过程单独说明如下：
// [解析] 【波特率是怎么算出来的 · 1 Mbps 的完整推导】
// [解析]   CAN 每一位的时间被切成若干个小时间片 TQ（Time Quantum）：
// [解析]     1 位 = SyncSeg + BS1 + BS2，其中 SyncSeg 固定 1 个 TQ。
// [解析]   本工程：BS1 = 12TQ、BS2 = 1TQ -> 1 位 = 1 + 12 + 1 = 14 个 TQ。
// [解析]   时间片时钟 = APB1 时钟 / Prescaler = 42 MHz / 3 = 14 MHz
// [解析]     （Prescaler 就是 .ioc 里的 Prescaler=3，写在 can.c 的 hcan1.Init.Prescaler 里）
// [解析]   波特率 = 14 MHz / 14 TQ = 1 Mbps ✔ —— 这就是 RoboMaster 电机总线的标准速率。
// [解析]   采样点位置 = (1+12)/14 ≈ 92.9%，位于位时间的靠后位置；
// [解析]   CAN 总线越长、节点越多，采样点越靠后越稳（业界的经验值在 75%~87.5% 附近，
// [解析]   这里偏高一点，属于本工程沿用的电机厂商推荐参数）。
// [解析]   SJW（SyncJumpWidth）= 1TQ：允许重同步时一次调整的最大 TQ 数，用于吸收
// [解析]   各节点晶振误差；波特率 1 Mbps 时 1TQ 是常规取值。
// [解析]   ▼ 其余几个开关的含义：
// [解析] 其余开关的含义：Mode=CAN_MODE_NORMAL 正常工作模式（不是回环/静默测试模式）；
// [解析]   AutoBusOff=DISABLE（总线严重出错后不自动恢复，需要软件干预）；
// [解析]   AutoRetransmission=DISABLE（发送失败不自动重发 —— 工程上是刻意的选择：
// [解析]   宁可丢一帧也不让发送邮箱被堵住，保证控制周期的实时性）；
// [解析]   ReceiveFifoLocked=DISABLE（FIFO 满了就覆盖旧帧，不锁死）；
// [解析]   TransmitFifoPriority=DISABLE（按邮箱号优先级发送，而不是按 ID 大小）。
// [解析]   中断：Core/Src/can.c 的 MspInit 里设置 CAN1_RX0_IRQn 抢占优先级 5、子优先级 0。
void MX_CAN1_Init(void);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

// [解析] ↑ 上面这对 #ifdef __cplusplus / #endif 结束了文件开头的 extern "C" 块。
// [解析] ↓ 这一行结束文件开头的 #ifndef __CAN_H__ 包含卫士；两者都要成对，缺一个就报错。
#endif /* __CAN_H__ */

