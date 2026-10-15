# 开发板 C 型 引脚与 FreeRTOS 任务配置 (Qust_version)

本配置依据 `applications/` 目录下**实际存在的程序**推导，并按开发板 C 型原理图
（`RoBoMaster 开发板 C 型原理图.pdf` Rev V1.0）核对引脚。

MCU: **STM32F407IGH6** / UFBGA176 / 168 MHz / HSE 12 MHz（PLLM=6, PLLN=168）/ APB1 42 MHz / APB2 84 MHz

## 1. 从 applications/ 推导出的需求

| 源文件 | 依赖的外设句柄 | 推导结果 |
|---|---|---|
| `imu_task.cpp` | `hspi1` + PA4/PB0 片选 | SPI1 + 2 个 GPIO 输出 |
| `uart_task.cpp` | `huart3` | USART3（DBUS 接收机） |
| `can_task.cpp` | `hcan1` | CAN1 |
| `led_task.cpp` | `htim5` | TIM5 CH1/2/3 |
| `buzzer_task.cpp` | `htim4` CH3 | TIM4 CH3 |
| `voltage_detect.cpp` | `hadc3` | ADC3 |
| `vision_task.cpp` | USB CDC（`usbd_cdc_if.h`） | **本次未配置**，见第 5 节 |

## 2. 已配置的引脚

| 功能 | 外设 | 引脚 | 说明 |
|---|---|---|---|
| BMI088 加速度计 CS | GPIO 输出 | PA4 | 原理图 `CS1_Accel`；`BMI088` 构造参数 `csb1` |
| BMI088 陀螺仪 CS | GPIO 输出 | PB0 | 原理图 `CS1_Gyro`；`BMI088` 构造参数 `csb2` |
| BMI088 中断 | EXTI4 / EXTI9_5 | PC4 / PC5 | `INT1_Accel` / `INT1_Gyro` |
| BMI088 SPI | SPI1 | PB3 / PA6 / PA7 | SCK / MISO / MOSI（原理图"陀螺仪SPI"） |
| DBUS 接收机 | USART3 | PB10 (TX) / PB11 (RX) | 100000 8N1 |
| 电机 CAN | CAN1 | PD0 (RX) / PD1 (TX) | 1 Mbps |
| RGB LED | TIM5_CH1/2/3 | PH10 / PH11 / PH12 | `sp::LED led(&htim5)`，Period 65535 |
| 蜂鸣器 | TIM4_CH3 | PD14 | `sp::Buzzer`，运行时会改写 PSC/ARR |
| 电池电压 | ADC3_IN8 | PF10 | `sp::Adc battery(&hadc3, 12, 3.3f, 222/22, 1.0f)` |
| 调试口 | SWD | PA13 / PA14 | |
| 晶振 | HSE | PH0 / PH1 | 12 MHz |

### 外设参数

- **SPI1**：Master、Full-Duplex、8-bit、CPOL=High / CPHA=2Edge（Mode 3）、
  Prescaler=8 → **10.5 Mbit/s**（BMI088 上限 10 MHz，如需保守可改为 Prescaler=16）
- **USART3**：100000 8N1，开启 **DMA1_Stream1 (RX)** 循环模式 + 空闲中断
  （`HAL_UARTEx_ReceiveToIdle_DMA` 需要 DMA + USART3 全局中断）
- **ADC3**：12-bit、单通道 IN8、扫描关闭、采样时间 56 cycles、
  ClockPrescaler = **SYNC_PCLK_DIV4**（`adc.cpp` 的 `init()` 里也会再设一次，此处保持一致）
- **TIM5**：Prescaler=0、Period=65535 → 约 1.28 kHz PWM
- **TIM1_UP_TIM10** 作为 HAL 时基（FreeRTOS 占用了 SysTick）
- **NVIC**：CAN1_RX0 / USART3 / DMA1_Stream1 / EXTI4 / EXTI9_5 抢占优先级均为 5
- **时钟**：HSE 12 MHz + PLL（PLLM=6, PLLN=168, PLLP=2）→ SYSCLK 168 MHz。
  HSE 必须处于开启状态，因此 **`PH0-OSC_IN` / `PH1-OSC_OUT` 必须同时出现在
  `Mcu.PinN` 引脚表和 `*.Signal` 声明中**（见第 8 节的坑）。
  PLLM=6 是为 12 MHz 晶振配的（VCO 输入 = 2 MHz）；若晶振换成 8 MHz 则 PLLM 应为 4，
  换 25 MHz 则 PLLM 应为 25。

## 3. FreeRTOS 任务

`FREERTOS.Tasks01` 已按 `applications/` 中的函数名配置 6 个任务
（`configMAX_PRIORITIES=16`，采用 CMSIS-RTOS **V1** 接口）：

| 任务名 | 优先级 | 栈（字） | 函数 | 对应文件 |
|---|---|---|---|---|
| DBUS | 3 | 256 | `uart_task` | uart_task.cpp |
| CAN | 2 | 128 | `can_task` | can_task.cpp |
| BUZZER | 1 | 128 | `buzzer_task` | buzzer_task.cpp |
| ADC | 1 | 128 | `adc_task` | voltage_detect.cpp |
| IMU | -1 | 256 | `imu_task` | imu_task.cpp |
| LED | -2 | 128 | `led_task` | led_task.cpp |

要点：

- 按你的选择**删除了 CubeMX 默认的 `defaultTask`**（原来是空转 `osDelay`）。
- `led_task` / `can_task` 的优先级沿用原工程（-2 / 2），其余为新增。
- 任务全部声明为 **As weak**（与工程原有风格一致），这样 `freertos.c` 里会生成
  `__weak` 空实现；只要 `CMakeLists.txt` 把 `applications/*.cpp` 编进来，
  链接时就会自动用你的实现覆盖。
- DBUS 任务优先级最高，因为遥控器失联 100 ms 就会触发保护，需要及时收包；
  IMU 任务次之（1 kHz 采样）；CAN 任务优先级 2 高于 IMU，保证电机指令按时发出。
- **注意**：`led_task` 用 `led.set()` 设置 RGB，而 `led.cpp` 的内部通道顺序可能与
  PH10/PH11/PH12 的 R/G/B 相反。若点灯颜色不对，交换 `led.set(r, g, b)` 的参数即可，
  或在 CubeMX 里调整 CH1/CH2/CH3 的引脚映射。

## 4. 本次刻意未配置的部分

| 项目 | 原因 |
|---|---|
| CAN2 | `applications/` 中没有使用 `hcan2` 的程序 |
| USB OTG (CDC) | `vision_task.cpp` 需要，但按你的选择本次跳过 |
| I2C1/2/3、用户 SPI2、USART1/6、TIM1、TIM10、用户按键 PH13、硬件版本 PC0-PC2 | 应用中无对应驱动代码 |

如需启用，在 CubeMX 中直接分配即可；完整 C 板引脚表见本文件第 6 节。

## 5. vision_task.cpp 当前无法编译

如果之后要启用视觉通信，需要先处理这几个问题：

1. **`supercap` 未定义**：`vision_task.cpp` 第 14 行引用了全局 `supercap`，
   但 `sp_middleware/motor/super_cap/super_cap.hpp` 只定义了 `sp::SuperCap` 类，
   工程里也没有该类型的全局对象。
2. **文件里混入了 Markdown 代码块**：第 24-36 行是 ``` 和说明文字，不是合法 C++。
3. **缺少 USB Device 中间件**：`vision.cpp` 依赖 `usbd_cdc_if.h`、`CDC_Transmit_FS`，
   需要启用 USB_OTG_FS（Device Only）+ CDC 中间件，并把 USB Device 库加入 CMake。

## 6. 生成代码后的步骤

`.ioc` 只是配置描述，`Core/Src/*.c` 需要 CubeMX 生成。
当前 `Drivers/STM32F4xx_HAL_Driver` 中**缺少 SPI / UART / ADC 的 HAL 源文件**
（现有的是 CAN、TIM、DMA、GPIO、RCC、PWR 等），所以必须完整生成一次：

1. 用 STM32CubeMX 打开 `Qust_version.ioc`；
2. 确认 Project Manager → Toolchain/IDE = **CMake**、Firmware Package = **STM32Cube FW_F4 V1.28.3**；
3. 点 **GENERATE CODE**。

生成后应当得到：`hspi1` / `huart3` / `hadc3` / `htim4` / `htim5` / `hcan1`，
以及 `Core/Src/freertos.c` 里 6 个任务的创建代码。

## 7. 完整 C 板引脚参考

以下引脚板上有连线，但本工程未使用，列在此处便于日后扩展：

| 功能 | 外设 | 引脚 |
|---|---|---|
| 磁力计 IST8310 | I2C1 | PB8 (SCL) / PB9 (SDA)，DRDY: PH6 |
| 外接摄像头 I2C | I2C2 | PH4 (SCL) / PH5 (SDA)（另一说为 PF0/PF1，需按实物确认） |
| 用户 SPI | SPI2 | PB13 / PB14 / PB15，CS: PB12 |
| 调试串口 | USART1 | PA9 (TX) / PA10 (RX) |
| 裁判系统 / VT03 / PM02 | USART6 | PG9 (RX) / PG14 (TX) |
| 舵机 PWM | TIM1_CH1~CH4 | PE9 / PE11 / PE13 / PE14 |
| 激光 | TIM10_CH1 | PF6 |
| 用户按键 | GPIO | PH13 |
| 硬件版本 | GPIO | PC0 / PC1 / PC2 |
| 预留 ADC | ADC3 | PF3/PF4/PF5/PF7/PF8/PF9 |

> 注：CAN2 在原理图上于 PB5/PB6 与 PB12/PB13 两处都有标注，而 PB12~PB15 的
> "用户SPI"分组写的是 `SPI2_*`。放大核对后判断 PB12~PB15 是用户 SPI 排针
> （带 12Ω 串阻），故 CAN2 应为 **PB5( RX) / PB6(TX)**。

## 8. 关于 HSE 被"关闭"的坑（已修复）

**现象**：用 CubeMX 打开 `.ioc` 后，Clock Configuration 里 HSE 变成灰色未启用，
报 "HSE is not enabled"，PLL 的输入源没有时钟。

**原因**：`.ioc` 里的引脚要**同时**具备两处声明才会被 CubeMX 认作"已配置"：

```
Mcu.Pin6=PH0-OSC_IN                 <- ① 出现在引脚表里
PH0-OSC_IN.Signal=RCC_OSC_IN        <- ② 有 Signal 声明
PH0-OSC_IN.Mode=HSE-External-Oscillator
```

只写 ② 不写 ①（或反之），CubeMX 重新加载时会把它当作**未配置的引脚**并直接丢弃，
于是 HSE 就"被关掉"了。`RCC.HSE_VALUE=12000000` 和 `RCC.PLLM=6` 这类参数
**不足以**让 HSE 生效——它们只是时钟树的数值，不表示 OSC 引脚已启用。

注意这不会报语法错误，只有在 CubeMX 里看时钟树才会发现，属于容易漏掉的静默失效。

**修复方式**：把 `PH0-OSC_IN` / `PH1-OSC_OUT` 补回 `Mcu.PinN` 引脚表（并同步 `Mcu.PinsNb`）。

**如果又想改 HSE 频率**：不要只改 `RCC.HSE_VALUE`，要同时改 `RCC.PLLM`，
保证 VCO 输入 = HSE / PLLM 落在 **1~2 MHz** 且尽量接近 2 MHz：

| 晶振 | PLLM | VCO 输入 |
|---|---|---|
| 8 MHz | 4 | 2 MHz |
| 12 MHz（C 板） | 6 | 2 MHz |
| 16 MHz | 8 | 2 MHz |
| 25 MHz | 25 | 1 MHz |

## 9. 关于外设在 CubeMX 里"整个不见"的坑（ADC3，已修复）

**现象**：`.ioc` 里明明有 `ADC3.Channel-0#...`、`Mcu.IP0=ADC3`、`PF10.Signal=ADC3_IN8`，
但 CubeMX 打开后 Analog 里没有 ADC3，PF10 也是未分配状态。

**原因**：外设除了要出现在 `Mcu.IPN`，**还必须出现在
`ProjectManager.functionlistsort`**（这个列表记录"要生成哪些 `MX_xxx_Init` 函数
以及它们在 `main()` 里的调用顺序"）：

```
Mcu.IP0=ADC3                                     <- 声明了这个外设
ProjectManager.functionlistsort=...,4-MX_ADC3_Init-ADC3-false-HAL-true,...   <- 少了这行
```

少了 functionlistsort 里的条目，CubeMX 就没有"该外设已配置"的记录，
界面上不显示、也不会生成 `MX_ADC3_Init`/`adc.c`，但文件本身**完全不报错**。

**修复方式**：把 `N-MX_<外设>_Init-<外设>-false-HAL-true` 补进 `functionlistsort`
并重排序号。

### 这两个坑的共同规律

`.ioc` 里"声明一个功能"往往需要**两处以上**同时成立，只写一处会被静默忽略：

| 功能 | 需要的声明 |
|---|---|
| 时钟引脚（HSE/LSE） | `Mcu.PinN` 条目 + `Pxy.Signal` + `Pxy.Mode` |
| 任意外设 | `Mcu.IPN` 条目 + `ProjectManager.functionlistsort` 里的 `MX_*_Init` |
| 引脚功能 | `Mcu.PinN` 条目 + `Pxy.Signal` |

所以**不要手工编辑 `.ioc` 的这几行**，或者改完务必用校验脚本过一遍。

## 10. 结构校验脚本

上面两个坑都是"少一行、不报错、功能静默消失"，所以加了个校验脚本：

```bash
python tools/check_ioc.py
```

它检查：

1. `Mcu.PinN` 索引连续且引脚不重复
2. 必需引脚（`PH0-OSC_IN`/`PH1-OSC_OUT` 等）确实在引脚表里
3. 每个引脚都有 `.Signal`
4. 外设信号没有被多个引脚占用
5. `Mcu.IPN` 索引连续、与 `Mcu.IPNb` 一致、无重复
6. **每个外设都在 `functionlistsort` 里有对应条目**（ADC3 消失的直接原因）
7. HSE/PLLM 组合落在 1~2 MHz
8. HAL 时基是否为 TIM1

返回码 0 表示通过。修改 `.ioc` 后建议都跑一次。


