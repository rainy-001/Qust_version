// [解析] ==========================================================================
// [解析] 【文件】sp_middleware/io/bmi088/bmi088_defs.h
// [解析] 【作用】BMI088 这颗六轴 IMU 的"寄存器字典"：把数据手册里的寄存器地址、
// [解析]         每个配置位应该填什么值、以及错误码，全部翻译成好记的宏名字。
// [解析]         本文件只有 #define，不产生任何代码，编译后不占空间。
// [解析] 【被谁用】bmi088.cpp（读写寄存器、配置量程、判断初始化是否成功）。
// [解析] 【关键点】1) BMI088 其实是两颗芯片封装在一起（加速度计 + 陀螺仪），
// [解析]             它们各自有独立的寄存器空间，所以同一个地址 0x00 在两边含义不同
// [解析]             （加速度计是 CHIP_ID=0x1E，陀螺仪是 CHIP_ID=0x0F）。
// [解析]           2) 大量宏是"先算好位移的值"，配寄存器时直接按位或即可，
// [解析]             例如 ACC_CONF = BWP | ODR | 0x80。
// [解析]           3) 带 _SHFITS 后缀的是"移位量"（拼写是原作者的小笔误，应为 SHIFTS），
// [解析]             带 << 的是"已经移好位的最终值"。
// [解析] ==========================================================================

#ifndef BMI088_DEFS_H
#define BMI088_DEFS_H

// BMI088(加速度计)寄存器地址及其存放的内容

// [解析] 加速度计的"身份证"寄存器：读它返回固定值 0x1E 才说明芯片活着、SPI 通了。
// [解析] 初始化时就是靠对比这两个宏来判断传感器有没有焊好 / 接线有没有错。
#define BMI088_ACC_CHIP_ID 0x00
#define BMI088_ACC_CHIP_ID_VALUE 0x1E

// [解析] 加速度数据寄存器起点：连续读 6 个字节就是 X/Y/Z 三轴的 int16 原始值。
// [解析] 每轴 2 字节，小端排列（低字节在前，见 bmi088.cpp 的字节拼接）。
#define BMI088_ACC_DATA 0x12

// [解析] 温度数据寄存器：2 个字节，实际温度只有 11 位有效位。
#define BMI088_TEMP_DATA 0x22

// [解析] 加速度计通用配置寄存器：bit[3:0] 选输出速率 ODR，bit[6:4] 选滤波带宽 BWP，
// [解析] bit7 是必须置 1 的保留位（见下面的 MUST_SET）。三部分按位或成一个字节写进去。
#define BMI088_ACC_CONF 0x40
// [解析] 这一位必须写 1，是数据手册要求的"保留位固定值"，写 0 芯片行为未定义。
#define BMI088_ACC_CONF_MUST_SET 0x80
// [解析] BWP(带宽)字段在 bit[6:4]，所以要左移 4 位。
#define BMI088_ACC_BWP_SHFITS 0x4
// [解析] OSR4 = 4 倍过采样，噪声最低但带宽最窄；三者按需选，通常用 NORMAL。
#define BMI088_ACC_BWP_OSR4 (0x0 << BMI088_ACC_BWP_SHFITS)
#define BMI088_ACC_BWP_OSR2 (0x1 << BMI088_ACC_BWP_SHFITS)
#define BMI088_ACC_BWP_NORMAL (0x2 << BMI088_ACC_BWP_SHFITS)
// [解析] ODR(输出数据率)字段在 bit[3:0]，不移位。
#define BMI088_ACC_ODR_SHFITS 0x0
// [解析] 下面是各档采样率对应的编码值：0x5=12.5Hz ... 0xC=1600Hz。
// [解析] 本项目实际用 800Hz（见 bmi088.cpp 的 ACC_INIT_TABLE）。
#define BMI088_ACC_ODR_12_5_HZ (0x5 << BMI088_ACC_ODR_SHFITS)
#define BMI088_ACC_ODR_25_HZ (0x6 << BMI088_ACC_ODR_SHFITS)
#define BMI088_ACC_ODR_50_HZ (0x7 << BMI088_ACC_ODR_SHFITS)
#define BMI088_ACC_ODR_100_HZ (0x8 << BMI088_ACC_ODR_SHFITS)
#define BMI088_ACC_ODR_200_HZ (0x9 << BMI088_ACC_ODR_SHFITS)
#define BMI088_ACC_ODR_400_HZ (0xA << BMI088_ACC_ODR_SHFITS)
#define BMI088_ACC_ODR_800_HZ (0xB << BMI088_ACC_ODR_SHFITS)
#define BMI088_ACC_ODR_1600_HZ (0xC << BMI088_ACC_ODR_SHFITS)

// [解析] 量程寄存器：给定这一档量程，最灵敏的一档是 ±3g（适合测小幅振动），
// [解析] 最粗的是 ±24g。量程越大越不容易饱和，但分辨率越差。
#define BMI088_ACC_RANGE 0x41
#define BMI088_ACC_RANGE_3G 0x0
#define BMI088_ACC_RANGE_6G 0x1
#define BMI088_ACC_RANGE_12G 0x2
#define BMI088_ACC_RANGE_24G 0x3

// [解析] INT1 引脚的电平/推挽配置。加速度计有 INT1、INT2 两个中断引脚，
// [解析] 本项目用 INT1 输出"数据准备好了(DRDY)"信号，通知单片机来取数。
#define BMI088_INT1_IO_CONF 0x53
// [解析] bit3 是 INT1 的输出使能（1=让 INT1 变成输出脚，能驱动电平）。
#define BMI088_ACC_INT1_OUT_SHFITS 0x3
#define BMI088_ACC_INT1_OUT_ENABLE (0x1 << BMI088_ACC_INT1_OUT_SHFITS)
// [解析] bit2 选择推挽(PP)还是开漏(OD)。这里用推挽，能主动输出高低电平。
#define BMI088_ACC_INT1_GPIO_MODE_SHFITS 0x2
#define BMI088_ACC_INT1_GPIO_PP (0x0 << BMI088_ACC_INT1_GPIO_MODE_SHFITS)
#define BMI088_ACC_INT1_GPIO_OD (0x1 << BMI088_ACC_INT1_GPIO_MODE_SHFITS)
// [解析] bit1 选择中断有效电平是高还是低。
#define BMI088_ACC_INT1_GPIO_LVL_SHFITS 0x1
#define BMI088_ACC_INT1_GPIO_LOW (0x0 << BMI088_ACC_INT1_GPIO_LVL_SHFITS)
#define BMI088_ACC_INT1_GPIO_HIGH (0x1 << BMI088_ACC_INT1_GPIO_LVL_SHFITS)

// [解析] 中断映射寄存器：决定"哪些事件"从"哪个引脚"输出。
// [解析] 本项目把 DRDY 映射到 INT1（值 0x04），配合上面的 INT1_IO_CONF 使用。
#define BMI088_INT1_INT2_MAP_DATA 0x58
#define BMI088_DRDY_TO_INT1_SHFITS 0x2
#define BMI088_DRDY_TO_INT1 (0x1 << BMI088_DRDY_TO_INT1_SHFITS)

// [解析] 电源模式寄存器：0x03=挂起(几乎不耗电，不采数据)，0x00=正常工作。
#define BMI088_ACC_PWR_CONF 0x7C
#define BMI088_ACC_PWR_SUSPEND_MODE 0x03
#define BMI088_ACC_PWR_ACTIVE_MODE 0x00

// [解析] 加速度计的"总开关"：0x04=打开加速度计，0x00=关闭。
// [解析] 注意它和 PWR_CONF 是两个不同寄存器，初始化时两个都要写。
#define BMI088_ACC_PWR_CTRL 0x7D
#define BMI088_ACC_ENABLE_ACC_OFF 0x00
#define BMI088_ACC_ENABLE_ACC_ON 0x04

// [解析] 软复位：向寄存器 0x7E 写入固定值 0xB6，芯片会自己回到上电默认状态。
// [解析] 0xB6 是数据手册规定的"魔数"，不能换成别的值。
#define BMI088_ACC_SOFTRESET 0x7E
#define BMI088_ACC_SOFTRESET_VALUE 0xB6

// BMI088(陀螺仪)寄存器地址及其存放的内容
// [解析] 注意：以下是另一颗芯片（陀螺仪）的寄存器，地址与上面加速度计的完全独立。
// [解析] 比如 0x00 在加速度计里是 CHIP_ID(0x1E)，在陀螺仪里也是 CHIP_ID 但值为 0x0F。

// [解析] 陀螺仪身份证：读回 0x0F 说明陀螺仪在正常应答。
#define BMI088_GYRO_CHIP_ID 0x00
#define BMI088_GYRO_CHIP_ID_VALUE 0x0F

// [解析] 陀螺仪数据起点：连续 6 字节 = X/Y/Z 三轴的角速度原始值，每轴 2 字节小端。
#define BMI088_GYRO_DATA 0x02

// [解析] 陀螺仪量程寄存器：编码值越小量程越大。本项目选 2000dps（见 gyro_init 表），
// [解析] 因为机器人快速旋转时角速度很容易超过 1000°/s，选小了会削顶失真。
#define BMI088_GYRO_RANGE 0x0F
#define BMI088_GYRO_2000 0x0
#define BMI088_GYRO_1000 0x1
#define BMI088_GYRO_500 0x2
#define BMI088_GYRO_250 0x3
#define BMI088_GYRO_125 0x4

// [解析] 带宽寄存器：名字形如"量程_截止频率"，例如 BMI088_GYRO_2000_230_HZ
// [解析] 表示量程 2000dps 时把低通滤波截止频率设为 230Hz（滤掉高频机械振动噪声）。
// [解析] 名字里的前半段必须和你实际选的量程一致，否则该编码在别的量程下含义不同。
#define BMI088_GYRO_BANDWIDTH 0x10
// [解析] 又是"必须写 1"的保留位。
#define BMI088_GYRO_BANDWIDTH_MUST_SET 0x80
#define BMI088_GYRO_2000_532_HZ 0x00
#define BMI088_GYRO_2000_230_HZ 0x01
#define BMI088_GYRO_1000_116_HZ 0x02
#define BMI088_GYRO_400_47_HZ 0x03
#define BMI088_GYRO_200_23_HZ 0x04
#define BMI088_GYRO_100_12_HZ 0x05
#define BMI088_GYRO_200_64_HZ 0x06
#define BMI088_GYRO_100_32_HZ 0x07

// [解析] 电源模式寄存器：0x00=正常，0x80=挂起，0x20=深度挂起（唤醒更慢、更省电）。
#define BMI088_GYRO_LPM1 0x11
#define BMI088_GYRO_NORMAL_MODE 0x00
#define BMI088_GYRO_SUSPEND_MODE 0x80
#define BMI088_GYRO_DEEP_SUSPEND_MODE 0x20

// [解析] 陀螺仪软复位：同样写固定魔数 0xB6（和加速度计一样，但地址不同）。
#define BMI088_GYRO_SOFTRESET 0x14
#define BMI088_GYRO_SOFTRESET_VALUE 0xB6

// [解析] 中断控制寄存器：0x80 打开"数据准备好(DRDY)"中断的产生。
#define BMI088_GYRO_INT_CTRL 0x15
#define BMI088_DRDY_OFF 0x00
#define BMI088_DRDY_ON 0x80

// [解析] INT3/INT4 引脚配置。陀螺仪的中断引脚叫 INT3、INT4（不是 INT1/INT2）。
// [解析] 本项目把 DRDY 从 INT3 引出，所以下面只关心 INT3 那几位。
#define BMI088_INT3_INT4_IO_CONF 0x16
#define BMI088_INT4_GPIO_MODE_SHFITS 0x3
#define BMI088_INT4_GPIO_PP (0x0 << BMI088_INT4_GPIO_MODE_SHFITS)
#define BMI088_INT4_GPIO_OD (0x1 << BMI088_INT4_GPIO_MODE_SHFITS)
#define BMI088_INT4_GPIO_LVL_SHFITS 0x2
#define BMI088_INT4_GPIO_LOW (0x0 << BMI088_INT4_GPIO_LVL_SHFITS)
#define BMI088_INT4_GPIO_HIGH (0x1 << BMI088_INT4_GPIO_LVL_SHFITS)
#define BMI088_INT3_GPIO_MODE_SHFITS 0x1
#define BMI088_INT3_GPIO_PP (0x0 << BMI088_INT3_GPIO_MODE_SHFITS)
#define BMI088_INT3_GPIO_OD (0x1 << BMI088_INT3_GPIO_MODE_SHFITS)
#define BMI088_INT3_GPIO_LVL_SHFITS 0x0
#define BMI088_INT3_GPIO_LOW (0x0 << BMI088_INT3_GPIO_LVL_SHFITS)
#define BMI088_INT3_GPIO_HIGH (0x1 << BMI088_INT3_GPIO_LVL_SHFITS)

// [解析] INT3/INT4 的事件映射：0x01 表示把 DRDY 事件送到 INT3，0x80 表示送到 INT4。
#define BMI088_INT3_INT4_IO_MAP 0x18
#define BMI088_DRDY_IO_INT3 0x01
#define BMI088_DRDY_IO_INT4 0x80

// BMI088错误码
// [解析] acc_init()/gyro_init() 的返回值就是下面这些码，0 表示一切正常。
// [解析] 每个"XX_ERROR"都精确对应初始化表里的某一步失败，
// [解析] 例如返回 0x01 就说明写 ACC_PWR_CTRL 寄存器后回读的值不对。
// [解析] 调试技巧：找不到问题时，只把返回值打印出来就能定位是哪一步配错了。

#define BMI088_NO_ERROR 0x00
#define BMI088_ACC_PWR_CTRL_ERROR 0x01
#define BMI088_ACC_PWR_CONF_ERROR 0x02
#define BMI088_ACC_CONF_ERROR 0x03
#define BMI088_ACC_SELF_TEST_ERROR 0x04
#define BMI088_ACC_RANGE_ERROR 0x05
#define BMI088_INT1_IO_CONF_ERROR 0x06
#define BMI088_INT1_INT2_MAP_DATA_ERROR 0x07
#define BMI088_GYRO_RANGE_ERROR 0x08
#define BMI088_GYRO_BANDWIDTH_ERROR 0x09
#define BMI088_GYRO_LPM1_ERROR 0x0A
#define BMI088_GYRO_CTRL_ERROR 0x0B
#define BMI088_INT3_INT4_IO_CONF_ERROR 0x0C
#define BMI088_INT3_INT4_IO_MAP_ERROR 0x0D
// [解析] 0xFF 是"读到的 CHIP_ID 不对"，也就是根本没检测到这颗芯片（未焊接/接线错/SPI 配置错）。
#define BMI088_NO_SENSOR 0xFF

// [解析] SP__ 开头的那些头文件（bmi088.hpp 等）用的是"每个头文件一套宏"的写法，
// [解析] 而这里沿用 #ifndef BMI088_DEFS_H 这种以文件名为准的老写法，效果一样：
// [解析] 防止同一个头文件被 include 两次导致宏重定义。
#endif  // BMI088_DEFS_H
