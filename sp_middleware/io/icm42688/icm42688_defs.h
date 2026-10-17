// [解析] ICM42688 的寄存器字典：寄存器地址、配置值位编码、错误码，全部做成宏。
// [解析] icm42688.cpp 靠这些宏配电源/量程/ODR，并用错误码定位初始化失败在哪一步。
// [解析] 量程宏已左移 5 位（字段在 bit[7:5]），ODR 宏未移位（bit[3:0]），配寄存器时直接按位或。
// [解析] 量程是"档位越大编码越小"（16G=0x00、2G=0x03），ODR 正好相反（0x01=32kHz、0x06=1kHz）。

#ifndef ICM42688_DEFS_H
#define ICM42688_DEFS_H

// ICM42688 寄存器地址及其存放的内容
#define ICM42688_DEVICE_CONFIG 0x11
#define ICM42688_SOFT_RESET_VALUE 0x01

// [解析] PWR_MGMT0 必须最先配好，否则量程/ODR 写了也不生效；0x0F = 加速度计、陀螺仪都进低噪声模式。
#define ICM42688_PWR_MGMT0 0x4E
#define ICM42688_PWR_LN_MODE 0x0F // 加速度计和陀螺仪开启低噪声模式 (Low Noise Mode)

#define ICM42688_WHO_AM_I 0x75
#define ICM42688_CHIP_ID_VALUE 0x47

// 寄存器配置宏
// [解析] ACCEL_CONFIG0 与 GYRO_CONFIG0 布局相同：bit[7:5] 量程、bit[4] 保留、bit[3:0] ODR，故用按位或拼装。
#define ICM42688_ACCEL_CONFIG0 0x50
#define ICM42688_GYRO_CONFIG0 0x4F

// 量程设置宏 (位移已经计算好，直接或运算即可)
#define ICM42688_ACC_RANGE_16G (0x00 << 5)
#define ICM42688_ACC_RANGE_8G  (0x01 << 5)
#define ICM42688_ACC_RANGE_4G  (0x02 << 5)
#define ICM42688_ACC_RANGE_2G  (0x03 << 5)

#define ICM42688_GYRO_RANGE_2000DPS (0x00 << 5)
#define ICM42688_GYRO_RANGE_1000DPS (0x01 << 5)
#define ICM42688_GYRO_RANGE_500DPS  (0x02 << 5)
#define ICM42688_GYRO_RANGE_250DPS  (0x03 << 5)

// 输出速率 ODR (Output Data Rate)
#define ICM42688_ODR_32KHZ 0x01
#define ICM42688_ODR_16KHZ 0x02
#define ICM42688_ODR_8KHZ  0x03
#define ICM42688_ODR_4KHZ  0x04
#define ICM42688_ODR_2KHZ  0x05
#define ICM42688_ODR_1KHZ  0x06  // 我们采用 1000Hz 满足要求
#define ICM42688_ODR_200HZ 0x07

// 数据寄存器起始地址 (突发读取起始点)
// [解析] 突发读取起点：0x1D~0x2A 共 14 字节连续排布，温度 -> 加速度 XYZ -> 角速度 XYZ，一次读完。
#define ICM42688_TEMP_DATA1 0x1D

// ICM42688 错误码
// [解析] device_init() 的返回值：0 表示全部配置成功，其他值对应配置表里某一步回读不一致。
#define ICM42688_NO_ERROR 0x00
#define ICM42688_PWR_MGMT_ERROR 0x01
#define ICM42688_ACCEL_CONF_ERROR 0x02
#define ICM42688_GYRO_CONF_ERROR 0x03
#define ICM42688_NO_SENSOR 0xFF

#endif  // ICM42688_DEFS_H