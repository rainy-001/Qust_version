// [解析] 裁判系统串口协议的常量表：帧头/命令码/帧尾长度、SOF、关心的命令码、数据段起点。
// [解析] sp_middleware/referee/vt02/vt02.cpp 的 VT02::update() 拆帧时全靠这几个宏算下标。
// [解析] 帧结构：0xA5 | len(2,小端) | seq | CRC8 | cmd_id(2,小端) | data | CRC16(2)，len 只算数据段长度。
// [解析] 这些数字都是协议规定的字节数/字节值，不要改成"看起来更整齐"的值。

#ifndef SP__REFEREE_DEF_H
#define SP__REFEREE_DEF_H

#define REFEREE_HEAD_LEN 5
#define REFEREE_CMD_ID_LEN 2
#define REFEREE_TAIL_LEN 2

#define REFEREE_SOF 0xA5

#define REFEREE_CUSTOM_CMD_ID 0x302
#define REFEREE_MOUSE_AND_KEYS_CMD_ID 0x304

#define REFEREE_DATA_START 7

#endif  // SP__REFEREE_DEF_H