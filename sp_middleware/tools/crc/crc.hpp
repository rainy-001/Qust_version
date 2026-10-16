// [解析] ==========================================================================
// [解析] 【文件】sp_middleware/tools/crc/crc.hpp
// [解析] 【作用】声明 CRC8 / CRC16 校验相关的 6 个函数（只有声明，实现都在 crc.cpp）。
// [解析] 【被谁用】裁判系统 referee/ 和视觉 io/vision/ 里的收发包代码，
// [解析]         例如 vt03.cpp 用 append_crc8/append_crc16 发帧、check_crc16 收帧。
// [解析] 【关键点】下面每个函数的 "len" 含义不同，是全文件最容易搞错的地方：
// [解析]   get_xxx 的 len 不含校验字节；check_xxx 的 len 含校验字节。
// [解析]   多传或少传 1~2 个字节，校验就会一直失败。
// [解析] ==========================================================================

#ifndef SP__CRC_HPP
#define SP__CRC_HPP

#include <cstdint>

namespace sp
{
// len不包括crc8
// [解析] 计算 CRC8。len = 需要参与计算的字节数（不含校验字节），返回 8 位校验值 0x00~0xFF。
// [解析] 典型用法：crc8 = get_crc8(buf, 4); 表示用前 4 个字节算出校验值。
uint8_t get_crc8(const uint8_t * data, uint16_t len);

// len包括crc8
// [解析] 校验一帧数据。len = 整帧长度（含最后 1 个 CRC8 字节）。
// [解析] 返回 true 表示校验通过、数据可信；false 表示数据被干扰，应当整帧丢弃。
// [解析] 注意：len 必须 ≥ 1，否则会越界读（原因见 crc.cpp 的实现注释）。
bool check_crc8(const uint8_t * data, uint16_t len);

// len不包括crc16
// [解析] 计算 CRC16。len = 需要参与计算的字节数（不含校验字节），返回 16 位校验值。
// [解析] 典型用法：crc16 = get_crc16(buf, frame_len - 2)。
uint16_t get_crc16(const uint8_t * data, uint32_t len);

// len包括crc16
// [解析] 校验一帧数据。len = 整帧长度（含末尾 2 个 CRC16 字节，低字节在前）。
// [解析] 返回 true 表示校验通过；false 表示数据损坏，应当整帧丢弃。
// [解析] 注意：len 必须 ≥ 2，否则会越界读。
bool check_crc16(const uint8_t * data, uint32_t len);

// [解析] 发送前一键补校验：算出前 len-1 个字节的 CRC8，写进 data[len-1]。
// [解析] 参数 len = 整帧长度（含 CRC8 的 1 个字节）。
// [解析] 副作用：会原地改写缓冲区最后一个字节，调用前要确认那里是留给校验位的。
void append_crc8(uint8_t * data, uint16_t len);

// [解析] 发送前一键补校验：算出前 len-2 个字节的 CRC16，写进最后 2 个字节（低字节在前）。
// [解析] 参数 len = 整帧长度（含 CRC16 的 2 个字节）。
// [解析] 副作用：会原地改写缓冲区最后 2 个字节。
void append_crc16(uint8_t * data, uint32_t len);

}  // namespace sp

#endif  // SP__CRC_HPP
