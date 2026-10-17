// [解析] CRC8 / CRC16 的接口声明，实现见 crc.cpp。
// [解析] 裁判系统与视觉的收发帧都用它：发送前 append_*，收到后 check_*。
// [解析] 最容易搞错的是 len：get_xxx 不含校验字节，check_xxx 含 1 或 2 个校验字节。

#ifndef SP__CRC_HPP
#define SP__CRC_HPP

#include <cstdint>

namespace sp
{
// len不包括crc8
uint8_t get_crc8(const uint8_t * data, uint16_t len);

// len包括crc8
bool check_crc8(const uint8_t * data, uint16_t len);

// len不包括crc16
uint16_t get_crc16(const uint8_t * data, uint32_t len);

// [解析] 末尾 2 字节按"低字节在前"拼回校验值，len 必须 ≥ 2，否则越界读。
// len包括crc16
bool check_crc16(const uint8_t * data, uint32_t len);

void append_crc8(uint8_t * data, uint16_t len);

void append_crc16(uint8_t * data, uint32_t len);

}  // namespace sp

#endif  // SP__CRC_HPP
