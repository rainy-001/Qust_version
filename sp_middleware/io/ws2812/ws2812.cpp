// [解析] WS2812 单总线灯珠驱动：没有时钟线，靠高电平长短区分 0/1，所以用 SPI 的 MOSI 去模拟时序。
// [解析] applications/ 的灯效任务调用 set()，一次性阻塞发完；当前实现只驱动一颗灯珠，也没有复位间隔。
// [解析] 每位占 4 个 SPI 位：0xC0 短高电平=0、0xF0 长高电平=1，>>1 把高电平挪进字节中部；三色按 G/R/B 填。
#include "ws2812.hpp"

namespace sp
{
constexpr uint8_t CODE0 = 0xC0;
constexpr uint8_t CODE1 = 0xF0;

WS2812::WS2812(SPI_HandleTypeDef * hspi) : hspi_(hspi) {}

void WS2812::set(uint8_t r, uint8_t g, uint8_t b)
{
  for (int i = 0; i < 8; i++) {
    buff_[07 - i] = (((g >> i) & 0x01) ? CODE1 : CODE0) >> 1;
    buff_[15 - i] = (((r >> i) & 0x01) ? CODE1 : CODE0) >> 1;
    buff_[23 - i] = (((b >> i) & 0x01) ? CODE1 : CODE0) >> 1;
  }

  HAL_SPI_Transmit(hspi_, buff_, 24, 0xFFFF);  // dismiss return
}

}  // namespace sp
