// [解析] WS2812/WS2812B 单总线 RGB 灯珠驱动，当前实现按一颗灯珠定长 24 字节码流。
// [解析] applications/ 的灯效任务使用，典型写法是 WS2812 led(&hspi1); led.set(255, 0, 0);
// [解析] 颜色是 0~255 的整数（和 led.hpp 的 [0,1] 浮点不同！）；SPI 波特率决定码形是否合时序。
#ifndef SP__WS2812_HPP
#define SP__WS2812_HPP

#include "spi.h"

namespace sp
{
class WS2812
{
public:
  WS2812(SPI_HandleTypeDef * hspi);
  void set(uint8_t r, uint8_t g, uint8_t b);

private:
  SPI_HandleTypeDef * hspi_;
  uint8_t buff_[24];
};

}  // namespace sp

#endif  // SP__WS2812_HPP