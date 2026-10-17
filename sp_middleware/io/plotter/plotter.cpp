// [解析] 波形调试器的实现：18 个 plot() 重载 + 1 个 send()，前者填帧、后者交串口。
// [解析] 除最后一个 send() 外全是同构函数：定长度 -> 逐个填 float -> send()，看懂第 1 个就看懂了全部。
// [解析] size 字段填的是"字节数" 4*N 而不是 float 个数；上位机按小端解析，接不上先查这两点。
// [解析] DMA 发送是异步的，plot_frame_ 必须常驻内存，调用完 send() 也不能马上改它。

#include "plotter.hpp"

namespace sp
{
Plotter::Plotter(UART_HandleTypeDef * huart, bool use_dma)
: huart_(huart), use_dma_(use_dma), hal_status_(HAL_OK)
{
}

// [解析] 单参数版 —— 18 个 plot() 的模板：static_assert 在编译期检查通道数没超过 data[] 容量，
// [解析] size = 4*1 是数据区的字节数（1 个 float 固定占 4 字节），填好 data[] 后交给 send()。
// [解析] 值的单位和物理含义由调用方约定，上位机只照单收数字。
void Plotter::plot(float value1)
{
  static_assert(PLOTTER_FLOAT_NUM >= 1);
  plot_frame_.size = 4 * 1;
  plot_frame_.data[0] = value1;
  send();
}

// [解析] 2~17 通道版与上面完全相同：size 和 data[] 的元素个数必须一起涨，只改一个就会发出畸形帧。
void Plotter::plot(float value1, float value2)
{
  static_assert(PLOTTER_FLOAT_NUM >= 2);
  plot_frame_.size = 4 * 2;
  plot_frame_.data[0] = value1;
  plot_frame_.data[1] = value2;
  send();
}

void Plotter::plot(float value1, float value2, float value3)
{
  static_assert(PLOTTER_FLOAT_NUM >= 3);
  plot_frame_.size = 4 * 3;
  plot_frame_.data[0] = value1;
  plot_frame_.data[1] = value2;
  plot_frame_.data[2] = value3;
  send();
}

void Plotter::plot(float value1, float value2, float value3, float value4)
{
  static_assert(PLOTTER_FLOAT_NUM >= 4);
  plot_frame_.size = 4 * 4;
  plot_frame_.data[0] = value1;
  plot_frame_.data[1] = value2;
  plot_frame_.data[2] = value3;
  plot_frame_.data[3] = value4;
  send();
}

void Plotter::plot(float value1, float value2, float value3, float value4, float value5)
{
  static_assert(PLOTTER_FLOAT_NUM >= 5);
  plot_frame_.size = 4 * 5;
  plot_frame_.data[0] = value1;
  plot_frame_.data[1] = value2;
  plot_frame_.data[2] = value3;
  plot_frame_.data[3] = value4;
  plot_frame_.data[4] = value5;
  send();
}

void Plotter::plot(
  float value1, float value2, float value3, float value4, float value5, float value6)
{
  static_assert(PLOTTER_FLOAT_NUM >= 6);
  plot_frame_.size = 4 * 6;
  plot_frame_.data[0] = value1;
  plot_frame_.data[1] = value2;
  plot_frame_.data[2] = value3;
  plot_frame_.data[3] = value4;
  plot_frame_.data[4] = value5;
  plot_frame_.data[5] = value6;
  send();
}

void Plotter::plot(
  float value1, float value2, float value3, float value4, float value5, float value6, float value7)
{
  static_assert(PLOTTER_FLOAT_NUM >= 7);
  plot_frame_.size = 4 * 7;
  plot_frame_.data[0] = value1;
  plot_frame_.data[1] = value2;
  plot_frame_.data[2] = value3;
  plot_frame_.data[3] = value4;
  plot_frame_.data[4] = value5;
  plot_frame_.data[5] = value6;
  plot_frame_.data[6] = value7;
  send();
}

void Plotter::plot(
  float value1, float value2, float value3, float value4, float value5, float value6, float value7,
  float value8)
{
  static_assert(PLOTTER_FLOAT_NUM >= 8);
  plot_frame_.size = 4 * 8;
  plot_frame_.data[0] = value1;
  plot_frame_.data[1] = value2;
  plot_frame_.data[2] = value3;
  plot_frame_.data[3] = value4;
  plot_frame_.data[4] = value5;
  plot_frame_.data[5] = value6;
  plot_frame_.data[6] = value7;
  plot_frame_.data[7] = value8;
  send();
}

void Plotter::plot(
  float value1, float value2, float value3, float value4, float value5, float value6, float value7,
  float value8, float value9)
{
  static_assert(PLOTTER_FLOAT_NUM >= 9);
  plot_frame_.size = 4 * 9;
  plot_frame_.data[0] = value1;
  plot_frame_.data[1] = value2;
  plot_frame_.data[2] = value3;
  plot_frame_.data[3] = value4;
  plot_frame_.data[4] = value5;
  plot_frame_.data[5] = value6;
  plot_frame_.data[6] = value7;
  plot_frame_.data[7] = value8;
  plot_frame_.data[8] = value9;
  send();
}

void Plotter::plot(
  float value1, float value2, float value3, float value4, float value5, float value6, float value7,
  float value8, float value9, float value10)
{
  static_assert(PLOTTER_FLOAT_NUM >= 10);
  plot_frame_.size = 4 * 10;
  plot_frame_.data[0] = value1;
  plot_frame_.data[1] = value2;
  plot_frame_.data[2] = value3;
  plot_frame_.data[3] = value4;
  plot_frame_.data[4] = value5;
  plot_frame_.data[5] = value6;
  plot_frame_.data[6] = value7;
  plot_frame_.data[7] = value8;
  plot_frame_.data[8] = value9;
  plot_frame_.data[9] = value10;
  send();
}

void Plotter::plot(
  float value1, float value2, float value3, float value4, float value5, float value6, float value7,
  float value8, float value9, float value10, float value11)
{
  static_assert(PLOTTER_FLOAT_NUM >= 11);
  plot_frame_.size = 4 * 11;
  plot_frame_.data[0] = value1;
  plot_frame_.data[1] = value2;
  plot_frame_.data[2] = value3;
  plot_frame_.data[3] = value4;
  plot_frame_.data[4] = value5;
  plot_frame_.data[5] = value6;
  plot_frame_.data[6] = value7;
  plot_frame_.data[7] = value8;
  plot_frame_.data[8] = value9;
  plot_frame_.data[9] = value10;
  plot_frame_.data[10] = value11;
  send();
}

void Plotter::plot(
  float value1, float value2, float value3, float value4, float value5, float value6, float value7,
  float value8, float value9, float value10, float value11, float value12)
{
  static_assert(PLOTTER_FLOAT_NUM >= 12);
  plot_frame_.size = 4 * 12;
  plot_frame_.data[0] = value1;
  plot_frame_.data[1] = value2;
  plot_frame_.data[2] = value3;
  plot_frame_.data[3] = value4;
  plot_frame_.data[4] = value5;
  plot_frame_.data[5] = value6;
  plot_frame_.data[6] = value7;
  plot_frame_.data[7] = value8;
  plot_frame_.data[8] = value9;
  plot_frame_.data[9] = value10;
  plot_frame_.data[10] = value11;
  plot_frame_.data[11] = value12;
  send();
}

void Plotter::plot(
  float value1, float value2, float value3, float value4, float value5, float value6, float value7,
  float value8, float value9, float value10, float value11, float value12, float value13)
{
  static_assert(PLOTTER_FLOAT_NUM >= 13);
  plot_frame_.size = 4 * 13;
  plot_frame_.data[0] = value1;
  plot_frame_.data[1] = value2;
  plot_frame_.data[2] = value3;
  plot_frame_.data[3] = value4;
  plot_frame_.data[4] = value5;
  plot_frame_.data[5] = value6;
  plot_frame_.data[6] = value7;
  plot_frame_.data[7] = value8;
  plot_frame_.data[8] = value9;
  plot_frame_.data[9] = value10;
  plot_frame_.data[10] = value11;
  plot_frame_.data[11] = value12;
  plot_frame_.data[12] = value13;
  send();
}

void Plotter::plot(
  float value1, float value2, float value3, float value4, float value5, float value6, float value7,
  float value8, float value9, float value10, float value11, float value12, float value13,
  float value14)
{
  static_assert(PLOTTER_FLOAT_NUM >= 14);
  plot_frame_.size = 4 * 14;
  plot_frame_.data[0] = value1;
  plot_frame_.data[1] = value2;
  plot_frame_.data[2] = value3;
  plot_frame_.data[3] = value4;
  plot_frame_.data[4] = value5;
  plot_frame_.data[5] = value6;
  plot_frame_.data[6] = value7;
  plot_frame_.data[7] = value8;
  plot_frame_.data[8] = value9;
  plot_frame_.data[9] = value10;
  plot_frame_.data[10] = value11;
  plot_frame_.data[11] = value12;
  plot_frame_.data[12] = value13;
  plot_frame_.data[13] = value14;
  send();
}

void Plotter::plot(
  float value1, float value2, float value3, float value4, float value5, float value6, float value7,
  float value8, float value9, float value10, float value11, float value12, float value13,
  float value14, float value15)
{
  static_assert(PLOTTER_FLOAT_NUM >= 15);
  plot_frame_.size = 4 * 15;
  plot_frame_.data[0] = value1;
  plot_frame_.data[1] = value2;
  plot_frame_.data[2] = value3;
  plot_frame_.data[3] = value4;
  plot_frame_.data[4] = value5;
  plot_frame_.data[5] = value6;
  plot_frame_.data[6] = value7;
  plot_frame_.data[7] = value8;
  plot_frame_.data[8] = value9;
  plot_frame_.data[9] = value10;
  plot_frame_.data[10] = value11;
  plot_frame_.data[11] = value12;
  plot_frame_.data[12] = value13;
  plot_frame_.data[13] = value14;
  plot_frame_.data[14] = value15;
  send();
}

void Plotter::plot(
  float value1, float value2, float value3, float value4, float value5, float value6, float value7,
  float value8, float value9, float value10, float value11, float value12, float value13,
  float value14, float value15, float value16)
{
  static_assert(PLOTTER_FLOAT_NUM >= 16);
  plot_frame_.size = 4 * 16;
  plot_frame_.data[0] = value1;
  plot_frame_.data[1] = value2;
  plot_frame_.data[2] = value3;
  plot_frame_.data[3] = value4;
  plot_frame_.data[4] = value5;
  plot_frame_.data[5] = value6;
  plot_frame_.data[6] = value7;
  plot_frame_.data[7] = value8;
  plot_frame_.data[8] = value9;
  plot_frame_.data[9] = value10;
  plot_frame_.data[10] = value11;
  plot_frame_.data[11] = value12;
  plot_frame_.data[12] = value13;
  plot_frame_.data[13] = value14;
  plot_frame_.data[14] = value15;
  plot_frame_.data[15] = value16;
  send();
}

void Plotter::plot(
  float value1, float value2, float value3, float value4, float value5, float value6, float value7,
  float value8, float value9, float value10, float value11, float value12, float value13,
  float value14, float value15, float value16, float value17)
{
  static_assert(PLOTTER_FLOAT_NUM >= 17);
  plot_frame_.size = 4 * 17;
  plot_frame_.data[0] = value1;
  plot_frame_.data[1] = value2;
  plot_frame_.data[2] = value3;
  plot_frame_.data[3] = value4;
  plot_frame_.data[4] = value5;
  plot_frame_.data[5] = value6;
  plot_frame_.data[6] = value7;
  plot_frame_.data[7] = value8;
  plot_frame_.data[8] = value9;
  plot_frame_.data[9] = value10;
  plot_frame_.data[10] = value11;
  plot_frame_.data[11] = value12;
  plot_frame_.data[12] = value13;
  plot_frame_.data[13] = value14;
  plot_frame_.data[14] = value15;
  plot_frame_.data[15] = value16;
  plot_frame_.data[16] = value17;
  send();
}

// [解析] 18 通道版是上限：data[] 只开了 18 个 float，想画更多得先改 plotter.hpp 的 PLOTTER_FLOAT_NUM。
void Plotter::plot(
  float value1, float value2, float value3, float value4, float value5, float value6, float value7,
  float value8, float value9, float value10, float value11, float value12, float value13,
  float value14, float value15, float value16, float value17, float value18)
{
  static_assert(PLOTTER_FLOAT_NUM >= 18);
  plot_frame_.size = 4 * 18;
  plot_frame_.data[0] = value1;
  plot_frame_.data[1] = value2;
  plot_frame_.data[2] = value3;
  plot_frame_.data[3] = value4;
  plot_frame_.data[4] = value5;
  plot_frame_.data[5] = value6;
  plot_frame_.data[6] = value7;
  plot_frame_.data[7] = value8;
  plot_frame_.data[8] = value9;
  plot_frame_.data[9] = value10;
  plot_frame_.data[10] = value11;
  plot_frame_.data[11] = value12;
  plot_frame_.data[12] = value13;
  plot_frame_.data[13] = value14;
  plot_frame_.data[14] = value15;
  plot_frame_.data[15] = value16;
  plot_frame_.data[16] = value17;
  plot_frame_.data[17] = value18;
  send();
}

// [解析] 唯一的发送出口：长度 = 2(帧头) + 1(长度字段) + plot_frame_.size = 3 + 4*N 字节。
// [解析] 不能图省事写 sizeof(plot_frame_)：那样恒为 75，会把未使用通道的旧值一起发出去。
// [解析] 结构体是 packed 的，内存布局就是线上帧格式，所以 (uint8_t *)&plot_frame_ 强转是安全的。
// [解析] 返回值存进 hal_status_ 但没人读：DMA 忙时这一帧被静默丢掉，表现为偶尔少一帧。
// [解析] 阻塞分支的超时给 255 ms（不是别处的 10 ms）：波特率低 + 通道多时一帧就要几毫秒。
void Plotter::send()
{
  if (use_dma_) {
    hal_status_ = HAL_UART_Transmit_DMA(
      huart_, (uint8_t *)&plot_frame_,
      sizeof(plot_frame_.start) + sizeof(plot_frame_.size) + plot_frame_.size);
  }
  else {
    hal_status_ = HAL_UART_Transmit(
      huart_, (uint8_t *)&plot_frame_,
      sizeof(plot_frame_.start) + sizeof(plot_frame_.size) + plot_frame_.size, 0xff);
  }
}

}  // namespace sp
