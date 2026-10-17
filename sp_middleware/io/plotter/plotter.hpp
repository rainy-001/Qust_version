// [解析] 波形调试器：把几个 float 通过串口丢给电脑上位机（VOFA+ 之类）实时画曲线。
// [解析] 调用 plotter.plot(a, b, c) 即可，plot() 有 1~18 个参数共 18 个重载，按参数个数自动选。
// [解析] 帧格式 = 2 字节 0xAA 0xBB + 1 字节长度 + N 个 float；长度填的是字节数 4*N，不是通道数。
// [解析] 本工程目前还没有任务用它，CMakeLists.txt 里也还没加入编译。

#ifndef SP__PLOTTER_HPP
#define SP__PLOTTER_HPP

#include "usart.h"

namespace sp
{
// [解析] 一帧最多 18 个 float，最大帧长 = 2 + 1 + 4*18 = 75 字节；想画更多通道改这个数字。
constexpr size_t PLOTTER_FLOAT_NUM = 18;

// [解析] packed 禁止编译器插入对齐填充，否则 size 后面会多 1 字节、data 从偏移 4 开始，上位机整体错位。
// [解析] 代价是 data 非 4 字节对齐，只能逐元素读写或 memcpy，不要整体按字访问。
struct __attribute__((packed)) PlotFrame
{
  uint8_t start[2] = {0xAA, 0xBB};
  uint8_t size;
  float data[PLOTTER_FLOAT_NUM];
};

// [解析] 一个对象对应一路串口，建议单独留给上位机；无状态，创建完就能直接 plot()。
class Plotter
{
public:
  // [解析] use_dma 默认 true：走 DMA 不占 CPU；false 是阻塞发送，会卡住调用者的任务。
  Plotter(UART_HandleTypeDef * huart, bool use_dma = true);

  // [解析] 18 个重载做的事完全一样：置 size = 4*参数个数 -> 依次填 data[] -> send()，只是通道数不同。
  // [解析] 参数顺序就是上位机的曲线顺序（第 1 个参数 = 第 1 条线），调换等于换曲线。
  void plot(float value1);
  void plot(float value1, float value2);
  void plot(float value1, float value2, float value3);
  void plot(float value1, float value2, float value3, float value4);
  void plot(float value1, float value2, float value3, float value4, float value5);
  void plot(float value1, float value2, float value3, float value4, float value5, float value6);
  void plot(
    float value1, float value2, float value3, float value4, float value5, float value6,
    float value7);
  void plot(
    float value1, float value2, float value3, float value4, float value5, float value6,
    float value7, float value8);
  void plot(
    float value1, float value2, float value3, float value4, float value5, float value6,
    float value7, float value8, float value9);
  void plot(
    float value1, float value2, float value3, float value4, float value5, float value6,
    float value7, float value8, float value9, float value10);
  void plot(
    float value1, float value2, float value3, float value4, float value5, float value6,
    float value7, float value8, float value9, float value10, float value11);
  void plot(
    float value1, float value2, float value3, float value4, float value5, float value6,
    float value7, float value8, float value9, float value10, float value11, float value12);
  void plot(
    float value1, float value2, float value3, float value4, float value5, float value6,
    float value7, float value8, float value9, float value10, float value11, float value12,
    float value13);
  void plot(
    float value1, float value2, float value3, float value4, float value5, float value6,
    float value7, float value8, float value9, float value10, float value11, float value12,
    float value13, float value14);
  void plot(
    float value1, float value2, float value3, float value4, float value5, float value6,
    float value7, float value8, float value9, float value10, float value11, float value12,
    float value13, float value14, float value15);
  void plot(
    float value1, float value2, float value3, float value4, float value5, float value6,
    float value7, float value8, float value9, float value10, float value11, float value12,
    float value13, float value14, float value15, float value16);
  void plot(
    float value1, float value2, float value3, float value4, float value5, float value6,
    float value7, float value8, float value9, float value10, float value11, float value12,
    float value13, float value14, float value15, float value16, float value17);
  void plot(
    float value1, float value2, float value3, float value4, float value5, float value6,
    float value7, float value8, float value9, float value10, float value11, float value12,
    float value13, float value14, float value15, float value16, float value17, float value18);

private:
  UART_HandleTypeDef * huart_;
  bool use_dma_;
  HAL_StatusTypeDef hal_status_;
  PlotFrame plot_frame_;

  void send();
  // [解析] plot_frame_ 必须是成员而不是局部变量：DMA 发送是异步的，函数返回后还在读这块内存。
  // [解析] hal_status_ 只写不读，发送失败（典型 HAL_BUSY）会被静默忽略，表现为偶尔丢帧。
};

}  // namespace sp

#endif  // SP__PLOTTER_HPP