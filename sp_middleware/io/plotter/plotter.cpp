// [解析] ==========================================================================
// [解析] 【文件】sp_middleware/io/plotter/plotter.cpp
// [解析] 【作用】plotter.hpp 里那 18 个 plot() 重载和 send() 的具体实现。
// [解析] 【本文件怎么读】虽然它有 350 行，但只有两类代码：
// [解析]   1) 结构一模一样的 18 个 plot()：定长度 -> 逐个填 float -> send()。
// [解析]      第一个（单参数版）写得最详细，看懂它 = 看懂全部 18 个，后面都是复制粘贴改数字；
// [解析]   2) 最后那个 send()：把结构体按字节交给串口，DMA 或阻塞二选一。
// [解析] 【三个必须知道的事】
// [解析]   1) size 字段填的是"字节数"4*N，不是 float 的个数；
// [解析]   2) 浮点数按 STM32 的"小端"字节序原样发出（低字节在前），上位机必须按小端解析，
// [解析]      否则数值会变成完全不可读的乱码 —— 这是接不上上位机时第一个要查的地方；
// [解析]   3) DMA 发送是异步的，plot_frame_ 必须常驻内存（所以它是类的成员），
// [解析]      而且刚调用完 send() 不能立刻改它，否则会把正在发的数据改坏。
// [解析] ==========================================================================

#include "plotter.hpp"

namespace sp
{
// [解析] 构造函数：只把两个参数存进成员变量，不做任何硬件动作
// [解析] （串口的初始化在 CubeMX 生成的 main.c / usart.c 里已经做完了）。
// [解析] hal_status_ 先给一个合法初值 HAL_OK，避免对象刚建出来时它是未定义的垃圾值。
Plotter::Plotter(UART_HandleTypeDef * huart, bool use_dma)
: huart_(huart), use_dma_(use_dma), hal_status_(HAL_OK)
{
}

// [解析] 【重点示例】单参数版 —— 18 个 plot() 里最值得精读的一个，整套流程都在这了：
// [解析]   static_assert(...)：编译期检查"要画的通道数没超过 data[] 的容量"。
// [解析]     它是"静态断言"：条件不成立就当场编译报错，运行时零开销、不占 Flash。
// [解析]     比如你把 PLOTTER_FLOAT_NUM 改小到 3，这行就会在你编译时直接报错提醒你。
// [解析]   plot_frame_.size = 4 * 1：告诉上位机"数据区一共多少字节"。
// [解析]     ★为什么乘 4？因为 1 个 float 在 STM32 上固定占 4 字节（IEEE-754 单精度，32 位）。
// [解析]     所以 1 个通道 = 4 字节，N 个通道 = 4N 字节，取值范围 4~72。
// [解析]     不能填成"通道个数 N"，那样上位机会按错误的长度去切帧，曲线全乱。
// [解析]   plot_frame_.data[0] = value1：把要观测的值写进数据区。
// [解析]     单位、含义由你自己约定（角度 rad、角速度、电流 A、PWM 占空比……），
// [解析]     上位机只是照单收数字，不关心物理意义。
// [解析]   send()：把这一帧真正发出去。
// [解析] 参数 value1：本次要画的那条曲线的当前值，类型 float。
void Plotter::plot(float value1)
{
  static_assert(PLOTTER_FLOAT_NUM >= 1);
  plot_frame_.size = 4 * 1;
  plot_frame_.data[0] = value1;
  send();
}

// [解析] 2 通道版：流程完全同上。size = 4*2 = 8 字节，按顺序填 data[0]、data[1]，
// [解析] 上位机那边就是"第 1 条线、第 2 条线"，顺序必须和这里一一对应。
void Plotter::plot(float value1, float value2)
{
  static_assert(PLOTTER_FLOAT_NUM >= 2);
  plot_frame_.size = 4 * 2;
  plot_frame_.data[0] = value1;
  plot_frame_.data[1] = value2;
  send();
}

// [解析] 3 通道版：size = 4*3 = 12 字节。
void Plotter::plot(float value1, float value2, float value3)
{
  static_assert(PLOTTER_FLOAT_NUM >= 3);
  plot_frame_.size = 4 * 3;
  plot_frame_.data[0] = value1;
  plot_frame_.data[1] = value2;
  plot_frame_.data[2] = value3;
  send();
}

// [解析] 4 通道版：size = 4*4 = 16 字节。
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

// [解析] 5 通道版：size = 4*5 = 20 字节。
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

// [解析] 6 通道版：size = 4*6 = 24 字节。
// [解析] 从这个版本开始参数一行写不下，于是换行排版 —— 换行对编译器没有任何影响。
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

// [解析] 7 通道版：size = 4*7 = 28 字节。
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

// [解析] 8 通道版：size = 4*8 = 32 字节。
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

// [解析] 9 通道版：size = 4*9 = 36 字节。
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

// [解析] 10 通道版：size = 4*10 = 40 字节。
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

// [解析] 11 通道版：size = 4*11 = 44 字节。
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

// [解析] 12 通道版：size = 4*12 = 48 字节。
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

// [解析] 13 通道版：size = 4*13 = 52 字节。
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

// [解析] 14 通道版：size = 4*14 = 56 字节。
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

// [解析] 15 通道版：size = 4*15 = 60 字节。
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

// [解析] 16 通道版：size = 4*16 = 64 字节。
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

// [解析] 17 通道版：size = 4*17 = 68 字节。
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

// [解析] 18 通道版：size = 4*18 = 72 字节，这是本类能画的最多条曲线（受 data[18] 限制）。
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

// [解析] 【发送函数】把 plot_frame_ 整块按字节发给串口。两种方式二选一：
// [解析]   1) use_dma_ == true —— HAL_UART_Transmit_DMA：把"要发的内存地址 + 长度"告诉 DMA，
// [解析]      函数立刻返回，CPU 不用一个字节一个字节地等，接着干别的活（推荐）。
// [解析]   2) use_dma_ == false —— HAL_UART_Transmit：阻塞式发送，CPU 会在这里空等到发完。
// [解析]      调试串口波特率低、数据量大时会明显拖慢控制周期，一般只在没配 DMA 时用。
// [解析] 长度计算：sizeof(plot_frame_.start) + sizeof(plot_frame_.size) + plot_frame_.size
// [解析]         = 2 字节帧头 + 1 字节长度字段 + 4*N 字节数据 = 3 + 4*N。
// [解析]   ★注意这里不能图省事写 sizeof(plot_frame_)：因为结构体是 packed 的，
// [解析]   sizeof 恒等于 3 + 4*18 = 75，会把 18 个通道全部发出去（未使用的通道是上一帧的旧值），
// [解析]   白白浪费串口带宽、也让上位机多画出一堆无意义的线，所以必须按 size 精确发。
// [解析] (uint8_t *)&plot_frame_：把结构体的地址强制转成"字节指针"。
// [解析]   HAL 的发送接口只认 uint8_t*，它并不理解结构体，只管把这段内存原样搬出去；
// [解析]   由于结构体是 packed 的、成员紧密排列，内存里的字节顺序就等于帧格式，
// [解析]   所以这样转换是安全的（这也是为什么必须加 packed，见 plotter.hpp）。
// [解析] 最后把返回值存进 hal_status_：HAL_OK 成功；HAL_BUSY 表示上一帧 DMA 还没发完，
// [解析]   这一帧被丢弃；HAL_ERROR / HAL_TIMEOUT 表示串口出问题了。注意目前没人读它。
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
    // [解析] ↑ 最后一个参数 0xff = 255 是超时时间，单位 ms。
    // [解析]   为什么用 255 而不是像别处那样的 10？因为阻塞发送本来就慢，
    // [解析]   波特率低 + 通道多时一帧可能要几毫秒，超时给宽一点避免误判失败。
  }
}

}  // namespace sp
