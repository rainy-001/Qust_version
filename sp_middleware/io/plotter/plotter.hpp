// [解析] ==========================================================================
// [解析] 【文件】sp_middleware/io/plotter/plotter.hpp
// [解析] 【作用】"波形调试器"：把单片机里的几个 float 变量通过串口丢给电脑上的上位机
// [解析]         （例如 VOFA+、或者自己写的 Python 脚本），实时画成曲线。
// [解析]         调 PID 时看超调、看传感器噪声，全靠它，比 printf 好用得多。
// [解析] 【被谁用】本工程目前还没有任务使用它，CMakeLists.txt 里也还没加入编译。
// [解析]         典型用法：sp::Plotter plotter(&huart1);   // 绑到调试串口
// [解析]                   plotter.plot(imu.yaw, imu.vyaw);  // 周期调用即可
// [解析] 【关键点】1) 帧格式 = 2 字节帧头 0xAA 0xBB + 1 字节长度 + N 个 float；
// [解析]             ★长度字段填的是"数据部分的字节数"= 4*N，不是 float 的个数，最容易搞错；
// [解析]           2) 结构体必须用 __attribute__((packed)) 紧凑排列。否则编译器会按 4 字节
// [解析]             对齐插入填充字节，上位机按固定偏移去解析就会整体错位、曲线全乱；
// [解析]           3) plot() 有 1 个参数到 18 个参数共 18 个"重载"版本，
// [解析]             编译器根据你传了几个参数自动挑对应的那个，写起来不用手动填数量。
// [解析] ==========================================================================

#ifndef SP__PLOTTER_HPP
#define SP__PLOTTER_HPP

#include "usart.h"
// [解析] 同其它 io 模块：usart.h 由 CubeMX 生成，提供 UART_HandleTypeDef 和 huart1 等句柄。

namespace sp
{
// [解析] 一帧最多能装多少个 float。取 18 只是作者的取舍：
// [解析] 帧最大长度 = 2(帧头) + 1(长度) + 4*18 = 75 字节，对串口来说不大；
// [解析] 想一次看更多通道，把这个数字改大即可（下面的 data 数组和 static_assert 会跟着变）。
constexpr size_t PLOTTER_FLOAT_NUM = 18;

// [解析] __attribute__((packed)) 是 GCC 的扩展语法，含义：禁止编译器在成员之间插入填充字节。
// [解析] 不加它会怎样？float 要求 4 字节对齐，编译器会在 size 后面塞 1 个填充字节，
// [解析] 让 data 从偏移 4 开始 —— 那么 sizeof 从 75 变成 76，上位机按偏移 3 去读数据就全错了。
// [解析] ★代价（要记住）：data 数组从奇数偏移 3 开始，是非 4 字节对齐的。
// [解析]   本文件里都是"单个元素赋值"和"按字节交给串口发送"，不会触发未对齐访问问题；
// [解析]   但如果你以后想对它整体做 memcpy 到 float 数组、或用 DMA 之外的方式按字读取，
// [解析]   在部分编译器/优化等级下可能变慢甚至产生硬件异常，那时要改用 memcpy 逐字节拷贝。
struct __attribute__((packed)) PlotFrame
{
  uint8_t start[2] = {0xAA, 0xBB};  // [解析] 帧头两个字节，上位机靠它找到一帧的开头
  uint8_t size;                     // [解析] 后面 data 部分的字节数 = 4 * 有效 float 个数，取值 4~72
  float data[PLOTTER_FLOAT_NUM];    // [解析] 数据区，每个通道 4 字节（IEEE-754 单精度浮点）
};

// [解析] Plotter 类：一个对象对应一路串口（一般专门留一个串口给上位机，别和电机共用）。
// [解析] 无状态、不需要初始化，创建完就能直接 plot()。
class Plotter
{
public:
  // [解析] 构造函数。参数：
  // [解析]   huart  ：用哪一路串口输出，例如 &huart1（记得 CubeMX 里要配好波特率，
  // [解析]            上位机那边必须用同样的波特率，常见 115200 / 460800 / 921600）。
  // [解析]   use_dma：true = 用 DMA 发送（不占 CPU，推荐）；false = 阻塞式发送（会卡住 CPU）。
  // [解析]            默认 true，所以平时写 sp::Plotter plotter(&huart1); 就行。
  Plotter(UART_HandleTypeDef * huart, bool use_dma = true);

  // [解析] ↓ 从这里到下面的 18 个 plot() 是"函数重载"：函数名一样、参数个数不同。
  // [解析]   编译器根据你实际传了几个 float 自动选对应的版本，所以调用时
  // [解析]   既不用写通道数量，也不用构造数组：plotter.plot(a, b, c); 就画 3 条线。
  // [解析]   每个版本内部做的事情都一样：置 size = 4*参数个数 -> 依次写入 data[] -> send()。
  // [解析]   ★代价：18 个版本写起来很啰嗦（源文件 350 行基本都是这个），
  // [解析]     但换来的是调用处最简洁，这是嵌入式波形调试里常见的取舍。
  // [解析] 1 通道版：只画 1 条曲线，size 会填 4*1 = 4。
  void plot(float value1);
  // [解析] 2 通道版：size = 4*2 = 8。
  void plot(float value1, float value2);
  // [解析] 3 通道版：size = 4*3 = 12。
  void plot(float value1, float value2, float value3);
  // [解析] 4 通道版：size = 4*4 = 16。
  void plot(float value1, float value2, float value3, float value4);
  // [解析] 5 通道版：size = 4*5 = 20。
  void plot(float value1, float value2, float value3, float value4, float value5);
  // [解析] 6 通道版：size = 4*6 = 24。
  void plot(float value1, float value2, float value3, float value4, float value5, float value6);
  // [解析] 从 7 通道开始参数一行写不下，换行书写 —— 换行只是排版，
  // [解析] 对编译器而言和写成一整行完全一样。下面同理。
  // [解析] 7 通道版：size = 4*7 = 28。
  void plot(
    float value1, float value2, float value3, float value4, float value5, float value6,
    float value7);
  // [解析] 8 通道版：size = 4*8 = 32。
  void plot(
    float value1, float value2, float value3, float value4, float value5, float value6,
    float value7, float value8);
  // [解析] 9 通道版：size = 4*9 = 36。
  void plot(
    float value1, float value2, float value3, float value4, float value5, float value6,
    float value7, float value8, float value9);
  // [解析] 10 通道版：size = 4*10 = 40。
  void plot(
    float value1, float value2, float value3, float value4, float value5, float value6,
    float value7, float value8, float value9, float value10);
  // [解析] 11 通道版：size = 4*11 = 44。
  void plot(
    float value1, float value2, float value3, float value4, float value5, float value6,
    float value7, float value8, float value9, float value10, float value11);
  // [解析] 12 通道版：size = 4*12 = 48。
  void plot(
    float value1, float value2, float value3, float value4, float value5, float value6,
    float value7, float value8, float value9, float value10, float value11, float value12);
  // [解析] 13 通道版：size = 4*13 = 52。
  void plot(
    float value1, float value2, float value3, float value4, float value5, float value6,
    float value7, float value8, float value9, float value10, float value11, float value12,
    float value13);
  // [解析] 14 通道版：size = 4*14 = 56。
  void plot(
    float value1, float value2, float value3, float value4, float value5, float value6,
    float value7, float value8, float value9, float value10, float value11, float value12,
    float value13, float value14);
  // [解析] 15 通道版：size = 4*15 = 60。
  void plot(
    float value1, float value2, float value3, float value4, float value5, float value6,
    float value7, float value8, float value9, float value10, float value11, float value12,
    float value13, float value14, float value15);
  // [解析] 16 通道版：size = 4*16 = 64。
  void plot(
    float value1, float value2, float value3, float value4, float value5, float value6,
    float value7, float value8, float value9, float value10, float value11, float value12,
    float value13, float value14, float value15, float value16);
  // [解析] 17 通道版：size = 4*17 = 68。
  void plot(
    float value1, float value2, float value3, float value4, float value5, float value6,
    float value7, float value8, float value9, float value10, float value11, float value12,
    float value13, float value14, float value15, float value16, float value17);
  // [解析] 18 通道版：size = 4*18 = 72。这是上限，因为 data[] 只开了 18 个 float。
  // [解析] 想画更多通道必须回上面改 PLOTTER_FLOAT_NUM，再照葫芦画瓢加一个重载。
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
  // [解析] huart_：要发给哪一路串口。
  // [解析] use_dma_：记住构造时选的是 DMA 发送还是阻塞发送。
  // [解析] hal_status_：保存最近一次 HAL 发送函数的返回值（HAL_OK / HAL_BUSY / HAL_ERROR…）。
  // [解析]   ★注意：它只被赋值，类里没有任何函数去读它、也没有对外提供 getter，
  // [解析]   所以发送失败（典型是上一次 DMA 还没发完导致的 HAL_BUSY）会被静默忽略，
  // [解析]   画面上表现为"偶尔丢一帧"。调试波形时不必太在意；
  // [解析]   但如果你要在正式控制逻辑里依赖它，记得先加个接口把它暴露出来。
  // [解析] plot_frame_：唯一的待发数据缓冲区。它是成员变量而不是局部变量，
  // [解析]   因为 DMA 发送是"异步"的：函数返回后 DMA 还在慢慢搬数据，
  // [解析]   缓冲区必须一直有效，绝不能用局部数组。
  // [解析]   ★注意：反过来说，如果上一次 DMA 还没发完就又调用 plot()，
  // [解析]   HAL 会返回 HAL_BUSY 且这一帧发不出去（数据被覆盖但没发出），属于良性丢帧。
  // [解析] send()：把 plot_frame_ 按计算出的长度发给串口，私有函数，由各 plot() 调用。
};

}  // namespace sp

#endif  // SP__PLOTTER_HPP