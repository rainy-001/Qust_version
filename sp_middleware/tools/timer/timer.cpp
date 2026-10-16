// [解析] ==========================================================================
// [解析] 【文件】sp_middleware/tools/timer/timer.cpp
// [解析] 【作用】timer.hpp 的实现：直接读写 ARM Cortex-M4 内核的 DWT 寄存器来数 CPU 周期数。
// [解析] 【被谁用】想量代码耗时的调试代码（用法见同目录 readme.md）。
// [解析] 【看代码的重点】1) 那几个 0xE000xxxx 地址是什么、为什么不用 HAL 库函数；
// [解析]   2) volatile 关键字在这里为什么不能少；
// [解析]   3) delta_us() 里一个减法就顺手处理了计数器溢出（回绕），这是本文件最巧的一处；
// [解析]   4) static 成员 initialized_ 保证全局只初始化一次。
// [解析] ==========================================================================

#include "timer.hpp"

// STM32 寄存器定义
// [解析] 下面 5 个宏是"把某个固定内存地址当成变量来用"的经典写法，拆开看有三层：
// [解析]   (volatile uint32_t *)0xE0001000 —— 把这个物理地址强制当成"指向 32 位无符号数的指针"
// [解析]   *(...)                        —— 解引用，于是 DWT_CTRL 这个"变量"其实就是那个寄存器
// [解析]   往 DWT_CTRL 赋值 = 往硬件寄存器写；读 DWT_CTRL = 读硬件寄存器。
// [解析] volatile 的作用：告诉编译器"这个值随时可能被硬件改掉"，禁止它做"读一次缓存起来用"的优化，
// [解析]   否则编译器可能把循环里的寄存器读取优化成只读一次，计时就永远是 0 了。
// [解析] 为什么不用 HAL 库？因为 HAL 没有提供 DWT 的接口，只能自己按《Cortex-M4 技术手册》的
// [解析]   固定地址访问。这些地址是 ARM 规定的内核外设地址，所有 Cortex-M3/M4 芯片都一样。
#define DWT_CTRL (*(volatile uint32_t *)0xE0001000)    // DWT 控制寄存器
// [解析] ↑ 地址 0xE0001000，bit0 = CYCCNTENA（周期计数器使能）。
#define DWT_CYCCNT (*(volatile uint32_t *)0xE0001004)  // DWT 周期计数器
// [解析] ↑ 地址 0xE0001004，32 位自由运行的计数器，每个 CPU 时钟周期 +1（168MHz 下约 6ns 一跳）。
#define DEM_CR (*(volatile uint32_t *)0xE000EDFC)  // Debug Exception and Monitor Control Register
// [解析] ↑ 地址 0xE000EDFC，调试异常与监控控制寄存器。它是 DWT 的"上级电源开关"：
// [解析]   不先打开它的 TRCENA 位，DWT 的计数器根本不会走（这是新手最容易漏的一步）。
#define DEM_CR_TRCENA (1 << 24)                    // Trace enable bit
// [解析] ↑ 1 << 24 = 0x01000000，只把 bit24 置 1，其余位保持不动（配合 |= 使用）。
#define DWT_CTRL_CYCCNTENA (1 << 0)                // Cycle count enable bit
// [解析] ↑ 1 << 0 = 0x00000001，DWT_CTRL 的最低位，置 1 后 CYCCNT 才开始计数。

// STM32 HAL 提供的系统时钟变量（由 SystemClock_Config() 更新）
// [解析] SystemCoreClock 是 HAL 里的全局变量，值就是 CPU 主频（本车一般是 168000000）。
// [解析] 它是 C 语言编译的，而本文件是 C++，所以要用 extern "C" 声明，
// [解析] 否则 C++ 的名字修饰（name mangling）会让链接器找不到这个符号。
extern "C" uint32_t SystemCoreClock;

namespace sp
{

// 静态成员初始化
// [解析] static 成员变量必须在 .cpp 里这样"定义"一次（.hpp 里只是声明），否则链接会报"未定义符号"。
// [解析] 初值 false = "DWT 还没初始化"。它是全类共享的，所以不管创建多少个 Timer 对象，
// [解析] 这个标志只会从 false 变成 true 一次。
bool Timer::initialized_ = false;

// [解析] 构造函数：初始化列表里把每个成员都赋上合理的初值。
// [解析]   start_tick_/stop_tick_ 归 0；count_ = 0（还没测过）。
// [解析]   min_delta_us_ 故意设成 1e9f（十亿微秒，一个不可能达到的大数），
// [解析]     这样第一次 stop() 时 "dt < min_delta_us_" 一定成立，最小值会被正确更新；
// [解析]     如果初值写 0，最小值就永远是 0 了 —— 这是求最小值时的常见技巧。
// [解析]   max_delta_us_ 反过来初值取 0，保证第一次一定能更新最大值。
Timer::Timer(bool auto_init)
: start_tick_(0), stop_tick_(0), min_delta_us_(1e9f), max_delta_us_(0.0f), count_(0)
{
  // [解析] auto_init 为 true 且"全局还没初始化过"时才真正去动寄存器。
  // [解析] 双重条件缺一不可：前者听调用者的，后者保证 DWT 只被配置一次。
  if (auto_init && !initialized_) {
    init();
  }
}

bool Timer::init()
{
  // [解析] 已经初始化过就直接返回成功，重复初始化会把计数器清零、打断正在进行的测量。
  if (initialized_) {
    return true;
  }

  // 使能 DWT 外设
  // [解析] 第一步（最容易漏）：打开调试模块的总开关 TRCENA。
  // [解析] |= 是"只把 bit24 置 1，不影响寄存器里的其他位"，不能用 = 覆盖，
  // [解析] 因为 DEM_CR 里还管着别的调试功能，覆盖会顺手把它们关掉。
  DEM_CR |= DEM_CR_TRCENA;

  // 重置周期计数器
  // [解析] 把计数器清零，让本次上电的计时从 0 开始（不清也行，只是 now_us() 的起点会变）。
  DWT_CYCCNT = 0;

  // 使能周期计数器
  // [解析] 打开 DWT_CTRL 的最低位，从此 CYCCNT 每个 CPU 周期自动 +1。
  DWT_CTRL |= DWT_CTRL_CYCCNTENA;

  // 验证是否使能成功
  // [解析] 回读寄存器确认 bit0 真的是 1 —— 有些调试器/低功耗状态下写进去也不生效，
  // [解析] 所以不能想当然地直接 return true，要读回来验证（硬件编程的好习惯）。
  if (DWT_CTRL & DWT_CTRL_CYCCNTENA) {
    initialized_ = true;
    return true;
  }

  // [解析] 走到这里说明使能位没置上，DWT 不可用，返回失败。
  // [解析] 注意：此时 initialized_ 仍是 false，下次构造对象还会再试一次。
  return false;
}

// [解析] 开始计时：就是把"此刻的计数器读数"存下来。
// [解析] 写成一行是因为它真的只有一条赋值，开销极小（读一个寄存器 + 写一个变量）。
void Timer::start() { start_tick_ = DWT_CYCCNT; }

void Timer::stop()
{
  // [解析] 记下结束时刻。注意 start_tick_ 并不会被清掉，
  // [解析] 所以 stop() 之后想再读一次 delta_us() 还是能读到同样的结果。
  stop_tick_ = DWT_CYCCNT;

  // 更新统计
  // [解析] 这里调的是 delta_us()，它用的是"刚更新的 stop_tick_"减去"上一次 start 的时刻"，
  // [解析] 所以统计的正是本次 start→stop 的耗时。
  // [解析] 下面两个 if 没有花括号（单语句可省略），分别是"刷新最小值/最大值"。
  float dt = delta_us();
  if (dt < min_delta_us_) min_delta_us_ = dt;
  if (dt > max_delta_us_) max_delta_us_ = dt;
  // [解析] 测量次数 +1，配合 min_us()/max_us() 可以算出平均值。
  count_++;
}

float Timer::delta_us() const
{
  // 处理溢出（32位计数器会回绕）
  // [解析] ★ 本文件最巧妙的一行：32 位计数器加到 4294967295 会回绕成 0，
  // [解析]   如果只做 start_tick_ - stop_tick_ 的对齐减法，结果会是负数（错得很离谱）。
  // [解析]   但这里用的是"无符号数的减法"：uint32_t 溢出在 C/C++ 里是良定义的模 2^32 运算，
  // [解析]   所以哪怕中途回绕过一次，stop_tick_ - start_tick_ 依然等于真实经过的周期数。
  // [解析] 前提：两次读数之间间隔不到 2^32 个周期（168MHz 下约 25.6 秒），
  // [解析]   量代码耗时绝不会超过这个尺度，所以完全够用。
  uint32_t ticks = stop_tick_ - start_tick_;
  // [解析] 换算成微秒：周期数 ÷ 主频(Hz) = 秒数，再 × 1000000 = 微秒。
  // [解析] 这里特意先乘 1e6 再除，是为了减少浮点误差（乘除顺序会影响最后几位小数）。
  // [解析] static_cast<float> 把整型转成浮点，避免整数除法把小数部分直接砍掉变成 0。
  return static_cast<float>(ticks) * 1000000.0f / static_cast<float>(SystemCoreClock);
}

// [解析] 微秒 → 毫秒：1 ms = 1000 us，所以除以 1000。
float Timer::delta_ms() const { return delta_us() / 1000.0f; }

// [解析] 微秒 → 秒：1 s = 1000000 us。
float Timer::delta_s() const { return delta_us() / 1000000.0f; }

// [解析] 直接给出原始周期差，不做任何浮点换算（没有精度损失）。
// [解析] 想自己手工换算的话：时间(秒) = delta_tick() / SystemCoreClock。
uint32_t Timer::delta_tick() const { return stop_tick_ - start_tick_; }

float Timer::now_us()
{
  // [解析] 和 delta_us() 的公式一样，只是分子换成"当前计数器读数"而不是差值，
  // [解析] 于是得到"开机到现在的微秒数"这个时间戳，可以当作一个高精度时钟用。
  // [解析] 注意：计数器约 25.6 秒回绕一次，所以这个时间戳会周期性地"跳回 0"，
  // [解析]   只能拿它算短时间间隔（两次 now_us() 相减），不能当日期用。
  return static_cast<float>(DWT_CYCCNT) * 1000000.0f / static_cast<float>(SystemCoreClock);
}

void Timer::reset_stats()
{
  // [解析] 注意这里把 min_delta_us_ 恢复成 1e9f 这个"哨兵值"，
  // [解析] 而不是清成 0 —— 原因和构造函数里一样：0 会让"求最小值"永远失效。
  min_delta_us_ = 1e9f;
  max_delta_us_ = 0.0f;
  count_ = 0;
}

}  // namespace sp
