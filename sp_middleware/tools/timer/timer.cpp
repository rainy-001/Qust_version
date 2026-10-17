// [解析] Timer 的实现：直接读写 Cortex-M4 内核的 DWT 寄存器，用 CPU 周期数计时。
// [解析] 只在调试量耗时用（同目录 readme.md 有示例），主干代码默认不调用它。
// [解析] 三个坑：先开 DEM_CR 的 TRCENA 总开关、volatile 不能省、32 位计数器约 25 秒回绕一次。

#include "timer.hpp"

// [解析] 这 5 个宏把固定物理地址当成变量用：*(volatile uint32_t *)addr，读写就是访问寄存器。
// [解析] DWT 没有 HAL 接口，只能按手册地址访问；volatile 防止编译器把读取优化成只读一次。
// STM32 寄存器定义
#define DWT_CTRL (*(volatile uint32_t *)0xE0001000)    // DWT 控制寄存器
#define DWT_CYCCNT (*(volatile uint32_t *)0xE0001004)  // DWT 周期计数器
#define DEM_CR (*(volatile uint32_t *)0xE000EDFC)  // Debug Exception and Monitor Control Register
#define DEM_CR_TRCENA (1 << 24)                    // Trace enable bit
#define DWT_CTRL_CYCCNTENA (1 << 0)                // Cycle count enable bit

// STM32 HAL 提供的系统时钟变量（由 SystemClock_Config() 更新）
// [解析] HAL 的全局变量（由 C 编译），这里必须 extern "C"，否则 C++ 名字修饰会导致链接失败。
extern "C" uint32_t SystemCoreClock;

namespace sp
{

// 静态成员初始化
// [解析] static 成员要在 .cpp 里定义一次；全类共享，保证 DWT 只被配置一次。
bool Timer::initialized_ = false;

// [解析] min_delta_us_ 初值取 1e9f 这个哨兵大数，第一次 stop() 才刷得出最小值（写 0 会永远是 0）。
Timer::Timer(bool auto_init)
: start_tick_(0), stop_tick_(0), min_delta_us_(1e9f), max_delta_us_(0.0f), count_(0)
{
  if (auto_init && !initialized_) {
    init();
  }
}

// [解析] 依次打开 TRCENA 总开关、清零计数器、置 CYCCNTENA，最后回读确认；重复调用直接返回成功。
bool Timer::init()
{
  if (initialized_) {
    return true;
  }

  // 使能 DWT 外设
  DEM_CR |= DEM_CR_TRCENA;

  // 重置周期计数器
  DWT_CYCCNT = 0;

  // 使能周期计数器
  DWT_CTRL |= DWT_CTRL_CYCCNTENA;

  // 验证是否使能成功
  if (DWT_CTRL & DWT_CTRL_CYCCNTENA) {
    initialized_ = true;
    return true;
  }

  return false;
}

void Timer::start() { start_tick_ = DWT_CYCCNT; }

// [解析] 记结束时刻并刷新统计；start_tick_ 不清，之后还能重复读 delta_*()。
void Timer::stop()
{
  stop_tick_ = DWT_CYCCNT;

  // 更新统计
  float dt = delta_us();
  if (dt < min_delta_us_) min_delta_us_ = dt;
  if (dt > max_delta_us_) max_delta_us_ = dt;
  count_++;
}

// [解析] 无符号减法天然处理计数器回绕（模 2^32），前提是两次读数间隔不到 2^32 个周期（约 25 秒）。
float Timer::delta_us() const
{
  // 处理溢出（32位计数器会回绕）
  uint32_t ticks = stop_tick_ - start_tick_;
  return static_cast<float>(ticks) * 1000000.0f / static_cast<float>(SystemCoreClock);
}

float Timer::delta_ms() const { return delta_us() / 1000.0f; }

float Timer::delta_s() const { return delta_us() / 1000000.0f; }

uint32_t Timer::delta_tick() const { return stop_tick_ - start_tick_; }

// [解析] 开机以来的时间戳，同样约 25.6 秒回绕一次，只能拿两次读数相减算短间隔。
float Timer::now_us()
{
  return static_cast<float>(DWT_CYCCNT) * 1000000.0f / static_cast<float>(SystemCoreClock);
}

void Timer::reset_stats()
{
  min_delta_us_ = 1e9f;
  max_delta_us_ = 0.0f;
  count_ = 0;
}

}  // namespace sp
