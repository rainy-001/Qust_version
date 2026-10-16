// [解析] ==========================================================================
// [解析] 【文件】sp_middleware/tools/timer/timer.hpp
// [解析] 【作用】用 CPU 内核自带的 DWT 硬件计数器做"微秒级秒表"，用来量某段代码跑多久，
// [解析]         也可以当作一个"开机以来的高精度时间戳"用（now_us()）。
// [解析] 【被谁用】性能调试时使用，用法见同目录 readme.md 的示例（sp::Timer timer;）。
// [解析]         目前主干代码里没有默认调用它，属于"想量耗时就自己加"的工具类。
// [解析] 【关键点】1) 精度 = 1/SystemCoreClock 秒，168MHz 主频下约 6ns；
// [解析]   2) 32 位计数器约 25 秒就会回绕一次（算差值时的减法天然能处理一次回绕）；
// [解析]   3) DWT 是全芯片共享的资源，初始化一次就够了，所以有 static 的 initialized_。
// [解析] ==========================================================================

#ifndef SP__TIMER_HPP
#define SP__TIMER_HPP

#include <cstdint>

namespace sp
{

/**
 * @brief 微秒级计时器类（基于 DWT 硬件计数器）
 * @note  用于测量代码执行时间，精度 = 1/SystemCoreClock 秒
 *
 * 使用示例：
 * @code
 *   sp::Timer timer;
 *   timer.start();
 *   // ... 待测代码 ...
 *   timer.stop();
 *   float dt_us = timer.delta_us();      // 获取耗时（微秒）
 *   float dt_ms = timer.delta_ms();      // 获取耗时（毫秒）
 * @endcode
 */
// [解析] 【什么是 DWT】Data Watchpoint and Trace，ARM Cortex-M3/M4 内核里的调试部件。
// [解析]   它有一个 32 位寄存器 DWT_CYCCNT，每个 CPU 时钟周期自动 +1，等于一个"免费的高精度计时器"。
// [解析] 【典型用法】start() 记开始、stop() 记结束、delta_xx() 读耗时，三步走。
// [解析] 【统计功能】同一个对象反复 start/stop 时，会自动记下最小/最大值和次数，
// [解析]   适合观察"控制循环最长跑了多久"（用来判断有没有超时风险）。
// [解析] 【注意】start/stop 本身只读寄存器，开销极小；但不要把它放在比 CPU 主频还高的场合。
class Timer
{
public:
  /**
   * @brief 构造函数
   * @param auto_init 是否自动初始化 DWT（首次使用需要初始化）
   */
  // [解析] auto_init 默认 true：第一次 new 出 Timer 对象时顺手把 DWT 打开，省得手动调 init()。
  // [解析] 如果你要严格控制"什么时候碰寄存器"，可以传 false 然后自己调 Timer::init()。
  // [解析] 注意：初始化只需要做一次（内部有 static 标志），构造 100 个对象也不会重复初始化。
  Timer(bool auto_init = true);

  /**
   * @brief 初始化 DWT 计数器（全局只需调用一次）
   * @return true 初始化成功，false 失败（硬件不支持）
   */
  // [解析] 真正去操作 DWT 寄存器的函数（详见 timer.cpp）：先给调试模块"总开关"上电，
  // [解析] 清零计数器，再打开 CYCCNT 使能位，最后回读寄存器确认真的开了。
  // [解析] 返回 true = 计数器可用；false = 使能位没置上（一般是调试模块被关掉了）。
  // [解析] 因为 DWT 是全芯片共享的，所以它是 static 函数，通常不用自己调（构造函数会调）。
  static bool init();

  /**
   * @brief 开始计时（记录起始时刻）
   */
  // [解析] 把当前 DWT_CYCCNT 记进 start_tick_，就一个赋值，开销约几十纳秒。
  // [解析] 注意：它不检查是否已经 start 过，重复调用等于"重新开始计时"。
  void start();

  /**
   * @brief 停止计时（记录结束时刻）
   */
  // [解析] 把当前 DWT_CYCCNT 记进 stop_tick_，顺便更新统计（最小/最大耗时、测量次数）。
  // [解析] 有了它，delta_us() 等函数才有意义（它们是拿 stop_tick_ - start_tick_ 算的）。
  // [解析] 注意：start() 和 stop() 必须成对；只 start 不 stop 的话 delta 读到的是旧值。
  void stop();

  /**
   * @brief 获取时间间隔（微秒）
   * @return float 时间间隔，单位：us
   */
  // [解析] 返回本次 start→stop 的耗时，单位微秒（us）。1 us = 1e-6 s。
  // [解析] 计算方式：tick 差值 ÷ SystemCoreClock × 1e6（见 timer.cpp）。
  // [解析] 因为是 float，大数值下有效位数有限；量几毫秒的代码完全够用。
  float delta_us() const;

  /**
   * @brief 获取时间间隔（毫秒）
   * @return float 时间间隔，单位：ms
   */
  // [解析] 就是 delta_us() / 1000，方便看"这一帧控制用了 0.8 ms"这种量级。
  float delta_ms() const;

  /**
   * @brief 获取时间间隔（秒）
   * @return float 时间间隔，单位：s
   */
  // [解析] 就是 delta_us() / 1000000；注意别把它和 dt（控制周期）搞混。
  float delta_s() const;

  /**
   * @brief 获取原始 tick 差值
   * @return uint32_t tick 数
   */
  // [解析] 不做任何换算，直接返回 stop_tick_ - start_tick_（多少个 CPU 周期）。
  // [解析] 好处是没有浮点误差；1 tick = 1/SystemCoreClock 秒 ≈ 5.95ns @168MHz。
  uint32_t delta_tick() const;

  /**
   * @brief 获取当前时刻（微秒，从系统启动开始）
   * @return float 当前时刻，单位：us
   */
  // [解析] 直接读 DWT_CYCCNT 换算成微秒，得到"开机到现在过了多久"的时间戳，不需要 start/stop。
  // [解析] 它是 static 的，所以可以直接写 sp::Timer::now_us() 而不用先建对象。
  // [解析] 注意：32 位计数器 @168MHz 约 25.6 秒就回绕归零（见 readme.md），
  // [解析]   所以这个"时刻"不能当长时间基准，只能用来算短时间间隔（用两次读数相减）。
  static float now_us();

  // 统计功能
  // [解析] ↓ 三个只在 stop() 里被更新的统计量，用来观察"最坏情况"：
  // [解析]   min_us / max_us 是历次测量的最小/最大耗时（单位 us），count 是测量次数。
  // [解析]   判断控制循环会不会超时，看 max_us 比看平均值更有意义。
  float min_us() const { return min_delta_us_; }  ///< 最小耗时
  float max_us() const { return max_delta_us_; }  ///< 最大耗时
  uint32_t count() const { return count_; }       ///< 测量次数
  void reset_stats();                             ///< 重置统计

private:
  // [解析] 下面这些是"每个对象各有一份"的状态（注意它们不是 static）。
  uint32_t start_tick_;  ///< 起始 tick
  uint32_t stop_tick_;   ///< 结束 tick
  float min_delta_us_;   ///< 最小耗时（微秒）
  float max_delta_us_;   ///< 最大耗时（微秒）
  uint32_t count_;       ///< 测量次数

  // [解析] 注意这里有 static：它属于"整个类"而不是某个对象，所以所有 Timer 对象共享同一个值，
  // [解析]   保证 DWT 只会被初始化一次（在 timer.cpp 里定义并初值为 false）。
  static bool initialized_;  ///< DWT 是否已初始化
};

}  // namespace sp

#endif  // SP__TIMER_HPP
