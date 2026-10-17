// [解析] 微秒级计时器：基于内核 DWT 的 32 位周期计数器，用来量某段代码跑了多久。
// [解析] 主干代码没有默认调用，需要时自己建对象：start -> stop -> delta_us()。
// [解析] 精度 1/SystemCoreClock（168MHz 约 6ns），但计数器约 25.6 秒回绕一次，只能算短间隔。

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
// [解析] DWT 是全芯片共享资源，初始化一次就够，所以 init() 和 initialized_ 都是 static。
// [解析] 同一对象反复 start/stop 会自动统计最小/最大耗时与次数，看最坏情况比看平均值更有用。
class Timer
{
public:
  /**
   * @brief 构造函数
   * @param auto_init 是否自动初始化 DWT（首次使用需要初始化）
   */
  // [解析] auto_init 默认 true，构造时顺手打开 DWT；内部有 static 标志，构造多少个都只初始化一次。
  Timer(bool auto_init = true);

  /**
   * @brief 初始化 DWT 计数器（全局只需调用一次）
   * @return true 初始化成功，false 失败（硬件不支持）
   */
  // [解析] 真正操作寄存器的函数，一般不用手动调；返回 false 说明使能位置不上。
  static bool init();

  /**
   * @brief 开始计时（记录起始时刻）
   */
  void start();

  /**
   * @brief 停止计时（记录结束时刻）
   */
  // [解析] 记结束时刻并刷新统计（最小/最大耗时、次数）；start/stop 要成对使用。
  void stop();

  /**
   * @brief 获取时间间隔（微秒）
   * @return float 时间间隔，单位：us
   */
  // [解析] 耗时的基础量，其余 delta_* 都由它换算；float 有效位数有限，量毫秒级代码够用。
  float delta_us() const;

  /**
   * @brief 获取时间间隔（毫秒）
   * @return float 时间间隔，单位：ms
   */
  // [解析] delta_us() / 1000，方便读"这一帧 0.8 ms"这种量级。
  float delta_ms() const;

  /**
   * @brief 获取时间间隔（秒）
   * @return float 时间间隔，单位：s
   */
  float delta_s() const;

  /**
   * @brief 获取原始 tick 差值
   * @return uint32_t tick 数
   */
  // [解析] 不换算成时间，直接给原始周期差，没有浮点误差。
  uint32_t delta_tick() const;

  /**
   * @brief 获取当前时刻（微秒，从系统启动开始）
   * @return float 当前时刻，单位：us
   */
  // [解析] 不用 start/stop，直接读计数器当"开机至今"的时间戳；会周期回绕，别当日期用。
  static float now_us();

  // [解析] min/max 是历次测量的极值（us），count 是次数；reset_stats() 会恢复哨兵初值。
  // 统计功能
  float min_us() const { return min_delta_us_; }  ///< 最小耗时
  float max_us() const { return max_delta_us_; }  ///< 最大耗时
  uint32_t count() const { return count_; }       ///< 测量次数
  void reset_stats();                             ///< 重置统计

private:
  uint32_t start_tick_;  ///< 起始 tick
  uint32_t stop_tick_;   ///< 结束 tick
  float min_delta_us_;   ///< 最小耗时（微秒）
  float max_delta_us_;   ///< 最大耗时（微秒）
  uint32_t count_;       ///< 测量次数

  static bool initialized_;  ///< DWT 是否已初始化
};

}  // namespace sp

#endif  // SP__TIMER_HPP
