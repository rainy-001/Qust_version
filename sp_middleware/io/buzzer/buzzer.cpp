// [解析] ==========================================================================
// [解析] 【文件】sp_middleware/io/buzzer/buzzer.cpp
// [解析] 【作用】buzzer.hpp 的实现。start()/stop() 是开关，set() 是本次要重点看懂的函数：
// [解析]         它把『想要的发声频率』换算成定时器的预分频(PSC)、重装值(ARR)、比较值(CCR)。
// [解析] 【被谁用】applications/ 下需要提示音的地方，通过 Buzzer 对象调用。
// [解析] 【关键点】PWM 频率公式与 1MHz 基准的换算，见下面 set() 的逐行说明。
// [解析] ==========================================================================
#include "buzzer.hpp"

namespace sp
{
// [解析] 构造函数：只把三个参数搬进成员变量，函数体为空 —— 真正的硬件配置在 start() 和 set() 里做。
Buzzer::Buzzer(TIM_HandleTypeDef * htim, uint16_t channel, float clock_hz)
: htim_(htim), channel_(channel), clock_hz_(clock_hz)
{
}

void Buzzer::start()
{
  // [解析] ↑ 打开这个通道的 PWM 输出，从此引脚上就会持续输出波形（频率由 set() 决定）。
  // [解析]   dismiss return 是本项目的写法约定：故意不检查 HAL 返回值，看到它就知道不是漏写判断。
  HAL_TIM_PWM_Start(htim_, channel_);  // dismiss return
}

void Buzzer::stop()
{
// [解析] ↑ 关掉 PWM 输出，蜂鸣器停止发声；定时器本身仍在跑，再次 start() 即可恢复。
  HAL_TIM_PWM_Stop(htim_, channel_);  // dismiss return
}

void Buzzer::set(float hz, float duty)
{
  // ref: https://www.bilibili.com/video/BV1th411z7sn?t=1393.4&p=15
  // [解析] ↑ 这一步把定时器计数时钟统一降到 1MHz（也就是计数一次 = 1 微秒），
  // [解析]   这样后面算 ARR(周期) 和 CCR(脉宽) 时，数字就正好等于『微秒数』，非常好心算。
  // [解析]   原理：计数频率 = 定时器输入时钟 / (PSC + 1)，要让结果等于 1MHz，
  // [解析]   就要 PSC + 1 = clock_hz / 1e6，所以 PSC = clock_hz/1e6 - 1。
  // [解析]   例：clock_hz = 84MHz 时 PSC = 84 - 1 = 83，84MHz/84 = 1MHz，符合预期。
  // [解析]   注意：PSC 寄存器是 16 位，clock_hz/1e6 - 1 超过 65535 就会溢出；
  // [解析]   而且这个式子是浮点算完隐式转成整数（直接截断），clock_hz 不是 1MHz 整数倍时会有误差。
  __HAL_TIM_SET_PRESCALER(htim_, clock_hz_ / 1e6 - 1);  // 预分频到1MHz
  // [解析] ↑ ARR 决定 PWM 的周期（也就是频率）：计数从 0 数到 ARR 翻一轮，所以一轮有 ARR+1 个计数。
  // [解析]   计数时钟 = clock_hz / (PSC + 1)（上面刚设成 1MHz），
  // [解析]   于是 频率 = 计数时钟 / (ARR + 1)，反解出 ARR = 计数时钟 / 目标频率 - 1。
  // [解析]   这里直接读 htim_->Instance->PSC 而不是用变量，是为了拿到『刚写进去的那个 PSC』。
  // [解析]   例：1MHz 计数、想发 1000Hz，则 ARR = 1000000/1000 - 1 = 999。
  // [解析]   注意：hz 传 0 会除以 0（浮点除零得到 inf，再转整数是未定义行为），调用者必须给正数频率；
  // [解析]   频率太低（如几 Hz）时 ARR 会超过 16 位上限 65535，输出频率就不准了。
  __HAL_TIM_SET_AUTORELOAD(htim_, clock_hz_ / (htim_->Instance->PSC + 1) / hz - 1);
  // [解析] ↑ CCR（比较值）决定占空比：计数器小于 CCR 时输出高电平，
  // [解析]   所以 占空比 = CCR / (ARR + 1)，反解出 CCR = (ARR + 1) × duty。
  // [解析]   例：ARR = 999（一轮 1000 个计数）、duty = 0.5，则 CCR = 500，正好一半时间高电平。
  // [解析]   占空比 0.5 时蜂鸣器得到的交流能量最大，所以听起来最响。
  __HAL_TIM_SET_COMPARE(htim_, channel_, (htim_->Instance->ARR + 1) * duty);
  // [解析] ↑ 把计数器清零，让新的频率/占空比立刻从这个时间点重新开始数。
  // [解析]   如果不清零，当前这一轮可能已经数到一半，会有一小段音调不准的过渡。
  __HAL_TIM_SET_COUNTER(htim_, 0);
}

}  // namespace sp
