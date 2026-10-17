// [解析] 一阶低通滤波的实现，核心只有一句加权平均：out = alpha × 新值 + (1 - alpha) × 上次输出。
// [解析] 它是一阶 IIR：噪声被摊平，代价是输出滞后；alpha 与截止频率约满足 fc ≈ alpha / (2π·dt)。
// [解析] 第一次 update() 走特例，直接把输入抄给 out —— 否则真实值恒定 100 时要爬十几拍才跟上去。

#include "low_pass_filter.hpp"

namespace sp
{
LowPassFilter::LowPassFilter(float alpha) : alpha_(alpha), inited_(false) {}

void LowPassFilter::update(float value)
{
  if (!inited_) {
    inited_ = true;
    last_ = value;
    this->out = value;
    return;
  }

  this->out = alpha_ * value + (1 - alpha_) * last_;
  last_ = this->out;
}

}  // namespace sp
