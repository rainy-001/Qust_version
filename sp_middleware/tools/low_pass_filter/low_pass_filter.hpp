// [解析] 一阶低通滤波（指数加权平均）：out = alpha × 新值 + (1 - alpha) × 上次输出。
// [解析] 给抖动的传感器读数/速度反馈做平滑，用法是 filter.update(raw) 后读 filter.out。
// [解析] alpha 越大越信任新值，越小越平滑但越滞后（必须落在 [0,1]）；有记忆，每一路信号各用一个对象。

#ifndef SP__LOW_PASS_FILTER_HPP
#define SP__LOW_PASS_FILTER_HPP

namespace sp
{
class LowPassFilter
{
public:
  // alpha: 为1时, 无滤波效果; 为0时, 无更新效果
  LowPassFilter(float alpha);

  float out;  // 只读! update()的计算结果

  // [解析] 每拿到一个新采样调一次；第一次调用直接采用输入值，避免输出从 0 慢慢爬上来。
  void update(float value);

private:
  const float alpha_;
  bool inited_;
  float last_;
};

}  // namespace sp

#endif  // SP__LOW_PASS_FILTER_HPP
