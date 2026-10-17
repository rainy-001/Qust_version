// [解析] 线性跟踪微分器的实现：连续域为 dx1/dt = x2、dx2/dt = -r²(x1-v) - 2r·x2。
// [解析] 特征方程 (s+r)² = 0 是双重根 -r，即"临界阻尼"——跟踪快又不过冲的经典极点配置。
// [解析] 离散化用前向欧拉，所以 dt 必须等于真实调用周期；dt 越大近似误差越大。

#include "linear_differentiator.hpp"

namespace sp
{

LinearDifferentiator::LinearDifferentiator(float r, float dt)
: x1(0.0f), x2(0.0f), r_(r), dt_(dt), inited_(false)
{
}

// [解析] x1 直接跳到 v、x2 清 0：重置时还没有任何"变化趋势"信息，硬给非零 x2 只会造出虚假速度。
void LinearDifferentiator::init(float v)
{
  x1 = v;
  x2 = 0.0f;
  inited_ = true;
}

void LinearDifferentiator::update(float v)
{
  if (!inited_) {
    init(v);
    return;
  }

  // 离散化线性跟踪微分器差分方程
  // [解析] 两个新值都先算完再统一写回 x1/x2；若先写 x1 再算 x2_next，就变成半隐式欧拉，行为不一样。
  float x1_next = x1 + dt_ * x2;
  float x2_next = x2 + dt_ * (-r_ * r_ * (x1 - v) - 2.0f * r_ * x2);

  x1 = x1_next;
  x2 = x2_next;
}

}  // namespace sp