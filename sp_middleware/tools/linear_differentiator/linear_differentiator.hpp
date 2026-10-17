// [解析] 线性跟踪微分器：输入带噪声的信号 v，同时输出 x1（平滑值）和 x2（微分/变化率）。
// [解析] 内部是二阶低通观测器，求导与平滑一步完成，比"先低通再差分"滞后更小、噪声更干净。
// [解析] r 是唯一的调参旋钮（越大跟得越快、噪声放大越多），dt 必须等于真实调用周期。

#ifndef SP__LINEAR_DIFFERENTIATOR_HPP
#define SP__LINEAR_DIFFERENTIATOR_HPP

namespace sp
{
// [解析] 有记忆（内部保留 x1/x2），每一路需要微分的信号各用一个对象；x2 的单位是"输入单位/秒"。
class LinearDifferentiator
{
public:
  /*
   * @param r 调节带宽参数，值越大跟踪越快，但对噪声的放大也越大
   * @param dt 积分步长，通常为系统的离散时间间隔（如 0.001s）
   */
  LinearDifferentiator(float r, float dt);

  float x1;  // 跟踪的平滑值
  float x2;  // 微分值（速度）

  /*
   * @param v 观测目标输入值
   */
  // [解析] 每周期喂一个新的原始采样；若从未 init() 过，第一拍只做初始化并返回，x2 仍为 0。
  void update(float v);

  /*
   * @brief 重置内部状态
   */
  // [解析] 手动指定起始跟踪值 v（一般填当前实际测量值），并把微分清零、标记已初始化。
  void init(float v);

private:
  float r_;
  float dt_;
  bool inited_;
};
}  // namespace sp

#endif  // SP__LINEAR_DIFFERENTIATOR_HPP
