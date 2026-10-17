// [解析] 四全向轮底盘（四个角各一个全向轮，滚动方向垂直于该角到底盘中心的连线）的正/反运动学解算器。
// [解析] 上层 applications/ 的底盘任务建对象后每周期调 calc() 下发四个轮速；update() 把轮速反馈换成整车速度做里程计。
// [解析] 最容易忽略的是 d_ = sqrt(l²+w²)（半对角线，既不是 l 也不是 w）；它同时充当"投影分母"和"自转半径"。
// [解析] 单位一律用米 / m/s / rad/s；下面原作者那段 ASCII 俯视图和向量推导是所有公式的出处，务必先读。

#ifndef SP__OMNI_HPP
#define SP__OMNI_HPP

#include "tools/math_tools/math_tools.hpp"

namespace sp
{
// 四全向轮底盘解算
// 布局: 四个全向轮位于底盘四个角, 每个轮子的滚动方向垂直于该角到底盘中心的连线(对角线)
// 正方形底盘时, 该滚动方向即为绕底盘中心的切线方向(与对角线垂直, 与坐标轴成 45°)
//
// 俯视图 (底盘坐标系: x 向前, y 向左, z 向上, 故逆时针为正)
//
//                  +y (左)
//                    ↑
//       LR  ╱        |        ╲  LF
//                    |
//   ─────────────────+──────────────────→ +x (前)
//                    |
//       RR  ╲        |        ╱  RF
//
// 图中 ╱ ╲ 为轮子滚动方向, 均垂直于所在角的对角线
// 各轮位置 p 与滚动方向 u (D = sqrt(half_length^2 + half_width^2)):
//   LF: p = (+l, +w), u = (-w, +l) / D
//   LR: p = (-l, +w), u = (-w, -l) / D
//   RF: p = (+l, -w), u = (+w, +l) / D
//   RR: p = (-l, -w), u = (+w, -l) / D
// (正方形底盘 l == w 时, 滚动方向与底盘坐标轴成 45°, 即常见的 X 型全向轮布局)
//
// 轮心速度为 v + wz × p, 轮子只能沿 u 滚动(垂直于 u 的方向由辊子自由滑动), 故
//   转速 speed = u · (v + wz × p) / r
// 默认(reverse 均为 false)时, 电机正转对应轮子沿逆时针切线方向滚动:
//   纯 vx > 0: LF/LR 反转, RF/RR 正转
//   纯 vy > 0: LF/RF 正转, LR/RR 反转
//   纯 wz > 0: 四轮同向正转
// 注意: 对角两轮(LF/RR, LR/RF)滚动方向相反, 故纯平移时左右两侧轮速反号, 与 mecanum 不同。
// [解析] 一个对象代表一辆四全向轮底盘：纯算数、不存状态，两个解算函数都不返回值，结果读下面的公共成员。
class Omni
{
public:
  // wheel_radius: 轮子半径, 单位: m
  // half_length: 前后轮距离的一半, 单位: m
  // half_width: 左右轮距离的一半, 单位: m
  // reverse_lf: left-front反向旋转
  // reverse_lr: left-rear反向旋转
  // reverse_rf: right-front反向旋转
  // reverse_rr: right-rear反向旋转
  // [解析] 四个 reverse 默认全 false（假定四轮装配方向一致），正好和 mecanum 的默认相反；装反了传 true，不要去改公式符号。
  Omni(
    float wheel_radius, float half_length, float half_width, bool reverse_lf = false,
    bool reverse_lr = false, bool reverse_rf = false, bool reverse_rr = false);

  // [解析] 单位是轮子自转角速度 rad/s，负值表示该轮反转，可直接作为电机 PID 的目标值。
  float speed_lf;  // 只读! calc()计算结果, left-front转速, 单位: rad/s
  float speed_lr;  // 只读! calc()计算结果, left-rear转速, 单位: rad/s
  float speed_rf;  // 只读! calc()计算结果, right-front转速, 单位: rad/s
  float speed_rr;  // 只读! calc()计算结果, right-rear转速, 单位: rad/s

  float vx;  // 只读! update()计算结果, 底盘x方向速度, 单位: m/s
  float vy;  // 只读! update()计算结果, 底盘y方向速度, 单位: m/s
  float wz;  // 只读! update()计算结果, 底盘z方向角速度, 单位: rad/s

  // [解析] 正解：把整车速度投影到各轮滚动方向 u 上，再叠加自转项 wz×p，最后 /r_ 转成角速度。
  // [解析] 形参 vx/vy/wz 与成员同名，函数体里裸写的是形参；本函数不限幅，指令过大需调用方自己限。
  // 底盘速度 -> 各轮转速
  // vx: 前进速度, 单位: m/s
  // vy: 左移速度, 单位: m/s
  // wz: 大拇指朝上，右手螺旋方向转速, 单位: rad/s
  void calc(float vx, float vy, float wz);

  // [解析] 反解：先 /sign_xx_、×r_ 把电机转速还原成标准约定下的轮子线速度，再按正交矩阵反解出整车速度。
  // [解析] 不含滤波也不做打滑补偿；vx 乘 d_/w_、vy 乘 d_/l_ 这两个系数都 ≥1，会放大轮速噪声。
  // 各轮转速 -> 底盘速度
  // speed_lf: left-front转速, 单位: rad/s
  // speed_lr: left-rear转速, 单位: rad/s
  // speed_rf: right-front转速, 单位: rad/s
  // speed_rr: right-rear转速, 单位: rad/s
  void update(float speed_lf, float speed_lr, float speed_rf, float speed_rr);

private:
  const float r_;
  const float l_;
  const float w_;
  // [解析] d_ 由勾股定理在构造时定死：平移分量里除以它做投影，自转分量里乘以它当旋转半径。
  const float d_;  // 轮心到底盘中心的距离, 即半对角线长度 sqrt(l^2 + w^2), 单位: m
  const float sign_lf_;
  const float sign_lr_;
  const float sign_rf_;
  const float sign_rr_;
};

}  // namespace sp

#endif  // SP__OMNI_HPP
