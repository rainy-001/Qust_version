// [解析] ==========================================================================
// [解析] 【文件】sp_middleware/tools/omni/omni.hpp
// [解析] 【作用】"四全向轮"底盘（四个角各一个全向轮）的运动学解算器。
// [解析]         正解 calc()：整车速度 (vx, vy, wz) -> 四个轮子该转多快。
// [解析]         反解 update()：四个轮子实际转速 -> 整车速度（做里程计用）。
// [解析] 【被谁用】本仓库里暂时没有调用者，属于中间件工具库（和 mecanum 二选一）。
// [解析]         上层 applications/ 的底盘任务自己建对象，例如：
// [解析]           sp::Omni omni(0.06f, 0.2f, 0.2f);
// [解析]           omni.calc(vx, vy, wz);
// [解析] 【和 mecanum 的区别】麦轮靠辊子 45° 斜置实现横移，四个轮子分两组；
// [解析]         本文件的全向轮把轮子装在四个角、滚动方向垂直于"角到中心"的连线，
// [解析]         纯平移时左右两侧轮速反号。用错模型会导致车"斜着走"或"原地打转"。
// [解析] 【关键点】1) 成员 d_ 是"半对角线长度" sqrt(l² + w²)，不是 l 也不是 w；
// [解析]           2) 下面那段原作者写的 ASCII 俯视图和向量推导一定要看，
// [解析]              它是本文件所有公式的出处；
// [解析]           3) 单位：长度 m，线速度 m/s，角速度 rad/s，轮速 rad/s。
// [解析] 【阅读顺序】先读下面原作者的推导注释 -> 再看 calc()/update() 的实现（omni.cpp）。
// [解析] ==========================================================================

#ifndef SP__OMNI_HPP
#define SP__OMNI_HPP

// [解析] 引用同工程的数学工具头文件（SP_PI、限幅、角度展开等）。
// [解析] 本类实际只用到加减乘除，真正用到 sqrt 的是 omni.cpp 里的 <cmath>。
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
// [解析] ---- 上面这段是原作者写的推导，下面是给新手补的"读法说明" ----
// [解析] 怎么读这张俯视图：
// [解析]   +x 指向车头前方，+y 指向车身左侧，+z 竖直向上；
// [解析]   这个坐标系是"右手系"，所以绕 +z 的正方向转动 = 从上往下看的逆时针。
// [解析] 为什么"滚动方向 u 垂直于对角线"？
// [解析]   全向轮（omni wheel）轱辘上有一圈小辊子，轮子只能沿一个方向产生驱动力，
// [解析]   垂直于该方向的运动由小辊子自由滚动"放行"。把轮子装在四个角、让其驱动
// [解析]   方向垂直于对角线，四个轮子的驱动方向就张成了平面内一组能覆盖任意平移
// [解析]   方向 + 自转的三个自由度，所以底盘可以做到"任意方向平移 + 原地自转"。
// [解析] 核心公式一句话：轮子只能沿 u 滚动，所以"轮子转多快"取决于
// [解析]   "轮心速度 v + wz × p 在 u 方向上的投影" —— 这就是 calc() 里那四行的全部来源。
// [解析] p 是轮心相对底盘中心的位置向量，wz × p 是自转给轮心带来的附加线速度。
// [解析] 单位提醒：下面所有长度都是 m，所有速度都是 m/s 或 rad/s；
// [解析]   构造参数传错单位（例如把 mm 当 m）会让车"跑得比预期快 1000 倍"。
class Omni
{
public:
  // [解析] 构造函数：只登记尺寸和装配方向，不做计算、不碰硬件。
  // [解析] 典型取值（以 RM 步兵为例）：wheel_radius ≈ 0.06 m（60mm 半径，
  // [解析]   注意是半径不是直径），half_length / half_width ≈ 0.2 m。
  // [解析] 单位一律是"米"，因为速度单位取的是 m/s，两者必须配套。
  // wheel_radius: 轮子半径, 单位: m
  // half_length: 前后轮距离的一半, 单位: m
  // half_width: 左右轮距离的一半, 单位: m
  // reverse_lf: left-front反向旋转
  // reverse_lr: left-rear反向旋转
  // reverse_rf: right-front反向旋转
  // reverse_rr: right-rear反向旋转
  // [解析] 和 mecanum 不同，本类四个 reverse 默认全是 false，即假定四个电机装配方向一致。
  // [解析] 装反了哪一个就把对应参数传 true；不要改公式里的符号，否则正反解会同时对不上。
  Omni(
    float wheel_radius, float half_length, float half_width, bool reverse_lf = false,
    bool reverse_lr = false, bool reverse_rf = false, bool reverse_rr = false);

  // [解析] ---- 输出区 1：calc() 的结果（整车速度 -> 轮速，给电机 PID 当目标值）----
  // [解析] 单位 rad/s（轮子自转的角速度），负值表示反转。
  float speed_lf;  // 只读! calc()计算结果, left-front转速, 单位: rad/s
  float speed_lr;  // 只读! calc()计算结果, left-rear转速, 单位: rad/s
  float speed_rf;  // 只读! calc()计算结果, right-front转速, 单位: rad/s
  float speed_rr;  // 只读! calc()计算结果, right-rear转速, 单位: rad/s

  // [解析] ---- 输出区 2：update() 的结果（轮速 -> 整车速度，里程计用）----
  float vx;  // 只读! update()计算结果, 底盘x方向速度, 单位: m/s
  float vy;  // 只读! update()计算结果, 底盘y方向速度, 单位: m/s
  float wz;  // 只读! update()计算结果, 底盘z方向角速度, 单位: rad/s

  // [解析] calc()：正运动学，把期望的整车速度分配到四个轮子。
  // [解析] 调用时机：每个控制周期调一次（一般 1ms 或 5ms），
  // [解析]   紧跟其后把 speed_lf/lr/rf/rr 发给对应电机的 PID。
  // [解析] 这三个输入都是"期望值"：vx/vy 单位 m/s，wz 单位 rad/s（不是角度！）。
  // 底盘速度 -> 各轮转速
  // vx: 前进速度, 单位: m/s
  // vy: 左移速度, 单位: m/s
  // wz: 大拇指朝上，右手螺旋方向转速, 单位: rad/s
  void calc(float vx, float vy, float wz);

  // [解析] update()：反运动学，把四个轮子的实测转速还原成整车速度。
  // [解析] 调用时机：拿到电机反馈转速之后（控制任务里、或 CAN 回调里）。
  // [解析] 结果不含滤波：轮速实测值抖动会直接反映到 vx/vy/wz 上，需要平滑请自己再套低通。
  // 各轮转速 -> 底盘速度
  // speed_lf: left-front转速, 单位: rad/s
  // speed_lr: left-rear转速, 单位: rad/s
  // speed_rf: right-front转速, 单位: rad/s
  // speed_rr: right-rear转速, 单位: rad/s
  void update(float speed_lf, float speed_lr, float speed_rf, float speed_rr);

private:
  // [解析] r_：轮子半径（m）。用来在"线速度 m/s"和"角速度 rad/s"之间换算。
  // [解析] l_：半轴距（m），前后轮中心距的一半。w_：半轮距（m），左右轮中心距的一半。
  const float r_;
  const float l_;
  const float w_;
  const float d_;  // 轮心到底盘中心的距离, 即半对角线长度 sqrt(l^2 + w^2), 单位: m
  // [解析] ↑ d_ 是本文件最容易被忽略的量：它是"轮心到中心的距离"，
  // [解析]   由勾股定理算出来，构造时就固定了，不会随运动变化。
  // [解析]   两个用途：① 平移分量里做投影（除以 d_）；
  // [解析]              ② 自转分量里当旋转半径（乘以 d_）。
  // [解析]   所以 l_ 和 w_ 只在反解 update() 里单独出现，正解里都藏在 d_ 中。
  // [解析] 下面 4 个是装配方向符号，取值 +1.0f 或 -1.0f，直接乘进公式里翻转结果。
  const float sign_lf_;
  const float sign_lr_;
  const float sign_rf_;
  const float sign_rr_;
};

}  // namespace sp

#endif  // SP__OMNI_HPP
