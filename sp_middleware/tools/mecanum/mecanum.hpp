// [解析] 麦克纳姆轮底盘的运动学解算器：calc() 正解（整车速度 → 四个轮速），update() 反解（轮速 → 整车速度，做里程计）。
// [解析] 上层 applications/ 的底盘任务自己建对象：sp::Mecanum m(0.06f, 0.2f, 0.2f); 之后每周期 m.calc(vx, vy, wz)。
// [解析] 单位统一用米 / m/s / rad/s（把 mm 当 m 传结果会差 1000 倍）；reverse_xx 只用来补电机装反，不要去改公式里的加减号。

#ifndef SP__MECANUM_HPP
#define SP__MECANUM_HPP

#include "tools/math_tools/math_tools.hpp"

namespace sp
{
// [解析] 一个对象代表一辆麦轮底盘：构造时把尺寸和转向符号存成 const，两个解算函数都不返回值，结果读下面这些公共成员。
class Mecanum
{
public:
  // [解析] 默认值 lf/lr=false、rf/rr=true 不是随便定的：麦轮左右两侧辊子镜像对称，同一套"往前开"的指令符号本来就相反。
  // wheel_radius: 轮子半径, 单位: m
  // half_length: 前后轮距离的一半, 单位: m
  // half_width: 左右轮距离的一半, 单位: m
  // reverse_lf: left-front反向旋转
  // reverse_lr: left-rear反向旋转
  // reverse_rf: right-front反向旋转
  // reverse_rr: right-rear反向旋转
  Mecanum(
    float wheel_radius, float half_length, float half_width, bool reverse_lf = false,
    bool reverse_lr = false, bool reverse_rf = true, bool reverse_rr = true);

  // [解析] 单位是轮子自转的角速度 rad/s，可以直接当电机 PID 目标；若电机内部按"转子侧"计角，还要再除减速比。
  float speed_lf;  // 只读! calc()计算结果, left-front转速, 单位: rad/s
  float speed_lr;  // 只读! calc()计算结果, left-rear转速, 单位: rad/s
  float speed_rf;  // 只读! calc()计算结果, right-front转速, 单位: rad/s
  float speed_rr;  // 只读! calc()计算结果, right-rear转速, 单位: rad/s

  float vx;  // 只读! update()计算结果, 底盘x方向速度, 单位: m/s
  float vy;  // 只读! update()计算结果, 底盘y方向速度, 单位: m/s
  float wz;  // 只读! update()计算结果, 底盘z方向角速度, 单位: rad/s

  // [解析] 坑：形参名和成员 vx/vy/wz 同名，函数体里裸写的 vx 是形参，写 this->vx 才是成员。
  // 底盘速度 -> 各轮转速
  // vx: 前进速度, 单位: m/s
  // vy: 左移速度, 单位: m/s
  // wz: 大拇指朝上，右手螺旋方向转速, 单位: rad/s
  void calc(float vx, float vy, float wz);

  // [解析] 输入是电机反馈转速（rad/s，形参同样与成员重名）；本函数不做滤波，轮速抖动会 1:1 传到结果里。
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
  const float sign_lf_;
  const float sign_lr_;
  const float sign_rf_;
  const float sign_rr_;
};

}  // namespace sp

#endif  // SP__MECANUM_HPP