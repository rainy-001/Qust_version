// [解析] 两轮差速底盘（左右各一个驱动轮）的运动学解算器：calc() 把 (v, w) 拆给左右轮，update() 反推回整车速度。
// [解析] 上层底盘任务建对象后每周期调 calc()，把 speed_l/speed_r 给两个电机 PID 当目标；update() 做里程计。
// [解析] 构造顺序是 (半轮距, 轮半径)：half_width 取"左右驱动轮中心距的一半"，轮距量错会让原地自转快慢不对。

#ifndef SP__DIFF_DRIVE_HPP
#define SP__DIFF_DRIVE_HPP

namespace sp
{
// [解析] 纯算数、不碰硬件也不保存状态，所以任务和中断里都能调；但两个函数都不返回结果，而是写下面这些公共成员。
class DiffDrive
{
public:
  DiffDrive(float half_width, float wheel_radius, bool reverse_l, bool reverse_r);

  float v = 0;  // 只读! update()更新结果, 前进速度, 单位: m/s
  float w = 0;  // 只读! update()更新结果, 旋转速度, 单位: m/s

  float speed_l = 0;  // 只读! calc()计算结果, left目标转速, 单位: rad/s
  float speed_r = 0;  // 只读! calc()计算结果, right目标转速, 单位: rad/s

  // [解析] 正解：左右轮各取"直行 v_set + 自转 w_set*w_"，左加右减（w>0 时右轮要多转），最后 /r_ 换成 rad/s。
  void calc(float v_set, float w_set);
  void update(float motor_speed_l, float motor_speed_r);

private:
  const float w_;
  const float r_;
  const float sign_l_;
  const float sign_r_;
};

}  // namespace sp

#endif  // SP__DIFF_DRIVE_HPP