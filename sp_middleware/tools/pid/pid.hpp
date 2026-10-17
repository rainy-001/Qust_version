// [解析] 通用 PID 控制器的声明：一个对象 = 一路闭环，每周期喂 (set, fdb)，从 out 取结果。
// [解析] tools/gimbal、swerve、omni、mecanum、joint 等上层控制器里各挂着若干个 PID 成员。
// [解析] 构造函数参数顺序是 dt, kp, ki, kd, max_out, max_iout，后两个限幅别写反；
// [解析] 4 个同名 calc() 靠参数个数区分，且 angular / dynamic 两个开关会明显改变算法行为。

#ifndef SP__PID_HPP
#define SP__PID_HPP

#include "tools/math_tools/math_tools.hpp"

namespace sp
{
// [解析] 内部状态 + 调试用的各项分解：set/fdb/err/dbuf/iout 是记忆，pout/iout/dout 供看波形；外部只读别写。
struct PIDData
{
  float set = 0;  // 设定值
  float fdb = 0;  // 反馈值(feedback)

  float pout = 0;  // P项输出值
  float iout = 0;  // I项输出值
  float dout = 0;  // D项输出值

  float err[3] = {0};   // 误差缓冲区
  float dbuf[3] = {0};  // D项滤波器缓冲区

  float trapezoid = 0;   // 梯形积分
  float dynamic_ki = 0;  // 动态积分
};

// [解析] 一个对象负责一路闭环控制；只做数学计算，不碰硬件也不自己定时，dt 必须与真实调用周期一致。
class PID
{
public:
  // dt: 控制周期, 单位: s
  // kp: P项系数
  // ki: I项系数
  // kd: D项系数
  // max_out: PID最大输出值
  // max_iout I项最大输出值
  // alpha: D项滤波系数, alpha=1时不滤波
  PID(
    float dt, float kp, float ki, float kd, float max_out, float max_iout, float alpha = 1,
    bool angular = false, bool dynamic = false);

  float out = 0;  // 只读! PID输出值
  PIDData data;   // 只读! PID计算数据

  // 计算PID输出值
  // set: 目标值
  // fdb: 反馈值(feedback)
  // [解析] 最常用版本：D 项"微分先行"（微分成 fdb 而非 err），目标值突变时不会产生微分尖峰。
  void calc(float set, float fdb);

  // [解析] 积分分离版：|P 项输出| 超过阈值时，本拍的积分增量置 0，避免起步阶段积分堆太高。
  void calc(float set, float fdb, float integral_pause_threshold);

  // [解析] 前馈版：D 项 = kd*(set_dot - fdb_dot)，不做差分也不滤波；下一个重载 = 本版 + 积分分离。
  void calc(float set, float fdb, float set_dot, float fdb_dot);
  void calc(float set, float fdb, float set_dot, float fdb_dot, float integral_pause_threshold);

  // 清零PID的所有状态（输出与缓存等）
  // [解析] 归零 out 与 data 并复位微分就绪标志；使能电机前、模式切换后不调用，残留 iout 会让车一上电就冲。
  void clear();

private:
  const float dt_;
  const float kp_, ki_, kd_;
  const float max_out_, max_iout_;
  const float alpha_;
  const bool angular_;  // 是否为角度控制
  const bool dynamic_;  // 是否为变速积分
  bool dout_ready_ = false;
};

}  // namespace sp

#endif  // SP__PID_HPP