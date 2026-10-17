// 底盘运动学 + 速度环的离线验证脚本（主机端运行，不参与固件编译）。
//
// 用途：在真机上电之前，先确认三件事
//   1) sp::Omni 的四个轮速符号 / 幅值与 readme 的表格一致，且 calc/update 互为逆运算
//   2) applications/chassis_task.cpp 里选的那组 PID 参数是收敛的、不超调的
//   3) 改 kp / kMaxTorque 时能拿到量化依据，而不是靠试
//
// 编译运行（在工程根目录；-std=gnu++17 必需，因为 math_tools.hpp 用了 M_PI）：
//   g++ -std=gnu++17 -O2 -I sp_middleware -o build/verify_chassis_sim \
//       tools/verify_chassis_sim.cpp \
//       sp_middleware/tools/omni/omni.cpp \
//       sp_middleware/tools/pid/pid.cpp \
//       sp_middleware/tools/math_tools/math_tools.cpp
//   ./build/verify_chassis_sim
//
// 注意：脚本里的参数是 chassis_task.cpp 的副本，改了固件里的参数要同步改这里，否则验证失去意义。
#include <cmath>
#include <cstdio>
#include <initializer_list>

#include "tools/math_tools/math_tools.hpp"
#include "tools/omni/omni.hpp"
#include "tools/pid/pid.hpp"

// ---- 与 applications/chassis_task.cpp 保持一致的参数 ----
constexpr float kR = 0.076f;
constexpr float kL = 0.190f;
constexpr float kW = 0.190f;
constexpr float kDt = 1e-3f;
constexpr float kKp = 0.60f;
constexpr float kKi = 0.40f;
constexpr float kKd = 0.0f;
constexpr float kMaxTorque = 1.2f;
constexpr float kMaxIout = 0.60f;
constexpr float kIntegralPause = 0.6f;

// 被控对象（轮端等效）：J*dw/dt = tau - b*w
constexpr float kJ = 0.034f;        // 转子折算 + 20kg 整车质量折算到单个轮子
constexpr float kB = 0.02f;         // 粘性阻尼
constexpr float kMechMass = 20.0f;  // 整车质量 kg，用于把轮上力矩换算成加速度

static int failures = 0;
static void check(bool ok, const char* what)
{
  std::printf("  [%s] %s\n", ok ? " OK " : "FAIL", what);
  if (!ok) failures++;
}

static void kinematics()
{
  std::printf("== 运动学符号 / 往返 ==\n");
  sp::Omni o(kR, kL, kW);

  o.calc(1.0f, 0.0f, 0.0f);
  const float kv = 1.0f / (kR * std::sqrt(2.0f));
  std::printf("  vx>0: lf=%.3f lr=%.3f rf=%.3f rr=%.3f (期望 -%.3f/-%.3f/+%.3f/+%.3f)\n", o.speed_lf,
              o.speed_lr, o.speed_rf, o.speed_rr, kv, kv, kv, kv);
  check(o.speed_lf < 0 && o.speed_lr < 0 && o.speed_rf > 0 && o.speed_rr > 0, "前进时四轮符号正确");
  check(std::fabs(o.speed_lf + kv) < 1e-4f && std::fabs(o.speed_rf - kv) < 1e-4f, "前进时幅值 = v/(r*sqrt2)");

  o.calc(0.0f, 0.0f, 1.0f);
  std::printf("  wz>0: lf=%.3f lr=%.3f rf=%.3f rr=%.3f\n", o.speed_lf, o.speed_lr, o.speed_rf, o.speed_rr);
  check(o.speed_lf > 0 && o.speed_lr > 0 && o.speed_rf > 0 && o.speed_rr > 0, "逆时针自转时四轮同向");

  const float a = 1.2f, b = -0.7f, c = 1.9f;
  o.calc(a, b, c);
  o.update(o.speed_lf, o.speed_lr, o.speed_rf, o.speed_rr);
  std::printf("  往返: (%.3f,%.3f,%.3f) -> (%.3f,%.3f,%.3f)\n", a, b, c, o.vx, o.vy, o.wz);
  check(std::fabs(o.vx - a) < 1e-4f && std::fabs(o.vy - b) < 1e-4f && std::fabs(o.wz - c) < 1e-4f,
        "calc/update 互为逆运算");
}

struct StepResult
{
  float final_w;
  float overshoot_pct;
  float settle_s;
  float min_settle_s;  // 力矩上限决定的物理下限
};

static StepResult closed_loop(float target, float max_torque, float kp)
{
  sp::PID pid(kDt, kp, kKi, kKd, max_torque, kMaxIout);
  float w = 0.0f, peak = 0.0f, settle = -1.0f;

  const int steps = 8000;  // 8 s
  for (int i = 0; i < steps; i++) {
    pid.calc(target, w, kIntegralPause);
    w += (pid.out - kB * w) / kJ * kDt;
    if (w > peak) peak = w;
    if (settle < 0 && std::fabs(w - target) < 0.05f * target) settle = (i + 1) * kDt;
  }

  const float min_settle = kJ * target / max_torque;  // 全程满力矩的最短加速时间
  return {w, (peak / target - 1) * 100, settle, min_settle};
}

static void closed_loop_check()
{
  std::printf("\n== 速度环闭环仿真（J=%.3f, b=%.2f, dt=1ms, kp=%.2f, ki=%.2f）==\n", kJ, kB, kKp, kKi);

  for (float target : {10.0f, 20.0f}) {
    const auto r = closed_loop(target, kMaxTorque, kKp);
    std::printf("  目标 %.0f rad/s: 终值 %.3f, 超调 %.1f%%, 5%% 调节 %.3f s (力矩上限物理下限 %.3f s)\n", target,
                r.final_w, r.overshoot_pct, r.settle_s, r.min_settle_s);
    check(std::fabs(r.final_w - target) < 0.03f * target, "稳态误差 < 3%");
    check(r.overshoot_pct < 15.0f, "超调 < 15%");
    check(r.settle_s < r.min_settle_s * 2.2f, "调节时间受力矩上限限制、未额外拖慢");
  }
}

static void kp_sweep()
{
  const float floor_s = kJ * 20.0f / kMaxTorque;
  std::printf("\n== kp 扫描（目标 20 rad/s, 力矩上限 %.1f N·m, 物理下限 %.3f s）==\n", kMaxTorque, floor_s);
  std::printf("  %-8s %-10s %-12s %-10s\n", "kp", "超调%", "调节时间s", "时间/下限");
  for (float kp : {0.30f, 0.60f, 0.90f, 1.20f}) {
    const auto r = closed_loop(20.0f, kMaxTorque, kp);
    std::printf("  %-8.2f %-10.1f %-12.3f %-10.2f\n", kp, r.overshoot_pct, r.settle_s,
                r.settle_s / r.min_settle_s);
  }
}

static void torque_tradeoff()
{
  std::printf("\n== 单轮力矩上限的取舍（目标 20 rad/s）==\n");
  std::printf("  %-10s %-12s %-16s %-12s\n", "上限N·m", "单轮电流A", "整车加速度m/s²", "调节时间s");
  for (float tq : {1.2f, 2.4f, 3.6f}) {
    const auto r = closed_loop(20.0f, tq, kKp);
    const float accel = tq / kR * 4 / kMechMass;  // 四轮推力 / 整车质量
    std::printf("  %-10.1f %-12.1f %-16.2f %-12.3f\n", tq, tq * 3.33f, accel, r.settle_s);
    check(r.overshoot_pct < 15.0f, "该上限下不超调");
  }
}

int main()
{
  kinematics();
  closed_loop_check();
  kp_sweep();
  torque_tradeoff();

  std::printf("\n%s (%d failures)\n", failures == 0 ? "ALL PASS" : "SOME FAILED", failures);
  return failures == 0 ? 0 : 1;
}
