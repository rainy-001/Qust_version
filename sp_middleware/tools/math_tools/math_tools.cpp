// [解析] 数学工具的实现：角度折叠/展开、限幅、线性映射、定点数换算、数值微分。
// [解析] PID、云台解算、底盘解算与各电机驱动都调用它，声明和用法见 math_tools.hpp。
// [解析] 两处值得留意：角度展开靠"增量补偿"消掉 ±π 跳变，map() 里的 1e-6f 是防除零阈值。

#include "math_tools.hpp"

namespace sp
{
// [解析] 加减整圈折进 (-π, π]；用 while 是因为输入可能绕了好几圈。
// [解析] 两个边界故意不对称（> SP_PI 与 <= -SP_PI），保证同一角度只有一种表示。
float limit_angle(float angle)
{
  while (angle > SP_PI) angle -= 2 * SP_PI;
  while (angle <= -SP_PI) angle += 2 * SP_PI;
  return angle;
}

// [解析] 只累加每帧的"增量"，所以输出连续且能超过 ±π；第一帧没有历史值，直接当基准。
// [解析] 跨 ±π 时两帧差值会跳 ≈±2π，减去/加上一整圈就还原成真实的小转动。
float AngleUnwrapper::update(float raw)
{
  if (!inited_) {
    last_raw_ = raw;
    out = raw;
    inited_ = true;
    return out;
  }

  float delta = raw - last_raw_;

  if (delta > SP_PI)
    delta -= 2.0f * SP_PI;
  else if (delta < -SP_PI)
    delta += 2.0f * SP_PI;

  out += delta;
  last_raw_ = raw;
  return out;
}

void AngleUnwrapper::reset()
{
  last_raw_ = 0.0f;
  out = 0.0f;
  inited_ = false;
}

float limit_min_max(float input, float min, float max)
{
  if (input > max)
    input = max;
  else if (input < min)
    input = min;
  return input;
}

float limit_max(float input, float max) { return limit_min_max(input, -max, max); }

bool scope_min_max(float input, float min, float max) { return (input >= min && input <= max); }

bool scope_max(float input, float max) { return scope_min_max(input, -max, max); }

// [解析] 定点数 → 物理量：先除以 (2^bits - 1) 归一化到 [0, 1]，再拉伸到 [min, max]。
// [解析] 协议用"最小值映射到 0"表达负数，例如力矩 -20~20 N·m 时整数 2048 约为 0。
float uint_to_float(uint32_t input, float min, float max, size_t bits)
{
  auto span = max - min;
  auto norm = static_cast<float>(input) / ((1 << bits) - 1);
  return norm * span + min;
}

// [解析] 上面的逆运算：先算比例再乘 (2^bits - 1)，float 转 uint32_t 是隐式截断取整。
// [解析] 内部不限幅，input 超出 [min, max] 会溢出到相邻位域，调用前务必先限幅。
uint32_t float_to_uint(float input, float min, float max, size_t bits)
{
  auto span = max - min;
  auto norm = (input - min) / span;
  return norm * ((1 << bits) - 1);
}

float map(float x, float in_min, float in_max, float out_min, float out_max)
{
  // [解析] 输入区间宽度趋近 0 时除法会得到 inf/NaN；1e-6f 是判零阈值，直接返回下界兜底。
  if (fabsf(in_max - in_min) < 1e-6f) {
    return out_min;
  }
  return out_min + (x - in_min) * (out_max - out_min) / (in_max - in_min);
}

// [解析] 用 (bool - bool) 实现：正数得 1、负数得 -1、零得 0，没有分支。
int8_t sgn(float input) { return (input > 0) - (input < 0); }

// [解析] 差分近似求导，先算 1/dt 再逐项相乘（省两次除法）；dt 传 0 会算出 inf/NaN。
void diff_vec3(const float v_next[3], const float v_prev[3], float result[3], float dt)
{
  // 数值微分：(v_next - v_prev) / dt
  float inv_dt = 1.0f / dt;
  result[0] = (v_next[0] - v_prev[0]) * inv_dt;
  result[1] = (v_next[1] - v_prev[1]) * inv_dt;
  result[2] = (v_next[2] - v_prev[2]) * inv_dt;
}

}  // namespace sp
