// [解析] ==========================================================================
// [解析] 【文件】sp_middleware/tools/math_tools/math_tools.cpp
// [解析] 【作用】math_tools.hpp 里那些数学工具函数的具体实现（都是有状态就记一下、没状态就纯算）。
// [解析] 【被谁用】见 math_tools.hpp 的说明；调用最密集的是 PID、云台解算和各种电机驱动。
// [解析] 【看代码的重点】1) 角度展开的"增量补偿"技巧（为什么它能消掉 ±π 跳变）；
// [解析]   2) uint_to_float / float_to_uint 的归一化公式和它的精度、溢出陷阱；
// [解析]   3) map() 里那个 1e-6f 是防除零的，不是随便写的魔数。
// [解析] ==========================================================================

#include "math_tools.hpp"

namespace sp
{
// [解析] 角度归一化：不停加减 2π（一整圈），直到落进 (-π, π] 这个区间。
// [解析] 用 while 而不是 if，是因为输入可能绕了好几圈（比如 20 rad），一次减法不够。
// [解析] 两个判断的边界故意不同：上界用 > SP_PI（所以 +π 要被折成 -π），
// [解析]   下界用 <= -SP_PI，这样最终结果恰好是"左开右闭"的 (-π, π]，保证同一角度只有一种表示。
// [解析] 注意：每绕一圈都要循环一次，如果传进来的是 1e9 这种天文数字会很慢；
// [解析]   正常传感器数据（|angle| < 100）完全不用担心。
float limit_angle(float angle)
{
  while (angle > SP_PI) angle -= 2 * SP_PI;
  while (angle <= -SP_PI) angle += 2 * SP_PI;
  return angle;
}

// [解析] 角度展开的核心：不关心"绝对角度"，只关心"这一帧相对上一帧转了多少"。
// [解析] 每次拿到的 raw 都是 (-π, π] 的单圈值，所以只要把每帧的"小增量"累加起来，
// [解析] 就能得到一个连续的、可以超过 ±π 的角度。
float AngleUnwrapper::update(float raw)
{
  // [解析] 第一帧没有历史值可比，直接把它当作基准：out = raw、记住它，然后就返回。
  // [解析] 这样做的代价是"上电时输出的绝对零点就是第一帧的读数"，一般正合需求。
  if (!inited_) {
    last_raw_ = raw;
    out = raw;
    inited_ = true;
    return out;
  }

  // [解析] 本帧相对上一帧的"原始增量"。因为 raw 被折叠在一圈内，
  // [解析] 正常转过 ±π 边界时这个差值会突然变成 ±2π 左右（看着像倒退一整圈）。
  float delta = raw - last_raw_;

  // [解析] 关键修正：如果差值超过半圈（|delta| > π），说明刚跨过 ±π 分界线，
  // [解析] 真实转动其实很小，只是数值跳变了，所以给它加/减一整圈补回来。
  // [解析] 举例：上一帧 +3.13，本帧 -3.13，差值是 -6.26（< -π），加 2π 后得 +0.02，
  // [解析]   正是"又往前转了一点点"，这就是消除跳变的全部原理。
  // [解析] 前提假设：一个控制周期内角度变化不会超过半圈（1kHz 控制下必然成立）。
  if (delta > SP_PI)
    delta -= 2.0f * SP_PI;
  else if (delta < -SP_PI)
    delta += 2.0f * SP_PI;

  // [解析] 把修正后的真实增量累加到输出上，并记住本帧读数供下一帧做差。
  out += delta;
  last_raw_ = raw;
  return out;
}

// [解析] 重置：把输出和内部状态都清干净，下一次 update() 会重新走"第一帧"分支。
// [解析] 注意 out 也会被清零（而不是保留当前角度），所以调用者如果拿 out 当角度用，
// [解析] 重置后需要自己重新对齐零点。
void AngleUnwrapper::reset()
{
  last_raw_ = 0.0f;
  out = 0.0f;
  inited_ = false;
}

// [解析] 双向限幅：超上界就压到 max，低于下界就抬到 min，其余原样返回。
// [解析] 用 else if 是为了少做一次比较（两个条件不可能同时成立）。
float limit_min_max(float input, float min, float max)
{
  if (input > max)
    input = max;
  else if (input < min)
    input = min;
  return input;
}

// [解析] 对称限幅：复用上面的双向限幅，把下界写成 -max 即可。
// [解析] 一行写完，编译器一般会直接内联，没有函数调用开销。
float limit_max(float input, float max) { return limit_min_max(input, -max, max); }

// [解析] 范围判断：&& 两边都成立才返回 true，注意两个都是"包含端点"的（>= 和 <=）。
bool scope_min_max(float input, float min, float max) { return (input >= min && input <= max); }

// [解析] 对称范围判断，同样是闭区间 [-max, max]。
bool scope_max(float input, float max) { return scope_min_max(input, -max, max); }

// [解析] 定点数 -> 物理量，两步走：
// [解析]   第 1 步：把整数归一化到 [0, 1]。分母 (1 << bits) - 1 是该字段能表示的最大整数
// [解析]     （bits = 12 时是 4095，bits = 16 时是 65535），所以 input=0 → 0.0，input=最大值 → 1.0。
// [解析]   第 2 步：把 [0,1] 拉伸到物理区间 [min, max]：norm * span + min。
// [解析] 注意 1：input 是"无符号整数"，协议里用"最小值映射到 0"的方式表达负数，
// [解析]   例如力矩量程 -20~20 N·m，整数 0 代表 -20 N·m，2048 左右代表 0 N·m。
// [解析] 注意 2：1 << bits 中的 1 是 int，bits ≥ 31 时属于未定义行为；本项目只用到 12/14/16。
// [解析] 注意 3：这里是 float 运算，只有约 7 位有效数字，超大 bits 时精度会不够。
float uint_to_float(uint32_t input, float min, float max, size_t bits)
{
  auto span = max - min;
  auto norm = static_cast<float>(input) / ((1 << bits) - 1);
  return norm * span + min;
}

// [解析] 物理量 -> 定点数，是上一个函数的逆运算：
// [解析]   先算 (input - min) / span 得到 [0,1] 的比例，再乘 (2^bits - 1) 换成整数。
// [解析] 注意 1：返回值是 uint32_t，但函数体最后没有显式转换，
// [解析]   所以 float → uint32_t 是"隐式截断"（直接砍掉小数），不是四舍五入。
// [解析] 注意 2：函数内部不做限幅。input > max 时结果会超过 (2^bits - 1)，
// [解析]   打包进位域（移位 + 或）时会跑到别的字段上去，造成"明明只发力矩却动了位置"。
// [解析]   所以调用前务必先用 limit_min_max 把物理量夹进 [min, max]。
// [解析] 注意 3：bits ≥ 31 时 1 << bits 是未定义行为。
uint32_t float_to_uint(float input, float min, float max, size_t bits)
{
  auto span = max - min;
  auto norm = (input - min) / span;
  return norm * ((1 << bits) - 1);
}

// [解析] 线性映射的本质就是"比例 + 平移"，公式：
// [解析]   out = out_min + (x - in_min) * (out_max - out_min) / (in_max - in_min)
// [解析] 也就是：x 在输入区间里占多少比例，就在输出区间里占同样的比例。
float map(float x, float in_min, float in_max, float out_min, float out_max)
{
  // [解析] 防除零：如果输入区间宽度接近 0（1e-6f 是"足够小"的经验阈值，也就是百万分之一），
  // [解析] 分母会趋近于 0，除法结果会变成 inf 或 NaN，把后面的控制量全污染掉，
  // [解析] 所以这里直接返回 out_min 作为兜底。
  if (fabsf(in_max - in_min) < 1e-6f) {
    return out_min;
  }
  return out_min + (x - in_min) * (out_max - out_min) / (in_max - in_min);
}

// [解析] 符号函数的小技巧：C++ 里比较运算的结果是 bool，参与算术时会变成 1 或 0。
// [解析]   正数：(true) - (false) = 1 - 0 = 1
// [解析]   负数：(false) - (true) = 0 - 1 = -1
// [解析]   零  ：0 - 0 = 0
// [解析] 比写三个 if-else 简洁，而且不会有分支预测负担，适合放在高频控制循环里。
int8_t sgn(float input) { return (input > 0) - (input < 0); }

void diff_vec3(const float v_next[3], const float v_prev[3], float result[3], float dt)
{
  // 数值微分：(v_next - v_prev) / dt
  // [解析] 先算 1/dt 再逐项相乘，比三个元素各除一次 dt 更省时间（除法比乘法慢得多），
  // [解析] 而且三个分量共用同一个倒数，结果一致。这在高频循环里是常见的优化写法。
  // [解析] 注意：dt 传 0 会得到 inf（1/0），进而算出 NaN；调用者要保证 dt > 0。
  float inv_dt = 1.0f / dt;
  result[0] = (v_next[0] - v_prev[0]) * inv_dt;
  result[1] = (v_next[1] - v_prev[1]) * inv_dt;
  result[2] = (v_next[2] - v_prev[2]) * inv_dt;
}

}  // namespace sp
