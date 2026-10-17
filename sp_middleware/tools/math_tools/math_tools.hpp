// [解析] 全项目的数学小工具箱：角度折叠/展开、限幅、线性映射、定点数换算、数值微分。
// [解析] PID、云台与底盘解算、各电机驱动都用它；本文件只有声明，实现在 math_tools.cpp。
// [解析] 约定：角度一律用弧度 rad；limit_* 会改数值，scope_* 只判断范围。

#ifndef SP__MATH_TOOLS_HPP
#define SP__MATH_TOOLS_HPP

#include <cmath>
#include <cstdint>

namespace sp
{
constexpr float SP_PI = M_PI;

// (-PI, PI]
// [解析] 折回 (-π, π]；电机/IMU 报出来的绕圈值要先折过再比较差值。
float limit_angle(float angle);

/**
 * @brief 角度展开类（消除 ±π 跳变）
 * @note  每个需要展开的角度应使用独立的实例
 *
 * 使用示例：
 * @code
 *   sp::AngleUnwrapper yaw_unwrapper;
 *   sp::AngleUnwrapper pitch_unwrapper;
 *   float yaw_unwrapped = yaw_unwrapper.update(yaw_raw);
 *   float pitch_unwrapped = pitch_unwrapper.update(pitch_raw);
 * @endcode
 */
// [解析] 有状态：每个要展开的角度都得有自己的实例，几个角度共用一个会把增量串在一起。
// [解析] 第一次 update() 没有上一帧，直接把输入当输出（上电零点 = 第一帧读数）。
class AngleUnwrapper
{
public:
  AngleUnwrapper() = default;

  /**
   * @brief 更新角度并返回展开后的值
   * @param raw 原始角度（应在 -π 到 π 范围内）
   * @return 展开后的角度（累积值，无跳变）
   */
  // [解析] 每周期喂进单圈角度，返回连续累积角度；同一个值也写在公开成员 out 里。
  float update(float raw);

  /**
   * @brief 重置状态
   */
  void reset();

  float out = 0.0f;  ///< 展开后的角度输出

private:
  float last_raw_ = 0.0f;
  bool inited_ = false;
};

// [min, max]
// [解析] 夹进闭区间 [min, max]；不检查 min ≤ max，传反了结果永远是某个端点。
float limit_min_max(float input, float min, float max);

// [-max, max]
float limit_max(float input, float max);

// [min, max]
// [解析] 只回答"在不在范围内"，不改输入（与 limit_min_max 的区别）。
bool scope_min_max(float input, float min, float max);

// [-max, max]
bool scope_max(float input, float max);

// [in_min, in_max] -> [out_min, out_max]
float map(float x, float in_min, float in_max, float out_min, float out_max);

// [min, max]
// [解析] 无符号定点数 → 物理量：先归一化到 [0, 1] 再拉伸到 [min, max]。
// [解析] 例：力矩量程 -20~20、12 位时，传 0 得 -20，传 2048 约为 0，传 4095 得 +20。
float uint_to_float(uint32_t input, float min, float max, size_t bits);

// [min, max]
// [解析] 逆运算；截断取整且内部不限幅，超量程会溢出到相邻位域，调用前先 limit_min_max。
uint32_t float_to_uint(float input, float min, float max, size_t bits);

// sgn
int8_t sgn(float input);

// 三维向量数值微分：result = (v_next - v_prev) / dt
// 输入：v_next[3] 下一时刻向量，v_prev[3] 上一时刻向量
// 输出：result[3] 微分结果（速度/加速度等）
// dt: 时间间隔，默认 1ms
// [解析] 单位是"输入单位/秒"；dt 不能为 0，数值微分对噪声敏感，输入一般要先低通滤波。
void diff_vec3(const float v_next[3], const float v_prev[3], float result[3], float dt = 1e-3f);

}  // namespace sp

#endif  // SP__MATH_TOOLS_HPP