// [解析] ==========================================================================
// [解析] 【文件】sp_middleware/tools/math_tools/math_tools.hpp
// [解析] 【作用】全项目的"数学小工具箱"：角度归一化、角度展开、限幅、线性映射、
// [解析]         定点数与物理量互转、符号函数、向量数值微分。都是无状态的纯函数。
// [解析] 【被谁用】几乎所有模块：tools/pid、tools/gimbal、tools/swerve、tools/omni、
// [解析]         tools/mecanum、tools/joint、tools/fuzzy_pid，以及 motor/ 下各种电机
// [解析]         （dm_motor、ddt_motor、lk_motor、rm_motor、cybergear_motor）。
// [解析] 【阅读顺序】先看本文件知道"有哪些工具、单位是什么"，再看 math_tools.cpp 看实现。
// [解析] 【关键点】1) 本文件所有角度单位都是弧度 rad，不是度；
// [解析]   2) [min, max] 与 [-max, max] 两种限幅容易看混，注意函数名后缀；
// [解析]   3) uint_to_float/float_to_uint 是"定点数换算"，DM 电机协议全靠它。
// [解析] ==========================================================================

#ifndef SP__MATH_TOOLS_HPP
#define SP__MATH_TOOLS_HPP

#include <cmath>
#include <cstdint>

namespace sp
{
// [解析] 圆周率 π ≈ 3.14159265。M_PI 由 <cmath> 提供（严格来说不是 C++ 标准宏，
// [解析] 但 GCC/ARM 工具链里一定有）。写成 constexpr 是为了让编译器把它直接内联成常量，
// [解析] 运行时不占 RAM，也不会像 #define 那样有宏替换的坑。
// [解析] 注意：本单位一律用弧度；如果要写角度请自己换算：rad = deg * SP_PI / 180。
constexpr float SP_PI = M_PI;

// (-PI, PI]
// [解析] 把任意角度折回 (-π, π] 这个"单圈"区间，单位 rad。
// [解析] 为什么需要：电机/IMU 报出来的角度可能是 7.5 rad、-13 rad 这种绕了好几圈的值，
// [解析]   直接拿去算角度差会得到错误结果，先折叠到一圈内再比较才对。
// [解析] 参数：angle 任意弧度值；返回：同方向的等价角度，落在 (-π, π]。
// [解析] 副作用：无。调用例子见 pid.cpp：angular_ ? limit_angle(set - fdb) : (set - fdb)。
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
// [解析] 【这是什么】"有状态"的小工具（和上面的纯函数不同，它要记住上一次的输入）。
// [解析] 【解决什么问题】如果角度传感器只给 (-π, π] 的单圈值，那么从 +179° 转到 -179° 时，
// [解析]   数值会从 +3.13 突然跳到 -3.13，看上去"倒退了一大圈"，PID 会在这一刻发疯。
// [解析]   本类把每次的"增量"累加起来，输出一个连续单调、可以超过 ±π 的角度。
// [解析] 【怎么用】每个需要展开的物理量都要有自己的实例（yaw 一个、pitch 一个），
// [解析]   绝对不能共用一个，否则两个角度的增量会串在一起。实际例子见
// [解析]   tools/gimbal/gimbal.hpp 里的 yaw_unwrapper_ 成员。
// [解析] 【注意】第一次 update() 时没有"上一帧"可比较，所以直接把输入当作输出（见 .cpp）。
class AngleUnwrapper
{
public:
  // [解析] AngleUnwrapper() = default：告诉编译器"用默认的构造就行"，
  // [解析] 也就是什么都不做，成员按下面的默认值初始化（out=0、inited_=false）。
  AngleUnwrapper() = default;

  /**
   * @brief 更新角度并返回展开后的值
   * @param raw 原始角度（应在 -π 到 π 范围内）
   * @return 展开后的角度（累积值，无跳变）
   */
  // [解析] 每个控制周期调用一次，喂进本周期读到的"单圈角度"（rad，理论上在 -π~π）。
  // [解析] 返回展开后的连续角度（rad，累积值，可能远大于 π，例如转 3 圈就是 ±6π 附近）。
  // [解析] 返回值和公开成员 out 是同一个数，两种取法都行。
  // [解析] 副作用：会更新内部状态 last_raw_，并且每次调用都会推进 out。
  float update(float raw);

  /**
   * @brief 重置状态
   */
  // [解析] 清空"上一次的值"，回到刚构造的状态（out=0、inited_=false）。
  // [解析] 什么时候用：云台重新上电、坐标系重新标定、或者 out 被别的逻辑改过之后。
  void reset();

  float out = 0.0f;  ///< 展开后的角度输出
  // [解析] ↑ 公开成员，只读！（请不要在外部直接改它，累加逻辑会被破坏）
  // [解析]   它就是"展开后的角度"，单位 rad，等价于 update() 的返回值。

private:
  // [解析] 上一次 update() 收到的原始角度，用来算本次增量 delta。
  float last_raw_ = 0.0f;
  // [解析] 是否已经收过第一帧。false 时 update() 走"初始化"分支（不做累加）。
  bool inited_ = false;
};

// [min, max]
// [解析] 双向限幅：把 input 夹到 [min, max] 区间内，返回夹好之后的值。
// [解析] 参数 min/max 是闭区间两端（单位由调用者决定，可以是 rad、N·m、A……）。
// [解析] 注意：没有检查 min <= max，如果传反了（min > max），结果永远是 max 或 min 之一。
// [解析] 用法见 joint_controller.cpp：set_ = sp::limit_min_max(value, min_, max_)。
float limit_min_max(float input, float min, float max);

// [-max, max]
// [解析] 对称限幅：等价于 limit_min_max(input, -max, max)，用在"正负对称量程"的场合，
// [解析] 比如电流、力矩、速度。max 请传正数（传负数会得到奇怪的结果）。
// [解析] 用法见 lk_motor.cpp、cybergear_motor.cpp 的力矩限幅。
float limit_max(float input, float max);

// [min, max]
// [解析] 范围判断（不是限幅！）：input 在 [min, max] 区间内返回 true，否则 false。
// [解析] 名字里的 scope 是"范围"的意思。注意和 limit_min_max 的区别：
// [解析]   限幅会改变数值，这个只回答"在不在范围内"，不会修改任何东西。
bool scope_min_max(float input, float min, float max);

// [-max, max]
// [解析] 对称范围判断：等价于 scope_min_max(input, -max, max)。返回 bool，不修改输入。
bool scope_max(float input, float max);

// [in_min, in_max] -> [out_min, out_max]
// [解析] 线性映射（区间缩放）：把 x 从 [in_min, in_max] 等比映射到 [out_min, out_max]。
// [解析] 常用场景：把遥控器摇杆的 [364, 1684] 映射成电机的 [-20, 20] rad/s。
// [解析] 注意：out_min > out_max 是允许的，那样会得到"反向"的映射。
float map(float x, float in_min, float in_max, float out_min, float out_max);

// [min, max]
// [解析] 定点数 -> 物理量。把"无符号原始整数"按位数线性映射到物理区间 [min, max]。
// [解析] 参数：input 读到的原始整数（无符号！无符号！）
// [解析]       min/max 该物理量的量程端点（例如力矩 -20.0f ~ 20.0f，单位 N·m）
// [解析]       bits 这个字段在报文里占多少位（常见 12 或 16）
// [解析] 返回：换算后的物理量，范围 [min, max]。
// [解析] 例：uint_to_float(0, -20.0f, 20.0f, 12) = -20.0f；传 4095 则得到 +20.0f。
// [解析] 实现见 math_tools.cpp，使用例子见 dm_motor.cpp 的反馈解析。
float uint_to_float(uint32_t input, float min, float max, size_t bits);

// [min, max]
// [解析] 物理量 -> 定点数（上面那个的逆运算）。把物理量按位数映射成 0 ~ 2^bits - 1 的整数，
// [解析] 以便打包进 CAN 报文的位域里发出去（见 dm_motor.cpp 的 write/write_mit）。
// [解析] 参数 input：物理量，单位与 min/max 一致；返回：uint32_t 整数。
// [解析] 注意1：结果是"直接截断"取整，不是四舍五入，最大会有不到 1 个 LSB 的误差。
// [解析] 注意2：内部没有限幅。如果 input 超出 [min, max]，返回值会超出该字段能表示的范围，
// [解析]   打包成位域时就会"溢出"到相邻字段去。所以调用前请自己先 limit_min_max。
uint32_t float_to_uint(float input, float min, float max, size_t bits);

// sgn
// [解析] 符号函数：正数返回 1，负数返回 -1，0 返回 0（返回类型 int8_t）。
// [解析] 常用来决定"该往哪个方向转/该加还是该减"。
// [解析] 注意：输入是 NaN 时两个比较都为假，会返回 0（不会崩，但要想清楚是否符合预期）。
int8_t sgn(float input);

// 三维向量数值微分：result = (v_next - v_prev) / dt
// 输入：v_next[3] 下一时刻向量，v_prev[3] 上一时刻向量
// 输出：result[3] 微分结果（速度/加速度等）
// dt: 时间间隔，默认 1ms
// [解析] 用一个"差分"近似求导：结果 ≈ 两帧之差 ÷ 时间间隔，单位是"输入单位/秒"。
// [解析] 例如输入位置(m)、dt(s)，输出就是速度(m/s)；输入速度、输出就是加速度。
// [解析] 参数：三个数组长度都必须是 3；result 是输出参数，会被写满 3 个元素（原地修改！）。
// [解析] dt 默认 1e-3f 秒 = 1kHz 控制周期，和本项目的主控制周期一致。
// [解析] 注意：dt 不能为 0，否则会算出 inf/NaN；另外数值微分对噪声非常敏感，
// [解析]   输入信号一般要先过一遍低通滤波（见 tools/low_pass_filter）。
void diff_vec3(const float v_next[3], const float v_prev[3], float result[3], float dt = 1e-3f);

}  // namespace sp

#endif  // SP__MATH_TOOLS_HPP