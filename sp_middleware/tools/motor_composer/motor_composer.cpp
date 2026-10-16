// [解析] ==========================================================================
// [解析] 【文件】sp_middleware/tools/motor_composer/motor_composer.cpp
// [解析] 【作用】motor_composer.hpp 里声明的 6 个函数的具体实现，每个都只有一两行公式。
// [解析] 【看代码的重点】
// [解析]   1) 构造函数把 bool 翻译成 ±1.0f，之后所有公式都靠"乘符号"统一方向；
// [解析]   2) 为什么角度/速度除以 2 而力矩不除（描述位姿 vs 描述总出力）；
// [解析]   3) this-> 前缀不是多余的：成员名和参数名同名（比如 speed），不写就变成了参数自己。
// [解析] 【阅读顺序】建议先看 hpp 了解成员含义和单位，再回来看这里的公式。
// [解析] ==========================================================================

#include "motor_composer.hpp"

namespace sp
{
// [解析] 构造函数只做一件事：把两个 bool 在"初始化列表"里翻译成 ±1.0f。
// [解析]   : sign_l_((reverse_l) ? -1.0f : 1.0f) —— 逗号左边是成员名，括号里是给它的初值。
// [解析]   写成初始化列表而不是在函数体里赋值，是因为 hpp 里 sign_l_/sign_r_ 声明成了 const，
// [解析]   const 成员只能在构造时初始化，函数体里再赋值会直接编译报错。
// [解析] 函数体是空的：本类不碰硬件、不读电机，所以没有别的事可做。
MotorComposer::MotorComposer(bool reverse_l, bool reverse_r)
: sign_l_((reverse_l) ? -1.0f : 1.0f), sign_r_((reverse_r) ? -1.0f : 1.0f)
{
}

// [解析] 记零位：把传进来的两个角度原样存下来，仅此而已。
// [解析]   之后 calc_angle() 会用 (angle_l - angle_l0_) 得到"相对零位转过的角度"。
// [解析] 调用时机：电机使能完成、机械处于你认定的初始姿态时调用一次。
// [解析] 副作用：只改 angle_l0_ / angle_r0_ 两个成员，不读硬件、不下指令。
// [解析] 注意：这里存的是电机原始角度，没有乘 sign_，所以两台电机的编码器零位必须"本身就是对齐的"，
// [解析]   否则等效角度会带一个固定偏差（不影响增量，但会影响绝对位置的判断）。
void MotorComposer::init_angles(float angle_l, float angle_r)
{
  angle_l0_ = angle_l;
  angle_r0_ = angle_r;
}

// [解析] 等效角度 = 两路"减零位、统一方向"之后的平均值。
// [解析]   逐项拆开看：
// [解析]     angle_l - angle_l0_ ：左电机相对零位的转角(rad)；
// [解析]     sign_l_ * (...)     ：把左电机的正方向统一到"等效正方向"；
// [解析]     右电机同理；
// [解析]     最后 / 2            ：取平均，得到唯一的等效角度。
// [解析] 注意 this-> 前缀：成员和参数同名，写成 angle 会是参数（这里是赋值给成员 this->angle）。
// [解析] 返回值：无。结果放在公开成员 this->angle 里，单位 rad。
void MotorComposer::calc_angle(float angle_l, float angle_r)
{
  this->angle = (sign_l_ * (angle_l - angle_l0_) + sign_r_ * (angle_r - angle_r0_)) / 2;
}

// [解析] 等效速度 = 两路"乘符号"之后的平均值（速度没有零位，所以不减 angle_xx0_）。
// [解析]   为什么用 /2 取平均而不是相加：两台电机本应同步转动，它们描述的是"同一个速度"，
// [解析]   取平均可以略微平滑掉测量噪声；而力矩是"总出力"，所以要相加，见下面 calc_torque()。
// [解析] 单位：输入 rad/s，结果 this->speed 也是 rad/s。
// [解析] 注意：两边速度不一致（打滑、机械间隙、电机故障）时，平均值会把这个差异掩盖掉，
// [解析]   如果你需要检测"两个电机是否同步"，请自己比较原始的两个 speed。
void MotorComposer::calc_speed(float speed_l, float speed_r)
{
  this->speed = (sign_l_ * speed_l + sign_r_ * speed_r) / 2;
}

// [解析] 等效力矩 = 两路"乘符号"之后相加（注意这里不除以 2）。
// [解析]   物理意义：两台电机都挂在同一个执行机构上，机构承受的总力矩就是两者之和。
// [解析]   如果把这里写成 /2，那么用它做力矩闭环时阈值就只能给到实际值的两倍大，容易过载。
// [解析] 单位：输入 N·m，结果 this->torque 也是 N·m。
// [解析] 注意：本函数只"读回来"力矩，本类没有对应的"把力矩拆成两路"的函数，
// [解析]   所以用力矩模式控制时，两路指令要自己在外面分别计算。
void MotorComposer::calc_torque(float torque_l, float torque_r)
{
  this->torque = sign_l_ * torque_l + sign_r_ * torque_r;
}

// [解析] 反向操作：把"一个等效速度"拆成两台电机各自要执行的速度指令。
// [解析]   因为 sign*sign = 1，所以拆出来的值再经过 calc_speed() 会原样还原成传入的 speed，
// [解析]   两个函数在数学上是自洽的（这也是"同一个符号要同时用在合成和拆分上"的原因）。
// [解析] 单位：输入 speed 为 rad/s，输出 speed_l / speed_r 也是 rad/s。
// [解析] 副作用：写公开成员 this->speed_l / this->speed_r；不调用电机、不发 CAN 帧。
// [解析] 注意：拆出来的是"指令"，别忘了真正下发给电机（motor_l.cmd(speed_l) 之类）。
void MotorComposer::calc_speeds(float speed)
{
  this->speed_l = sign_l_ * speed;
  this->speed_r = sign_r_ * speed;
}

}  // namespace sp
