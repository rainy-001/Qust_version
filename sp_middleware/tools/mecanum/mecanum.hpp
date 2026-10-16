// [解析] ==========================================================================
// [解析] 【文件】sp_middleware/tools/mecanum/mecanum.hpp
// [解析] 【作用】麦克纳姆轮（简称"麦轮"）底盘的"运动学解算器"。
// [解析]         正解 calc()：给定整车速度 (vx, vy, wz) -> 算出四个轮子各自该转多快。
// [解析]         反解 update()：给定四个轮子的实际转速 -> 还原出整车速度（做里程计用）。
// [解析] 【被谁用】本仓库里暂时没有调用者（它属于中间件工具库，需要哪个底盘就引哪个）。
// [解析]         上层 applications/ 里的底盘任务自己创建对象，例如：
// [解析]           sp::Mecanum mecanum(0.06f, 0.2f, 0.2f);   // 半径 6cm、半轴距/半轮距 20cm
// [解析]           mecanum.calc(vx, vy, wz);                // 每个控制周期调一次
// [解析]         （这套中间件源自同济 SP 战队的开源电控 sp_middleware，故保持与上游一致的接口。）
// [解析] 【关键点】1) lf / lr / rf / rr 分别是 左前 / 左后 / 右前 / 右后；
// [解析]           2) reverse_xx 是"电机装配方向"补偿：电机装反了就在这里翻符号，
// [解析]              千万不要去改公式里的加减号，否则正解和反解会同时对不上；
// [解析]           3) 长度一律用米(m)，线速度用 m/s，角速度用 rad/s，轮速用 rad/s。
// [解析]              单位弄错（比如把 mm 当 m 传）结果会差 1000 倍，且很难查。
// [解析] 【阅读顺序】先看本文件知道有哪些接口，再看 mecanum.cpp 看公式怎么来的。
// [解析] ==========================================================================

#ifndef SP__MECANUM_HPP
#define SP__MECANUM_HPP

// [解析] 引用同工程的数学工具头文件（里面是 SP_PI、限幅、角度展开等小工具）。
// [解析] 注意：本类的实现其实只用到加减乘除，并没有用到 math_tools 里的任何函数，
// [解析]   但工程里各个 tools 子目录都保持统一写法，所以这里照旧引入。
#include "tools/math_tools/math_tools.hpp"

namespace sp
{
// [解析] Mecanum 类：一个对象代表一辆麦轮底盘。
// [解析] 构造时把尺寸参数和转向标志一次性存进私有成员（都是 const，之后不能再改）。
// [解析] 之后每个控制周期调用 calc() 求目标轮速，或者用 update() 做里程计。
// [解析] 注意：这两个函数都没有返回值！结果靠读对象的公共成员变量 speed_xx / vx / vy / wz。
class Mecanum
{
public:
  // [解析] 构造函数：只登记参数，不做任何计算，也不碰硬件。
  // [解析] 典型取值（以 RM 步兵 120mm 麦轮为例）：
  // [解析]   wheel_radius ≈ 0.06 m（就是 60mm，注意是"半径"不是直径）
  // [解析]   half_length  ≈ 0.20 m（左前轮到左后轮的中心距 / 2，即半轴距）
  // [解析]   half_width   ≈ 0.20 m（左前轮到右前轮的中心距 / 2，即半轮距）
  // [解析] 量的时候要量"轮子与地面接触点"之间的距离，不要量轮子外沿的宽度。
  // wheel_radius: 轮子半径, 单位: m
  // half_length: 前后轮距离的一半, 单位: m
  // half_width: 左右轮距离的一半, 单位: m
  // reverse_lf: left-front反向旋转
  // reverse_lr: left-rear反向旋转
  // reverse_rf: right-front反向旋转
  // reverse_rr: right-rear反向旋转
  // [解析] 默认值之所以是 lf/lr = false、rf/rr = true，是因为麦轮左右两侧的辊子方向
  // [解析]   是镜像对称的：同一套"往前开"的动作，左右两侧电机收到的指令符号必须相反，
  // [解析]   所以构造时默认就把右侧取反了。
  // [解析] （注意：这只是"指令符号"的约定，不是说车往前走时右轮物理上在倒转。）
  // [解析] 如果实车上发现某个轮子转向和预期相反，正确做法是把这个轮对应的 reverse_xx
  // [解析]   传 true（而不是去改公式，也不是去改电调设置），这样正反解仍然自洽。
  Mecanum(
    float wheel_radius, float half_length, float half_width, bool reverse_lf = false,
    bool reverse_lr = false, bool reverse_rf = true, bool reverse_rr = true);

  // [解析] ---- 输出区 1：calc() 的结果（整车速度 -> 轮速，通常作为电机 PID 的目标值）----
  // [解析] 负值表示该轮反转。单位是"轮子自转的角速度"，可以直接当电机 PID 的目标值；
  // [解析] （如果所用电机内部用的是"转子侧"角度，还差一个减速比的换算，详见 motor 目录。）
  float speed_lf;  // 只读! calc()计算结果, left-front转速, 单位: rad/s
  float speed_lr;  // 只读! calc()计算结果, left-rear转速, 单位: rad/s
  float speed_rf;  // 只读! calc()计算结果, right-front转速, 单位: rad/s
  float speed_rr;  // 只读! calc()计算结果, right-rear转速, 单位: rad/s

  // [解析] ---- 输出区 2：update() 的结果（轮速 -> 整车速度，里程计/打点用）----
  // [解析] 底盘坐标系约定：x 轴指向车头前方，y 轴指向车身左侧，z 轴竖直向上，
  // [解析] 所以 wz > 0 表示"从上往下看逆时针转"（右手螺旋，大拇指朝上）。
  float vx;  // 只读! update()计算结果, 底盘x方向速度, 单位: m/s
  float vy;  // 只读! update()计算结果, 底盘y方向速度, 单位: m/s
  float wz;  // 只读! update()计算结果, 底盘z方向角速度, 单位: rad/s

  // [解析] calc()：正运动学，把"我想要的整车速度"翻译成"四个轮子要转多快"。
  // [解析] 调用时机：每个控制周期（一般 1ms 或 5ms）在底盘任务里调一次，
  // [解析]   紧接着把 speed_lf 等四个值分别送给四个电机的 PID 作为目标。
  // [解析] 参数是"期望值"，不是实测值；wz 是角速度而不是角度，不要直接把遥控器的
  // [解析]   拨杆量当角度传进来。
  // 底盘速度 -> 各轮转速
  // vx: 前进速度, 单位: m/s
  // vy: 左移速度, 单位: m/s
  // wz: 大拇指朝上，右手螺旋方向转速, 单位: rad/s
  void calc(float vx, float vy, float wz);

  // [解析] update()：反运动学，把四个轮子的实测转速还原成整车速度。
  // [解析] 调用时机：拿到电机反馈转速之后（比如在 CAN 接收回调或控制任务末尾），
  // [解析]   再对 vx/vy 积分就能得到车辆位置，对 wz 积分就能得到车头朝向（里程计）。
  // [解析] 注意它只做矩阵运算，不做滤波也不处理打滑，实测值抖动时结果也会抖。
  // 各轮转速 -> 底盘速度
  // speed_lf: left-front转速, 单位: rad/s
  // speed_lr: left-rear转速, 单位: rad/s
  // speed_rf: right-front转速, 单位: rad/s
  // speed_rr: right-rear转速, 单位: rad/s
  void update(float speed_lf, float speed_lr, float speed_rf, float speed_rr);

private:
  // [解析] 下面 3 个是几何参数，构造时由参数决定，单位都是 m。
  // [解析] 注意 l_ + w_ 在公式里反复出现：麦轮解算里的"等效旋转半径"就是半轴距 + 半轮距。
  // [解析] 打个比方：绕底盘中心原地自转时，每个轮子接触点画出的圆半径并不是 l_ 或 w_，
  // [解析]   而是它们在"轮子滚动方向"上的投影之和，也就是 (l_ + w_)。
  const float r_;
  const float l_;
  const float w_;
  // [解析] 下面 4 个是转向符号，取值只有 +1.0f 或 -1.0f，方便在公式里直接相乘翻符号。
  // [解析] 用 float 而不是 int，是为了和公式里其它 float 运算保持同一类型、少一次隐式转换。
  const float sign_lf_;
  const float sign_lr_;
  const float sign_rf_;
  const float sign_rr_;
};

}  // namespace sp

#endif  // SP__MECANUM_HPP