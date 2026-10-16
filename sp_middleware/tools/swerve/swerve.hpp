// [解析] ==========================================================================
// [解析] 【文件】sp_middleware/tools/swerve/swerve.hpp
// [解析] 【作用】四轮独立转向底盘（swerve，俗称"舵轮底盘"）的运动学：
// [解析]          正解 calc()  ：底盘想要的速度 vx/vy/wz -> 4 个轮子的转速 + 4 个舵的角度；
// [解析]          反解 update()：传感器读到的 4 轮转速 + 4 舵角度 -> 底盘实际速度 vx/vy/wz。
// [解析] 【被谁用】上层"底盘任务/底盘类"里放一个 sp::Swerve 成员，
// [解析]         每个控制周期先 calc()，再把 speed_xx 下发给 4 个轮电机、angle_xx 下发给 4 个舵电机。
// [解析] 【关键点】1) 单位统一是：长度 m、角度 rad、平动 m/s、角速度 rad/s、轮转速 rad/s；
// [解析]   2) init() 只是"记零位"，调用瞬间 4 个舵轮的朝向必须一致（这是最容易做错的一步）；
// [解析]   3) convert() 里的 cos(a) 是舵轮底盘的精髓：舵角差得太多时干脆让"轮子反转"代替转 180°，
// [解析]     详见 swerve.cpp 里那一大段注释。
// [解析] 【阅读顺序】先看本文件了解成员和单位，再看 swerve.cpp 看公式怎么来的。
// [解析] ==========================================================================

#ifndef SP__SWERVE_HPP
#define SP__SWERVE_HPP

namespace sp
{
// [解析] Swerve：一个对象 = 一台四舵轮底盘。
// [解析] 【坐标系约定 · 必须先看懂这个，否则公式全是天书】
// [解析]   底盘本体坐标系，原点在底盘的旋转中心：
// [解析]     x 轴 = 车头正前方，y 轴 = 车身正左方，z 轴 = 正上方（右手系）；
// [解析]     vx = 前进速度(m/s)，vy = 左移速度(m/s)；
// [解析]     wz = 绕 z 轴的角速度(rad/s)，逆时针（从上往下看）为正 —— 即"右手大拇指朝上"的螺旋方向。
// [解析]   四个轮子在底盘坐标系中的位置，用 (前后偏移, 左右偏移) 表示：
// [解析]     LF 左前 = (+half_length, +half_width)     RF 右前 = (+half_length, -half_width)
// [解析]     LR 左后 = (-half_length, +half_width)     RR 右后 = (-half_length, -half_width)
// [解析]   轮子不是凭空跑的：它的速度 = 底盘平动速度 + 底盘自转带出来的切向速度，
// [解析]     即 v_轮 = v_底盘 + wz × r_轮位置。calc() 里那 4 行 { vx ± wz*w, vy ± wz*l } 就是它展开的结果。
// [解析] 【典型调用流程】
// [解析]   sp::Swerve swerve(0.05f, 0.2f, 0.2f);       // 轮半径 5cm、半轴距 20cm、半轮距 20cm
// [解析]   swerve.init(yaw_lf, yaw_lr, yaw_rf, yaw_rr);  // 四舵同向时记零位，只调一次
// [解析]   每个控制周期：
// [解析]     swerve.calc(vx, vy, wz, 四个舵的当前角度);
// [解析]     舵电机_lf.cmd(swerve.angle_lf);  轮电机_lf.cmd(swerve.speed_lf);  // 其余三个同理
// [解析]   想做速度估计时再调 update(四个轮速, 四个舵角)，读 swerve.vx/vy/wz。
class Swerve
{
public:
  // [解析] 构造函数：把几何参数存成常量，并把 invert_pivot 翻译成 sign_ = ±1。不碰硬件。
  // [解析]   wheel_radius —— 轮子半径(m)，只用于"线速度 m/s <-> 轮转速 rad/s"的换算。
  // [解析]     注意量的是"轮胎实际滚动半径"，压扁后会比标称值略小；传 0 会导致除零(inf/nan)，绝不能传 0。
  // [解析]   half_length  —— 前轮中心到后轮中心距离的一半(m)，内部记为 l_。
  // [解析]   half_width   —— 左轮中心到右轮中心距离的一半(m)，内部记为 w_。
  // [解析]     l_ / w_ 只影响"自转 wz 带给每个轮子的速度增量有多大"，
  // [解析]     量错的表现是"原地旋转时轮速偏大或偏小"，直行时看不出来。
  // [解析]   invert_pivot —— 舵电机（转向电机）是否倒装，默认 true。
  // [解析]     true -> sign_ = -1（推荐保持默认，详见 swerve.cpp 里关于符号约定的说明）。
  // [解析] 副作用：无（只存参数）。
  // wheel_radius: 轮子半径, 单位: m
  // half_length: 前后轮距离的一半, 单位: m
  // half_width: 左右轮距离的一半, 单位: m
  // invert_pivot: 各舵电机倒置
  Swerve(float wheel_radius, float half_length, float half_width, bool invert_pivot = true);

  // [解析] ↓ 8 个"只读"输出成员，由 calc() 一次性刷新，单位见行尾注释。
  // [解析]   命名规则：speed_xx 给轮电机，angle_xx 给舵电机，xx = lf/lr/rf/rr（左前/左后/右前/右后）。
  float speed_lf;  // 只读! calc()计算结果, 左前轮转速, 单位: rad/s
  // [解析] ↑ 轮子的"角"速度，是"轮子转多快"（rad/s），不是"车跑多快"(m/s)。
  // [解析]   想换成轮缘线速度，乘轮半径 r_ 即可（update() 里就是这么做的）。
  float speed_lr;  // 只读! calc()计算结果, 左后轮转速, 单位: rad/s
  // [解析] ↑ 同上，左后轮。
  float speed_rf;  // 只读! calc()计算结果, 右前轮转速, 单位: rad/s
  // [解析] ↑ 同上，右前轮。可以是负数：负数表示这个轮子往"反方向"转。
  float speed_rr;  // 只读! calc()计算结果, 右后轮转速, 单位: rad/s
  // [解析] ↑ 同上，右后轮。

  float angle_lf;  // 只读! calc()计算结果, 左前舵角度, 单位: rad
  // [解析] ↑ 舵电机应该转到的"绝对角度"(rad)。注意它是"已经加回安装零位"的值，
  // [解析]   也就是直接下发给舵电机 cmd() 的数，不需要你再去加 yaw_offset。
  float angle_lr;  // 只读! calc()计算结果, 左后舵角度, 单位: rad
  // [解析] ↑ 同上，左后舵。
  float angle_rf;  // 只读! calc()计算结果, 右前舵角度, 单位: rad
  // [解析] ↑ 同上，右前舵。
  float angle_rr;  // 只读! calc()计算结果, 右后舵角度, 单位: rad
  // [解析] ↑ 同上，右后舵。

  // [解析] ↓ 3 个"只读"输出成员，由 update()（不是 calc()）刷新，是底盘实际速度的估计值。
  float vx;  // 只读! update()计算结果, 底盘x方向速度, 单位: m/s
  // [解析] ↑ 底盘实际前进速度估计(m/s)，正 = 往前。
  float vy;  // 只读! update()计算结果, 底盘y方向速度, 单位: m/s
  // [解析] ↑ 底盘实际左移速度估计(m/s)。注意它的符号约定与 calc() 的输入相反，见 swerve.cpp。
  float wz;  // 只读! update()计算结果, 底盘z方向角速度, 单位: rad/s
  // [解析] ↑ 底盘实际自转角速度估计(rad/s)。同样与 calc() 输入的正方向相反，见 swerve.cpp。

  // [解析] ↓ 中间变量：由 update() 算出的"每个轮子的线速度矢量"，单位 m/s。
  // [解析]   下标 x/y 是底盘坐标系的两个分量；它们已经包含"自转带来的切向速度"。
  // [解析]   平时不用管，调试打滑/单轮异常时对比 4 组数据很有用。
  float v_lf_x, v_lf_y; //轮子线速度在底盘坐标系x轴、y轴分量
  // [解析] ↑ 左前轮线速度的 x 分量和 y 分量，m/s。
  float v_lr_x, v_lr_y;
  // [解析] ↑ 左后轮的两个分量，m/s。
  float v_rf_x, v_rf_y;
  // [解析] ↑ 右前轮的两个分量，m/s。
  float v_rr_x, v_rr_y;
  // [解析] ↑ 右后轮的两个分量，m/s。

  float wz_lf ; //轮子对底盘角速度的分量
  // [解析] ↑ 只拿"左前轮一个轮子"反解出来的底盘角速度，单位 rad/s。
  // [解析]   理论上 4 个值应该相同，实际的差异来自测量噪声/打滑；最终发布的 wz 是它们的平均值。
  // [解析]   排查故障很好用：某个轮子打滑或编码器坏了，它这一项会明显偏离其他三个。
  float wz_lr ;
  // [解析] ↑ 左后轮单独反解出的角速度，rad/s。
  float wz_rf ;
  // [解析] ↑ 右前轮单独反解出的角速度，rad/s。
  float wz_rr;
  // [解析] ↑ 右后轮单独反解出的角速度，rad/s。

  // [解析] 记录四个舵的"安装零位"：把此刻四个舵的读数存起来，之后算舵角时都拿它当基准。
  // [解析]   yaw_lf/lr/rf/rr —— 左前/左后/右前/右后舵电机当前的角度，单位 rad。
  // [解析] ★ 调用条件（最容易做错的一步）：调用这一瞬间，四个轮子必须朝向同一个方向
  // [解析]   （比如四个轮都朝正前方）。绝对不能用"两两对称相对"的姿态来标定，
  // [解析]   否则四个零位各不相同，之后算出的舵角会全盘错乱、车会乱扭。
  // [解析] 调用时机：上电、舵电机回零/校准完成之后，底盘还没开始动之前，调用一次。
  // [解析] 副作用：只改 yaw_offset_xx_ 这 4 个私有成员，不读硬件、不下指令。
  // yaw_lf/lr/rf/rr: 各舵电机的初始角度, 注意此时各轮电机输出轴朝向要相同, 而不是相对! 单位: rad
  void init(float yaw_lf, float yaw_lr, float yaw_rf, float yaw_rr);

  // [解析] 正解：底盘"想要"的运动 -> 四个舵的角度 + 四个轮的速度。每个控制周期调一次。
  // [解析]   vx/vy/wz —— 底盘期望速度，单位 m/s、m/s、rad/s（旋转正方向见 hpp 开头的坐标系说明）。
  // [解析]   yaw_lf/lr/rf/rr —— 四个舵"当前真实朝向"的角度，单位 rad，
  // [解析]     一般直接传舵电机反馈的 angle。★ 这个参数很关键：convert() 靠它判断
  // [解析]     "是就近小角度转过去，还是让轮子反转 180°"，传错会导致轮子乱转。
  // [解析] 返回值：无。结果全部写进成员：
  // [解析]   angle_lf ~ angle_rr —— 四个舵的目标角度(rad)，下发给舵电机；
  // [解析]   speed_lf ~ speed_rr —— 四个轮的目标转速(rad/s)，下发给轮电机。
  // [解析] 注意：当底盘速度和平动/自转全为 0 时，convert() 会输出"角度 = 当前角度、速度 = 0"，
  // [解析]   也就是停车时舵保持不动，避免原地来回抖舵。
  // 底盘速度 -> 各轮转速和各舵角度
  // vx: 前进速度, 单位: m/s
  // vy: 左移速度, 单位: m/s
  // wz: 单位: rad/s, 正方向: 大拇指朝上右手螺旋方向
  // yaw_lf/lr/rf/rr: 各舵当前角度, 单位: rad
  void calc(float vx, float vy, float wz, float yaw_lf, float yaw_lr, float yaw_rf, float yaw_rr);

  // [解析] 反解（速度估计）：把四个轮子的实测转速和四个舵的实测角度，还原成底盘实际速度。
  // [解析]   speed_lf/lr/rf/rr —— 四个轮电机的反馈转速，单位 rad/s；
  // [解析]   angle_lf/lr/rf/rr —— 四个舵电机的反馈角度，单位 rad。
  // [解析] 算法：每个轮子先"转速 -> 线速度矢量"，再单独反解出一个底盘角速度，
  // [解析]   平动速度取四轮平均、角速度取四个反解值的平均（详见 swerve.cpp 的 update()）。
  // [解析] 返回值：无。结果写进 vx/vy/wz 以及 v_xx_x/v_xx_y、wz_xx 这些调试用成员。
  // [解析] ★ 注意符号约定：update() 输出的 vy 和 wz 与 calc() 输入的正方向是相反的
  // [解析]   （等价于把 y 轴取反）。如果你想把 update() 的结果直接引回 calc() 做速度闭环，
  // [解析]   必须先确认方向、必要时取负，否则会变成正反馈。详见 swerve.cpp 里的说明。
  // [解析] 用它的场合：判断底盘实际有没有按预期运动（打滑检测、超功率保护、里程估计）。
  void update(
    float speed_lf, float speed_lr, float speed_rf, float speed_rr, float angle_lf, float angle_lr,
    float angle_rf, float angle_rr);


private:
  // [解析] ↓ 四个几何/方向常量：构造时定死，之后永不改变（所以是 const，想改只能重新构造对象）。
  // [解析]   r_ = 轮半径(m)、l_ = 半轴距(m)、w_ = 半轮距(m)、sign_ = 舵电机方向 ±1。
  // [解析]   r_ 用于线速度<->轮转速换算，l_/w_ 用于算自转带来的切向速度，sign_ 用于统一舵角方向。
  const float r_;
  const float l_;
  const float w_;
  const float sign_;

  // [解析] ↓ 四个舵的安装零位（rad），即 init() 时记下的角度。
  // [解析]   作用：把"舵电机的绝对读数"换算成"轮子相对底盘的朝向"。
  // [解析]   calc() 里是"减零位"，update()/convert() 里是"加回去"。
  float yaw_offset_lf_;
  float yaw_offset_lr_;
  float yaw_offset_rf_;
  float yaw_offset_rr_;

  // 各轮速度向量 -> 各轮转速和各舵角度
  // [解析] 单个轮子的换算函数（私有，只被 calc() 调用 4 次）。
  // [解析]   v[2]       —— 该轮"想要的速度矢量"，底盘坐标系，单位 m/s，v[0]=x 分量、v[1]=y 分量；
  // [解析]   yaw        —— 该舵当前真实角度(rad)；
  // [解析]   yaw_offset —— 该舵的安装零位(rad)；
  // [解析]   angle      —— 输出参数：该舵应该转到的目标角度(rad)；
  // [解析]   speed      —— 输出参数：该轮应该有的转速(rad/s)。
  // [解析] ★ 后两个是引用（&）而不是返回值：函数会直接改调用者传进来的变量，
  // [解析]   所以 calc() 里写 convert(v_lf, ..., this->angle_lf, this->speed_lf) 就填好了成员。
  // [解析] 注意：v[2] 形参数组会退化成指针，函数内部不检查长度，调用者必须保证至少有 2 个元素。
  void convert(const float v[2], float yaw, float yaw_offset, float & angle, float & speed);
};

}  // namespace sp

#endif  // SP__SWERVE_HPP