// [解析] ==========================================================================
// [解析] 【文件】sp_middleware/tools/mecanum/mecanum.cpp
// [解析] 【作用】mecanum.hpp 里声明的三个函数的具体实现。核心其实就是 4 行公式，
// [解析]         一行对应一个轮子，把整车速度 (vx, vy, wz) 拆给四个轮子。
// [解析] 【被谁用】上层底盘控制任务（applications/ 里）创建 sp::Mecanum 对象后调用：
// [解析]         calc() 每个控制周期调一次，把结果喂给四个电机的 PID；
// [解析]         update() 拿到电机反馈转速后调一次，做里程计。
// [解析] 【看代码的重点】四个地方最容易看不懂，下面逐条标注：
// [解析]   1) 四个公式里 vx / vy / wz 前面的加减号是怎么来的（记忆口诀见 calc()）；
// [解析]   2) (l_ + w_) 是什么物理量，为什么 wz 要乘它；
// [解析]   3) 为什么最后要除以 r_（单位换算：m/s -> rad/s）；
// [解析]   4) update() 为什么能"精确反解"（因为矩阵列正交，最小二乘退化成精确解）。
// [解析] 【阅读顺序】先看本文件顶部的公式解释，再回头看 mecanum.hpp 的接口会更顺。
// [解析] ==========================================================================

// [解析] 只包含本类自己的头文件。mecanum.hpp 里已经带了 math_tools，这里不用重复包含。
#include "mecanum.hpp"

namespace sp
{
// [解析] 构造函数：函数体里没有硬件操作，只有"参数登记"。
// [解析] : r_(wheel_radius) 这种写法叫"初始化列表"，作用和 r_ = wheel_radius; 一样，
// [解析]   但对于 const 成员（本类全是 const）只能这么写，不能放到大括号里赋值。
// [解析] 初始化列表的执行顺序只跟"成员在类里声明的顺序"有关，跟这里写的顺序无关；
// [解析]   本类碰巧两边顺序一致，所以没问题。
Mecanum::Mecanum(
  float wheel_radius, float half_length, float half_width, bool reverse_lf, bool reverse_lr,
  bool reverse_rf, bool reverse_rr)
: r_(wheel_radius),
  l_(half_length),
  w_(half_width),
  sign_lf_((reverse_lf) ? -1.0f : 1.0f),
  sign_lr_((reverse_lr) ? -1.0f : 1.0f),
  sign_rf_((reverse_rf) ? -1.0f : 1.0f),
  sign_rr_((reverse_rr) ? -1.0f : 1.0f)
{
  // [解析] 把四个输出变量清零，避免对象刚创建、还没调过 calc() 时读到随机值。
  // [解析] 这是"防御性编程"：新手常见 bug 就是先读结果后调函数，读到栈上的垃圾数据。
  this->speed_lf = 0.0f;
  this->speed_lr = 0.0f;
  this->speed_rf = 0.0f;
  this->speed_rr = 0.0f;
}

// [解析] calc()：正运动学。输入"我期望整车怎么动"，输出"四个轮子各转多快"。
// [解析] 【三个输入】
// [解析]   vx：沿车头方向的速度，+ 为前进，单位 m/s；典型遥控车速 1~4 m/s。
// [解析]   vy：沿车身左侧方向的速度，+ 为向左平移，单位 m/s。
// [解析]   wz：绕竖直轴的角速度，+ 为俯视逆时针，单位 rad/s；原地自转一般 1~6 rad/s。
// [解析] 【副作用】写 this->speed_lf / speed_lr / speed_rf / speed_rr（单位 rad/s）。
// [解析] 【陷阱】三个形参的名字和成员变量 vx / vy / wz 同名！
// [解析]   在函数体里写 vx 指的是"形参"，写 this->vx 才是"成员"。
// [解析]   本函数只用了形参，所以没问题；但你自己改代码时千万别写错这一个 this->。
// [解析] 【调用时机】控制周期里先调 calc()，再把四个 speed_xx 分别给电机 PID 当目标值。
void Mecanum::calc(float vx, float vy, float wz)
{
  // [解析] 正解公式（每行算出一个轮子的线速度，最后 /r_ 转成角速度）：
  // [解析]   轮子线速度 = vx  ± vy  ± (l_ + w_) * wz
  // [解析] 用"线性叠加"拆成三部分来理解，三项互不干扰：
  // [解析]   ① 纯前进 vx：四个轮子都得转，所以四行都有 +vx；
  // [解析]   ② 纯左移 vy：麦轮辊子与轮轴成 45°，横移要靠四轮差速完成，
  // [解析]       左前与右后这一对"反向"，左后与右前这一对"正向"
  // [解析]       → 所以 vy 的符号是 左前 -、左后 +、右前 +、右后 -；
  // [解析]   ③ 纯自转 wz：(l_ + w_) 是"轮子接地点绕底盘中心的等效旋转半径"，
  // [解析]       对应物理式 线速度 = 角速度 × 半径，故乘 (l_ + w_)；
  // [解析]       符号按左右分：左侧两轮 -、右侧两轮 +（逆时针时右侧轮往前推）。
  // [解析] 记忆口诀：vx 全正；vy 反对角同号；wz 左负右正。
  // [解析] 最后 /r_：线速度(m/s) 除以 轮半径(m) = 角速度(rad/s)，这才是电机认的量。
  // [解析] sign_xx_ 只在最外层乘一次：公式先按"标准装配朝向"算，
  // [解析]   再统一翻符号，这样"数学对不对"和"电机线接没接反"两件事互不干扰。
  // [解析] 注意：这里没有做任何限幅。如果上位机给的 vx 特别大，算出来的轮速会超电机上限，
  // [解析]   限幅应该在这一层之后、或者给 PID 目标值那一步做（不要指望本函数帮你兜底）。
  // ref: https://modernrobotics.northwestern.edu/nu-gm-book-resource/13-2-omnidirectional-wheeled-mobile-robots-part-1-of-2
  this->speed_lf = sign_lf_ * (vx - vy - (l_ + w_) * wz) / r_;
  this->speed_lr = sign_lr_ * (vx + vy - (l_ + w_) * wz) / r_;
  this->speed_rf = sign_rf_ * (vx + vy + (l_ + w_) * wz) / r_;
  this->speed_rr = sign_rr_ * (vx - vy + (l_ + w_) * wz) / r_;
}

// [解析] update()：反运动学（里程计）。输入四个轮子的实际转速，反推整车速度。
// [解析] 【四个输入】speed_lf/lr/rf/rr：各轮实测转速，单位 rad/s。一般来自电机反馈
// [解析]   （例如 RM 电机回传的转子转速除以减速比），正负号代表转向。
// [解析] 【副作用】写 this->vx / this->vy / this->wz（单位分别是 m/s、m/s、rad/s）。
// [解析] 【和 calc() 的关系】calc() 是"正着算"，update() 是"倒着算"，两者互为逆运算：
// [解析]   把 calc() 的输出直接喂给 update()，理论上应该还原出原来的 (vx, vy, wz)。
// [解析]   实车调试时可以用这一条来验证公式和符号有没有接错。
// [解析] 【陷阱】和 calc() 一样，四个形参名和成员变量 speed_xx 同名，
// [解析]   函数体里裸写的 speed_lf 是形参，this->speed_lf 才是 calc() 留下的旧结果。
// [解析] 【调用时机】拿到电机反馈之后调；对 vx/vy 积分得位置，对 wz 积分得车头朝向。
// [解析]   本函数不做滤波，轮速抖动会 1:1 传到结果里，需要平滑请自己外面加低通。
void Mecanum::update(float speed_lf, float speed_lr, float speed_rf, float speed_rr)
{
  // [解析] 第一步：先把"电机侧的量"还原成"按标准装配方向、单位 m/s 的轮子线速度"。
  // [解析]   ÷ sign_xx_ 就是 × sign_xx_（符号只有 ±1，除以 -1 和乘 -1 效果一样），
  // [解析]   作用是先抵消掉构造时登记的装配方向，后面的公式才能用统一的标准约定推导。
  // [解析]   × r_ 是单位换算：rad/s × m = m/s。
  float sign_speed_lf = speed_lf / sign_lf_;
  float sign_speed_lr = speed_lr / sign_lr_;
  float sign_speed_rf = speed_rf / sign_rf_;
  float sign_speed_rr = speed_rr / sign_rr_;

  // [解析] 第二步：反解出整车速度。系数规律正好是 calc() 的"转置"：
  // [解析]   vx = ( +u_lf +u_lr +u_rf +u_rr ) / 4 × r_            ← 全加
  // [解析]   vy = ( -u_lf +u_lr +u_rf -u_rr ) / 4 × r_            ← 反对角反号
  // [解析]   wz = ( -u_lf -u_lr +u_rf +u_rr ) / 4 × r_ / (l_+w_)  ← 左负右正
  // [解析] 其中 u_xx 就是上面算出来的轮子线速度。
  // [解析] 为什么能这么简单地"加起来除以 4"？
  // [解析]   麦轮的正解可以写成一个 4×3 矩阵 H（4 个轮子 × 3 个自由度）：u = H·(vx,vy,wz)。
  // [解析]   H 不是方阵，没有普通逆矩阵；但它的三列两两正交（点乘为 0），
  // [解析]   于是 H^T·H 退化成对角阵，最小二乘解 (H^T H)^-1 H^T u 里的逆矩阵
  // [解析]   就只剩"除以列模长平方"，也就是那个 /4 和 /(l_+w_) 的来源 —— 结果是精确解，
  // [解析]   不是近似值。这也是为什么麦克纳姆轮能做到"不损失自由度"的里程计。
  // [解析] 自检小技巧（推不出结果时可以这样验算）：
  // [解析]   假设车在原地纯自转、四个轮子线速度大小都是 u，按 calc() 的符号规律
  // [解析]   有 u_lf = u_lr = -u、u_rf = u_rr = +u，代入 wz 那一行：
  // [解析]   (-(-u) - (-u) + (+u) + (+u)) / 4 / (l_+w_) = u / (l_+w_)，
  // [解析]   而物理上 u = (l_+w_)·ω，正好还原出 ω，说明公式自洽。
  // [解析] 注意：wz 那一行除了除以 4，还要再多除一个 (l_ + w_)，
  // [解析]   因为轮子线速度到角速度之间隔了一个旋转半径。
  this->vx = (sign_speed_lf + sign_speed_lr + sign_speed_rf + sign_speed_rr) / 4 * r_;
  this->vy = (-sign_speed_lf + sign_speed_lr + sign_speed_rf - sign_speed_rr) / 4 * r_;
  this->wz = (-sign_speed_lf - sign_speed_lr + sign_speed_rf + sign_speed_rr) / 4 * r_ / (l_ + w_);
}

}  // namespace sp
