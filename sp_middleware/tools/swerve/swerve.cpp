// [解析] ==========================================================================
// [解析] 【文件】sp_middleware/tools/swerve/swerve.cpp
// [解析] 【作用】舵轮底盘运动学的实现，一共 4 个函数：
// [解析]         构造函数（存参数）、init()（记零位）、calc()+convert()（正解）、update()（反解）。
// [解析] 【看代码的重点】1) calc() 里 { vx ± wz*w, vy ± wz*l } 那 4 行的物理意义（自转贡献的切向速度）；
// [解析]   2) convert() 为什么要先算两个候选朝向 v_angle / v_angle_flipped 再挑一个；
// [解析]   3) 最后乘的 cos(a) 为什么能实现"轮子反转代替转 180°"；
// [解析]   4) update() 的符号约定与 calc() 相反（等价于 y 轴取反），拿它做闭环要当心。
// [解析] ==========================================================================

#include "swerve.hpp"

#include "tools/math_tools/math_tools.hpp"

namespace sp
{
// [解析] 构造函数：用初始化列表把参数存成 const 成员，并把 invert_pivot 翻译成 sign_ = ±1。
// [解析]   invert_pivot ? -1.0f : 1.0f —— 舵电机倒装时取 -1，之后所有涉及"舵角"的公式都乘它。
// [解析]   这样一个符号就解决了"舵电机正装/倒装"的差异，不用在每个公式里写 if。
// [解析] 函数体是空的：本类不碰硬件，几何参数存下来就够了。
Swerve::Swerve(float wheel_radius, float half_length, float half_width, bool invert_pivot)
: r_(wheel_radius), l_(half_length), w_(half_width), sign_(invert_pivot ? -1.0f : 1.0f)
{
}

// [解析] 记零位：把四个舵当前的角度原样存下来，之后一律用"绝对角度 - 零位"来表示轮子朝向。
// [解析] ★ 调用这一瞬间四个轮子必须同向（比如都朝前），详见 hpp 里的说明。
// [解析] 副作用：只改四个成员，不读硬件、不下指令。
void Swerve::init(float yaw_lf, float yaw_lr, float yaw_rf, float yaw_rr)
{
  yaw_offset_lf_ = yaw_lf;
  yaw_offset_lr_ = yaw_lr;
  yaw_offset_rf_ = yaw_rf;
  yaw_offset_rr_ = yaw_rr;
}

// [解析] ★ 正解第一步（也是整个文件的核心思想）：先算出"每个轮子应该以多快的速度、朝哪个方向跑"。
// [解析] 物理公式：轮子的速度 = 底盘平动速度 + 底盘自转带来的切向速度，即 v_轮 = v_底盘 + ω × r_轮。
// [解析] 把 ω 取成绕 z 轴、轮子位置取成 (前后偏移, 左右偏移)，叉乘展开后就是下面 4 行：
// [解析]   下标 [0] 是"前后(x)方向"的分量，写成 vx ± wz*w_：里面那个 w_ 是半轮距，
// [解析]     因为轮子左右方向离旋转中心越远，底盘自转时把它带着走的前后速度就越大；
// [解析]   下标 [1] 是"左右(y)方向"的分量，写成 vy ± wz*l_：里面那个 l_ 是半轴距，
// [解析]     同理，轮子前后方向离旋转中心越远，自转带给它的左右速度就越大。
// [解析] 四个轮子的差别只在两个偏移量的正负号上：
// [解析]   LF(+l,+w)  LR(-l,+w)  RF(+l,-w)  RR(-l,-w)，对照 hpp 里的位置表。
// [解析] 单位：vx/vy 是 m/s，wz 是 rad/s，w_/l_ 是 m，wz*w_ 得到的仍是 m/s，量纲自洽。
// [解析] 注意：wz 为正 = 逆时针（从上往下看），此时左前轮会被"甩"向前，所以它的 [1] 分量是 +wz*l_。
void Swerve::calc(
  float vx, float vy, float wz, float yaw_lf, float yaw_lr, float yaw_rf, float yaw_rr)
{
  // 计算中间变量: 各轮速度向量
  float v_lf[2] = {vx - wz * w_, vy + wz * l_};
  float v_lr[2] = {vx - wz * w_, vy - wz * l_};
  float v_rf[2] = {vx + wz * w_, vy + wz * l_};
  float v_rr[2] = {vx + wz * w_, vy - wz * l_};

  // [解析] 正解第二步：把这 4 个速度矢量分别换算成"舵该转多少度、轮该转多快"。
  // [解析] 注意最后两个实参是 this->angle_xx / this->speed_xx，convert() 通过引用直接把结果写进成员，
  // [解析]   所以调用完这 4 行，对外输出的 8 个量就都准备好了。
  // [解析] 第 2、3 个实参分工：yaw_xx 是舵当前角度（用来选"就近转"还是"反转"），
  // [解析]   yaw_offset_xx_ 是安装零位（用来在绝对角度和相对角度之间换算）。
  // 计算各舵转角和各舵角度
  convert(v_lf, yaw_lf, yaw_offset_lf_, this->angle_lf, this->speed_lf);
  convert(v_lr, yaw_lr, yaw_offset_lr_, this->angle_lr, this->speed_lr);
  convert(v_rf, yaw_rf, yaw_offset_rf_, this->angle_rf, this->speed_rf);
  convert(v_rr, yaw_rr, yaw_offset_rr_, this->angle_rr, this->speed_rr);
}

// [解析] 单个轮子的换算：输入"这个轮子想要的速度矢量"，输出"舵的目标角度 + 轮的目标转速"。
// [解析] 参数：v[2] = 速度矢量(m/s，v[0] 是 x 分量、v[1] 是 y 分量)；yaw = 舵当前绝对角度(rad)；
// [解析]       yaw_offset = 该舵安装零位(rad)；angle/speed 是引用输出参数（直接改调用者的变量）。
// [解析] 返回值：无（结果在 angle / speed 里）。不读硬件、不发指令，纯计算。
void Swerve::convert(const float v[2], float yaw, float yaw_offset, float & angle, float & speed)
{
  // ref: https://github.com/rm-controls/rm_controllers/blob/master/rm_chassis_controllers/src/swerve.cpp

  // [解析] 特判：这个轮子一点速度都不要（底盘平动和自转都恰好为 0）时，
  // [解析]   保持当前舵角不动、转速给 0，然后直接返回。
  // [解析] 为什么必须特判：atan2(0, 0) 虽然不崩，但结果是没有意义的 0，
  // [解析]   不特判的话停车时每个舵都会拼命往 0 度转，出现"车停了四个舵还在乱摆"的现象。
  if (v[0] == 0.0f && v[1] == 0.0f) {
    angle = yaw;
    speed = 0.0f;
    return;
  }

  // [解析] v_angle：这个轮子"希望前进的方向"，单位 rad，范围 (-π, π]。
  // [解析]   atan2(y, x) 比 atan(y/x) 好：它能正确区分四个象限，x=0 时也不会除零。
  float v_angle = std::atan2(v[1], v[0]);
  // [解析] v_angle_flipped：把上面的方向反过来 180°（+π）。
  // [解析]   用途："轮子装着不动、只把转速取反"和"舵转 180°"是等效的；
  // [解析]   如果当前舵角离这个反向更近，就选它，省掉一次大角度打舵 —— 这是舵轮底盘的精髓。
  // [解析]   +SP_PI 之后再过 limit_angle()，把结果也折回 (-π, π]，方便后面比较差值大小。
  float v_angle_flipped = limit_angle(v_angle + SP_PI);
  // [解析] pivot_angle：把"舵当前朝向"换算到与 v_angle 同一个坐标系里。
  // [解析]   先减 yaw_offset 得到"轮子相对底盘的真实朝向"，再乘 sign_ 统一舵电机的安装方向。
  // [解析]   注意 limit_angle 保证结果落在 (-π, π]，这样下面做减法不会绕远路。
  float pivot_angle = sign_ * limit_angle(yaw - yaw_offset);

  // [解析] a、b：两个候选方向分别与"舵当前朝向"的夹角（已折到 (-π, π]）。
  // [解析]   取绝对值就是"要转过的角度大小"，越小越划算。
  // [解析]   a 对应正常方向，b 对应掉头 180° 的方向。
  float a = limit_angle(v_angle - pivot_angle);
  float b = limit_angle(v_angle_flipped - pivot_angle);
  // [解析] 就近原则：谁需要转的角度小就选谁。注意比较的是 |a| 与 |b|，
  // [解析]   因为 a、b 本身带符号，直接比大小会得出错误结论。
  float pivot_angle_set = (std::abs(a) < std::abs(b)) ? v_angle : v_angle_flipped;

  // [解析] 输出舵角：把选中的方向乘回 sign_（抵消 pivot_angle 里的符号），再加回安装零位，
  // [解析]   得到"舵电机应该读到的绝对角度"，单位 rad，最后再折一次到 (-π, π]。
  // [解析]   推导：由 pivot_angle = sign_*limit_angle(yaw - offset) 反解出
  // [解析]   yaw = offset + sign_*pivot_angle_set（sign_ 是 ±1，乘两次等于没乘）。
  angle = limit_angle(sign_ * pivot_angle_set + yaw_offset);
  // [解析] 输出轮速：线速度大小 |v| = sqrt(vx²+vy²)，除以轮半径 r_ 得到轮子该转多快，单位 rad/s。
  // [解析] ★ 关键是最后乘的 cos(a)，它是本函数的点睛之笔：
  // [解析]   ① 如果刚才选中的是"掉头方向"，那么 a 会接近 ±π，cos(a) ≈ -1，
  // [解析]      于是 speed 自动变成负数 —— 舵不用真的转 180°，靠"轮子反转"等效实现；
  // [解析]   ② 如果选中的是"正常方向"，a 是个小角度，cos(a) 接近 +1（一般不等于 1，因为舵还没转到位），
  // [解析]      相当于顺手做了一点"没对准就先降速"的补偿，避免轮子斜着蹭地。
  // [解析]   注意 a 用的是"目标方向与当前舵角之差"，也就是舵还没转过去的那个角度，
  // [解析]     所以车会一边打舵一边按比例加速，转到位时 cos(a) 才回到 1。
  // [解析]   行尾被注释掉的 * std::cos(a) * std::cos(a) 是作者试过的备选写法（相当于把投影平方），
  // [解析]     补偿更狠、加速更慢，最终选了只乘一次。原样保留，供调试时切换。
  speed = std::sqrt(v[0] * v[0] + v[1] * v[1]) / r_ * std::cos(a) ;//* std::cos(a) * std::cos(a);
}


// [解析] 反解：四个轮子的实测转速/舵角 -> 底盘实际速度。用途是速度估计、打滑判断、超功率保护。
// [解析] 参数：speed_xx 是四个轮电机的反馈转速(rad/s)，angle_xx 是四个舵电机的反馈角度(rad)。
// [解析] 思路分三步：① 每个轮子"转速 -> 线速度矢量"；② 四轮平均得到平动速度；
// [解析]   ③ 每个轮子单独反解一个角速度，再平均得到底盘角速度。
void Swerve::update(
  float speed_lf, float speed_lr, float speed_rf, float speed_rr, float angle_lf, float angle_lr,
  float angle_rf, float angle_rr)
{
  // 角度修正
  // [解析] 先减掉安装零位，把"舵电机的绝对角度"变成"轮子相对底盘的真实朝向"，单位 rad。
  // [解析] ★ 注意这里没有乘 sign_：舵角的符号翻转被放到了下面 v_xx_y 的负号里一起处理，
  // [解析]   这也正是 update() 输出符号与 calc() 相反的原因（详见本函数末尾的说明）。
  float theta_lf = limit_angle(angle_lf - yaw_offset_lf_);
  float theta_lr = limit_angle(angle_lr - yaw_offset_lr_);
  float theta_rf = limit_angle(angle_rf - yaw_offset_rf_);
  float theta_rr = limit_angle(angle_rr - yaw_offset_rr_);

  // 各轮线速度分解（车体坐标系）
  // [解析] 把"轮子转多快(rad/s)"还原成"轮子以多快的速度往哪跑(m/s)"：
  // [解析]   轮缘线速度大小 = 角速度 × 半径 = speed_xx * r_，单位 m/s；
  // [解析]   方向由 theta_xx 决定，分解成 x/y 两个分量。
  // [解析]   注意 y 分量前面有负号，写成 (cos, -sin)；这与 convert() 里
  // [解析]     atan2(v[1], v[0]) 隐含的 (cos, +sin) 并不一致，属于 y 轴取向的差异。
  // [解析]   在默认 invert_pivot = true（sign_ = -1）时这套符号是自洽的，
  // [解析]     所以这两处不能只改一处，改符号前请先想清楚整车坐标系。
  this->v_lf_x = speed_lf * r_ * std::cos(theta_lf);
  this->v_lf_y = - speed_lf * r_ * std::sin(theta_lf);

  // [解析] 左后轮同理，(cos, -sin) 分解，单位 m/s。
  this->v_lr_x = speed_lr * r_ * std::cos(theta_lr);
  this->v_lr_y = - speed_lr * r_ * std::sin(theta_lr);

  // [解析] 右前轮同理。
  this->v_rf_x = speed_rf * r_ * std::cos(theta_rf);
  this->v_rf_y = - speed_rf * r_ * std::sin(theta_rf);

  // [解析] 右后轮同理。这 4 组 v_xx_x / v_xx_y 同时也是公开的调试成员。
  this->v_rr_x = speed_rr * r_ * std::cos(theta_rr);
  this->v_rr_y = - speed_rr * r_ * std::sin(theta_rr);

  // 位置向量
  // [解析] 四个轮子在底盘坐标系里的位置，单位 m，写成 (前后偏移, 左右偏移)：
  // [解析]   第 0 个分量是"沿车长方向"的偏移（±半轴距 l_），
  // [解析]   第 1 个分量是"沿车宽方向"的偏移（±半轮距 w_）。
  // [解析]   注意它既不是 (x, y) 的直观顺序、也不是"左前/右前/左后/右后"的常见顺序，
  // [解析]   顺序固定为 LF、LR、RF、RR，与上面 v_xx_* 的排列一一对应，别记混。
  // [解析]   LF(+l,+w) 与 RR(-l,-w) 是一条对角线，LR(-l,+w) 与 RF(+l,-w) 是另一条。
  const float pos[4][2] = {
    {+l_, +w_},  // LF
    {-l_, +w_},  // LR
    {+l_, -w_},  // RF
    {-l_, -w_},  // RR
  };

  // 平均线速度
  // [解析] 底盘平动速度 = 四个轮子线速度的平均值。
  // [解析]   为什么可以这样算：四个轮子都跟着底盘一起平动，这部分速度完全相同；
  // [解析]   而自转带来的那部分切向速度在四个轮子上方向各异，取平均后正好互相抵消。
  // [解析]   写成 / 4.0f（浮点字面量）而不是 / 4，避免整数除法把结果截断成整数。
  // [解析]   单位 m/s。结果稍后写进公开成员 vx / vy。
  float vx_sum = (v_lf_x + v_lr_x + v_rf_x + v_rr_x) / 4.0f;
  float vy_sum = (v_lf_y + v_lr_y + v_rf_y + v_rr_y) / 4.0f;

  // 平均角速度
  // [解析] 单轮反解角速度：分子是"速度分量与位置分量交叉相乘再相减"，
  // [解析]   也就是位置向量与速度向量叉乘的 z 分量（这里写成了 v_x*p_y - v_y*p_x 的次序）；
  // [解析]   分母 l²+w² 是位置向量模长的平方，也就是该轮到旋转中心距离的平方。
  // [解析]   物理含义：绕中心转动的点，它的"旋转分量速度"除以半径就是角速度 ω = (r × v)/|r|²。
  // [解析] ★ 注意：这个次序算出来的 ω 与 calc() 里 wz 的正方向是相反的（等价于把 y 轴取反），
  // [解析]   如果你拿 update() 的 wz 去做旋转闭环，方向会反，需要自己取负。
  // [解析] 为什么要算 4 个再平均：每个轮子单独解出来的 ω 里都混进了"底盘平动"的干扰项，
  // [解析]   但 4 个轮子的干扰项正负恰好两两抵消（LF/LR/RF/RR 的 ±l_、±w_ 组合），
  // [解析]   所以平均之后剩下的就是真正的角速度。
  this->wz_lf = (v_lf_x * pos[0][1] - v_lf_y * pos[0][0]) / (l_ * l_ + w_ * w_);
  this->wz_lr = (v_lr_x * pos[1][1] - v_lr_y * pos[1][0]) / (l_ * l_ + w_ * w_);
  this->wz_rf = (v_rf_x * pos[2][1] - v_rf_y * pos[2][0]) / (l_ * l_ + w_ * w_);
  this->wz_rr = (v_rr_x * pos[3][1] - v_rr_y * pos[3][0]) / (l_ * l_ + w_ * w_);
  // [解析] 四个轮子反解值的平均，同样除以 4.0f（轮子个数），单位 rad/s。
  float wz_sum = (wz_lf + wz_lr + wz_rf + wz_rr) / 4.0f;

  // [解析] 发布结果：底盘平动速度(m/s) 和角速度(rad/s)，供上层做速度估计/打滑判断。
  // [解析] ★ 再次提醒：这里的 vy 和 wz 的符号约定与 calc() 的输入相反，
  // [解析]   不要把 update() 的输出不加处理地引回 calc()（会变成正反馈）。
  this->vx = vx_sum;
  this->vy = vy_sum;
  this->wz = wz_sum;
}

}  // namespace sp