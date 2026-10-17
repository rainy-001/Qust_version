// [解析] 四轮独立转向（舵轮）底盘的运动学：calc() 正解出 4 个轮速 + 4 个舵角，update() 反解出底盘速度。
// [解析] 坐标系：x 车头前、y 车身左、z 朝上；四轮位置(前后,左右) LF(+l,+w) LR(-l,+w) RF(+l,-w) RR(-l,-w)。
// [解析] 单位 m / rad / m/s / rad/s；init() 必须四舵同向时调用；update() 的 vy/wz 符号与 calc() 相反。

#ifndef SP__SWERVE_HPP
#define SP__SWERVE_HPP

namespace sp
{
// [解析] 一个对象 = 一台四舵轮底盘。轮速 = 底盘平动 + 自转切向速度（v_轮 = v_底盘 + wz × r_轮）。
// [解析] 用法：init(四舵角) 记零位 -> 每周期 calc(vx,vy,wz,四舵角) -> 下发 angle_xx / speed_xx。
class Swerve
{
public:
  // [解析] 构造只存几何参数 r_ 轮半径、l_ 半轴距、w_ 半轮距；invert_pivot 翻译成 sign_ = ±1。
  // wheel_radius: 轮子半径, 单位: m
  // half_length: 前后轮距离的一半, 单位: m
  // half_width: 左右轮距离的一半, 单位: m
  // invert_pivot: 各舵电机倒置
  Swerve(float wheel_radius, float half_length, float half_width, bool invert_pivot = true);

  // [解析] 8 个只读输出由 calc() 刷新：speed_xx 给轮电机(rad/s)、angle_xx 给舵电机(rad)，xx = lf/lr/rf/rr。
  float speed_lf;  // 只读! calc()计算结果, 左前轮转速, 单位: rad/s
  float speed_lr;  // 只读! calc()计算结果, 左后轮转速, 单位: rad/s
  float speed_rf;  // 只读! calc()计算结果, 右前轮转速, 单位: rad/s
  float speed_rr;  // 只读! calc()计算结果, 右后轮转速, 单位: rad/s

  float angle_lf;  // 只读! calc()计算结果, 左前舵角度, 单位: rad
  float angle_lr;  // 只读! calc()计算结果, 左后舵角度, 单位: rad
  float angle_rf;  // 只读! calc()计算结果, 右前舵角度, 单位: rad
  float angle_rr;  // 只读! calc()计算结果, 右后舵角度, 单位: rad

  float vx;  // 只读! update()计算结果, 底盘x方向速度, 单位: m/s
  float vy;  // 只读! update()计算结果, 底盘y方向速度, 单位: m/s
  float wz;  // 只读! update()计算结果, 底盘z方向角速度, 单位: rad/s

  float v_lf_x, v_lf_y; //轮子线速度在底盘坐标系x轴、y轴分量
  float v_lr_x, v_lr_y;
  float v_rf_x, v_rf_y;
  float v_rr_x, v_rr_y;

  float wz_lf ; //轮子对底盘角速度的分量
  float wz_lr ;
  float wz_rf ;
  float wz_rr;

  // [解析] 记四个舵的安装零位，只调一次，且必须"四轮同向"时调用 —— 用两两对称的姿态标定会让舵角全盘错乱。
  // yaw_lf/lr/rf/rr: 各舵电机的初始角度, 注意此时各轮电机输出轴朝向要相同, 而不是相对! 单位: rad
  void init(float yaw_lf, float yaw_lr, float yaw_rf, float yaw_rr);

  // [解析] 正解：底盘期望速度 -> 四个舵角 + 四个轮速。yaw_xx 必须传舵的当前真实角度，convert() 靠它决定"就近转还是轮子反转"。
  // 底盘速度 -> 各轮转速和各舵角度
  // vx: 前进速度, 单位: m/s
  // vy: 左移速度, 单位: m/s
  // wz: 单位: rad/s, 正方向: 大拇指朝上右手螺旋方向
  // yaw_lf/lr/rf/rr: 各舵当前角度, 单位: rad
  void calc(float vx, float vy, float wz, float yaw_lf, float yaw_lr, float yaw_rf, float yaw_rr);

  // [解析] 反解（速度估计）：四轮实测转速/舵角 -> 底盘 vx/vy/wz；注意 vy/wz 与 calc() 输入的正方向相反。
  void update(
    float speed_lf, float speed_lr, float speed_rf, float speed_rr, float angle_lf, float angle_lr,
    float angle_rf, float angle_rr);

private:
  const float r_;
  const float l_;
  const float w_;
  const float sign_;

  float yaw_offset_lf_;
  float yaw_offset_lr_;
  float yaw_offset_rf_;
  float yaw_offset_rr_;

  // 各轮速度向量 -> 各轮转速和各舵角度
  // [解析] 单轮换算（私有，calc() 调 4 次）；angle/speed 是引用输出参数，直接传成员即可。
  void convert(const float v[2], float yaw, float yaw_offset, float & angle, float & speed);
};

}  // namespace sp

#endif  // SP__SWERVE_HPP