// [解析] 差速齿轮的解算器：两个输入电机的转角经一套锥齿轮叠加成末端 pitch（共模）/ roll（差模）两个自由度。
// [解析] 上层云台/关节任务先 init_gear_angle() 标定零位，再用 calc_end_*() 取反馈、calc_gear_speed() 算指令。
// [解析] 两个坑：angle_l0_/angle_r0_ 构造时未初始化，不先调 init 就是随机值；ratio_ 在力矩里是"乘"不是"除"。

#ifndef SP__DIFF_GEAR_HPP
#define SP__DIFF_GEAR_HPP

namespace sp
{
// [解析] 输出都是"相对开机零位"的累计量，不归一化到 ±π —— 关节可能转好几圈，累计值才能反映真实位置。
class DiffGear
{
public:
  // ratio: 锥齿轮减速比
  // reverse_l: left反向旋转
  // reverse_r: right反向旋转
  DiffGear(float ratio, bool reverse_l = false, bool reverse_r = true);

  float pitch;    // 只读! calc_end_angle()计算结果, 单位: rad
  float roll;     // 只读! calc_end_angle()计算结果,  单位: rad
  float v_pitch;  // 只读! calc_end_speed()计算结果, 单位: rad/s
  float v_roll;   // 只读! calc_end_speed()计算结果, 单位: rad/s
  float t_pitch;  // 只读! calc_end_torque()计算结果, 单位: N·m
  float t_roll;   // 只读! calc_end_torque()计算结果, 单位: N·m
  float v_left;   // 只读! calc_gear_speed()计算结果, left转速, 单位: rad/s
  float v_right;  // 只读! calc_gear_speed()计算结果, right转速, 单位: rad/s

  // [解析] 上电后必须先调一次：运行中反复调等于每周期都把当前姿态当零位，输出永远是 0。
  // 初始化各齿轮转角
  // angle_l: 左侧齿轮转角, 单位: rad
  // angle_r: 右侧齿轮转角, 单位: rad
  void init_gear_angle(float angle_l, float angle_r);

  // [解析] 共模取平均得 pitch，差模取差值再 /ratio_ 得 roll（锥齿轮副：齿轮转 ratio_ 圈、末端才转 1 圈）。
  // 各齿轮转角 -> 末端转角
  // angle_l: 左侧齿轮转角, 单位: rad
  // angle_r: 右侧齿轮转角, 单位: rad
  void calc_end_angle(float angle_l, float angle_r);

  // 各齿轮转速 -> 末端转速
  // speed_l: 左侧齿轮转速, 单位: rad/s
  // speed_r: 右侧齿轮转速, 单位: rad/s
  void calc_end_speed(float speed_l, float speed_r);

  // [解析] 力矩是速度的功率共轭量，所以既没有 /2、ratio_ 也从"除"变"乘"：转速降 ratio_ 倍则力矩升 ratio_ 倍。
  // [解析] 未计传动效率，结果是理想上限。
  // 各齿轮转矩 -> 末端转矩
  // torque_l: 左侧齿轮转矩, 单位: N·m
  // torque_r: 右侧齿轮转矩, 单位: N·m
  void calc_end_torque(float torque_l, float torque_r);

  // [解析] 反解：v_roll*ratio_ 正是 calc_end_speed() 里 /ratio_ 的逆运算，结果已含 sign_，可直接下发电机。
  // 末端转速 -> 各齿轮转速
  // v_pitch: 大拇指朝左，右手螺旋方向转速, 单位: rad/s
  // v_roll: 大拇指朝前，右手螺旋方向转速, 单位: rad/s
  void calc_gear_speed(float v_pitch, float v_roll);

private:
  const float ratio_;
  const float sign_l_;
  const float sign_r_;

  float angle_l0_;
  float angle_r0_;
};

}  // namespace sp

#endif  // SP__DIFF_GEAR_HPP