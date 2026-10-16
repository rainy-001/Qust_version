// [解析] ==========================================================================
// [解析] 【文件】sp_middleware/tools/motor_composer/motor_composer.hpp
// [解析] 【作用】把"机械上并排/对装的两台电机"在数学上等效成"一台电机"来思考和控制。
// [解析]         两个方向：
// [解析]           calc_angle / calc_speed / calc_torque —— 读反馈：两个电机 -> 一个等效量；
// [解析]           calc_speeds                            —— 下指令：一个等效量 -> 两个电机。
// [解析] 【被谁用】上层机构类（例如升降、伸缩、夹爪这种"两根电机一起推同一根轴"的机构）。
// [解析]         典型写法：在机构类里放一个 sp::MotorComposer composer_ 成员，
// [解析]         然后在每个控制周期里：先 calc_* 把反馈合成一个量，再 PID 算，最后 calc_speeds 拆回去。
// [解析] 【关键点】1) 两个电机的"正方向"往往相反（左右镜像安装），所以要用 reverse_l/reverse_r
// [解析]         把符号统一到同一个物理方向，否则"取平均"那一步两边会直接抵消成 0；
// [解析]         2) 力矩是"相加"（两台电机一起出力），角度和速度是"取平均"（它们本该同步）；
// [解析]         3) 用之前必须先 init_angles() 记零位，等效角度是"相对零位"的角度。
// [解析] 【阅读顺序】先看本文件了解有哪些量、单位是什么，再看 motor_composer.cpp 看公式。
// [解析] ==========================================================================

#ifndef SP__MOTOR_COMPOSER_HPP
#define SP__MOTOR_COMPOSER_HPP

namespace sp
{
// [解析] MotorComposer：双电机合成器。内部只保存两个方向符号和两个零位角度，
// [解析] 不做任何硬件操作、不加锁、不延时，在同一个控制周期里串行调用即可。
// [解析] 【典型调用流程】
// [解析]   sp::MotorComposer composer(false, true);   // 左正装、右倒装
// [解析]   composer.init_angles(motor_l.angle, motor_r.angle);  // 使能后、开始控制前，记零位
// [解析]   每个控制周期：
// [解析]     composer.calc_angle(motor_l.angle, motor_r.angle);    // -> composer.angle  (rad)
// [解析]     composer.calc_speed(motor_l.speed, motor_r.speed);    // -> composer.speed  (rad/s)
// [解析]     composer.calc_torque(motor_l.torque, motor_r.torque); // -> composer.torque (N·m)
// [解析]     pid.calc(target, composer.angle);                    // 上层闭环
// [解析]     composer.calc_speeds(pid.out);                       // 拆成两个电机的速度指令
// [解析]     motor_l.cmd(composer.speed_l); motor_r.cmd(composer.speed_r);
class MotorComposer
{
public:
  // [解析] 构造函数：只把 reverse_l / reverse_r 翻译成 ±1.0f 存起来，不碰硬件、不读电机。
  // [解析]   reverse_l / reverse_r —— 该侧电机的"实际正方向"与"期望正方向"相反时为 true。
  // [解析]     false -> 符号 +1；true -> 符号 -1。
  // [解析]     默认 false / true 表示"左边正装、右边倒装"，这是左右镜像机械最常见的装法。
  // [解析]   为什么用"乘 ±1"而不是 if 取反：全部公式统一成 sign*值 的形式，
  // [解析]     写起来短，也不会漏掉某一处忘了反号。
  // [解析]   注意：符号传错的典型现象是"两个量互相抵消"，表现为等效角度/速度恒为 0，很好判断。
  // [解析] 副作用：无（只存两个数）。
  // 将两个电机等效为一个电机
  // reverse_l: 左侧电机反向旋转
  // reverse_r: 右侧电机反向旋转
  MotorComposer(bool reverse_l = false, bool reverse_r = true);

  // [解析] ↓ 五个公开成员就是本类的"输入输出接口"：不靠函数返回值传递数据，
  // [解析]   而是 calc_xxx() 算完直接写进这些成员，外面读成员即可。
  // [解析]   带"只读!"是作者的约定：这些值是算出来的，请不要从外部改它，否则和内部状态对不上。
  float angle;    // 只读! calc_angle()计算结果, 单位: rad
  // [解析] ↑ 两台电机的"等效角度"，单位 rad（弧度，一圈 = 2π ≈ 6.283）。
  // [解析]   它是相对 init_angles() 那次记录的零位的角度，不是绝对角度，可以为负。
  float speed;    // 只读! calc_speed()计算结果, 单位: rad/s
  // [解析] ↑ 等效角速度，单位 rad/s，注意不是 rpm！1 圈/秒 = 2π ≈ 6.283 rad/s。
  float torque;   // 只读! calc_torque()计算结果, 单位: N·m
  // [解析] ↑ 两台电机力矩之"和"（不是平均），可以理解为这台"等效电机"的总出力。
  float speed_l;  // 只读! calc_speeds()计算结果, 单位: rad/s
  // [解析] ↑ 拆给"左侧"电机的速度指令（已经乘过 sign_l_），单位 rad/s。
  float speed_r;  // 只读! calc_speeds()计算结果, 单位: rad/s
  // [解析] ↑ 拆给"右侧"电机的速度指令（已经乘过 sign_r_），单位 rad/s。
  // [解析]   直接 motor_l.cmd(speed_l)、motor_r.cmd(speed_r) 用即可，不用再手动反号。

  // [解析] 记零位：把此刻两台电机的角度存起来，之后 calc_angle() 每次都先减去它们。
  // [解析]   angle_l / angle_r —— 左、右电机当前读到的角度，单位 rad，直接传 motor.angle 即可。
  // [解析] 调用时机：电机使能完成、机械停在"你希望当作零点"的姿态时，调用且只调用一次。
  // [解析]   如果忘了调用，angle_l0_ / angle_r0_ 默认是 0，等效角度就变成"绝对角度"，
  // [解析]   上电瞬间会有一个大跳变，PID 会立刻猛冲一下。
  // [解析] 副作用：只改两个私有成员，不读硬件、不下指令。
  // [解析] 注意：这里存的是"电机原始角度"，没有乘 sign_、也没有减 mid，
  // [解析]   所以两台电机编码器零位本身对不齐时，这个固定偏差会原样留在 angle 里。
  // 初始化各电机转角
  // angle_l: 左侧电机转角, 单位: rad
  // angle_r: 右侧电机转角, 单位: rad
  void init_angles(float angle_l, float angle_r);

  // [解析] 读反馈用：把两台电机的当前角度合成为一个等效角度，结果写进公开成员 this->angle。
  // [解析] 公式 = ( sign_l*(angle_l - 零位_l) + sign_r*(angle_r - 零位_r) ) / 2
  // [解析]   也就是"先各自减零位、再统一方向、最后取平均"。
  // [解析] 为什么要除以 2：两台电机描述的是"同一个等效角度"，取平均才是它的估计值。
  // [解析]   angle_l / angle_r 单位 rad（DM/RM 电机反馈的 angle 就是 rad）。
  // [解析] 返回值：无（结果在 this->angle）。典型用法：pid.calc(target, composer.angle)。
  // [解析] 注意：两台电机之间有机械间隙/编码器零位差时，这个偏差会带进等效角度里。
  // 各电机转角 -> 等效转角
  // angle_l: 左侧电机转角, 单位: rad
  // angle_r: 右侧电机转角, 单位: rad
  void calc_angle(float angle_l, float angle_r);

  // [解析] 与 calc_angle() 同理，只是速度没有"零位"的概念，不需要减零点。
  // [解析] 公式 = ( sign_l*speed_l + sign_r*speed_r ) / 2，结果写进 this->speed。
  // [解析]   speed_l / speed_r 单位 rad/s；结果也可直接当"等效电机的反馈速度"喂给速度环。
  // [解析] 注意：原注释把这里单位写成了 rad（上方成员 speed 的注释写的是 rad/s）。
  // [解析]   速度的单位是 rad/s，特此说明，原注释保持不动。
  // 各电机转速 -> 等效转速
  // speed_l: 左侧电机转速, 单位: rad
  // speed_r: 右侧电机转速, 单位: rad
  void calc_speed(float speed_l, float speed_r);

  // [解析] 公式 = sign_l*torque_l + sign_r*torque_r —— 注意这里是"相加"，不除以 2。
  // [解析] 为什么：力矩是可以叠加的。两台电机同时出力，等效电机受到的合力矩就是两者之和。
  // [解析]   对比记忆：角度/速度描述"同一个位姿"，只能取平均；力矩描述"总出力"，要相加。
  // [解析]   torque_l / torque_r 单位 N·m，结果写进 this->torque，单位 N·m。
  // [解析] 注意：如果上层拿它做力矩闭环，它代表的是"两台电机的总力矩"，
  // [解析]   阈值判断（比如夹紧力上限）要按总力矩来定。
  // 各电机转矩 -> 等效转矩
  // torque_l: 左侧电机转矩, 单位: N·m
  // torque_r: 右侧电机转矩, 单位: N·m
  void calc_torque(float torque_l, float torque_r);

  // [解析] 下发指令用：把"等效速度"按符号拆成两台电机各自的速度指令。
  // [解析] 公式 speed_l = sign_l*speed、speed_r = sign_r*speed，
  // [解析]   结果分别写进公开成员 this->speed_l / this->speed_r。
  // [解析] 与 calc_speed() 是自洽的：因为 sign*sign = 1，
  // [解析]   把拆出来的两个值再喂回 calc_speed() 得到的正好还是这里传入的 speed。
  // [解析] 参数 speed：等效角速度，单位 rad/s（通常就是速度环 PID 的输出）。
  // [解析] 注意：本类只提供"速度"的拆分，没有"力矩拆分"。
  // [解析]   如果想让两台电机出力不同（例如消间隙、防过载），需要自己在外面分别下 cmd。
  // 等效转速 -> 各电机转速
  // speed: 等效转速, 单位: rad/s
  void calc_speeds(float speed);

private:
  // [解析] ↓ 两个"方向符号"，构造时定死、之后永不改变（所以加了 const）。
  // [解析]   所有公式里都用"乘符号"的方式把两台电机统一到同一个物理正方向。
  // [解析]   取值为 +1.0f 或 -1.0f，不可能有别的值。
  const float sign_l_;
  const float sign_r_;

  // [解析] ↓ 两台电机在 init_angles() 那一刻的角度，作为"零位"。
  // [解析]   calc_angle() 每次都先减掉它们，所以等效角度是"从零位转过了多少"，单位 rad。
  float angle_l0_;
  float angle_r0_;
};

}  // namespace sp

#endif  // SP__MOTOR_COMPOSER_HPP