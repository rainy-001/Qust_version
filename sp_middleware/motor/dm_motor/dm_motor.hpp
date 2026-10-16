// [解析] ==========================================================================
// [解析] 【文件】sp_middleware/motor/dm_motor/dm_motor.hpp
// [解析] 【作用】封装达妙(DM)系列电机(如 J4310)：把 CAN 收到的 8 字节原始数据翻译成
// [解析]         角度/速度/力矩/温度，并把上层给的目标值打包成 8 字节 CAN 数据帧。
// [解析] 【被谁用】tools/joint/joint_controller.cpp 里有
// [解析]           `template class JointMotorController<sp::DM_Motor>;`
// [解析]           ——它被当作"关节电机"的模板参数使用，再由上层机器人配置创建具体对象。
// [解析] 【关键点】DM 电机的物理量是"定点数"传输：12 位或 16 位的整数按量程线性映射到
// [解析]           [-max, +max]。构造函数传进来的 pmax/vmax/tmax 必须和电机在上位机
// [解析]           (达妙调试助手)里设置的量程一致，否则读出来的角度/力矩全是错的值。
// [解析] 【阅读顺序】先看本文件了解"有什么功能"，再看 dm_motor.cpp 了解"怎么实现的"。
// [解析] ==========================================================================

#ifndef SP__DM_MOTOR_HPP
#define SP__DM_MOTOR_HPP

#include <cstdint>
// [解析] 本文件只用到固定宽度整型(uint16_t / uint32_t / int32_t)，所以只依赖这个头文件。

namespace sp
{
// [解析] J4310 电机的额定最大力矩 = 7.0 N·m。
// [解析] 注意：它有减速箱，这里的 7 N·m 指的是"输出轴"上的力矩，不是电机转子上的力矩。
// [解析] 目前项目里没有别处引用它(搜索结果只有本行)，属于给使用者准备的参考常量。
constexpr float J4310_MAX_TORQUE = 7.0f;  // N·m

// [解析] DM_Motor 类：一个对象 = 一台达妙电机。
// [解析] 典型使用流程(和 can_task.cpp 里的 RM 电机套路一样)：
// [解析]   1) 构造：sp::DM_Motor m(0x01, 0x11, 12.5f, 30.0f, 7.0f);  // 具体量程按电机配置填
// [解析]   2) 使能：m.write_enable(can.tx_data); can.send(m.tx_id);
// [解析]   3) 发控制量：m.cmd(力矩) 或 m.cmd_mit(...) -> m.write(can.tx_data) -> can.send(...)
// [解析]   4) 收反馈：CAN 中断里 if (can.rx_id == m.rx_id) m.read(can.rx_data, 时间戳);
// [解析] 说明：cmd 系列函数只是"把目标值存进成员变量"，真正打包成字节的是 write 系列。
class DM_Motor
{
public:
  // [解析] 构造函数：只把参数存进成员变量，不碰硬件、不发任何 CAN 帧。
  // [解析] can_id / master_id 是电机在"达妙调试助手"里设置的两个 ID，收发各用一个：
  // [解析]   主机发给电机用 can_id，电机回报给主机用 master_id。
  // [解析] pmax/vmax/tmax 是定点数的"量程上限"，直接决定 CAN 数据里 1 个最低位代表多少物理量：
  // [解析]   例如 pmax=12.5rad 且用 16 位传输时，角度分辨率 = 2*12.5/65535 ≈ 0.00038 rad。
  // [解析]   这三个值填错不会编译报错，只会让读到的角度/力矩整体偏大或偏小，非常难查。
  // can_id: 电机控制帧ID, 对应tx_id
  // master_id: 电机反馈帧ID, 对应rx_id
  // pmax: position最大值, 单位: rad
  // vmax: velocity最大值, 单位: rad/s
  // tmax: torque最大值, 单位: N·m
  DM_Motor(
    uint16_t can_id, uint16_t master_id, float pmax, float vmax, float tmax,
    bool multi_circle = false);
  // [解析] 最后的 multi_circle 默认 false：false 时角度只在 [-pmax, pmax] 内单圈变化；
  // [解析]   true 时会把转过的整圈数累计起来，输出连续的角度(见 dm_motor.cpp 的 read())。

  // [解析] 下面三个 ID 都带 const，说明"构造完就固定"，因为它们在电机上电前就已经配置好了。
  const uint16_t rx_id;      // 电机反馈帧ID
  // [解析] ↑ 收到的 CAN 帧 ID 与它相等，就说明"这一帧是本电机的反馈"。
  const uint16_t tx_id;      // 电机控制帧ID
  const uint16_t tx_id_vel;  // 速度模式 控制帧ID (0x200 + CAN ID)
  // [解析] ↑ 速度模式的控制帧 ID 是 can_id + 0x200，这是达妙协议规定的固定偏移；
  // [解析]   注意是十六进制加法：can_id = 0x01 时 tx_id_vel = 0x201。

  // 只读! 8: 超压, 9: 欠压, A: 过流, B: MOS过温, C: 线圈过温, D: 通讯丢失, E: 过载
  // [解析] "只读"是指：只由 read() 在收到反馈帧时写入，用户代码不要去改它。
  // [解析] 具体含义看上面这行(数值是十六进制)：8=0x08 超压, 9=0x09 欠压, A=0x0A 过流,
  // [解析]   B=0x0B MOS 过温, C=0x0C 线圈过温, D=0x0D 通讯丢失, E=0x0E 过载。
  // [解析]   0 表示一切正常。排查"电机不动"时，第一个该看的就是它。
  uint8_t error = 0;

  float angle = 0;   // 只读! 单位: rad
  // [解析] ↑ 电机输出轴角度。单圈模式下范围约 [-pmax, pmax]；多圈模式下会一直累加。
  float speed = 0;   // 只读! 单位: rad/s
  // [解析] ↑ 输出轴角速度(不是转速 rpm)，正负号表示方向。
  float torque = 0;  // 只读! 单位: N·m
  // [解析] ↑ 输出轴实际力矩，可以用来做过载判断或力矩闭环反馈。

  float T_MOS = 0;    // 电机MOS管温度 单位: ℃
  // [解析] ↑ 驱动器 MOS 管温度。这个升得比线圈快，是过温保护的主要参考。
  float T_Rotor = 0;  // 电机转子温度 单位: ℃
  // [解析] ↑ 线圈/转子温度。持续大扭矩堵转时它会快速升高，注意别超 100℃ 太多。

  // [解析] 是否曾经收到过一帧反馈：用来判断"电机是否已经上电并连上总线"。
  bool is_open() const;
  // [解析] 是否"还活着"：既要求 open，又要求距上次收到反馈不超过 100ms(见 .cpp)。
  // [解析] now_ms：调用时传入的当前毫秒时间戳(通常来自 HAL_GetTick())。
  // [解析] 返回值含义：true = 通信正常；false = 掉线，上层应当停止发指令以免失控。
  bool is_alive(uint32_t now_ms) const;

  // [解析] read()：解析一帧反馈。data 指向 CAN 收到的 8 字节原始数组，stamp_ms 是当前毫秒数。
  // [解析] 调用时机：CAN 接收中断/任务里判断 rx_id 匹配之后调用。
  // [解析] 副作用：会改写 error/speed/torque/angle/T_MOS/T_Rotor 以及内部的时间戳。
  void read(uint8_t * data, uint32_t stamp_ms);
  // [解析] write()：把 cmd() 缓存的力矩打包成 8 字节，写进 data(一般是 can.tx_data)。
  void write(uint8_t * data) const;
  // [解析] write_enable()：打包"使能"专用帧(数据固定为 7 个 0xFF + 0xFC)。
  // [解析] 上电后必须发一次，电机才会输出力矩；否则你发什么它都不动。
  void write_enable(uint8_t * data) const;
  // [解析] write_disable()：打包"失能"帧(...+0xFD)，电机立刻卸力自由旋转。
  void write_disable(uint8_t * data) const;
  // [解析] write_clear_error()：打包"清错"帧(...+0xFB)，用于清掉 error 里的故障标志。
  void write_clear_error(uint8_t * data) const;

  // [解析] write_velocity()：速度模式专用打包，直接把 float 的 4 个字节拷进 data。
  void write_velocity(uint8_t * data) const;

  // [解析] cmd()：给"MIT 力控模式"下发力矩目标值。
  // [解析] torque 单位 N·m，超过 ±tmax 会被自动截断(内部调用 limit_max)，
  // [解析] 所以传一个很大的值不会报错，只是被限制在量程内。
  // MIT力控模式指令缓存, 单位: N·m
  void cmd(float torque);
  // [解析] cmd_velocity()：给"速度模式"下发目标转速，单位 rad/s，同样会自动截断到 ±vmax。
  // 速度模式指令缓存, 单位: rad/s
  void cmd_velocity(float velocity);

  // [解析] cmd_mit()：完整的 MIT 五参数控制指令(比单纯发力矩更灵活)。
  // [解析] 五个参数的物理含义(对应电机内部公式 t = kp*(p_des-p) + kd*(v_des-v) + t_ff)：
  // [解析]   p_des：目标位置，单位 rad，范围 ±pmax
  // [解析]   v_des：目标速度，单位 rad/s，范围 ±vmax
  // [解析]   kp   ：位置环增益，范围 0~500(内部会截断)，越大越"硬"地掰向目标位置
  // [解析]   kd   ：速度环(阻尼)增益，范围 0~5(内部会截断)，用来抑制抖动
  // [解析]   t_ff ：前馈力矩，单位 N·m，范围 ±tmax，用于抵消重力等已知负载
  // [解析] 想只做力矩控制就把 kp、kd 都设成 0；想只做速度控制就把 kp 设成 0。
  void cmd_mit(float p_des, float v_des, float kp, float kd, float t_ff);

  // MIT速度控制模式
  // velocity: 目标速度 (rad/s)
  // kd: 速度环增益 (建议根据实际负载调试，范围 0~5)
  // [解析] 它只是把 cmd_mit 简化了一下：位置和 kp 固定为 0，只保留速度环，
  // [解析] 等价于 cmd_mit(0, velocity, 0, kd, 0)。
  void cmd_mit_velocity(float velocity, float kd);

  // 完整的MIT模式数据打包
  // [解析] write_mit()：把 cmd_mit() 缓存的 5 个目标值，按达妙协议压成 8 个字节写入 data。
  // [解析] 这是本文件位运算最密集的地方，逐位含义在 dm_motor.cpp 里有详细标注。
  void write_mit(uint8_t * data) const;

private:
  // [解析] 以下三个是"量程上限"的副本，加上 _ 后缀表示私有成员。
  // [解析] 用 const 修饰是为了防止运行时被误改，因为改了读出来的物理量就全错了。
  const float pmax_;
  const float vmax_;
  const float tmax_;

  // [解析] has_read_：是否已经收到过第一帧反馈。
  // [解析] 初学者常见困惑："为什么 is_open() 一开始是 false？"——因为还没收到过帧。
  bool has_read_;
  // [解析] last_read_ms_：最近一次收到反馈的毫秒时间戳，is_alive() 用它算通信超时。
  uint32_t last_read_ms_;

  // [解析] cmd_torque_：cmd() 存下来的力矩目标值，等待 write() 打包发送。
  float cmd_torque_;
  // [解析] cmd_velocity_：cmd_velocity() 存下来的速度目标值，等待 write_velocity() 打包发送。
  float cmd_velocity_;

  //多圈计数
  // [解析] angle_raw_：本次反馈解析出的"单圈角度"，范围 ±pmax，会在 ±pmax 处跳变。
  float angle_raw_ = 0;
  // [解析] last_angle_raw_：上一次的单圈角度，用来判断有没有跨过量程边界(即转过一整圈)。
  float last_angle_raw_ = 0.0f;
  // [解析] circle_：累计圈数，可以为负(反转方向)。angle = angle_raw_ + circle_*2*pmax。
  int32_t circle_;
  // [解析] multi_circle_：是否启用上面的多圈累计；由构造函数最后一个参数决定。
  bool multi_circle_;

  // 新增 MIT 模式参数缓存
  // [解析] 下面 5 个就是 cmd_mit() 的五个输入参数，先缓存起来，等 write_mit() 再打包。
  // [解析] 初始值全为 0，意味着"没调用 cmd_mit 之前，发出去的是零力矩"。
  float cmd_p_des_ = 0.0f;
  float cmd_v_des_ = 0.0f;
  float cmd_kp_ = 0.0f;
  float cmd_kd_ = 0.0f;
  float cmd_t_ff_ = 0.0f;
};

}  // namespace sp

#endif  // SP__DM_MOTOR_HPP