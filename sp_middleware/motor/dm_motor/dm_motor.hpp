// [解析] 达妙(DM)电机封装(如 J4310)：把 CAN 的 8 字节原始帧翻译成角度/速度/力矩/温度，并把目标值打包回去。
// [解析] 由 tools/joint/joint_controller.cpp 以 JointMotorController<sp::DM_Motor> 当关节电机使用，再由上层机器人配置创建对象。
// [解析] 物理量走定点数，1 个最低位代表多少物理量由构造时的 pmax/vmax/tmax 决定，必须和电机里设的量程一致。
// [解析] cmd_xxx() 只缓存目标值，write_xxx() 才打包成字节，真正发送由外层 can.send() 完成。

#ifndef SP__DM_MOTOR_HPP
#define SP__DM_MOTOR_HPP

#include <cstdint>

namespace sp
{
constexpr float J4310_MAX_TORQUE = 7.0f;  // N·m

// [解析] 一个对象 = 一台电机：反馈帧按 rx_id 匹配后 read()，控制帧发往 tx_id。
class DM_Motor
{
public:
  // [解析] 只存参数不碰硬件；量程填错不会报错，只会让读到的角度/力矩整体偏大或偏小，很难查；multi_circle 决定 read() 是否累计圈数。
  // can_id: 电机控制帧ID, 对应tx_id
  // master_id: 电机反馈帧ID, 对应rx_id
  // pmax: position最大值, 单位: rad
  // vmax: velocity最大值, 单位: rad/s
  // tmax: torque最大值, 单位: N·m
  DM_Motor(
    uint16_t can_id, uint16_t master_id, float pmax, float vmax, float tmax,
    bool multi_circle = false);

  const uint16_t rx_id;      // 电机反馈帧ID
  const uint16_t tx_id;      // 电机控制帧ID
  // [解析] tx_id_vel = can_id + 0x200，达妙速度模式规定的固定偏移(十六进制加法，can_id=0x01 时是 0x201)。
  const uint16_t tx_id_vel;  // 速度模式 控制帧ID (0x200 + CAN ID)

  // 只读! 8: 超压, 9: 欠压, A: 过流, B: MOS过温, C: 线圈过温, D: 通讯丢失, E: 过载
  uint8_t error = 0;

  float angle = 0;   // 只读! 单位: rad
  float speed = 0;   // 只读! 单位: rad/s
  float torque = 0;  // 只读! 单位: N·m

  float T_MOS = 0;    // 电机MOS管温度 单位: ℃
  float T_Rotor = 0;  // 电机转子温度 单位: ℃

  // [解析] has_read_ 判断是否收到过帧；is_alive() 额外要求最近 100ms 内有帧，掉线时上层应停发指令。
  bool is_open() const;
  bool is_alive(uint32_t now_ms) const;

  // [解析] read() 把 8 字节反馈解析进上面的反馈量并刷新时间戳，只在 CAN 中断里 rx_id 匹配后调用。
  void read(uint8_t * data, uint32_t stamp_ms);
  // [解析] write_xxx() 把缓存的目标值打包进 data(一般是 can.tx_data)；write_enable() 上电后必须先发一次电机才出力。
  void write(uint8_t * data) const;
  void write_enable(uint8_t * data) const;
  void write_disable(uint8_t * data) const;
  void write_clear_error(uint8_t * data) const;

  void write_velocity(uint8_t * data) const;

  // MIT力控模式指令缓存, 单位: N·m
  void cmd(float torque);
  // 速度模式指令缓存, 单位: rad/s
  void cmd_velocity(float velocity);

  // [解析] 五参数 MIT 控制：t = kp*(p_des-p) + kd*(v_des-v) + t_ff，参数在内部限幅到协议量程(kp 0~500、kd 0~5)。
  // [解析] 纯力矩控制令 kp=kd=0；纯速度控制令 kp=0。
  void cmd_mit(float p_des, float v_des, float kp, float kd, float t_ff);

  // MIT速度控制模式
  // velocity: 目标速度 (rad/s)
  // kd: 速度环增益 (建议根据实际负载调试，范围 0~5)
  void cmd_mit_velocity(float velocity, float kd);

  // 完整的MIT模式数据打包
  // [解析] 把五个参数按 16+12+12+12+12 = 64 位压进 8 字节，data[3]、data[6] 各由两个参数共用，位分配见 dm_motor.cpp。
  void write_mit(uint8_t * data) const;

private:
  const float pmax_;
  const float vmax_;
  const float tmax_;

  bool has_read_;
  uint32_t last_read_ms_;

  float cmd_torque_;
  float cmd_velocity_;

  //多圈计数
  // [解析] angle = angle_raw_ + circle_*2*pmax_；单圈角跨越 ±pmax_ 边界时 circle_ ±1。
  float angle_raw_ = 0;
  float last_angle_raw_ = 0.0f;
  int32_t circle_;
  bool multi_circle_;

  // 新增 MIT 模式参数缓存
  float cmd_p_des_ = 0.0f;
  float cmd_v_des_ = 0.0f;
  float cmd_kp_ = 0.0f;
  float cmd_kd_ = 0.0f;
  float cmd_t_ff_ = 0.0f;
};

}  // namespace sp

#endif  // SP__DM_MOTOR_HPP