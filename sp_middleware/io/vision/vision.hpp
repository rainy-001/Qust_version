// [解析] 电控板 <-> 视觉/自瞄上位机之间的 USB CDC 通信协议：定结构体 + 收包解析 + 发包。
// [解析] 用法见 applications/vision_task.cpp：vis.update(buf, len) 收、vis.send(...) 周期发（该任务还没编进工程）。
// [解析] 所有结构体都带 packed，sizeof 就是线上字节数；CRC16 在最后 2 字节（小端），覆盖它前面的全部字节。
// [解析] 帧头靠默认成员初始化器 head[2] = {'S','P'} 自动就位，send() 里从不给 head 赋值。

#ifndef SP__VISION_HPP
#define SP__VISION_HPP

#include <cstdint>

namespace sp
{
// [解析] 上位机 -> 电控板 的控制指令，30 字节：帧头 + 模式 + 6 个 float + 目标编号 + CRC16。
struct __attribute__((packed)) VisionToGimbal
{
  uint8_t head[2] = {'S', 'P'};
  uint8_t mode;  // 0: 不控制, 1: 控制云台但不开火，2: 控制云台且开火
  float yaw;
  float yaw_vel;
  float yaw_acc;
  float pitch;
  float pitch_vel;
  float pitch_acc;
  uint8_t target;  //0:英雄,1:工程,2:3号步兵,3:4号步兵,4:5号步兵,5:哨兵,6:前哨,7:基地,8:无目标
  uint16_t crc16;
};
static_assert(sizeof(VisionToGimbal) <= 64);
// [解析] 编译期卡住帧长：USB 全速 CDC 单包最多 64 字节，超了宁可编译报错，也别上线后莫名丢包。

// [解析] 电控板 -> 上位机 的本车状态，57 字节；上位机靠它做弹道解算，与上面的指令构成闭环。
struct __attribute__((packed)) GimbalToVision
{
  uint8_t head[2] = {'S', 'P'};
  uint8_t mode;  // 0: 空闲, 1: 自瞄, 2: 小符, 3: 大符
  float q[4];    // wxyz顺序
  float yaw;
  float yaw_vel;
  float pitch;
  float pitch_vel;
  // [解析] 弹速 m/s，弹道解算的关键输入；换弹种、改摩擦轮都要重新标定，标错就"瞄得准、打不中"。
  float bullet_speed;
  uint16_t bullet_count;  // 子弹累计发送次数
  float supercap_power_in;
  float supercap_power_out;
  float supercap_voltage;
  uint8_t supercap_temperature;
  uint8_t supercap_status;
  uint16_t crc16;
};

static_assert(sizeof(GimbalToVision) <= 64);

// [解析] 英雄专用版，比上面多 1 个 game_progress 共 58 字节；单独复制一份而不是给原结构体加字段，
// [解析] 是为了不动其它兵种已经调好的协议 —— 代价是改字段要两处同步。
struct __attribute__((packed)) GimbalToVisionHero
{
  uint8_t head[2] = {'S', 'P'};
  uint8_t mode;  // 0: 空闲, 1: 自瞄, 2: 小符, 3: 大符
  float q[4];    // wxyz顺序
  float yaw;
  float yaw_vel;
  float pitch;
  float pitch_vel;
  float bullet_speed;
  uint16_t bullet_count;  // 子弹累计发送次数
  float supercap_power_in;
  float supercap_power_out;
  float supercap_voltage;
  uint8_t supercap_temperature;
  uint8_t supercap_status;
  uint8_t game_progress;
  uint16_t crc16;
};

static_assert(sizeof(GimbalToVisionHero) <= 64);

// [解析] 'S''C' 是另一种大包：293 字节 = 2 帧头 + 1 序号 + 288 数据 + 2 CRC，和 'S''P' 是两类数据。
struct __attribute__((packed)) VisionToHanging
{
  uint8_t head[2] = {'S', 'C'};
  uint8_t seq;
  // [解析] 288 是上位机定死的长度（仓库里没文档说明来历），改之前两端必须一起改，否则 CRC 永远不过。
  uint8_t data[288];
  uint16_t crc16;
};
static_assert(sizeof(VisionToHanging) <= 293);
// [解析] 上限 293 而不是 64：USB 会拆包，靠 update() 的 512 字节缓冲累积凑满再解析。

// [解析] 收和发两条路的入口。★没有构造函数：只能用 sp::Vision vis; 创建，
// [解析] 写成 sp::Vision vis(false, false) 会编译报错；成员初值全靠默认成员初始化器。
class Vision
{
public:
  bool control = false;
  bool fire = false;
  float yaw = 0;
  float yaw_vel = 0;
  float yaw_acc = 0;
  float pitch = 0;
  float pitch_vel = 0;
  float pitch_acc = 0;
  uint8_t auto_aim_target = 8;  //默认无目标

  bool autoaim_alive = false;
  // [解析] 两次收到数据的间隔 < 100 ms 就算在线；因为先算后刷时间戳，上电后第一帧必为 false。

  uint8_t data[288] = {0};
  uint8_t seq = 0;

  uint32_t autoaim_last_read_ms_ = 0;
  void update(uint8_t * buf, uint32_t len);
  // [解析] 解析一批 USB 字节：追加 -> 找帧头 -> 校验 CRC16 -> 通过才更新上面那些 public 成员。
  void send(
    uint8_t mode, float q[4], float yaw, float yaw_vel, float pitch, float pitch_vel,
    float bullet_speed, uint16_t bullet_count, float supercap_power_in, float supercap_power_out,
    float supercap_voltage, uint8_t supercap_temperature, uint8_t supercap_status);
  // [解析] 把本车状态打包成 GimbalToVision（57 字节）发出，CRC 自动算、head 不用填。
  // [解析] 单位：q 为 wxyz 四元数，yaw/pitch 用弧度、角速度 rad/s，弹速 m/s，超电功率 W、电压 V、温度 ℃。
  void send(
    uint8_t mode, float q[4], float yaw, float yaw_vel, float pitch, float pitch_vel,
    float bullet_speed, uint16_t bullet_count, float supercap_power_in, float supercap_power_out,
    float supercap_voltage, uint8_t supercap_temperature, uint8_t supercap_status,
    uint8_t game_progress);
  // [解析] 英雄版：多一个 game_progress，发出去的是 58 字节的 GimbalToVisionHero，其余参数相同。
  VisionToHanging rx_data_hanging_;

private:
  VisionToGimbal rx_data_;
  GimbalToVision tx_data_;
  GimbalToVisionHero tx_data_with_game_progress_;

  // [解析] 512 字节累积缓冲：USB 一次回调最多 64 字节，大包要攒几次，且可能尾巴和头一起到。
  uint8_t rx_buffer_[512] = {0};
  uint16_t rx_len_ = 0;
};

}  // namespace sp

#endif  // SP__VISION_HPP
