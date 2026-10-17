// [解析] 达妙 DM-IMU-L1 驱动接口：请求-应答式，IMU 内部自带解算，直接给出加速度/角速度/欧拉角/四元数。
// [解析] 本类不自己收发 CAN：write_request() 只填 8 字节请求，read() 只解析别人递进来的 8 字节。
// [解析] 最容易踩的坑：euler[] 顺序是 pitch/yaw/roll（不是常见的 roll/pitch/yaw），下标用错数据就全乱。

#ifndef SP__DM_IMU_HPP
#define SP__DM_IMU_HPP

#include <cstdint>

#include "dm_imu_defs.h"
namespace sp
{

class DM_IMU
{
public:
  // [解析] 两个 ID 都属于 IMU 那一侧：tx_id 你发给它、rx_id 它回给你，别按 MCU 视角反着理解。
  // [解析] r_ab 是 {b}->{a} 的安装方向修正矩阵，装正了传单位阵 {{1,0,0},{0,1,0},{0,0,1}}。
  // tx_id: 上位机设置的 CAN ID (用于发送请求)
  // rx_id: 上位机设置的 MST ID (用于接收应答)
  // r_ab: dm_imu原始坐标系{b}在机器人坐标系{a}下的坐标矩阵
  DM_IMU(uint32_t tx_id, uint32_t rx_id, const float r_ab[3][3]);

  const uint32_t tx_id;
  const uint32_t rx_id;

  // [解析] 以下都是只读结果，由 read() 按 data[0] 分派填写；acc/gyro 已乘 r_ab 转到机器人坐标系。
  float acc[3] = {0.0f};   // 只读! 单位: m/s^2
  float gyro[3] = {0.0f};  // 只读! 单位: rad/s
  float euler[3];          // 只读! pitch, yaw, roll
  float quat[4];           // 只读! w, x, y, z
  int8_t temp;             // 只读! 单位: 摄氏度

  bool is_open() const;
  bool is_alive(uint32_t now_ms) const;

  // 将接收到的 CAN 帧数据解析并更新至公有变量
  // [解析] data 至少要有 8 字节（本类不做长度检查）；一次只请求一种数据，因为全靠 data[0] 分派。
  void read(uint8_t * data, uint32_t stamp_ms);

  // 根据指定寄存器，填装请求发送缓存 (不直接调用FDCAN)
  // [解析] 只负责填这 8 字节，发去哪、用什么外设发由调用者决定（这样本类不绑定 FDCAN/bxCAN）。
  void write_request(uint8_t * data, uint8_t reg) const;

private:
  bool has_read_;
  uint32_t last_read_ms_;
  const float r_ab_[3][3];
  // [解析] 定点转物理量的核心工具：把 bits 位无符号整数线性映射到 [x_min, x_max]。
  static float uint_to_float(int x_int, float x_min, float x_max, int bits);
};

}  // namespace sp

#endif  // SP__DM_IMU_HPP