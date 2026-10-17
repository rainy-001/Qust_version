// [解析] DM-IMU 驱动实现：填 8 字节请求帧、按寄存器号解析应答帧、把定点数换算成物理量。
// [解析] 目前没有任务调用它（见 hpp）；将来由收发 CAN 的那一层调用 read() / write_request()。
// [解析] 三处要点：uint_to_float 的线性映射、请求帧的 0xCC 头、四元数的 14 位压缩。

#include "dm_imu.hpp"

#include "dm_imu_defs.h"

namespace sp
{

// [解析] r_ab_ 是 const 数组，初始化列表里不能整体赋值，只能像下面这样逐个元素拷进来。
DM_IMU::DM_IMU(uint32_t tx_id, uint32_t rx_id, const float r_ab[3][3])
: tx_id(tx_id),
  rx_id(rx_id),
  temp(0),
  has_read_(false),
  last_read_ms_(0),
  r_ab_{
    {r_ab[0][0], r_ab[0][1], r_ab[0][2]},
    {r_ab[1][0], r_ab[1][1], r_ab[1][2]},
    {r_ab[2][0], r_ab[2][1], r_ab[2][2]}}
{
  for (int i = 0; i < 3; ++i) {
    acc[i] = 0.0f;
    gyro[i] = 0.0f;
    euler[i] = 0.0f;
  }
  // [解析] 四元数不能全零：w=1 才表示"没有旋转"，全零的四元数模长为 0，姿态运算会出垃圾。
  quat[0] = 1.0f;
  for (int i = 1; i < 4; ++i) {
    quat[i] = 0.0f;
  }
}

bool DM_IMU::is_open() const { return has_read_; }

// [解析] 100ms 阈值比请求周期宽松得多；无符号相减判断时间差，毫秒计数绕回也不会误判。
bool DM_IMU::is_alive(uint32_t now_ms) const { return is_open() && (now_ms - last_read_ms_ < 100); }

// [解析] 线性映射：原始值 0 -> x_min，满量程(2^bits-1) -> x_max，中间按比例插值。
// [解析] 必须先把 x_int 转成 float，否则整数除法会把小数部分全丢掉。
float DM_IMU::uint_to_float(int x_int, float x_min, float x_max, int bits)
{
  float span = x_max - x_min;
  float offset = x_min;
  return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

// [解析] 请求帧固定 8 字节；0xCC 是达妙协议的请求帧头(厂家魔数)，后 5 字节是数据域，读操作不用带。
// [解析] 只填不发：发送交给调用者，这样本类不绑定具体的 CAN 驱动。
void DM_IMU::write_request(uint8_t * data, uint8_t reg) const
{
  data[0] = 0xCC;  // 请求帧头
  data[1] = reg;   // 寄存器 RID
  data[2] = 0x00;  // 操作类型读
  data[3] = 0x00;
  data[4] = 0x00;
  data[5] = 0x00;
  data[6] = 0x00;
  data[7] = 0x00;
}

// [解析] 靠 data[0] 的寄存器号决定怎么解析 —— IMU 会把收到的寄存器号原样放进应答第一字节。
// [解析] switch 没有 default：不认识的包静默忽略，所以"没反应"也可能是收到了别的编号。
void DM_IMU::read(uint8_t * data, uint32_t stamp_ms)
{
  // [解析] 先无条件记时间戳：哪怕这包不认识，也算"IMU 在通信"，别让 is_alive() 误报掉线。
  last_read_ms_ = stamp_ms;
  if (!has_read_) {
    has_read_ = true;
  }

  // 首字节区分回传的数据包种类
  switch (data[0]) {
    case DM_IMU_REG_ACCEL: {
      // [解析] 温度只跟着这一包发：有符号，必须转 int8_t，否则 -10℃ 会被读成 246。
      temp = (int8_t)data[1];

      // [解析] 三轴是小端 16 位无符号数：负加速度靠下面映射到 -235.2 得来，数据本身没有符号位。
      uint16_t acc_x_int = (data[3] << 8) | data[2];
      uint16_t acc_y_int = (data[5] << 8) | data[4];
      uint16_t acc_z_int = (data[7] << 8) | data[6];

      float acc_x = uint_to_float(acc_x_int, DM_IMU_ACCEL_MIN, DM_IMU_ACCEL_MAX, 16);
      float acc_y = uint_to_float(acc_y_int, DM_IMU_ACCEL_MIN, DM_IMU_ACCEL_MAX, 16);
      float acc_z = uint_to_float(acc_z_int, DM_IMU_ACCEL_MIN, DM_IMU_ACCEL_MAX, 16);

      // 应用旋转矩阵
      // [解析] a = R × b：把传感器坐标系下的向量转到机器人坐标系，方向一致时就是单位阵。
      acc[0] = r_ab_[0][0] * acc_x + r_ab_[0][1] * acc_y + r_ab_[0][2] * acc_z;
      acc[1] = r_ab_[1][0] * acc_x + r_ab_[1][1] * acc_y + r_ab_[1][2] * acc_z;
      acc[2] = r_ab_[2][0] * acc_x + r_ab_[2][1] * acc_y + r_ab_[2][2] * acc_z;
      break;
    }
    case DM_IMU_REG_GYRO: {
      // [解析] 这包没有温度字节；单位是 rad/s 不是 °/s，和 euler（度）混算前要先统一。
      uint16_t gyro_x_int = (data[3] << 8) | data[2];
      uint16_t gyro_y_int = (data[5] << 8) | data[4];
      uint16_t gyro_z_int = (data[7] << 8) | data[6];

      float gyro_x = uint_to_float(gyro_x_int, DM_IMU_GYRO_MIN, DM_IMU_GYRO_MAX, 16);
      float gyro_y = uint_to_float(gyro_y_int, DM_IMU_GYRO_MIN, DM_IMU_GYRO_MAX, 16);
      float gyro_z = uint_to_float(gyro_z_int, DM_IMU_GYRO_MIN, DM_IMU_GYRO_MAX, 16);

      // 应用旋转矩阵
      gyro[0] = r_ab_[0][0] * gyro_x + r_ab_[0][1] * gyro_y + r_ab_[0][2] * gyro_z;
      gyro[1] = r_ab_[1][0] * gyro_x + r_ab_[1][1] * gyro_y + r_ab_[1][2] * gyro_z;
      gyro[2] = r_ab_[2][0] * gyro_x + r_ab_[2][1] * gyro_y + r_ab_[2][2] * gyro_z;
      break;
    }
    case DM_IMU_REG_EULER: {
      // [解析] 下面三行定死了 euler 的顺序：pitch/yaw/roll；这个分支不做 r_ab_ 修正。
      uint16_t pitch_int = (data[3] << 8) | data[2];
      uint16_t yaw_int = (data[5] << 8) | data[4];
      uint16_t roll_int = (data[7] << 8) | data[6];

      euler[0] = uint_to_float(pitch_int, DM_IMU_PITCH_MIN, DM_IMU_PITCH_MAX, 16);
      euler[1] = uint_to_float(yaw_int, DM_IMU_YAW_MIN, DM_IMU_YAW_MAX, 16);
      euler[2] = uint_to_float(roll_int, DM_IMU_ROLL_MIN, DM_IMU_ROLL_MAX, 16);
      break;
    }
    case DM_IMU_REG_QUAT: {
      // [解析] data[1]~data[7] 共 56 位，正好装 4 个 14 位分量（4×14=56），一个 bit 都不浪费。
      // [解析] 从 data[1] 最高位起连续切 14 位：w 占 8+6，x 占 2+8+4，y 占 4+8+2，z 占 6+8。
      // [解析] 掩码 0x03/0x0F/0x3F 就是取低 2/4/6 位，用来抹掉属于上一个分量的高位。
      int w_int = (data[1] << 6) | (data[2] >> 2);
      int x_int = ((data[2] & 0x03) << 12) | (data[3] << 4) | (data[4] >> 4);
      int y_int = ((data[4] & 0x0F) << 10) | (data[5] << 2) | (data[6] >> 6);
      int z_int = ((data[6] & 0x3F) << 8) | data[7];

      quat[0] = uint_to_float(w_int, DM_IMU_QUAT_MIN, DM_IMU_QUAT_MAX, 14);
      quat[1] = uint_to_float(x_int, DM_IMU_QUAT_MIN, DM_IMU_QUAT_MAX, 14);
      quat[2] = uint_to_float(y_int, DM_IMU_QUAT_MIN, DM_IMU_QUAT_MAX, 14);
      quat[3] = uint_to_float(z_int, DM_IMU_QUAT_MIN, DM_IMU_QUAT_MAX, 14);
      break;
    }
  }
}

}  // namespace sp