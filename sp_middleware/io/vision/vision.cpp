// [解析] Vision 类的实现：从 USB CDC 收到的字节流里切出合法帧，并把本车状态打包发回上位机。
// [解析] 收在 USB 中断里（vision_task.cpp 的 vision_callback），发在任务里周期调用，两条路用的成员不重叠。
// [解析] update() 是最难的一段：帧头匹配 + 长度检查 + CRC16 + 失败时只丢 1 个字节重新对齐。
// [解析] 文件顶部还包了一层 CDC_Transmit_Select()，用来兼容 FS / HS 两种 USB 发送函数名。

#include "vision.hpp"

#include <cstring>

#include "cmsis_os.h"
#include "tools/crc/crc.hpp"
#include "tools/math_tools/math_tools.hpp"
#include "usbd_cdc_if.h"
// [解析] 声明了 CDC_Transmit_FS/HS；工程没启用 USB 时这个头文件会找不到，直接编译报错。

extern "C" {
// [解析] 这两个函数定义在 C 文件 usbd_cdc_if.c 里，不套 extern "C" 链接时会找不到符号。
#if defined(__GNUC__)
uint8_t CDC_Transmit_HS(uint8_t * Buf, uint16_t Len) __attribute__((weak));
uint8_t CDC_Transmit_FS(uint8_t * Buf, uint16_t Len) __attribute__((weak));
// [解析] weak 弱声明：工程里没人实现该函数时不报链接错误，符号地址就是 0。
// [解析] 本工程目前还没启用 USB，所以两个地址都是 0，CDC_Transmit_Select() 实际返回 0xFF。
#else
/* For non-GNU toolchains, declare without weak attribute; the project
  should ensure one of these is provided. */
uint8_t CDC_Transmit_HS(uint8_t * Buf, uint16_t Len);
uint8_t CDC_Transmit_FS(uint8_t * Buf, uint16_t Len);
#endif
}

// [解析] 适配层：C 板可能用 USB_OTG_FS 也可能 HS，两者发送函数名不同，这里自动挑能用的那个。
// [解析] 返回 0=成功 / 1=忙（本帧被丢） / 2=失败 / 0xFF=两个都没实现（本函数自定的错误码）。
static inline uint8_t CDC_Transmit_Select(uint8_t * Buf, uint16_t Len)
{
#if defined(__GNUC__)
  // [解析] 函数名单独出现在 if 里会退化成函数指针，配合上面的 weak 声明就等于"这个功能存不存在"。
  if (CDC_Transmit_HS) return CDC_Transmit_HS(Buf, Len);
  if (CDC_Transmit_FS) return CDC_Transmit_FS(Buf, Len);
  return 0xFF; /* no implementation available */
#else
  /* Prefer HS when both declared; fall back to FS. Linker will error
    if neither is provided for non-GNU toolchains. */
  extern uint8_t CDC_Transmit_HS(uint8_t *, uint16_t);
  extern uint8_t CDC_Transmit_FS(uint8_t *, uint16_t);
  /* Try HS first */
  return CDC_Transmit_HS ? CDC_Transmit_HS(Buf, Len) : CDC_Transmit_FS(Buf, Len);
  // [解析] 非 GNU 分支语义不同：普通声明取地址永远非 0，缺实现时是链接错误而不是优雅降级。
#endif
}

namespace sp
{

// [解析] 收包主函数：把 USB 送来的一批字节追加进 rx_buffer_，逐帧切分校验后写进那些 public 成员。
// [解析] 它由 USB 中断回调调用，里面不能打印、不能延时、不能分配内存。
void Vision::update(uint8_t * buf, uint32_t len)
{
  if (len == 0 || buf == nullptr) return;

  // Append new data to the buffer, discard all if overflow
  // [解析] 溢出策略：装不下就整段丢弃重来；只丢最老的数据会把一帧切成两半，反而更难对齐。
  if (rx_len_ + len > sizeof(rx_buffer_)) {
    rx_len_ = 0;
  }
  // [解析] 追加到已有数据后面。★若某次 len 本身就超过 512，上面的清零救不了，这里会越界写。
  std::copy(buf, buf + len, rx_buffer_ + rx_len_);
  rx_len_ += len;

  // [解析] 每轮只处理缓冲区最前面的一帧：成功取走一整帧，或丢掉 1 个字节重新对齐；
  // [解析] 数据不够一帧就 break 等下一批。用 while 是因为一次 update 可能送来好几帧。
  while (rx_len_ >= 2) {
    if (rx_buffer_[0] == 'S' && rx_buffer_[1] == 'P') {
      if (rx_len_ >= sizeof(VisionToGimbal)) {
        // [解析] check_crc16 把末尾 2 字节当校验值（小端），对前面 len-2 字节重算比对；错帧绝不能当控制指令用。
        if (check_crc16(rx_buffer_, sizeof(VisionToGimbal))) {
          std::copy(
            rx_buffer_, rx_buffer_ + sizeof(VisionToGimbal),
            reinterpret_cast<uint8_t *>(&rx_data_));

          // [解析] ★注意顺序：这里先用"旧"时间戳算心跳、再刷新时间戳，所以 autoaim_alive 反映的是
          // [解析] 上一帧与上上帧的间隔（100 ms 阈值），上电后第一帧必为 false。
          this->autoaim_alive = (osKernelSysTick() - this->autoaim_last_read_ms_ < 100);
          this->control = (rx_data_.mode != 0);
          this->fire = (rx_data_.mode == 2);
          this->yaw = rx_data_.yaw;
          this->yaw_vel = rx_data_.yaw_vel;
          this->yaw_acc = rx_data_.yaw_acc;
          this->pitch = rx_data_.pitch;
          this->pitch_vel = rx_data_.pitch_vel;
          this->pitch_acc = rx_data_.pitch_acc;
          this->auto_aim_target = rx_data_.target;
          this->autoaim_last_read_ms_ = osKernelSysTick();

          // Shift remaining data forward
          rx_len_ -= sizeof(VisionToGimbal);
          // [解析] 剩余数据前移（源和目的重叠，必须用 memmove，不能用 memcpy）。
          if (rx_len_ > 0) std::memmove(rx_buffer_, rx_buffer_ + sizeof(VisionToGimbal), rx_len_);
        }
        else {
          // CRC failed, invalid frame, shift 1 byte forward to re-sync
          // [解析] CRC 没过不能一次扔掉 30 字节：帧头很可能是巧合撞上的，真正的帧在后面；
          // [解析] 所以只丢 1 个字节让帧头后移一位重新对齐 —— 串口协议解析的标准做法。
          rx_len_--;
          if (rx_len_ > 0) std::memmove(rx_buffer_, rx_buffer_ + 1, rx_len_);
        }
      }
      else {
        break;  // Wait for more data
      }
    }
    else if (rx_buffer_[0] == 'S' && rx_buffer_[1] == 'C') {
      if (rx_len_ >= sizeof(VisionToHanging)) {
        if (check_crc16(rx_buffer_, sizeof(VisionToHanging))) {
          std::copy(
            rx_buffer_, rx_buffer_ + sizeof(VisionToHanging),
            reinterpret_cast<uint8_t *>(&rx_data_hanging_));

          this->seq = rx_data_hanging_.seq;
          // [解析] 288 是写死的数字（没用 sizeof）；改 VisionToHanging::data 长度时必须同步改这里。
          for (int i = 0; i < 288; i++) this->data[i] = rx_data_hanging_.data[i];

          // [解析] 大包也算"上位机还活着"，但这里只刷新时间戳，没有重算 autoaim_alive。
          this->autoaim_last_read_ms_ = osKernelSysTick();

          // Shift remaining data forward
          rx_len_ -= sizeof(VisionToHanging);
          if (rx_len_ > 0) std::memmove(rx_buffer_, rx_buffer_ + sizeof(VisionToHanging), rx_len_);
        }
        else {
          // CRC failed, invalid frame, shift 1 byte forward to re-sync
          rx_len_--;
          if (rx_len_ > 0) std::memmove(rx_buffer_, rx_buffer_ + 1, rx_len_);
        }
      }
      else {
        break;  // Wait for more data
      }
    }
    else {
      // Invalid head, shift 1 byte forward to find next 'S'
      rx_len_--;
      if (rx_len_ > 0) std::memmove(rx_buffer_, rx_buffer_ + 1, rx_len_);
    }
  }
}

// [解析] 把本车状态填进 GimbalToVision（57 字节）发出；CRC16 覆盖除末尾 2 字节外的全部字段。
void Vision::send(
  uint8_t mode, float q[4], float yaw, float yaw_vel, float pitch, float pitch_vel,
  float bullet_speed, uint16_t bullet_count, float supercap_power_in, float supercap_power_out,
  float supercap_voltage, uint8_t supercap_temperature, uint8_t supercap_status)
{
  // [解析] head 不用填：靠结构体里的默认初始化器 {'S','P'} 自动就位，这个值永远不变。
  tx_data_.mode = mode;
  tx_data_.q[0] = q[0];
  tx_data_.q[1] = q[1];
  tx_data_.q[2] = q[2];
  tx_data_.q[3] = q[3];
  tx_data_.yaw = yaw;
  tx_data_.yaw_vel = yaw_vel;
  tx_data_.pitch = pitch;
  tx_data_.pitch_vel = pitch_vel;
  tx_data_.bullet_speed = bullet_speed;
  tx_data_.bullet_count = bullet_count;
  tx_data_.supercap_power_in = supercap_power_in;
  tx_data_.supercap_power_out = supercap_power_out;
  tx_data_.supercap_voltage = supercap_voltage;
  tx_data_.supercap_temperature = supercap_temperature;
  tx_data_.supercap_status = supercap_status;
  // [解析] CRC16 只覆盖末尾 crc16 之前的字节，所以 crc16 必须在结构体最后，且两端覆盖范围要一致。
  tx_data_.crc16 =
    get_crc16(reinterpret_cast<uint8_t *>(&tx_data_), sizeof(tx_data_) - sizeof(tx_data_.crc16));

  // [解析] 返回值没有接收：上次没发完时 CDC 返回 USBD_BUSY，这一帧被静默丢掉（1 kHz 下很常见）。
  CDC_Transmit_Select(reinterpret_cast<uint8_t *>(&tx_data_), sizeof(tx_data_));
}

// [解析] 英雄版：逻辑和上面完全一样，只是换成 58 字节的 GimbalToVisionHero 并多填 game_progress。
void Vision::send(
  uint8_t mode, float q[4], float yaw, float yaw_vel, float pitch, float pitch_vel,
  float bullet_speed, uint16_t bullet_count, float supercap_power_in, float supercap_power_out,
  float supercap_voltage, uint8_t supercap_temperature, uint8_t supercap_status,
  uint8_t game_progress)
{
  tx_data_with_game_progress_.mode = mode;
  tx_data_with_game_progress_.q[0] = q[0];
  tx_data_with_game_progress_.q[1] = q[1];
  tx_data_with_game_progress_.q[2] = q[2];
  tx_data_with_game_progress_.q[3] = q[3];
  tx_data_with_game_progress_.yaw = yaw;
  tx_data_with_game_progress_.yaw_vel = yaw_vel;
  tx_data_with_game_progress_.pitch = pitch;
  tx_data_with_game_progress_.pitch_vel = pitch_vel;
  tx_data_with_game_progress_.bullet_speed = bullet_speed;
  tx_data_with_game_progress_.bullet_count = bullet_count;
  tx_data_with_game_progress_.supercap_power_in = supercap_power_in;
  tx_data_with_game_progress_.supercap_power_out = supercap_power_out;
  tx_data_with_game_progress_.supercap_voltage = supercap_voltage;
  tx_data_with_game_progress_.supercap_temperature = supercap_temperature;
  tx_data_with_game_progress_.supercap_status = supercap_status;
  tx_data_with_game_progress_.game_progress = game_progress;
  tx_data_with_game_progress_.crc16 = get_crc16(
    reinterpret_cast<uint8_t *>(&tx_data_with_game_progress_),
    sizeof(tx_data_with_game_progress_) - sizeof(tx_data_with_game_progress_.crc16));

  CDC_Transmit_Select(
    reinterpret_cast<uint8_t *>(&tx_data_with_game_progress_), sizeof(tx_data_with_game_progress_));
}
}  // namespace sp