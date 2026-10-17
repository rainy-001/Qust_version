// [解析] GY-56 激光测距模块的驱动实现：发命令 + 逐字节拼帧 + 解析距离。
// [解析] 数据流：DMA 空闲回调 receive_complete() -> update() 滑动窗口 -> frame_valid() 校验 -> parse_frame()。
// [解析] 串口是连续字节流、还可能从半截开始收，所以必须靠 8 字节滑动窗口对齐帧头，不能按固定长度切。
// [解析] 距离是两个字节拼出来的大端数；状态字节 frame_[6] 高 4 位是故障码、低 2 位是模式。

#include "laser.hpp"

namespace sp
{
LaserGY56::LaserGY56(UART_HandleTypeDef * huart)
: huart_(huart),
  rx_buff_{0},
  frame_{0},
  frame_size_(0),
  distance_cm_(0),
  last_frame_tick_(0),
  mode_(LaserMode::UNKNOWN),
  has_reliable_distance_(false)
{
}

uint16_t LaserGY56::get_distance() const { return distance_cm_; }

// [解析] 数据新鲜度：收到过有效帧、且距最后一帧不超过 timeout_ms 才算有效。
// [解析] 两个 uint32_t 相减即使 HAL_GetTick() 回绕（约 49.7 天）也依然正确，不用额外防护。
bool LaserGY56::is_valid(uint32_t timeout_ms) const
{
  if (!has_reliable_distance_) {
    return false;
  }

  return HAL_GetTick() - last_frame_tick_ <= timeout_ms;
}

LaserMode LaserGY56::get_mode() const { return mode_; }

// [解析] 把模式枚举翻译成命令字节（0x51/0x52/0x53 = 短/中/长）再发给模块。
// [解析] 这三个字在不同批次的手册里可能是"单次/连续/停止测量"，实测不对先翻模块附带的手册。
bool LaserGY56::send_mode(LaserMode mode)
{
  switch (mode) {
    case LaserMode::SHORT:
      return send_command(0x51);
    case LaserMode::MEDIUM:
      return send_command(0x52);
    case LaserMode::LONG:
      return send_command(0x53);
    default:
      return false;
  }
}

// [解析] 改波特率（0xAE=9600 / 0xAF=115200）和存配置（0x25）；改完必须 save_config()，
// [解析] 并同步改 CubeMX 里该串口的参数，否则模块掉电后设置丢失，就再也收不到数据。
bool LaserGY56::set_baud_rate_9600() { return send_command(0xAE); }

bool LaserGY56::set_baud_rate_115200() { return send_command(0xAF); }

bool LaserGY56::save_config() { return send_command(0x25); }

// [解析] 启动 DMA + 空闲中断接收：true 表示已经在收、或本次启动成功。
// [解析] 最常见的失败是 CubeMX 里这个串口没开 DMA 接收通道（hdmarx 为空），其次是串口正忙。
bool LaserGY56::start_receive()
{
  if (huart_ == nullptr || huart_->hdmarx == nullptr) {
    return false;
  }
  if (huart_->RxState == HAL_UART_STATE_BUSY_RX) {
    return true;
  }
  if (huart_->RxState != HAL_UART_STATE_READY) {
    return false;
  }

  if (HAL_UARTEx_ReceiveToIdle_DMA(huart_, rx_buff_, RX_BUFFER_SIZE) != HAL_OK) {
    return false;
  }

  // 仅在空闲或缓冲区满时处理数据, 避免半满回调重复解析
  // [解析] 半传输(HT)中断会在一半(32 字节)时再回调一次，同一批数据被解析两遍就会出脏帧，所以关掉。
  __HAL_DMA_DISABLE_IT(huart_->hdmarx, DMA_IT_HT);
  return true;
}

// [解析] HAL 的串口空闲/收满回调最终转到这里，运行在中断上下文，不能阻塞、不能等锁。
// [解析] 把本次收到的 size 个字节逐个喂给 update()，喂完立刻重新挂起接收。
void LaserGY56::receive_complete(uint16_t size)
{
  if (size > RX_BUFFER_SIZE) {
    size = RX_BUFFER_SIZE;
  }

  for (uint16_t i = 0; i < size; i++) {
    update(rx_buff_[i]);
  }
  // [解析] 这里没检查 start_receive() 的返回值：若串口正好在发送，重启失败、接收就此静默中断。
  start_receive();
}

// [解析] 滑动窗口拼帧：串口是字节流、还可能从半截开始收，按固定 8 字节切一刀会永久错位，
// [解析] 所以永远只看"最近 8 个字节"，每来一个新字节左移一格，凑满且校验通过才算一帧。
void LaserGY56::update(uint8_t byte)
{
  // 维护8字节窗口, 使帧头错位或分段接收时仍能重新找到完整数据帧
  if (frame_size_ < FRAME_SIZE) {
    frame_[frame_size_] = byte;
    frame_size_++;
  }
  else {
    for (uint8_t i = 0; i < FRAME_SIZE - 1; i++) {
      frame_[i] = frame_[i + 1];
    }
    frame_[FRAME_SIZE - 1] = byte;
  }

  if (frame_size_ < FRAME_SIZE || !frame_valid()) {
    return;
  }

  parse_frame();
  frame_size_ = 0;
}

// [解析] 命令固定 3 字节：0xA5 + 命令字 + 累加和（前两字节之和，自然溢出到 8 位）。
// [解析] static_cast 是必需的：0xA5 + 命令字在 C++ 里先提升成 int，结果可能超过 255。
bool LaserGY56::send_command(uint8_t command)
{
  if (huart_ == nullptr) {
    return false;
  }

  uint8_t data[3] = {0xA5, command, static_cast<uint8_t>(0xA5 + command)};
  return HAL_UART_Transmit(huart_, data, sizeof(data), 10) == HAL_OK;
}

// [解析] 检查帧头 4 字节 + 最后 1 字节的逐字节累加和；只在 frame_size_ 满 8 之后调用。
bool LaserGY56::frame_valid() const
{
  // 检查帧头、数据类型和逐字节累加校验
  if (frame_[0] != 0x5A || frame_[1] != 0x5A || frame_[2] != 0x15 || frame_[3] != 0x03) {
    return false;
  }

  uint8_t checksum = 0;
  for (uint8_t i = 0; i < FRAME_SIZE - 1; i++) {
    checksum = static_cast<uint8_t>(checksum + frame_[i]);
  }
  return checksum == frame_[FRAME_SIZE - 1];
}

// [解析] 把校验通过的 8 字节拆成距离/模式/时间戳；状态非 0 表示本次测量失败，整帧作废。
// [解析] 距离是大端（frame_[4] 为高字节），必须手动拼，直接整体强转成 uint16_t 会高低字节颠倒。
void LaserGY56::parse_frame()
{
  // 状态不为0时不更新距离、模式和有效时间
  // [解析] 一个字节塞了两样东西：高 4 位状态/故障码，低 2 位模式（0~3，与 LaserMode 枚举一一对应）。
  uint8_t status = static_cast<uint8_t>((frame_[6] >> 4) & 0x0F);
  if (status != 0) {
    return;
  }

  // 实机8字节协议的距离单位为cm, 直接保存帧中的数值
  distance_cm_ = static_cast<uint16_t>((static_cast<uint16_t>(frame_[4]) << 8) | frame_[5]);
  mode_ = static_cast<LaserMode>(frame_[6] & 0x03);
  last_frame_tick_ = HAL_GetTick();
  has_reliable_distance_ = true;
}

}  // namespace sp
