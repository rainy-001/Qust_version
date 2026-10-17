// [解析] VT02 类的实现：启动串口接收、拆帧、校验，把 0x0302 的数据段拷进 custom。
// [解析] 上层遥控器任务先 request() 挂接收，再在串口空闲回调里 update(HAL_GetTick())。
// [解析] request() 用 ReceiveToIdle（总线静默即一帧结束）适配变长帧，并且必须关掉 DMA 半传输中断。

#include "vt02.hpp"

#include <cstring>

#include "referee_def.h"
#include "tools/crc/crc.hpp"

namespace sp
{
VT02::VT02(UART_HandleTypeDef * huart, bool use_dma) : huart_(huart), use_dma_(use_dma) {}

void VT02::request()
{
  if (use_dma_) {
    // dismiss return
    HAL_UARTEx_ReceiveToIdle_DMA(huart_, buff_, VT02_BUFF_SIZE);

    // ref: basic_framework/bsp/usart/bsp_usart.c at master · HNUYueLuRM/basic_framework
    // [解析] 关掉 DMA 半传输(HT)中断：50 字节缓冲搬到一半也会回调，那时只有半帧，解析必然失败。
    __HAL_DMA_DISABLE_IT(huart_->hdmarx, DMA_IT_HT);
  }
  else {
    // dismiss return
    HAL_UARTEx_ReceiveToIdle_IT(huart_, buff_, VT02_BUFF_SIZE);
  }
}

// [解析] 核心函数：把 buff_ 里的字节按 SOF -> CRC8 -> 帧长 -> CRC16 -> cmd_id==0x0302 的顺序过滤，
// [解析] 任一关不过就直接 return 丢掉这帧；全过才把数据段拷进 custom。
void VT02::update(uint32_t stamp_ms)
{
  if (buff_[0] != REFEREE_SOF) return;
  if (!check_crc8(buff_, REFEREE_HEAD_LEN)) return;

  // [解析] 小端：buff_[1] 是低字节、buff_[2] 是高字节，写成 (buff_[1]<<8)|buff_[2] 会算出错的长度。
  size_t data_len = (buff_[2] << 8) | buff_[1];
  size_t len = REFEREE_HEAD_LEN + REFEREE_CMD_ID_LEN + data_len + REFEREE_TAIL_LEN;
  if (!check_crc16(buff_, len)) return;

  uint16_t cmd_id = (buff_[6] << 8) | buff_[5];
  if (cmd_id == REFEREE_CUSTOM_CMD_ID) {
    last_read_ms_ = stamp_ms;
    has_read_ = true;
    // [解析] ⚠ 这里没校验 data_len 是否等于 sizeof(RefereeCustomData)，数据段偏短会读到帧尾甚至缓冲区外。
    std::memcpy(&(this->custom), buff_ + REFEREE_DATA_START, sizeof(RefereeCustomData));
  }
}

bool VT02::is_open() const { return has_read_; }

bool VT02::is_alive(uint32_t now_ms) const { return is_open() && (now_ms - last_read_ms_ < 2000); }

}  // namespace sp