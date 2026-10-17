// [解析] FDCAN 通信封装：start / recv / send 三件套，接口基本照抄 can.hpp。
// [解析] 本工程(F407)片内没有 FDCAN，这是给达妙板(H7)预留的；换板并在 CubeMX 里开 FDCAN 才用得上。
// [解析] 帧格式在构造函数里一次定死为"经典 CAN + 8 字节"，过滤器则必须在 CubeMX 里配，本类不管。

#ifndef SP__FDCAN_HPP
#define SP__FDCAN_HPP

#include "fdcan.h"

namespace sp
{
constexpr size_t FDCAN_DATA_LEN = 8;

// [解析] 一个对象管一路 FDCAN，收发数据靠下面的公共缓冲区交换。
class FDCAN
{
public:
  FDCAN(FDCAN_HandleTypeDef * hfdcan);

  // [解析] recv() 之后读 rx_*，send() 之前写 tx_data；rx_id 是 uint16_t，装不下 29 位扩展 ID。
  uint16_t rx_id;
  uint8_t rx_data[FDCAN_DATA_LEN];
  uint8_t tx_data[FDCAN_DATA_LEN];

  // [解析] 必须在构造之后、收发之前调一次；BUS_OFF 中断也在这里开，原因见 cpp。
  void start();

  // [解析] 在 HAL_FDCAN_RxFifo0Callback 中断里调用，一次取走 FIFO0 里的一帧。
  void recv();

  void send(uint16_t tx_id);

private:
  FDCAN_HandleTypeDef * hfdcan_;

  FDCAN_RxHeaderTypeDef rx_header_;
  FDCAN_TxHeaderTypeDef tx_header_;
};

}  // namespace sp

#endif  // SP__FDCAN_HPP