// [解析] FDCAN 类的实现，基本都是 HAL 调用的薄封装。
// [解析] 构造函数把发送帧头一次性配好（标准 ID + 8 字节 + 经典 CAN），之后每次 send() 只改 ID。
// [解析] start() 里多开了一条 BUS_OFF 通知：H7 的 FDCAN 丢了自动恢复机制，得自己救总线。

#include "fdcan.hpp"

namespace sp
{
// [解析] BRS 与 FDFormat 必须一起关：退化成经典 CAN 才能和总线上不支持 FD 的普通设备共存。
FDCAN::FDCAN(FDCAN_HandleTypeDef * hfdcan) : hfdcan_(hfdcan)
{
  tx_header_.IdType = FDCAN_STANDARD_ID;
  tx_header_.TxFrameType = FDCAN_DATA_FRAME;
  tx_header_.DataLength = FDCAN_DLC_BYTES_8;
  tx_header_.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  tx_header_.BitRateSwitch = FDCAN_BRS_OFF;
  tx_header_.FDFormat = FDCAN_CLASSIC_CAN;
  tx_header_.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  tx_header_.MessageMarker = 0x00;
}

void FDCAN::start()
{
  HAL_FDCAN_Start(hfdcan_);                                                   // dismiss return
  HAL_FDCAN_ActivateNotification(hfdcan_, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);  // dismiss return
  // [解析] 总线严重错误会让 FDCAN 进 BUS_OFF 永久哑掉；光开这条通知还不够，
  // [解析] 要真正恢复还得自己写 HAL_FDCAN_ErrorStatusCallback() 里重新 HAL_FDCAN_Start()。
  HAL_FDCAN_ActivateNotification(
    hfdcan_, FDCAN_IT_BUS_OFF,
    0);  // dismiss return ref: https://community.st.com/t5/stm32-mcus-products/stm32h7-fdcan-has-lost-the-automatic-bus-off-recovery-mechanism/td-p/187400?utm_source=chatgpt.com
}

void FDCAN::recv()
{
  HAL_FDCAN_GetRxMessage(hfdcan_, FDCAN_RX_FIFO0, &rx_header_, this->rx_data);  // dismiss return
  this->rx_id = rx_header_.Identifier;
}

// [解析] 帧头已配好，这里只改 ID；AddMessageToTxFifoQ 只是入队，返回不代表帧已发上总线。
void FDCAN::send(uint16_t tx_id)
{
  tx_header_.Identifier = tx_id;
  HAL_FDCAN_AddMessageToTxFifoQ(hfdcan_, &tx_header_, this->tx_data);  // dismiss return
}

}  // namespace sp
