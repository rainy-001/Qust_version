// [解析] ==========================================================================
// [解析] 【文件】sp_middleware/io/can/can.cpp
// [解析] 【作用】can.hpp 里声明的那些函数的具体实现，内容基本都是调用 STM32 HAL 库。
// [解析] 【看代码的重点】三个地方最容易看不懂，已在下面逐行标注：
// [解析]   1) config() 里的过滤器编号（FilterBank）为什么是 0 或 14；
// [解析]   2) 每处 HAL 调用后面的 "dismiss return" 是什么意思；
// [解析]   3) send_ext() 里扩展帧 ID 的"位拼接"是怎么算的。
// [解析] ==========================================================================

#include "can.hpp"

namespace sp
{
// [解析] 构造函数：初始化列表 : hcan_(hcan) 就是把参数存进成员变量。
// [解析] 函数体是空的，因为此时还不该操作硬件（硬件初始化在 main.c 里做完了）。
CAN::CAN(CAN_HandleTypeDef * hcan) : hcan_(hcan) {}

void CAN::config()
{
  // [解析] 下面这 8 行是在填一个"过滤器"结构体，作用是决定"哪些 CAN 帧能被收下"。
  CAN_FilterTypeDef filter;
  filter.FilterActivation = CAN_FILTER_ENABLE;  // [解析] 打开这个过滤器
  filter.FilterFIFOAssignment = CAN_RX_FIFO0;   // [解析] 通过筛选的帧放进接收 FIFO0
  filter.FilterMode = CAN_FILTERMODE_IDMASK;    // [解析] 按"ID + 掩码"方式筛选
  filter.FilterScale = CAN_FILTERSCALE_32BIT;   // [解析] 用 32 位模式（F4 支持 16/32 位两种）
  // [解析] ↓ 关键：掩码(Mask)全是 0，表示"ID 的每一位都不关心"，
  // [解析]   也就是无论收到什么 ID 都放行 —— 这就是"全通配/收所有帧"。
  filter.FilterIdHigh = 0x0000;
  filter.FilterIdLow = 0x0000;
  filter.FilterMaskIdHigh = 0x0000;
  filter.FilterMaskIdLow = 0x0000;
  // [解析] STM32F4 一共有 28 个过滤器组，CAN1 和 CAN2 平分：
  // [解析]   CAN1 只能用 0~13，CAN2 只能用 14~27。
  // [解析]   所以这里用 hcan1 就选 0 号，否则（hcan2）选 14 号。
  filter.FilterBank = (hcan_ == &hcan1) ? 0 : 14;
  // [解析] SlaveStartFilterBank 告诉硬件"从第几号开始算 CAN2 的过滤器"，固定填 14。
  filter.SlaveStartFilterBank = 14;

  // [解析] 填好后交给真正干事的有参版 config()。
  config(filter);
}

void CAN::config(const CAN_FilterTypeDef & filter)
{
  HAL_CAN_ConfigFilter(hcan_, &filter);  // dismiss return
  // [解析] "dismiss return" 是本项目的写法约定：故意忽略 HAL 的返回值，
  // [解析] 表示"这里不检查成功失败"。看到这个注释就知道不是漏写了判断。
}

void CAN::start()
{
  HAL_CAN_Start(hcan_);                                              // dismiss return
  // [解析] ↑ 让 CAN 外设退出初始化状态，正式开始收发。
  HAL_CAN_ActivateNotification(hcan_, CAN_IT_RX_FIFO0_MSG_PENDING);  // dismiss return
  HAL_CAN_ActivateNotification(hcan_, CAN_IT_RX_FIFO1_MSG_PENDING);  // dismiss return
  // [解析] ↑ 这两行打开中断通知：只要 FIFO0 / FIFO1 里进了新帧，
  // [解析]   硬件就会调用你在 applications/can_task.cpp 里写的
  // [解析]   HAL_CAN_RxFifo0MsgPendingCallback() 回调。
  // [解析]   注意：FIFO1 的中断也开了，但项目里只写了 FIFO0 的回调函数。
}

void CAN::recv(uint32_t rx_fifo)
{
  // [解析] static：这个"帧头"结构体只分配一次，每次复用，避免反复进出中断时压栈。
  // [解析] 因为 recv() 只在中断里被调用，不会被重入，所以这样是安全的。
  static CAN_RxHeaderTypeDef rx_header;

  // [解析] 从硬件取走一帧：帧的"描述信息"（ID、长度、帧类型）进 rx_header，
  // [解析] 真正的 8 字节数据直接写进成员 rx_data。
  HAL_CAN_GetRxMessage(hcan_, rx_fifo, &rx_header, rx_data);  // dismiss return

  //接收时可以根据IDE判断是标准帧还是扩展帧
  // [解析] IDE = Identifier Extension，标识符扩展位，用来区分两种帧格式：
  // [解析]   扩展帧(CAN_ID_EXT)：ID 有 29 位，保存在 rx_header.ExtId
  // [解析]   标准帧：ID 只有 11 位，保存在 rx_header.StdId
  // [解析] 所以必须分开取值，否则读到的 ID 是错的。
  if (rx_header.IDE == CAN_ID_EXT) {
    rx_id = rx_header.ExtId;
    frame_type = true;  // [解析] 顺便告诉调用者：刚收到的是扩展帧
  }
  else {
    rx_id = rx_header.StdId;
    frame_type = false;
  }
}

//发送时查电机说明书确定使用标准帧还是扩展帧
void CAN::send(uint32_t tx_id)
{
  static uint32_t mailbox;              // [解析] 硬件用它返回"这帧被放进哪个发送邮箱"
  static CAN_TxHeaderTypeDef tx_header; // [解析] 同 recv()，static 复用一份即可
  tx_header.RTR = CAN_RTR_DATA;         // [解析] RTR=DATA 表示这是"数据帧"（不是远程请求帧）
  tx_header.DLC = CAN_DATA_LEN;         // [解析] DLC = Data Length Code，本帧带 8 字节

  // TODO 扩展数据帧
  // [解析] 这里写死成标准帧。要发扩展帧得走下面的 send_ext()。
  tx_header.IDE = CAN_ID_STD;
  tx_header.StdId = tx_id;

  // [解析] 把 tx_data 的内容按 tx_header 描述的格式发出去。
  HAL_CAN_AddTxMessage(hcan_, &tx_header, tx_data, &mailbox);  // dismiss return
}

//小米电机发送一帧扩展数据帧
void CAN::send_ext(uint8_t communication_type, uint16_t torque,  uint8_t motor_id, uint8_t master_id)
{
  static uint32_t mailbox;
  static CAN_TxHeaderTypeDef tx_header;
  tx_header.RTR = CAN_RTR_DATA;
  tx_header.DLC = CAN_DATA_LEN;
  tx_header.StdId = 0;      // [解析] 扩展帧不用 StdId，清零免得残留旧值
  tx_header.ExtId = 0;
  tx_header.IDE = CAN_ID_EXT;  // [解析] 明确本次用扩展帧（29 位 ID）

  
  //除了力矩控制之外，其他控制帧帧头格式相同
  // [解析] ↓ 这是本文件最"烧脑"的地方：小米电机把多个信息"塞进"一个 29 位 ID 里。
  // [解析]   用的是"移位 + 按位或"拼接，<< 24 表示放到最高 8 位，<< 8 表示放到中间。
  // [解析]   格式核对请翻 cybergear_motor/ 目录下的使用说明书 PDF。
  if (communication_type == 0x01) {
    // [解析] 0x01 = 力矩控制：高 8 位通信类型 | 中 16 位力矩 | 低 8 位电机 ID
    tx_header.ExtId = communication_type << 24 | torque << 8 | motor_id;
  }
  else {
    // [解析] 其他控制帧：高 8 位通信类型 | 中 8 位主机 ID | 低 8 位电机 ID
    tx_header.ExtId = communication_type << 24 | master_id << 8 | motor_id;
    // tx_header.ExtId = communication_type << 24 | motor_id << 8 | master_id;
    // [解析] ↑ 上一行是备用的另一种拼法，被作者注释掉了。
    // [解析]   实际接线测试时如果电机不理你，可以试着换回这一行。
  }

  HAL_CAN_AddTxMessage(hcan_, &tx_header, tx_data, &mailbox);  // dismiss return
}

}  // namespace sp
