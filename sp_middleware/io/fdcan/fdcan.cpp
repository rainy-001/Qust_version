// [解析] ==========================================================================
// [解析] 【文件】sp_middleware/io/fdcan/fdcan.cpp
// [解析] 【作用】fdcan.hpp 里声明的那些函数的具体实现，内容基本都是调用 STM32 HAL 库。
// [解析] 【看代码的重点】两个地方最容易看不懂，已在下面逐行标注：
// [解析]   1) 构造函数里那 8 个 FDCAN 帧头字段分别是什么、为什么这么填（尤其"经典CAN"那两行）；
// [解析]   2) start() 里为什么要额外打开 BUS_OFF（总线关闭）中断通知。
// [解析] 【和 can.cpp 的差别】can.cpp 每次 send 都现填一遍帧头；
// [解析]   这里把帧头"提前配好存在成员里"，send() 只改 ID，少干点活。
// [解析] ==========================================================================

#include "fdcan.hpp"

namespace sp
{
// [解析] 构造函数：初始化列表 : hfdcan_(hfdcan) 只负责存句柄；
// [解析] 花括号里那 8 行才是重点 —— 把"发送帧头"的固定字段预先配好，
// [解析] 这样以后每次 send() 就只需要改一下 ID，不用重复填。
// [解析] 下面逐个字段解释（都是 STM32H7 的 FDCAN 帧头字段，F4 的 bxCAN 没有这些概念）：
FDCAN::FDCAN(FDCAN_HandleTypeDef * hfdcan) : hfdcan_(hfdcan)
{
  // [解析] IdType = 标准 ID：ID 只有 11 位（0~0x7FF）。这也解释了为什么 hpp 里
  // [解析]   rx_id / tx_id 用 uint16_t 就够了。
  // [解析]   如果要用 29 位扩展 ID，得改成 FDCAN_EXTENDED_ID 并把 ID 换成 uint32_t。
  tx_header_.IdType = FDCAN_STANDARD_ID;
  // [解析] TxFrameType = 数据帧：这一帧里装的是数据。
  // [解析]   另一种是 FDCAN_REMOTE_FRAME（远程帧），只发"我要这个 ID 的数据"不带内容，本项目不用。
  tx_header_.TxFrameType = FDCAN_DATA_FRAME;
  // [解析] DataLength = 8 字节：注意这里填的是 HAL 的"枚举值"而不是数字 8。
  // [解析]   FD 帧最多可以填到 FDCAN_DLC_BYTES_64（64 字节），这里只填 8，配合下面的"经典CAN"。
  tx_header_.DataLength = FDCAN_DLC_BYTES_8;
  // [解析] ErrorStateIndicator = 主动错误状态：这是 FDCAN 新增的 ESI 位，
  // [解析]   表示"我（发送方）当前是不是错误主动状态"。正常通信时固定填 ACTIVE 即可，
  // [解析]   它只影响总线上别人怎么看待这次发送，不影响数据内容。
  tx_header_.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  // [解析] ★ BitRateSwitch = OFF：关闭"位速率切换(BRS)"，这是"用不用 CAN-FD 加速"的开关。
  // [解析]   打开(BRS_ON)时，一帧里"仲裁段"用普通波特率、"数据段"用更快的波特率，速度更快；
  // [解析]   关掉就全程一个波特率，也就是退化成"经典 CAN"的行为。
  // [解析]   之所以关掉：为了和总线上那些不支持 FD 的普通设备（大多数电机）和平共处。
  tx_header_.BitRateSwitch = FDCAN_BRS_OFF;
  // [解析] ★ FDFormat = 经典 CAN 格式：决定帧本身是"CAN-FD 帧"还是"经典 CAN 帧"。
  // [解析]   选了 CLASSIC_CAN 之后，本帧在物理层上和老式 CAN 帧一模一样，
  // [解析]   所以 hpp 里说的"一帧还是 8 字节"才能成立。
  // [解析]   注意：这两个字段(BRS/FDFormat)必须一起关，只关一个会发出总线上没人认识的怪帧。
  tx_header_.FDFormat = FDCAN_CLASSIC_CAN;
  // [解析] TxEventFifoControl = 不产生发送事件：FDCAN 可以把"某帧已成功发出去"记录到
  // [解析]   一个叫 Tx Event FIFO 的硬件队列里，方便精确知道发送完成时刻和时间戳。
  // [解析]   本中间件不需要这个功能，填 NO_TX_EVENTS 省掉这份开销（也省得队列满了报错）。
  tx_header_.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  // [解析] MessageMarker = 0：消息标记，只有启用了上面的发送事件 FIFO 时才用来配对"哪条记录对应哪次发送"。
  // [解析]   既然不用发送事件，这里固定填 0。
  tx_header_.MessageMarker = 0x00;
  // [解析] 小提醒：构造函数里只配置了"发送"帧头，接收方那些过滤器设置
  // [解析]   必须在 CubeMX 的 FDCAN 配置页里设好（生成到 fdcan.c 里），本类不做这件事。
}

void FDCAN::start()
{
  // [解析] 让 FDCAN 外设退出初始化状态，正式开始收发（和 bxCAN 的 HAL_CAN_Start 一个意思）。
  HAL_FDCAN_Start(hfdcan_);                                                   // dismiss return
  // [解析] "dismiss return" 是本项目的写法约定：故意忽略 HAL 的返回值，
  // [解析]   表示"这里不检查成功失败"。看到这个注释就知道不是漏写了判断。
  // [解析] 打开"接收 FIFO0 里有新消息"的中断通知：
  // [解析]   一旦有新帧，硬件就会调用你写的 HAL_FDCAN_RxFifo0Callback() 回调，
  // [解析]   你在回调里调用本类的 recv() 把数据取走。
  // [解析]   最后的 0 是"通知相关的附加参数"（本场景不用），填 0 即可。
  HAL_FDCAN_ActivateNotification(hfdcan_, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);  // dismiss return
  // [解析] ★ 这一行是 H7 系列的一个经典"坑"，值得单独讲：
  // [解析]   总线上的严重错误（比如接线松动、两端波特率不一致）会让控制器进入 BUS_OFF 状态，
  // [解析]   也就是"我自己退出总线，不再收发"。老式 bxCAN 会自动恢复，但 STM32H7 的 FDCAN
  // [解析]   在某些 HAL 版本里丢掉了自动恢复机制（见作者贴的 ST 社区链接），一旦 BUS_OFF 就永久哑掉。
  // [解析]   所以这里主动打开 BUS_OFF 中断通知，好在回调里手动重新调用 HAL_FDCAN_Start() 把总线救回来。
  // [解析]   注意：光打开通知还不够，还得自己写 HAL_FDCAN_ErrorStatusCallback() 才能真正恢复，
  // [解析]     本中间件没有提供这个回调模板，换板调试时如果发现"通信突然全断"，先查这里。
  // [解析]   （下面这行的换行和缩进是格式化工具按 100 列折的，0 就是那个附加参数。）
  HAL_FDCAN_ActivateNotification(
    hfdcan_, FDCAN_IT_BUS_OFF,
    0);  // dismiss return ref: https://community.st.com/t5/stm32-mcus-products/stm32h7-fdcan-has-lost-the-automatic-bus-off-recovery-mechanism/td-p/187400?utm_source=chatgpt.com
}

void FDCAN::recv()
{
  // [解析] 从 RX FIFO0 取走一帧：帧的"描述信息"（ID、长度、帧类型等）进成员 rx_header_，
  // [解析]   真正的 8 字节数据直接写进成员 rx_data。
  // [解析] 这里用 this->rx_data 只是把"这是本对象的成员"写明白，等价于直接写 rx_data。
  HAL_FDCAN_GetRxMessage(hfdcan_, FDCAN_RX_FIFO0, &rx_header_, this->rx_data);  // dismiss return
  // [解析] 把 ID 单独拎出来给上层用。FDCAN 的 rx_header_.Identifier 是"标准ID/扩展ID 共用"的字段，
  // [解析]   标准帧时就是 11 位 ID，扩展帧时才是 29 位 —— 具体是哪种由 rx_header_.IdType 区分。
  // [解析] 注意：这里没有像 CAN::recv() 那样保存 IdType，也没有判断扩展帧，
  // [解析]   所以上层拿到的 rx_id 是"哪一种 ID"是不知道的（见 hpp 里的提醒）。
  this->rx_id = rx_header_.Identifier;
}

void FDCAN::send(uint16_t tx_id)
{
  // [解析] 每帧只需要改 ID：其他字段（标准帧/数据帧/8字节/经典CAN）在构造函数里已经配好了。
  // [解析] 这也是把 tx_header_ 做成成员变量而不是局部变量的最大好处 —— 省掉了每次发送的重复填表。
  tx_header_.Identifier = tx_id;
  // [解析] 把 tx_data 的 8 字节按 tx_header_ 描述的格式，塞进硬件发送队列(TxFifoQ)后立刻返回。
  // [解析] 函数名里的 Q = Queue：只要队列没满，这行就不会阻塞、也不会等真正发完，
  // [解析]   所以"调用完 send() 了"≠"总线上已经发出去了"。要确认发出去得看发送事件 FIFO（本项目没开）。
  // [解析] 队列满了会返回错误，但按惯例这里 dismiss 掉了，不会报错也不会重试。
  HAL_FDCAN_AddMessageToTxFifoQ(hfdcan_, &tx_header_, this->tx_data);  // dismiss return
}

}  // namespace sp
