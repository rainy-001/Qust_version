// [解析] CAN 通信封装。
// [解析] 做什么：把 HAL 的 bxCAN 操作包成一个类，上层只需"填 8 字节 -> send(id)"
// [解析]          或 "recv() -> 读 rx_data"，不必直接接触 HAL 句柄和过滤器结构体。
// [解析] 谁在用：applications/can_task.cpp 建对象，按 config -> start -> 循环收发 使用；
// [解析]          接收是在 HAL_CAN_RxFifo0MsgPendingCallback 中断里调 recv() 完成的。
// [解析] 要点：收发数据走 rx_id/rx_data/tx_data 这几个公共缓冲区，而不是函数返回值；
// [解析]       标准帧与扩展帧靠接收帧头的 IDE 位区分。

#ifndef SP__CAN_HPP
#define SP__CAN_HPP

#include "can.h"

namespace sp
{
// [解析] CAN 一帧固定 8 字节。
constexpr size_t CAN_DATA_LEN = 8;

// [解析] 一个对象对应一路 CAN（如 CAN1）。缓冲区的读写方向见各自注释。
class CAN
{
public:
  CAN(CAN_HandleTypeDef * hcan);

  // [解析] 收发数据靠这几个缓冲区交换：recv() 之后读 rx_*，send() 之前写 tx_data。
  uint32_t rx_id;                 // 只读! recv()的输出
  uint8_t rx_data[CAN_DATA_LEN];  // 只读! recv()的输出
  uint8_t tx_data[CAN_DATA_LEN];  // 只写! send()的输入
  bool frame_type = false;    // 只写! send()的输入, false: 标准帧, true: 扩展帧

  // 使用C板官方示例的CAN过滤器配置
  // ref: https://github.com/RoboMaster/Development-Board-C-Examples/blob/master/20.standard_robot/bsp/boards/bsp_can.c
  // [解析] 过滤器全通配，总线上所有帧都收。调试阶段用这个最省事。
  void config();

  // 手动配置CAN过滤器的过滤规则
  // ref: https://github.com/HNUYueLuRM/basic_framework/blob/master/bsp/can/bsp_can.c
  // [解析] 只放行关心的 ID，减轻中断压力。上面的无参 config() 内部就是调它。
  void config(const CAN_FilterTypeDef & filter);

  // 启动CAN通信
  // [解析] 让外设开始收发，并打开"收到帧就进中断"的通知。必须排在 config() 之后。
  void start();

  // 接收1帧标准数据帧或扩展数据帧
  // rx_fifo: 缓冲区位置, 默认为CAN_RX_FIFO0
  // [解析] 只在中断里调用，结果写进 rx_* 缓冲区（见 can_task.cpp 的 FIFO0 回调）。
  void recv(uint32_t rx_fifo = CAN_RX_FIFO0);

  // 发送1帧标准数据帧
  void send(uint32_t tx_id);
  // 发送1帧CyberGear的扩展数据帧
  // [解析] 小米电机用扩展帧、且帧头要按说明书拼位，所以单独一条发送路径。
  void send_ext(uint8_t communication_type, uint16_t data, uint8_t tx_id, uint8_t master_id);

private:
  // [解析] 本对象管的是哪一路 CAN（&hcan1 / &hcan2）。
  CAN_HandleTypeDef * hcan_;
};

}  // namespace sp

#endif  // SP__CAN_HPP
