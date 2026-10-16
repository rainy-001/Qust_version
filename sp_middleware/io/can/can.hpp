// [解析] ==========================================================================
// [解析] 【文件】sp_middleware/io/can/can.hpp
// [解析] 【作用】把 STM32 HAL 库那套繁琐的 CAN 操作，封装成一个好用的 CAN 类。
// [解析]         上层只需要：填 tx_data -> send(id)，或者 recv() -> 读 rx_data。
// [解析] 【被谁用】applications/can_task.cpp：sp::CAN can1(&hcan1); 创建对象。
// [解析] 【背景知识】CAN 是 RoboMaster 车上电机、超电、裁判系统互相通信的总线。
// [解析]             它一帧最多带 8 字节数据（下面 CAN_DATA_LEN 就是 8）。
// [解析] 【阅读顺序】先看本文件了解"有什么功能"，再看 can.cpp 了解"怎么实现的"。
// [解析] ==========================================================================

#ifndef SP__CAN_HPP
#define SP__CAN_HPP

#include "can.h"  // CubeMX 生成的 CAN 外设句柄声明（hcan1、hcan2 等在这里）

namespace sp
{
// [解析] 给"8"起个名字，避免代码里到处出现魔法数字 8。
constexpr size_t CAN_DATA_LEN = 8;

// [解析] CAN 类：一个对象管理一路 CAN（比如 CAN1 一个对象、CAN2 另一个对象）。
// [解析] 典型用法见 applications/can_task.cpp，一共 4 步：
// [解析]   1) can1.config();   // 配置过滤器（决定收哪些帧）
// [解析]   2) can1.start();    // 启动外设，并打开"收到帧就进中断"的通知
// [解析]   3) 往 can1.tx_data 填 8 字节 -> can1.send(目标ID)
// [解析]   4) 在中断回调里 can1.recv() -> 读 can1.rx_id / can1.rx_data
class CAN
{
public:
  // [解析] 构造函数只把 HAL 句柄存起来，不碰任何硬件。
  // [解析] 传 &hcan1 表示"这个对象管 CAN1"。
  CAN(CAN_HandleTypeDef * hcan);

  // [解析] 下面 4 个成员是"公共缓冲区"：收发数据不靠函数返回值传递，
  // [解析] 而是靠读写这些成员变量。请特别注意每个后面的"只读/只写"提示。
  uint32_t rx_id;                 // 只读! recv()的输出
  // [解析] ↑ 收到的帧 ID，用来判断"这是哪个电机的回包"。
  uint8_t rx_data[CAN_DATA_LEN];  // 只读! recv()的输出
  // [解析] ↑ 收到的 8 字节原始数据。
  uint8_t tx_data[CAN_DATA_LEN];  // 只写! send()的输入
  // [解析] ↑ 发送前，先把要发的 8 字节写到这里。
  bool frame_type = false;    // 只写! send()的输入, false: 标准帧, true: 扩展帧
  // [解析] ↑ recv() 也会顺手更新它，好让调用者知道刚收到的这帧是标准帧还是扩展帧。

  // 使用C板官方示例的CAN过滤器配置
  // ref: https://github.com/RoboMaster/Development-Board-C-Examples/blob/master/20.standard_robot/bsp/boards/bsp_can.c
  // [解析] 无参版 config()：用"全通配"过滤器，即总线上所有 CAN 帧都收，不做任何筛选。
  // [解析] 新手调试阶段直接用这个最省事，代价是无关帧也会进中断。
  void config();

  // 手动配置CAN过滤器的过滤规则
  // ref: https://github.com/HNUYueLuRM/basic_framework/blob/master/bsp/can/bsp_can.c
  // [解析] 有参版 config()：自己指定过滤规则，只放行关心的 ID，可以减轻中断压力。
  void config(const CAN_FilterTypeDef & filter);

  // 启动CAN通信
  // [解析] 真正让 CAN 外设跑起来，并打开"收到消息就触发中断"的通知。
  // [解析] 必须在 config() 之后调用，顺序反了过滤器不生效。
  void start();

  // 接收1帧标准数据帧或扩展数据帧
  // rx_fifo: 缓冲区位置, 默认为CAN_RX_FIFO0
  // [解析] 从硬件接收 FIFO 里取出一帧，结果写进 rx_id / rx_data / frame_type。
  // [解析] 实际调用位置：HAL_CAN_RxFifo0MsgPendingCallback() 中断回调里，
  // [解析] 配合 while (HAL_CAN_GetRxFifoFillLevel(...) > 0) 把积压的帧一次取完。
  void recv(uint32_t rx_fifo = CAN_RX_FIFO0);

  // 发送1帧标准数据帧
  // [解析] 把 tx_data 里的 8 字节，以"标准帧"格式发到 tx_id 这个 ID 上。
  void send(uint32_t tx_id);
  // 发送1帧CyberGear的扩展数据帧
  // [解析] 小米 CyberGear 电机用的是"扩展帧"，而且帧头要按它的说明书拼位，
  // [解析] 和普通标准帧差别较大，所以单独写成一个 send_ext()。
  void send_ext(uint8_t communication_type, uint16_t data, uint8_t tx_id, uint8_t master_id);

private:
  // [解析] 记住本对象管的是哪一路 CAN 外设（&hcan1 或 &hcan2）。
  // [解析] 下面所有 HAL_xxx(hcan_, ...) 调用都要把它当第一个参数传进去。
  CAN_HandleTypeDef * hcan_;
};

}  // namespace sp

#endif  // SP__CAN_HPP
