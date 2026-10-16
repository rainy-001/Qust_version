// [解析] ==========================================================================
// [解析] 【文件】sp_middleware/io/fdcan/fdcan.hpp
// [解析] 【作用】把 STM32 HAL 库的 FDCAN（新一代 CAN 外设）封装成一个好用的类。
// [解析]         上层只需要：填 tx_data -> send(id)，或者 recv() -> 读 rx_data。
// [解析] 【被谁用】本工程（STM32F407）目前还没有人用它：
// [解析]           F407 片内只有"老一辈"的 bxCAN 外设，根本没有 FDCAN 这个模块，
// [解析]           Core/Src 里只有 can.c、没有 fdcan.c，fdcan.cpp 也没加进 CMakeLists.txt。
// [解析]           它是给"达妙板"(STM32H7 系列)这类自带 FDCAN 的板子预留的中间件，
// [解析]           换板子并在 CubeMX 里打开 FDCAN 之后，才需要照着 can_task.cpp 的写法去用它。
// [解析] 【和 can.hpp 的关系】接口几乎是照抄 can.hpp（start/recv/send + 公共缓冲），
// [解析]           但 FDCAN 的帧头字段比 bxCAN 多得多，ID 也只有 11 位标准 ID 这一种用法。
// [解析] 【阅读顺序】先看本文件了解"有什么功能"，再看 fdcan.cpp 了解"怎么实现的"。
// [解析] ==========================================================================

#ifndef SP__FDCAN_HPP
#define SP__FDCAN_HPP

#include "fdcan.h"
// [解析] ↑ CubeMX 生成的 FDCAN 外设句柄声明（例如 hfdcan1）就在这里。

// [解析] 给"8"起个名字，避免代码里到处出现魔法数字 8。
// [解析] 说明：FDCAN 的"FD 帧"理论上单帧最多能带 64 字节数据，
// [解析]   但本中间件只跑"经典 CAN"模式（见 fdcan.cpp 里的 FDCAN_CLASSIC_CAN），
// [解析]   所以一帧依旧是 8 字节，和 can.hpp 里的 CAN_DATA_LEN 保持一致，
// [解析]   这样上层"填 tx_data[8] -> send()"的写法可以直接照搬过来。
namespace sp
{
constexpr size_t FDCAN_DATA_LEN = 8;

// [解析] FDCAN 类：一个对象管理一路 FDCAN（比如 FDCAN1 一个对象、FDCAN2 另一个对象）。
// [解析] 用法（和 sp::CAN 基本一致）：
// [解析]   1) 定义全局对象 sp::FDCAN fdcan1(&hfdcan1);   // 构造函数里顺便把发送帧头配好
// [解析]   2) fdcan1.start();                            // 启动外设 + 打开接收中断通知
// [解析]   3) 往 fdcan1.tx_data 填 8 字节 -> fdcan1.send(目标ID)
// [解析]   4) 在中断回调里 fdcan1.recv() -> 读 fdcan1.rx_id / fdcan1.rx_data
// [解析] 注意：本类没有 config()，过滤器配置要在 CubeMX 里提前设好（生成到 fdcan.c 里）。
class FDCAN
{
public:
  // [解析] 构造函数：存下 HAL 句柄，并把"发送帧头"里那些固定不变的字段一次性填好
  // [解析]   （具体填了什么、为什么这么填，见 fdcan.cpp 里的逐行说明）。
  // [解析] 传 &hfdcan1 表示"这个对象管 FDCAN1"。
  FDCAN(FDCAN_HandleTypeDef * hfdcan);

  // [解析] 下面 3 个成员是"公共缓冲区"：收发数据不靠函数返回值传递，而是靠读写这些成员变量。
  uint16_t rx_id;                      // [解析] 收到的帧 ID（11 位标准 ID），用来判断"这是哪个设备的回包"。
  // [解析] 注意：类型是 uint16_t，最多表示 0~0x7FF(2047)，装不下扩展帧的 29 位 ID。
  // [解析]   如果总线上真有扩展帧，这里只能拿到低 16 位，会读出一个错的 ID。
  uint8_t rx_data[FDCAN_DATA_LEN];     // [解析] 只读! recv()的输出：收到的 8 字节原始数据。
  uint8_t tx_data[FDCAN_DATA_LEN];     // [解析] 只写! send()的输入：发送前先把 8 字节写到这里。

  // [解析] 启动 FDCAN：让外设跑起来，并打开"收到消息 / 总线关闭"两类中断通知。
  // [解析] ★ 必须在对象构造之后、收发之前调用，而且只能调一次。
  void start();

  // [解析] 从硬件接收 FIFO0 里取出一帧，结果写进 rx_id / rx_data。
  // [解析] 实际调用位置：HAL_FDCAN_RxFifo0Callback() 中断回调里，
  // [解析]   配合 while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan_, FDCAN_RX_FIFO0) > 0) 把积压的帧一次取完。
  void recv();

  // [解析] 把 tx_data 里的 8 字节，以 tx_id 为目的 ID 发出去。
  // [解析] 帧格式（标准帧/数据帧/8字节/经典CAN）在构造函数里就定死了，这里只改 ID。
  void send(uint16_t tx_id);

private:
  // [解析] 记住本对象管的是哪一路 FDCAN 外设（&hfdcan1 或 &hfdcan2）。
  // [解析] 下面所有 HAL_FDCAN_xxx(hfdcan_, ...) 调用都要把它当第一个参数传进去。
  FDCAN_HandleTypeDef * hfdcan_;

  // [解析] ★ 这两个帧头做成了"成员变量"而不是 can.cpp 里那样的 static 局部变量。
  // [解析]   原因：发送帧头需要在构造函数里预配置一次、之后一直复用，所以必须活得比函数久。
  // [解析]   代价是每个 FDCAN 对象都会多占一份（几十字节），对象少的话无所谓。
  // [解析]   也是因为这个原因，recv() 不能像 CAN::recv() 那样随便重入 —— 收到帧的
  // [解析]   那一瞬间如果别处也在调 recv()，两边会互相踩 rx_header_。
  FDCAN_RxHeaderTypeDef rx_header_;  // [解析] 接收帧头：每收一帧由 HAL 填好，供 recv() 读 ID。
  FDCAN_TxHeaderTypeDef tx_header_;  // [解析] 发送帧头：构造函数配好固定字段，send() 只改 Identifier。
};

}  // namespace sp

#endif  // SP__FDCAN_HPP