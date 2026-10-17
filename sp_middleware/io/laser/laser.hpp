// [解析] GY-56 激光测距模块封装，一只模块 = 一路串口 + 一个 LaserGY56 对象；本工程还没有任务用它。
// [解析] 模块自己按周期往串口吐 8 字节帧，主循环只调 is_valid()/get_distance() 读，不用发查询。
// [解析] 读出来的距离可能是旧的甚至从没收到过，所以必须先问 is_valid()；只有改模式/波特率才主动发命令。

#ifndef SP__LASER_GY56_HPP
#define SP__LASER_GY56_HPP

#include <cstdint>

#include "usart.h"

namespace sp
{
enum class LaserMode : uint8_t
{
  UNKNOWN = 0,  // 尚未从有效数据帧获取模式
  SHORT = 1,    // 短距离模式
  MEDIUM = 2,   // 中距离模式
  LONG = 3      // 长距离模式
};

// [解析] 一个对象管一路串口上的一只模块：被动收帧刷新内部数据，只有改配置时才主动发命令。
class LaserGY56
{
public:
  explicit LaserGY56(UART_HandleTypeDef * huart);

  // 返回最近一帧有效距离, 单位: cm; 使用前通过is_valid()判断是否过期
  uint16_t get_distance() const;
  // 收到过有效帧且距上一帧不超过timeout_ms时返回true, 单位: ms
  bool is_valid(uint32_t timeout_ms = 500) const;
  // 返回最近一帧有效数据中报告的模式
  LaserMode get_mode() const;

  // 返回命令是否通过串口发送成功
  bool send_mode(LaserMode mode);
  bool set_baud_rate_9600();
  bool set_baud_rate_115200();
  bool save_config();
  // [解析] 返回值只代表"这 3 个字节发出去了"，模块不回 ACK，true 不等于设置生效。
  // [解析] 改完波特率务必再调 save_config()，并同步改 CubeMX 里该串口的波特率，否则掉电即丢。

  // 启动USART的DMA空闲接收; 回调传入本次收到的字节数
  bool start_receive();
  void receive_complete(uint16_t size);
  void update(uint8_t byte);
  // [解析] start_receive() 用 HAL_UARTEx_ReceiveToIdle_DMA，回调里再转手调 receive_complete(size)。
  // [解析] update(byte) 是核心：每来 1 字节维护一次 8 字节滑动窗口，凑满且校验通过才解析。

private:
  // [解析] 一帧固定 8 字节：[0][1] 0x5A 0x5A | [2] 0x15 | [3] 0x03 | [4][5] 距离(大端) | [6] 状态+模式 | [7] 累加和。
  static constexpr uint8_t FRAME_SIZE = 8;
  static constexpr uint16_t RX_BUFFER_SIZE = 64;

  bool send_command(uint8_t command);
  bool frame_valid() const;
  void parse_frame();

  UART_HandleTypeDef * huart_;
  uint8_t rx_buff_[RX_BUFFER_SIZE];
  uint8_t frame_[FRAME_SIZE];
  uint8_t frame_size_;

  volatile uint16_t distance_cm_;
  volatile uint32_t last_frame_tick_;
  volatile LaserMode mode_;
  volatile bool has_reliable_distance_;
  // [解析] 这 4 个成员带 volatile：它们在串口中断里被改写、主循环里被读，不加可能永远读到旧值。
  // [解析] 前三者由 parse_frame() 每帧刷新，has_reliable_distance_ 是 is_valid() 的第一道门槛。
};

}  // namespace sp

#endif  // SP__LASER_GY56_HPP
