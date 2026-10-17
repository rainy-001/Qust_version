// [解析] VT02 图传接收机的数据定义 + 解析类：把串口来的裁判系统帧拆成摇杆/拨轮/按键/鼠标键盘。
// [解析] 上层遥控器任务创建 sp::VT02 vt02(&huart3)，先 request() 挂接收，再在串口回调里 update()。
// [解析] 要点：RefereeCustomData 的 packed 不能删（字节布局必须和协议一致）；update() 只认 0x0302。

#ifndef SP__VT02_HPP
#define SP__VT02_HPP

#include "usart.h"  // CubeMX 生成的串口句柄声明（huart1、huart3 等在这里）

namespace sp
{
// [解析] 接收缓冲 50 字节：帧头5 + 命令码2 + 数据段约33 + 帧尾2 ≈ 42，留点粘包余量；超了会被硬截断。
constexpr size_t VT02_BUFF_SIZE = 50;

// [解析] 0x0302 自定义控制器（图传遥控器）的控制数据；packed 不能删 —— 少了它编译器会插空隙，
// [解析] 成员偏移就和串口字节错开。0x0302 官方只定义 30 字节，这里多出的 float 是本项目和上位机的约定。
struct __attribute__((packed)) RefereeCustomData
{
  float control;
  float j0;
  float j1;
  float j2;
  float j3;
  float j4;
  float j5;
  float gripper;
  uint8_t reserved;  //保留位
};

// [解析] 0x0304 的鼠标数据，vx/vy/vs 已归一化到 [-1,1]；update() 只处理 0x0302，所以它不会被填充。
struct RefereeMouseData
{
  float vx;  // 取值范围: [-1, 1]
  float vy;  // 取值范围: [-1, 1]
  float vs;  // 取值范围: [-1, 1], 鼠标滚轮, s代表scroll
  bool left;
  bool right;
};

// [解析] 0x0304 的键盘数据：16 个 bool 对应 16 位键盘掩码的 bit0~bit15；同样不会被 update() 填充。
struct RefereeKeysData
{
  bool w;
  bool s;
  bool a;
  bool d;
  bool shift;
  bool ctrl;
  bool q;
  bool e;
  bool r;
  bool f;
  bool g;
  bool z;
  bool x;
  bool c;
  bool v;
  bool b;
};

// [解析] 一个对象管一路串口上的 VT02 图传接收机；解析结果放在下面的 public 成员里，不走返回值。
// [解析] 用法：构造 -> request() 挂接收 -> 串口空闲回调里 update(HAL_GetTick()) -> 主循环读 custom。
class VT02
{
public:
  VT02(UART_HandleTypeDef * huart, bool use_dma = true);

  RefereeCustomData custom;  // 只读!
  RefereeMouseData mouse;    // 只读!
  RefereeKeysData keys;      // 只读!

  // [解析] 挂接收：HAL 的 ReceiveToIdle 只收一次，每帧处理完必须再调一次，否则串口不再收数据。
  void request();
  // [解析] 拆帧入口，一般放串口空闲回调里；stamp_ms 传 HAL_GetTick()，解析成功时刷新在线时间戳。
  void update(uint32_t stamp_ms);
  bool is_open() const;
  bool is_alive(uint32_t now_ms) const;

private:
  UART_HandleTypeDef * huart_;
  bool use_dma_;
  bool has_read_;
  uint32_t last_read_ms_;
  // [解析] 接收缓冲：DMA/中断写、update() 读，两者同在中断上下文，才没有竞态。
  uint8_t buff_[VT02_BUFF_SIZE];
};

}  // namespace sp

#endif  // SP__VT02_HPP