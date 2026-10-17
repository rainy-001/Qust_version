// [解析] DBus 遥控器接收机的数据结构：18 字节原始帧解析出的摇杆、拨杆、鼠标、键盘。
// [解析] applications/uart_task.cpp 在串口回调里调 update() 解析，其他任务直接读 ch_xx / sw_x / mouse / keys。
// [解析] 关键点：摇杆是 11 位无符号(中位 1024)，多字节数值是小端，按键靠 16 位掩码区分。

#ifndef SP__DBUS_HPP
#define SP__DBUS_HPP

#include <cstdint>
#include "usart.h"

namespace sp
{
// [解析] 一帧固定 18 字节，update() 长度不对就整帧丢弃，防止错位。
constexpr size_t DBUS_BUFF_SIZE = 18;
// [解析] 键盘 16 位掩码：每一位一个键，1 = 按下；顺序是遥控器固件的编号，不是键盘物理布局。
// [解析] W 是 bit0、S 是 bit1、A 是 bit2……B 是 bit15，别凭直觉推（A 在 bit2 而不是 bit1）。
constexpr uint16_t DBUS_KEY_W_MASK     = 0x0001;
constexpr uint16_t DBUS_KEY_S_MASK     = 0x0002;
constexpr uint16_t DBUS_KEY_A_MASK     = 0x0004;
constexpr uint16_t DBUS_KEY_D_MASK     = 0x0008;
constexpr uint16_t DBUS_KEY_SHIFT_MASK = 0x0010;
constexpr uint16_t DBUS_KEY_CTRL_MASK  = 0x0020;
constexpr uint16_t DBUS_KEY_Q_MASK     = 0x0040;
constexpr uint16_t DBUS_KEY_E_MASK     = 0x0080;
constexpr uint16_t DBUS_KEY_R_MASK     = 0x0100;
constexpr uint16_t DBUS_KEY_F_MASK     = 0x0200;
constexpr uint16_t DBUS_KEY_G_MASK     = 0x0400;
constexpr uint16_t DBUS_KEY_Z_MASK     = 0x0800;
constexpr uint16_t DBUS_KEY_X_MASK     = 0x1000;
constexpr uint16_t DBUS_KEY_C_MASK     = 0x2000;
constexpr uint16_t DBUS_KEY_V_MASK     = 0x4000;
constexpr uint16_t DBUS_KEY_B_MASK     = 0x8000;

// [解析] 三位拨杆的三个挡位；原始值 1->UP、3->MID、2(及异常值)->DOWN，和枚举顺序并不对应。
enum class DBusSwitchMode
{
  DOWN,
  MID,
  UP
};

// [解析] 鼠标数据：三个分量是本帧位移增量（不动就是 0，不累加），归一化后上层还要乘灵敏度。
struct DBusMouseData
{
  float vx;  // 取值范围: [-1, 1]
  float vy;  // 取值范围: [-1, 1]
  float vs;  // 取值范围: [-1, 1], 鼠标滚轮, s代表scroll
  bool left;
  bool right;
};

// [解析] 16 个按键"当前这一帧是否按下"的快照，来源是 keyboard_value 的对应位。
// [解析] 只有当前状态，要做"按一下触发一次"得自己存上一帧做边沿检测。
struct DBusKeysData
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

// [解析] 一个对象对应一个接收机；数据在中断里写、任务里读，单字段读写天然原子，不用加锁。
class DBus
{
public:
  // [解析] use_dma 是接线选项而不是性能选项：C 板走 DMA、达妙板走中断，选错一个字节都收不到。
  DBus(UART_HandleTypeDef * huart, bool use_dma = true);

  // [解析] 以下都是 update() 写、上层只读的解析结果，物理含义见各自的原注释。
  float ch_rh;  // 只读! 右水平摇杆, 取值范围: [-1, 1], 右正左负
  float ch_rv;  // 只读! 右垂直摇杆, 取值范围: [-1, 1], 上正下负
  float ch_lh;  // 只读! 左水平摇杆, 取值范围: [-1, 1], 右正左负
  float ch_lv;  // 只读! 左垂直摇杆, 取值范围: [-1, 1], 上正下负
  float ch_lu;  // 只读! 左上方拨轮, 取值范围: [-1, 1], 左正右负

  DBusSwitchMode sw_r;  // 只读! 右三位开关
  DBusSwitchMode sw_l;  // 只读! 左三位开关

  DBusMouseData mouse;  // 只读!
  DBusKeysData keys;    // 只读!
  uint16_t keyboard_value;  // 只读! 键盘16位原始值

  // [解析] 发起一次异步接收，非阻塞；每收完一帧都要在回调里再调一次，否则只收一帧就停了。
  void request();
  // [解析] 由串口接收回调（中断上下文）调用：size 必须正好 18，stamp_ms 用于掉线判断。
  void update(uint16_t size, uint32_t stamp_ms);

  // [解析] is_alive = 收到过数据 且 距最后一帧 <100ms（约丢 7 帧），做掉线保护用。
  bool is_open() const;
  bool is_alive(uint32_t now_ms) const;

private:
  UART_HandleTypeDef * huart_;
  bool use_dma_;

  // [解析] has_read_ 保证 is_alive() 不会用到还没赋过值的 last_read_ms_。
  bool has_read_;
  uint32_t last_read_ms_;

  // [解析] buff_ 必须是成员：DMA 在 request() 返回后还要继续往里写，不能是局部数组。
  uint8_t buff_[DBUS_BUFF_SIZE];
};

}  // namespace sp

#endif  // SP__DBUS_HPP