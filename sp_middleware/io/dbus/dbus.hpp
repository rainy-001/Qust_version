// [解析] ==========================================================================
// [解析] 【文件】sp_middleware/io/dbus/dbus.hpp
// [解析] 【作用】把 DBus 遥控器接收机通过串口送来的 18 字节原始数据，
// [解析]         解析成好用的摇杆浮点数、拨杆挡位、鼠标、键盘布尔量。
// [解析] 【被谁用】applications/uart_task.cpp：
// [解析]           sp::DBus remote(&huart3);   // 创建对象（C 板接在 USART3）
// [解析]           然后在 HAL_UARTEx_RxEventCallback() 里 remote.update(Size, stamp_ms)。
// [解析]           其他任务再读 remote.ch_lh / remote.sw_l / remote.mouse / remote.keys。
// [解析] 【关键点】DBus 协议最难看懂的是它的"位打包"：
// [解析]   18 个字节里，4 个摇杆通道每个占 11 位，而且不对齐字节边界，一个通道会横跨 2~3 个字节，
// [解析]   所以只能靠"移位 + 按位或"把位拼回来（详见 dbus.cpp 的 update()）。
// [解析]   另外，摇杆是 11 位无符号值(0~2047，中位 1024)；
// [解析]   而多字节的数值（鼠标的 vx/vy/vs、键盘 16 位值）都是"低字节在前"的小端排列，
// [解析]   所以代码里写成 (buff_[7] << 8) | buff_[6] 这种"把后一个字节当高 8 位"的形式。
// [解析] 【阅读顺序】先看本文件了解"有哪些数据"，再看 dbus.cpp 了解"怎么解析的"。
// [解析] ==========================================================================

#ifndef SP__DBUS_HPP
#define SP__DBUS_HPP

#include <cstdint>
#include "usart.h"
// [解析] ↑ 里面声明了 UART_HandleTypeDef 和 huart3 等串口句柄（CubeMX 生成在 usart.c 里）。

namespace sp
{
// [解析] DBus 遥控器一帧永远是 18 字节，这个数字是协议规定的，不能改。
// [解析] update() 里会检查"本次收到的字节数必须正好等于 18"，否则整帧丢弃 ——
// [解析]   这是防止"串口因为噪声多收/少收几个字节"导致后面所有数据全部错位。
constexpr size_t DBUS_BUFF_SIZE = 18;
// [解析] ★ 下面这一组常量是"键盘按键位掩码"：遥控器发来的键盘数据只有 2 个字节(16 位)，
// [解析]   这 16 个二进制位就代表了 16 个按键，每一位对应一个键，1 = 按下，0 = 松开。
// [解析]   W 对应最低位 bit0（值 0x0001），往后依次是 S、A、D、Shift、Ctrl、Q、E、R、F、G、Z、X、C、V，
// [解析]   最后 B 对应最高位 bit15（值 0x8000）。
// [解析]   ⚠ 这个顺序是"遥控器固件里的编号顺序"（来自官方 DBus 协议表），
// [解析]   并不是键盘上的物理位置顺序（比如 A 在 bit2 而不是 bit1），所以千万不要凭直觉推。
// [解析]   解析方式见 dbus.cpp 最后那些 keys.x = ((keyboard_value & 掩码) != 0)。
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
// [解析] 上面 16 个掩码正好把 16 个位用满：0x0001 是 bit0，每次左移一位，
// [解析] 到 0x8000 就是 bit15。用键盘掩码的写法比手写 (1 << n) 更直观，也不容易数错位。

// [解析] 三位拨杆（就是遥控器上左右各一个的三段开关）的三个位置。
// [解析] 枚举顺序 DOWN/MID/UP 和下面 get_switch() 里的 raw 值不是顺序对应的，注意看：
// [解析]   遥控器原始值 1 -> UP(上)   3 -> MID(中)   2(以及其它任何异常值) -> DOWN(下)
// [解析] 它本质上是"档位"而不是"布尔开关"，所以用 enum class 而不是 bool。
enum class DBusSwitchMode
{
  DOWN,
  MID,
  UP
};

// [解析] 鼠标数据。三个分量都是"这一帧的位移增量"而不是绝对坐标，
// [解析]   也就是说鼠标动了才有值，不动就是 0，每帧都会被新值覆盖（不会累加）。
// [解析] 原注释写的取值范围 [-1, 1] 是归一化后的理论上限（见 dbus.cpp 里除以 32768 的说明），
// [解析]   实际使用中数值通常很小（比如 0.003），上层一般还要再乘一个灵敏度系数才用于控制。
struct DBusMouseData
{
  float vx;  // 取值范围: [-1, 1]
  // [解析] ↑ 鼠标左右移动量，右为正 —— 原注释保留。
  float vy;  // 取值范围: [-1, 1]
  // [解析] ↑ 鼠标前后移动量，上(前)为正 —— 原注释保留。
  float vs;  // 取值范围: [-1, 1], 鼠标滚轮, s代表scroll
  // [解析] ↑ 滚轮滚动量，一般向前滚为正 —— 原注释保留。
  bool left;
  // [解析] ↑ 鼠标左键是否按下：true = 按下，false = 松开。
  bool right;
  // [解析] ↑ 鼠标右键是否按下：true = 按下，false = 松开。
};

// [解析] 键盘 16 个按键的"是否按下"状态，统一用 bool 表示，读起来比位掩码直观。
// [解析] 每个成员的真假来源见 dbus.cpp 里对应的 DBUS_KEY_xxx_MASK。
// [解析] ⚠ 这里只表示"当前这一帧按下与否"的快照，所以上层想实现"按一下触发一次"的话，
// [解析]   必须自己保存上一帧状态做边沿检测，不能直接拿 keys.x 当触发信号。
struct DBusKeysData
{
  bool w;      // [解析] W 键：bit0  (0x0001)
  bool s;      // [解析] S 键：bit1  (0x0002)
  bool a;      // [解析] A 键：bit2  (0x0004)
  bool d;      // [解析] D 键：bit3  (0x0008)
  bool shift;  // [解析] Shift 键：bit4 (0x0010)
  bool ctrl;   // [解析] Ctrl 键：bit5 (0x0020)
  bool q;      // [解析] Q 键：bit6  (0x0040)
  bool e;      // [解析] E 键：bit7  (0x0080)
  bool r;      // [解析] R 键：bit8  (0x0100)
  bool f;      // [解析] F 键：bit9  (0x0200)
  bool g;      // [解析] G 键：bit10 (0x0400)
  bool z;      // [解析] Z 键：bit11 (0x0800)
  bool x;      // [解析] X 键：bit12 (0x1000)
  bool c;      // [解析] C 键：bit13 (0x2000)
  bool v;      // [解析] V 键：bit14 (0x4000)
  bool b;      // [解析] B 键：bit15 (0x8000)
};

// [解析] DBus 类：一个对象代表"一个遥控器接收机"。
// [解析] 用法（完整例子见 applications/uart_task.cpp）：
// [解析]   1) 全局定义对象 sp::DBus remote(&huart3);
// [解析]   2) 任务里调一次 remote.request(); 发起接收
// [解析]   3) 串口收满一帧触发回调 -> remote.update(Size, 时间戳) 解析 -> 再 request() 接力
// [解析]   4) 业务代码随时读 remote.ch_lh / remote.sw_l / remote.mouse / remote.keys
// [解析] 注意：数据是在中断上下文里被写的，业务任务里读的时候不需要加锁 ——
// [解析]   因为每个成员都是"单个 32 位以内的量"（float/bool），读写本身是原子的，
// [解析]   最坏情况只是某一帧的新旧数据混用，对遥控器这种应用完全无所谓。
class DBus
{
public:
  // [解析] 构造函数：只把串口句柄和"是否用 DMA"记下来，不碰硬件。
  // [解析] huart   —— 遥控器接收机接在哪一路串口上，C 板是 &huart3，达妙板是 &huart5。
  // [解析] use_dma —— 接收方式：true 用 DMA（C 板用法），false 用普通中断（达妙板用法）。
  // [解析]   有默认值 true，所以平时的写法 sp::DBus remote(&huart3); 就是"走 DMA"。
  // [解析] ★ 这个参数不是"性能选项"而是"硬件接线选项"：板子上有没有把串口的 DMA 请求通道配好，
  // [解析]   决定了必须选哪种，选错了就一个字节都收不到。
  DBus(UART_HandleTypeDef * huart, bool use_dma = true);

  // [解析] 下面这些 float / bool 是"公共数据"，都由 update() 解析后写入，
  // [解析] 上层只管读。每个字段的物理含义和单位看原注释。
  float ch_rh;  // 只读! 右水平摇杆, 取值范围: [-1, 1], 右正左负
  // [解析] ↑ ch = channel(通道)，rh = right horizontal(右水平)。这是遥控器右摇杆的左右方向。
  float ch_rv;  // 只读! 右垂直摇杆, 取值范围: [-1, 1], 上正下负
  // [解析] ↑ rv = right vertical。右摇杆的上下方向，往上推为正。
  float ch_lh;  // 只读! 左水平摇杆, 取值范围: [-1, 1], 右正左负
  // [解析] ↑ lh = left horizontal。左摇杆的左右方向。
  float ch_lv;  // 只读! 左垂直摇杆, 取值范围: [-1, 1], 上正下负
  // [解析] ↑ lv = left vertical。左摇杆的上下方向，底盘前后运动一般用它。
  float ch_lu;  // 只读! 左上方拨轮, 取值范围: [-1, 1], 左正右负
  // [解析] ↑ lu = left wheel/up。遥控器左上角那个"拨轮"（不是摇杆），左右拨动时变化。
  // [解析]   它单独占第 16、17 字节（另外 4 个摇杆挤在第 0~5 字节里）。

  DBusSwitchMode sw_r;  // 只读! 右三位开关
  // [解析] ↑ 遥控器右上角的三段拨杆，一般用来切换"底盘跟随/不跟随"这类模式。
  DBusSwitchMode sw_l;  // 只读! 左三位开关
  // [解析] ↑ 遥控器左上角的三段拨杆，一般用来切"开火/停火"或安全开关。

  DBusMouseData mouse;  // 只读!
  // [解析] ↑ 鼠标数据（键鼠模式下才有效，用遥控器时可能一直是 0/false）。
  DBusKeysData keys;    // 只读!
  // [解析] ↑ 键盘按键的"按下与否"状态。
  uint16_t keyboard_value;  // 只读! 键盘16位原始值
  // [解析] ↑ 键盘那两个字节的原始值。平时用上面的 keys 就行；
  // [解析]   只有想用某个"keys 结构体里没定义的键"时才需要直接操作这 16 位。

  // [解析] 发起一次异步接收：让串口在后台把最多 DBUS_BUFF_SIZE(18) 个字节收进内部缓冲。
  // [解析] 这个函数是"非阻塞"的，调用后立刻返回，收满/空闲时由 HAL 触发回调。
  // [解析] ★ 每收完一帧都要再调一次（写在回调里），否则只收一帧就永久停了。
  void request();
  // [解析] 解析一帧数据，把结果写进上面那些公有成员。由串口回调调用。
  // [解析] size     —— 本次实际收到的字节数，必须正好等于 18，否则直接丢弃不解析。
  // [解析] stamp_ms —— 这一帧的接收时刻（毫秒，来自 osKernelSysTick()），用于 is_alive() 判断掉线。
  // [解析] 副作用：会更新内部的 has_read_ / last_read_ms_，所以调用后 is_open() 会变成 true。
  // [解析] ⚠ 这个函数运行在中断里，所以实现上刻意做得很短（没有循环、没有阻塞调用、没有浮点除法以外的重活）。
  void update(uint16_t size, uint32_t stamp_ms);

  // [解析] 是否"曾经成功收到过一帧"。返回 true 只说明通信建立过，不代表现在还有数据。
  bool is_open() const;
  // [解析] 是否"现在还活着"：要求收到过数据，并且距最后一次成功解析不超过 100ms。
  // [解析] now_ms 传当前毫秒时间戳（一般还是 osKernelSysTick()）。
  // [解析] 典型用法：if (!remote.is_alive(osKernelSysTick())) { 进入安全状态/停车 }
  // [解析] ★ 100ms 这个阈值：DBus 大约 14ms 来一帧，100ms 相当于连续丢了 7 帧，
  // [解析]   基本可以断定接收机掉线或串口卡死。（这个 100 是直接写在 cpp 里的魔法数字。）
  bool is_alive(uint32_t now_ms) const;

private:
  // [解析] 记住本对象用哪一路串口（&huart3 / &huart5）。所有 HAL_UART 调用都靠它。
  UART_HandleTypeDef * huart_;
  // [解析] 记住用 DMA 还是中断接收（对应构造函数的第 2 个参数）。
  // [解析] 只在 request() 里被用来二选一，运行中不会变。
  bool use_dma_;

  // [解析] has_read_：是否已经成功收到过至少一帧。它让 is_alive() 在"从没收到过数据"时
  // [解析]   直接返回 false，而不会去用还没赋过值的 last_read_ms_。
  bool has_read_;
  // [解析] last_read_ms_：最近一次成功收到帧的时刻（毫秒）。注意它在 update() 的**开头**就被更新，
  // [解析]   也就是说哪怕后面发现数据异常、这一帧被丢弃，也仍然算"接收机在发数据"。
  uint32_t last_read_ms_;

  // [解析] 接收缓冲：DMA/中断把串口收到的原始字节直接写到这里，update() 再从这个数组解析。
  // [解析] ★ 它是被 DMA 直接写入的，属于"C 语言看不见的并发修改"：
  // [解析]   所以绝不能在接收进行中随意改写这个数组（会撞车），也正因如此它必须是成员变量
  // [解析]   而不能是 request() 里的局部变量 —— DMA 在 request() 返回后还要继续往里写。
  uint8_t buff_[DBUS_BUFF_SIZE];
};

}  // namespace sp

#endif  // SP__DBUS_HPP