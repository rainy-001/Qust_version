// [解析] ==========================================================================
// [解析] 【文件】sp_middleware/referee/vt03/vt03.hpp
// [解析] 【作用】VT03 图传链路的协议定义 + 解析类。VT03 是新版图传遥控器，
// [解析]         它把"遥控器摇杆/鼠标/键盘"和"裁判系统串口数据"混在同一个串口里发过来，
// [解析]         所以这个文件既定义了遥控器私有帧（0xA9 0x53 开头），
// [解析]         也定义了裁判系统标准帧里的几个结构体。
// [解析] 【被谁用】applications/ 下的遥控器/图传任务：sp::VT03 vt03(&huart3);
// [解析]         vt03.request(); 中断里 vt03.update(size, HAL_GetTick());
// [解析] 【关键点】1) 大量 __attribute__((packed)) 和位域(bitfield)，布局必须和协议逐 bit 对上；
// [解析]         2) 摇杆原始值 0~2047 通过 (x - 1024) / 660.0f 归一化成 -1~1；
// [解析]         3) 键盘是 16 位掩码，下面的 VT03_KEY_xxx_MASK 就是各 bit 的"选择子"；
// [解析]         4) static_assert 是"编译期保险丝"，结构体大小写错会直接编译不过。
// [解析] 【阅读顺序】先看本文件了解数据长什么样，再看 vt03.cpp 了解怎么解出来的。
// [解析] ==========================================================================

#ifndef SP__VT03_HPP
#define SP__VT03_HPP

#include <array>
#include <cstring>

#include "referee/referee_protocol/referee_protocol.hpp"
#include "usart.h"

namespace sp
{

// [解析] 下面 16 个常量是"键盘 16 位掩码"的每一位对应的按键。
// [解析] 用法：keyboard_value & VT03_KEY_W_MASK，结果非 0 就说明 W 键被按下了。
// [解析] 为什么用 1/2/4/8/.../0x8000 这种"每次翻倍"的值？因为它们分别是二进制的
// [解析]   bit0 ~ bit15，每一个常量只占一个 bit，多个键同时按下也不会互相干扰，
// [解析]   这样一个 uint16_t 就能同时表达 16 个按键的状态（这叫位图/bitmask）。
// [解析] 注意：这些值不能随便改，它们必须和裁判系统协议里"键盘键值"定义一致；
// [解析]   想做自定义键位映射，应该在自己代码里转换，而不是改这里的位。
constexpr uint16_t VT03_KEY_W_MASK = 0x0001;  // [解析] bit0，二进制 ...0001
constexpr uint16_t VT03_KEY_S_MASK = 0x0002;  // [解析] bit1
constexpr uint16_t VT03_KEY_A_MASK = 0x0004;  // [解析] bit2
constexpr uint16_t VT03_KEY_D_MASK = 0x0008;  // [解析] bit3
constexpr uint16_t VT03_KEY_SHIFT_MASK = 0x0010;  // [解析] bit4
constexpr uint16_t VT03_KEY_CTRL_MASK = 0x0020;   // [解析] bit5
constexpr uint16_t VT03_KEY_Q_MASK = 0x0040;      // [解析] bit6
constexpr uint16_t VT03_KEY_E_MASK = 0x0080;      // [解析] bit7
constexpr uint16_t VT03_KEY_R_MASK = 0x0100;      // [解析] bit8
constexpr uint16_t VT03_KEY_F_MASK = 0x0200;      // [解析] bit9
constexpr uint16_t VT03_KEY_G_MASK = 0x0400;      // [解析] bit10
constexpr uint16_t VT03_KEY_Z_MASK = 0x0800;      // [解析] bit11
constexpr uint16_t VT03_KEY_X_MASK = 0x1000;      // [解析] bit12
constexpr uint16_t VT03_KEY_C_MASK = 0x2000;      // [解析] bit13
constexpr uint16_t VT03_KEY_V_MASK = 0x4000;      // [解析] bit14
constexpr uint16_t VT03_KEY_B_MASK = 0x8000;      // [解析] bit15

// [解析] 自定义控制器（图传遥控器）发给机器人的"关节级"指令，命令码 0x0302。
// [解析] 6 个 float 分别对应 6 个关节的期望值（具体是位置/角度/速度，由上位机约定），
// [解析] 单位建议统一成弧度或角度，并且和上位机对齐，否则会出现"数值对但动作差很多"的坑。
// [解析] 整个结构体 = 6*4 + 6 = 30 字节，正好等于裁判系统 0x0302 的数据长度（30 字节）。
// [解析] `= 0.0f` 是 C++11 的"默认成员初始化器"：对象一构造，这些成员就自动清零，
// [解析]   好处是即使从未收到过数据，读到的也是 0 而不是垃圾值，对新手非常友好。
// [解析] packed 依然不能删：必须保证成员偏移和协议字节流一一对应。
struct __attribute__((packed)) RefereeCustomData
{
  float j0 = 0.0f;  // [解析] 关节 0 的数值，单位由上位机约定
  float j1 = 0.0f;  // [解析] 关节 1
  float j2 = 0.0f;  // [解析] 关节 2
  float j3 = 0.0f;  // [解析] 关节 3
  float j4 = 0.0f;  // [解析] 关节 4
  float j5 = 0.0f;  // [解析] 关节 5
  uint8_t reserved[6];  //保留位
  // [解析] ↑ 6 字节保留位，作用是"凑长度"：6*4 + 6 = 30 字节，正好等于协议规定长度。
  // [解析]   如果这里写成 uint8_t reserved[1]，结构体就只有 25 字节，static_assert 会直接报错。
  // [解析]   这 6 字节里目前没有有效数据，读取时应当忽略。
};

// [解析] 机器人回发给自定义控制器（上位机）的状态数据，命令码 0x0309。
// [解析] 这是"反向通道"：让图传界面上能显示机器人当前关节的位置和速度，方便操作手看状态。
// [解析] 7 个 float + 2 字节保留 = 30 字节，同样等于 0x0309 的数据长度。
struct __attribute__((packed)) RefereeRobotData
{
  float mode = 0.0f;     // [解析] 机器人当前模式/状态，具体编码由双方约定（例如 0=待机 1=手动）
  float j0_pos = 0.0f;   // [解析] 关节 0 的当前位置反馈
  float j1_pos = 0.0f;   // [解析] 关节 1 的当前位置反馈
  float j2_pos = 0.0f;   // [解析] 关节 2 的当前位置反馈
  float j0_vel = 0.0f;   // [解析] 关节 0 的当前速度反馈（单位/时间基准由双方约定）
  float j1_vel = 0.0f;   // [解析] 关节 1 的当前速度反馈
  float j2_vel = 0.0f;   // [解析] 关节 2 的当前速度反馈
  uint8_t reserved[2];  //保留位 2位
  // [解析] ↑ 注意原注释写的是"2位"，但代码里其实是 2 **字节**（uint8_t[2]）。
  // [解析]   这是原注释的措辞不严谨，不是代码问题；长度 28+2=30 才是真正要保证的。
};

// [解析] 编译期断言（static_assert）：如果 RefereeCustomData 的大小和裁判系统协议里的
// [解析] CustomRobotData（30 字节）不一致，编译器会直接报错，而不是等到上车跑飞了才发现。
// [解析] 这类"保险丝"非常值得学：协议一改，编译阶段就把问题暴露出来，代价最低。
static_assert(sizeof(RefereeCustomData) == sizeof(referee::CustomRobotData));

// [解析] ★本文件最难的★ 遥控器私有帧（以 0xA9 0x53 开头）的原始布局。
// [解析] 它把"5 个通道 + 2 个开关 + 2 个自定义键 + 拨轮 + 扳机"全部压进一个 64 位整数里，
// [解析] 后面紧跟着鼠标三个轴、鼠标三个键、键盘掩码和 CRC16。
// [解析] 位域语法 `uint64_t ch_0 : 11;` 的意思是"这个成员只占 11 个 bit"，
// [解析] 编译器会把同一个 uint64_t 里声明的所有位域**从低位往高位依次紧密排列**，
// [解析] 所以 ch_0 在 bit0~10、ch_1 在 bit11~21 …… 这不是"多个变量"，而是"一个 64 位数被切成了好几段"。
// [解析] ⚠ 位域的具体排布规则是由编译器实现的（GCC/ARM 上从低位开始），跨编译器/跨平台不保证一致，
// [解析]   好在本项目固定用 arm-none-eabi-gcc，收发两端也都是同一套约定，才能这样写。
// [解析] ⚠ 位域的"打包"行为对 packed 并不敏感 —— 位域本身就不会被填充，但联合使用时仍需小心。
struct __attribute__((packed)) VT03RemoteData
{
  uint8_t sof_1;  // [解析] 帧头第 1 字节，固定 0xA9
  uint8_t sof_2;  // [解析] 帧头第 2 字节，固定 0x53（两个字节合起来构成"遥控器私有帧"的标志）
  uint64_t ch_0 : 11;  // [解析] 通道 0（右水平摇杆）原始值，范围 0~2047，中位约 1024
  uint64_t ch_1 : 11;  // [解析] 通道 1（右垂直摇杆）原始值，范围 0~2047
  uint64_t ch_2 : 11;  // [解析] 通道 2（左垂直摇杆）原始值，范围 0~2047
  uint64_t ch_3 : 11;  // [解析] 通道 3（左水平摇杆）原始值，范围 0~2047
  uint64_t mode_sw : 2;  // [解析] 档位开关，2 bit，取值 0/1/2 -> C/N/S 三个档（见 update_remote）
  uint64_t pause : 1;    // [解析] 暂停键，1 bit，1=按下
  uint64_t fn_1 : 1;     // [解析] 左侧自定义按键（拨杆），1 bit
  uint64_t fn_2 : 1;     // [解析] 右侧自定义按键，1 bit
  uint64_t wheel : 11;   // [解析] 左上方拨轮，原始值 0~2047，中位约 1024
  uint64_t trigger : 1;  // [解析] 扳机键，1 bit，1=扣下
  uint64_t padding1 : 3;  // [解析] 填充 3 bit，把上面这些凑成正好 64 bit（11*5+2+1+1+1+1+3 = 64）
  // [解析] ↑ 算一下：ch_0..ch_3 共 44 bit，mode_sw 2，pause/fn_1/fn_2 各 1，wheel 11，trigger 1，
  // [解析]   44+2+3+11+1 = 61，再补 padding1 的 3 bit 正好 64。对齐之后鼠标字段才是字节边界，
  // [解析]   否则 mouse_x 会读错位置 —— 这就是"为什么必须补 3 个空 bit"的原因。

  int16_t mouse_x;  // [解析] 鼠标横向位移增量，有符号 16 位，范围 -32768~32767
  int16_t mouse_y;  // [解析] 鼠标纵向位移增量，有符号 16 位
  int16_t mouse_z;  // [解析] 鼠标滚轮增量，有符号 16 位

  uint8_t mouse_left : 2;    // [解析] 鼠标左键状态，占 2 bit（协议这么做是为了留出"长按/双击"等扩展值）
  uint8_t mouse_right : 2;   // [解析] 鼠标右键状态，占 2 bit
  uint8_t mouse_middle : 2;  // [解析] 鼠标中键状态，占 2 bit
  uint8_t padding2 : 2;      // [解析] 填充 2 bit，把 4 个字段凑满 1 字节（2+2+2+2=8）
  // [解析] ↑ 取"是否按下"时直接把它当布尔用（见 update_remote 里的 mouse.left = data->mouse_left;），
  // [解析]   所以只要不是 0 就当作按下；如果将来协议用到 2/3 这些值，这个写法就需要改。

  uint16_t keys;   // [解析] 键盘 16 位掩码，每一位对应一个按键，配合上面的 VT03_KEY_xxx_MASK 使用
  uint16_t crc16;  // [解析] CRC16 校验值，覆盖本结构体除它自己以外的所有字节（校验方式见 check_crc16）
  // [解析] 结构体总大小 = 2(帧头) + 8(那个 64 位) + 6(三个 int16 鼠标轴) + 1(鼠标键) + 2 + 2 = 21 字节。
  // [解析]   在 vt03.cpp 里 update() 用 sizeof(VT03RemoteData) 当"这一帧多长"，
  // [解析]   所以这个布局任何一个成员改动都会改变帧长，必须和遥控器固件同步修改。
};

// [解析] 档位开关的三个档，用 enum class（强类型枚举）表示，避免和整数混用。
// [解析] C / N / S 分别对应遥控器拨杆的三个位置，含义由队伍约定（常见是 C=云台 N=中立 S=底盘）。
// [解析] 用 enum class 的好处：不能隐式转成 int，写 if (mode == 1) 会直接编译报错，逼你用 VT03Mode::N。
enum class VT03Mode
{
  C,  // [解析] 档位 C（mode_sw == 0）
  N,  // [解析] 档位 N（mode_sw == 1）
  S   // [解析] 档位 S（mode_sw == 2）
};

// [解析] 给上层用的鼠标数据（归一化后的版本，不是协议原始字节）。
// [解析] 为什么不直接用 VT03RemoteData 里的 int16_t？因为那些是"增量/脉冲"，
// [解析]   和具体鼠标 DPI、上报率强相关；归一化成 -1~1 之后，控制逻辑就不用关心硬件细节了。
// [解析] 换算公式见 vt03.cpp：vx = mouse_x / 32768.0f（int16 满量程）。
struct VT03MouseData
{
  float vx;  // 取值范围: [-1, 1]
  // [解析] ↑ 归一化横向速度，右正左负（符号由传感器方向决定）。
  float vy;  // 取值范围: [-1, 1]
  // [解析] ↑ 归一化纵向速度。
  float vs;  // 取值范围: [-1, 1], 鼠标滚轮, s代表scroll
  // [解析] ↑ 滚轮量，s = scroll。
  bool left;  // [解析] 左键是否按下（由 2 bit 的原始值转换而来）
  bool middle;  //中键
  // [解析] ↑ 中键（按下滚轮）是否按下。
  bool right;  // [解析] 右键是否按下
};

// [解析] 给上层用的键盘数据：把 16 位掩码"翻译"成 16 个 bool，用起来更直观
// [解析]   （想判断 W 键，直接写 keys.w，不用记 VT03_KEY_W_MASK 是多少）。
// [解析] 这里加了 packed，其实 16 个 bool 本来就紧凑（每个 1 字节、共 16 字节），
// [解析]   加不加 packed 结果一样，属于作者的防御性写法。
// [解析] ⚠ 与 VT03MouseData 不同，这个结构体没有"默认初始化"，所以构造后是未定义值？
// [解析]   不会 —— VT03 的构造函数里写了 keys()，值初始化会把 16 个 bool 全置 false。
struct __attribute__((packed)) VT03KeysData
{
  bool w;      // [解析] 对应掩码 bit0
  bool s;      // [解析] bit1
  bool a;      // [解析] bit2
  bool d;      // [解析] bit3
  bool shift;  // [解析] bit4
  bool ctrl;   // [解析] bit5
  bool q;      // [解析] bit6
  bool e;      // [解析] bit7
  bool r;      // [解析] bit8
  bool f;      // [解析] bit9
  bool g;      // [解析] bit10
  bool z;      // [解析] bit11
  bool x;      // [解析] bit12
  bool c;      // [解析] bit13
  bool v;      // [解析] bit14
  bool b;      // [解析] bit15
};

// [解析] 机器人发给自定义客户端（上位机图形界面）的数据块，命令码 0x0310。
// [解析] 协议规定这一帧的数据段最大 300 字节，所以这里就用一个 300 字节的数组占位。
// [解析] 为什么单独定义一个"只有 padding"的结构体？因为 0x0310 的内容是"自定义的"，
// [解析]   中间件不关心里面装什么，只需要一个固定 300 字节的"信封"来承载任意数据。
// [解析] 用法：发送前把 300 字节塞进自定义数据结构里，再强转成 CustomByteBlock 传进去。
// [解析] ⚠ 名字叫 padding 容易误会：它并不是"无用的填充"，而是真正承载用户数据的那 300 字节。
struct __attribute__((packed)) CustomByteBlock
{
  uint8_t padding[300];  // [解析] 300 字节用户数据区，内容格式完全由使用者自定义
};

// [解析] 又一处编译期保险丝：确保 CustomByteBlock 就是 300 字节。
// [解析]   第二个参数是"断言失败时的提示文字"，编译报错时会一起打印出来，方便定位。
// [解析]   为什么必须是 300？因为裁判系统 0x0310 的数据段上限就是 300 字节，
// [解析]   发超过 300 字节的包协议不接受，发少了又对不齐，所以只能正好 300。
static_assert(sizeof(CustomByteBlock) == 300, "CustomByteBlock MUST be exactly 300 bytes!");

// [解析] 从自定义客户端（0x0311）收到的数据 + 它的元信息。
// [解析] data：最多 30 字节的原始载荷（协议规定 0x0311 最大 30 字节）。
// [解析]   `{}` = 值初始化，保证 30 个字节全为 0，不会残留上一帧的数据。
// [解析] size：本帧实际收到了多少字节（0~30），读 data 时按 size 截取，不要傻读 30 个。
// [解析] is_valid：本帧是否解析成功（长度不超限才算有效），必须在读 data 前先判断它，
// [解析]   否则可能读到上一帧残留或全 0 的无效数据。
struct VT03CustomClientData
{
  std::array<uint8_t, 30> data{};  // [解析] 实际载荷，std::array 比裸数组安全（自带 size()、边界信息）
  uint8_t size = 0;                // [解析] 本帧有效字节数，范围 0~30；单位：字节
  bool is_valid = false;           // [解析] 本帧是否有效；false 时 data 里的内容不可信
};

// [解析] VT03 类：一个对象管理一路串口上的 VT03 图传链路。
// [解析] 它比 VT02 复杂，原因是"一个串口上混跑两种协议"：
// [解析]   ① 遥控器私有帧：以 0xA9 0x53 开头，装摇杆/鼠标/键盘；
// [解析]   ② 裁判系统标准帧：以 0xA5 开头，装 0x0302 / 0x0309 / 0x0311 等业务数据。
// [解析] update() 会先试 ①，不是再试 ②，并且**递归**解析缓冲区里粘连的多帧。
// [解析] 典型用法：
// [解析]   1) sp::VT03 vt03(&huart3);
// [解析]   2) vt03.request();
// [解析]   3) 串口空闲回调里 vt03.update(Size, HAL_GetTick());
// [解析]   4) 主循环读 vt03.ch_rh / vt03.keys.w / vt03.is_alive(...)
class VT03
{
public:
  // [解析] 构造函数。huart：串口句柄（不可为 nullptr）；use_dma：true 用 DMA 收，false 用中断收。
  // [解析] 它会把所有成员初始化到"安全的初始值"（摇杆 0、按键 false、缓冲清零），
  // [解析]   这样在还没收到任何数据前读这些变量也不会控制乱飞。
  VT03(UART_HandleTypeDef * huart, bool use_dma = true);
  // [解析] 串口句柄直接公开成 public 成员（名字没有下划线）。
  // [解析]   对比：vt02.hpp 里是私有的 huart_，风格不统一；这里公开是为了外部也能拿它做串口操作。
  UART_HandleTypeDef * huart;

  // [解析] 下面这一段是"解析结果"，全部标记为只读：只应由 update() 写、由你的控制代码读。
  // [解析] 之所以用 public 成员而不是 getter 函数，是为了让上层代码写起来短（vt03.ch_rh）。
  VT03Mode mode;  // 只读! 档位切换开关
  // [解析] ↑ 三档开关的当前档位，取值 VT03Mode::C / N / S。
  float ch_rh;    // 只读! 右水平摇杆, 取值范围: [-1, 1], 右正左负
  // [解析] ↑ 归一化值而不是原始 0~2047：公式 (原始值 - 1024) / 660.0f，详见 vt03.cpp。
  float ch_rv;    // 只读! 右垂直摇杆, 取值范围: [-1, 1], 上正下负
  float ch_lh;    // 只读! 左水平摇杆, 取值范围: [-1, 1], 右正左负
  float ch_lv;    // 只读! 左垂直摇杆, 取值范围: [-1, 1], 上正下负
  float wheel;    // 只读! 左上方拨轮, 取值范围: [-1, 1], 右正左负
  bool fn_l;      // 只读! 左自定义按键
  bool fn_r;      // 只读! 右自定义按键
  bool pause;     // 只读! 暂停按键
  bool trigger;   // 只读! 扳机按键

  RefereeCustomData custom;            // 只读! 自定义控制器数据
  // [解析] ↑ 来自裁判系统 0x0302 帧，需要 custom_2_robot_is_open() 为 true 才有意义。
  RefereeRobotData robot;              // 只读！机器人向自定义控制器发送数据
  // [解析] ↑ 来自 0x0309 帧（这个方向其实是"自定义控制器接收机器人数据"，见协议注释）。
  VT03MouseData mouse;                 // 只读! 鼠标数据
  VT03KeysData keys;                   // 只读! 键盘数据
  VT03CustomClientData custom_client;  // 只读! 自定义客户端向机器人发送的数据 0x0311

  uint16_t keyboard_value;  // 只读! 键盘16位原始值
  // [解析] ↑ 未拆位的原始掩码。想自己判断某个键，可以写 (keyboard_value & 0x0001)。
  // [解析]   保留它的意义：万一协议新增了键，上面 keys 结构体里没有对应成员，还能用原始值兜底。

  // [解析] 是否收到过合法的遥控器私有帧（0xA9 0x53）。true 只代表"曾经收到过"。
  bool is_open() const;
  // [解析] 遥控器是否在线：曾经收到过 && 距今不到 100ms。
  // [解析] now_ms 传 HAL_GetTick()。100ms 是个很紧的阈值 —— 遥控器帧率约 100Hz 以上，
  // [解析]   超过 100ms 没数据基本就是掉线/失联了（对比 VT02 用了宽松的 2000ms）。
  bool is_alive(uint32_t now_ms) const;
  // [解析] 是否收到过合法的 0x0302（自定义控制器数据）。判断图传业务数据通道是否通。
  bool custom_2_robot_is_open() const;
  // [解析] 0x0302 通道是否还在刷新：曾经收到过 && 距今不到 100ms。
  // [解析] ⚠ 注意函数名少了一个下划线（custom_2_robotis_alive 而不是 ..._is_alive），
  // [解析]   这是原代码的拼写，调用时别写错；改名会破坏兼容性，所以保留原样。
  bool custom_2_robotis_alive(uint32_t now_ms) const;

  // [解析] 挂载接收：启动一次 ReceiveToIdle（DMA 或中断），并关掉 DMA 半传输中断。
  // [解析] 每处理完一轮数据后都要再调用一次，否则串口不会再收（HAL 的特性）。
  void request();
  // [解析] 解析入口（在串口空闲回调里调用）。
  // [解析] size：本次实际收到的字节数（HAL_UARTEx_RxEventCallback 的 Size 参数）；
  // [解析] stamp_ms：当前毫秒时间戳（HAL_GetTick()），用来刷新 is_alive 的时间基准。
  // [解析] 它只是把内部 buff_ 转发给私有重载版本，真正干活的是那个 (frame_start, size, stamp) 版本。
  void update(uint16_t size, uint32_t stamp_ms);

  // 发送自定义客户端数据 (0x0310)
  // [解析] 把一块 300 字节的数据按裁判系统帧格式（帧头 + CRC8 + 命令码 + 数据 + CRC16）发出去。
  // [解析] 参数 custom_data：恰好 300 字节的载荷，内容随便定义，协议不管。
  // [解析] 副作用：每次调用会把内部序列号 seq_ 加 1 并写进帧头；同时会占用串口发送（DMA/中断）。
  void send_custom_client_data(const CustomByteBlock & custom_data);

private:
  // [解析] const 成员：构造后不能改，所以只能用初始化列表赋值（构造函数里正是这么做的）。
  const bool use_dma_;
  // [解析] 单帧接收缓冲，255 字节。为什么是 255？因为裁判系统单帧最大就是 300 字节，
  // [解析]   但 0x0310 那类大帧是由本机**发送**的；这里接收用的 0x0311 最多 30 字节，
  // [解析]   255 已经远远够用（同时也和 DMA 的 NDTR 配置配套）。
  std::array<uint8_t, 255> buff_;
  // [解析] "双缓冲"，两个 255 字节的数组。定义在这里但目前代码里没有用到
  // [解析]   （request() 用的是 buff_），看起来是为"交替接收/乒乓缓冲"预留的。
  // [解析] ⚠ 它会让 VT03 对象占用约 765 字节 RAM，比较占内存；如果确认没用可以删，但这里不动。
  std::array<std::array<uint8_t, 255>, 2> multi_buff_;

  // [解析] 接收缓冲大小常量。注意它写死 255，和 buff_ 的实际大小一致，
  // [解析]   改其中一个忘了改另一个会导致越界，属于需要留意的"重复定义"。
  static constexpr uint32_t RX_BUF_SIZE = 255;
  // [解析] DMA 的 NDTR（剩余传输量）初值 = 255*2 = 510。
  // [解析] 但代码里没有用到这个常量（multi_buff_ 也没用），属于预留/历史遗留。
  static constexpr uint32_t DMA_NDTR_SIZE = RX_BUF_SIZE * 2;

  // [解析] 是否收到过合法遥控器帧，is_open() 直接返回它。
  bool has_read_ = false;
  // [解析] 最近一次成功解析遥控器帧的时间戳（ms），is_alive() 用它算超时。
  // [解析] 没有给初值，但构造函数里显式初始化成了 0，所以是安全的。
  uint32_t last_read_ms_;
  // [解析] 是否收到过合法 0x0302 帧，custom_2_robot_is_open() 返回它。
  bool custom_2_robot_has_read_ = false;
  // [解析] 最近一次收到合法 0x0302 帧的时间戳（ms）。
  uint32_t custom_2_robot_last_read_ms_;

  uint8_t seq_ = 0;  // 发送序列号，随每次发送递增
  // [解析] ↑ 裁判系统帧头里的"包序号"字段。取值 0~255，发一次加 1，溢出后自然回绕到 0。
  // [解析]   它的作用：让接收方（上位机/裁判系统）能发现丢包或重复包，属于轻量级的可靠性手段。

  // [解析] 私有重载：真正的解析函数。参数 frame_start 指向"这一帧的起始字节"，
  // [解析]   之所以要这么设计，是为了在解析完一帧后能用 frame_start + frame_len 继续解析下一帧
  // [解析]   （递归调用，处理一个缓冲里同时到达的多帧）。
  void update(uint8_t * frame_start, uint16_t size, uint32_t stamp_ms);
  // [解析] 把遥控器帧（VT03RemoteData）里的位域/原始值，换算成上面那些归一化成员。
  // [解析] 参数 data 必须已经通过 CRC16 校验，否则换算出来的是垃圾。
  void update_remote(const VT03RemoteData * data);
};

}  // namespace sp

#endif  // SP__VT03_HPP