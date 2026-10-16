// [解析] ==========================================================================
// [解析] 【文件】sp_middleware/referee/ui/manager.hpp
// [解析] 【作用】声明 UI_Manager（UI 管理器）。它负责把 element.hpp 里定义的图形
// [解析]         "打包"成一帧符合裁判系统协议的 0x0301 机器人交互数据，
// [解析]         并算好帧头的 CRC8 和帧尾的 CRC16，上层直接整帧发出去即可。
// [解析] 【被谁用】applications/uart_task.cpp（参考同目录 readme.md）：
// [解析]           sp::UI_Manager ui_manager;                       // 全局定义一个
// [解析]           ui_manager.set_sender_id(...);                   // 声明"我是谁"
// [解析]           ui_manager.pack(&line, &rect);                   // 打包要画的图形
// [解析]           pm02.send(ui_manager.data(), ui_manager.size()); // 串口发出去
// [解析] 【关键点】1) frame_ 是一个 __attribute__((packed)) 的匿名结构体，成员顺序就是
// [解析]              裁判系统帧的字节顺序，所以能把它当成一段裸字节直接发送；
// [解析]           2) 最烧脑的一点：帧尾的 CRC16 并不是一个独立成员，而是被写进了
// [解析]              data.user_data[] 数组内部（原因见成员定义处的注释）；
// [解析]           3) pack() 有 5 个重载，参数个数决定调用哪个（1/2/5/7 个图形或 1 个字符），
// [解析]              少传的参数会用"空图形"占位，帧长不会缩短。
// [解析] ==========================================================================

#ifndef SP__UI_MANAGER_HPP
#define SP__UI_MANAGER_HPP

// [解析] ↓ 要用到 ui::Element / ui::String / ui::ComposableElement / ui::Layer 这些类型。
#include "element.hpp"
// [解析] ↓ 要用到裁判系统协议结构体：FrameHeader、RobotInteractionData、
// [解析]   InteractionFigure，以及协议常量 HEAD_LEN / CMD_ID_LEN / TAIL_LEN 等。
#include "referee/referee_protocol/referee_protocol.hpp"

namespace sp
{
// [解析] UI_Manager：整个 UI 功能的"发送端大脑"。
// [解析] 一个对象内部只有一块可复用的发送缓冲区 frame_（约 125 字节），
// [解析] 每次 pack() 都往这块缓冲区里重新填内容并更新校验，不会动态申请内存，
// [解析] 所以在 FreeRTOS 任务里反复调用也不会造成内存碎片。
// [解析] 典型使用流程（顺序不能反）：
// [解析]   1) 构造对象后先 set_sender_id(robot_id)，告诉裁判系统"我是哪台车"；
// [解析]   2) 需要清屏时先 delete_all() 或 delete_layer()；
// [解析]   3) 每个发送周期调一次 pack(...) 把这一帧要画的图形打包好；
// [解析]   4) data() + size() 交给串口/CAN 发出去（通常 30~40 ms 发一次）。
class UI_Manager
{
public:
  // [解析] 构造函数：只初始化帧头和空图形，不碰硬件。
  // [解析] 注意：它不会设置 data_len，所以在第一次 pack()/delete_xxx() 之前
  // [解析]   调用 size()/data() 是不安全的（帧长还是未初始化的值）。
  UI_Manager();

  // [解析] size()：返回"这一帧实际要发的字节数"。
  // [解析] 返回值 = 5(帧头) + 2(命令ID) + data_len + 2(帧尾CRC16)，
  // [解析] 不是 sizeof(frame_)！因为 frame_ 里的 user_data 有 112 字节，
  // [解析] 但通常只用到前几十字节，多发的部分裁判系统不认。
  // [解析] 调用时机：必须在 pack() 或 delete_xxx() 之后，否则读到的帧长是错的。
  size_t size() const;
  // [解析] data()：返回指向帧缓冲区首字节的指针（就是把结构体"当成字节数组"看）。
  // [解析] 配合 size() 使用：send(data(), size())。
  // [解析] 注意：指针指向的是成员变量 frame_，下一个人调用 pack() 就会把它改写，
  // [解析]   所以不要把这个指针存起来长期使用，发完就作废。
  const uint8_t * data() const;

  // [解析] set_sender_id：设置"发送者 ID"，也就是告诉裁判系统这台车是谁。
  // [解析] 参数 robot_id 用 referee::robot_id 里的常量，例如
  // [解析]   referee::robot_id::RED_HERO（红方英雄，值 1）；
  // [解析]   用南航模拟器调试时可以随便填一个，用官方裁判系统时建议在循环里
  // [解析]   用 pm02.robot_status.robot_id 动态设置。
  // [解析] 副作用：同时会把 receiver_id（接收者）设成该机器人对应的选手端 Client ID，
  // [解析]   映射关系见 manager.cpp 的 get_client_id()。
  void set_sender_id(uint8_t robot_id);

  // [解析] delete_all：请求选手端"清空所有图层上的所有图形"。
  // [解析] 调用后同样要用 send(data(), size()) 发出去才生效。
  // [解析] 常用时机：任务启动时先清一次屏，避免上次运行残留的图形留在画面上。
  // [解析] 注意：它清的是"选手端画面"，不会重置 element.cpp 里那个自增的图形编号。
  void delete_all();
  // [解析] delete_layer：只删除指定图层上的所有图形（其他图层不受影响）。
  // [解析] 参数 layer 取值 ui::Layer::LAYER_0 ~ LAYER_9。
  // [解析] 典型套路：把静态背景放第 0 层，把每帧刷新的数据放第 1 层，
  // [解析]   需要重画时只 delete_layer(LAYER_1)，省时又省带宽。
  void delete_layer(ui::Layer layer);

  // [解析] 下面 5 个 pack() 是重载函数：参数个数不同，打包出的帧长/命令也不同。
  // [解析] 它们都会做三件事：填帧头 data_len、填子命令号、把图形数据拷进 user_data，
  // [解析] 最后统一调 apply_crc() 算校验并让帧序号 seq 自增。
  // [解析] 注意：pack() 只读图形对象里的 data 成员，不会修改你传进来的对象。

  // [解析] 打包 1 个字符图形（对应协议子命令 0x0110 EXT_CLIENT_CUSTOM_CHARACTER）。
  // [解析] 为什么字符图形要单独一个函数？因为它的长度是 15(图形) + 30(字符数据) 字节，
  // [解析] 和普通图形（15 字节）不一样，而且协议规定一帧只能带一个字符图形。
  // [解析] 参数 str：字符图形对象指针，内部字符串超过 30 字节的部分会被截断。
  void pack(const ui::String * str);
  // [解析] 打包 1 个普通图形（子命令 0x0101 INTERACTION_FIGURE），帧长 6+15 = 21 字节。
  void pack(const ui::ComposableElement * e1);
  // [解析] 打包 2 个普通图形（子命令 0x0102），一帧塞两个可以省一半的发送次数。
  void pack(const ui::ComposableElement * e1, const ui::ComposableElement * e2);
  // [解析] 打包 5 个普通图形（子命令 0x0103）。
  // [解析] 注意：e4 / e5 有默认值 nullptr，理论上 pack(a,b,c) 也能编译，
  // [解析]   但那样最后两个位置会被"空图形"占着，帧长仍然是 5 个图形的长度，
  // [解析]   等于白白多发 30 字节。所以用这个重载时建议把 5 个都填满。
  void pack(
    const ui::ComposableElement * e1, const ui::ComposableElement * e2,
    const ui::ComposableElement * e3, const ui::ComposableElement * e4 = nullptr,
    const ui::ComposableElement * e5 = nullptr);
  // [解析] 打包 7 个普通图形（子命令 0x0104），这是协议一帧能带的图形上限。
  // [解析] 帧长 = 6 + 15*7 = 111 字节，是 UI 里最长的一帧（串口波特率不够时会明显变慢）。
  // [解析] 同样只有 e7 有默认值，所以最少要传 6 个图形。
  void pack(
    const ui::ComposableElement * e1, const ui::ComposableElement * e2,
    const ui::ComposableElement * e3, const ui::ComposableElement * e4,
    const ui::ComposableElement * e5, const ui::ComposableElement * e6,
    const ui::ComposableElement * e7 = nullptr);

private:
  // [解析] frame_ 就是"发送缓冲区"，它的字节排布完全照着裁判系统协议来：
  // [解析]   head      : 5 字节帧头（sof=0xA5、data_len、seq、crc8）
  // [解析]   cmd_id    : 2 字节命令 ID，固定 0x0301（机器人交互数据）
  // [解析]   data      : 118 字节交互数据（data_cmd_id + sender_id + receiver_id + user_data[112]）
  // [解析] __attribute__((packed)) 的作用：禁止编译器做内存对齐填充。
  // [解析]   不加它的话，编译器可能在成员之间插入空字节，发出去的字节位置就全错了。
  // [解析]   代价是成员地址可能不对齐，访问时 CPU 需要多花几个周期（甚至不支持非对齐访问），
  // [解析]   但在 STM32F407（Cortex-M4）上单个字节/半字访问是可以容忍的。
  // [解析] 注意：这是一个"匿名结构体 + 变量名 frame_"的写法（GCC 扩展），
  // [解析]   为了能直接写 frame_.head / frame_.data，同时又不用另起一个类型名。
  struct __attribute__((packed))
  {
    referee::FrameHeader head;
    uint16_t cmd_id;
    referee::RobotInteractionData data;
    // 绘制UI所需的user_data最大长度为105, 而其定义长度为112, 有剩余, 因此帧尾的crc16在data内部
    // [解析] ↑ 上面是原作者的关键说明，原样保留。展开解释一下为什么可行：
    // [解析]   1) 帧长由帧头的 data_len 决定，data_len = 6(交互头) + 图形数据长度；
    // [解析]   2) 7 个普通图形最长：6 + 15*7 = 111，其中图形数据占 105 字节，
    // [解析]      写进 user_data[0] ~ user_data[104]；
    // [解析]   3) 帧尾还要 2 字节 CRC16，正好接着写在 user_data[105]、user_data[106]，
    // [解析]      而 user_data 有 112 字节（下标 0~111），所以放得下，不会越界；
    // [解析]   4) 也就是说 CRC16 借用（复用了）user_data 数组末尾没用到的那几个字节，
    // [解析]      这样整个 frame_ 就是一段"连续的、可以直接发送的字节"，
    // [解析]      不需要再额外拼一次缓冲区。这是本文件最巧妙也最容易看懵的地方。
  } frame_;

  // [解析] empty_：一个"空图形"模板，除了 operate_type = EMPTY 之外全是 0。
  // [解析] 用途：pack(e1, nullptr, e3) 这种参数缺省时，copy() 会拿它来占位，
  // [解析]   保证每个图形槽位都有 15 字节数据，帧长和图形个数严格对应。
  referee::InteractionFigure empty_;

  // [解析] copy()：把第 i 个图形数据拷进 frame_.data.user_data 的第 i 个槽位。
  // [解析] 参数 i 从 0 开始，槽位大小固定 15 字节（FIG_LEN）；
  // [解析] 如果传入的指针是 nullptr，就自动改用上面的 empty_。
  void copy(const ui::Element * e, size_t i);
  // [解析] apply_crc()：填充校验（本文件的核心）。
  // [解析] 它做三件事：1) 帧序号 seq 自增；2) 算帧头 CRC8；3) 算整帧 CRC16 写到帧尾。
  // [解析] 所有 pack() 和 delete_xxx() 的最后一行都会调用它，忘记调用就会发出去一帧废数据。
  void apply_crc();
};

}  // namespace sp

#endif  // SP__UI_MANAGER_HPP