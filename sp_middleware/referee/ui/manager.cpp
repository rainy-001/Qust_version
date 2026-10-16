// [解析] ==========================================================================
// [解析] 【文件】sp_middleware/referee/ui/manager.cpp
// [解析] 【作用】实现 manager.hpp 里声明的 UI_Manager：
// [解析]         把图形元素打包成裁判系统 0x0301 机器人交互数据帧（含 CRC8/CRC16），
// [解析]         并且支持"删除全部/删除某一层"两条指令。
// [解析] 【看代码的重点】
// [解析]   1) 帧长 data_len 到底怎么算出来的（每个 pack 重载都不一样）；
// [解析]   2) apply_crc() 里 CRC16 的位置为什么要用 data_len 去算指针；
// [解析]   3) 为什么 CRC16 写在 user_data[] 数组内部（见 manager.hpp 的说明）。
// [解析] 【被谁用】applications/uart_task.cpp 里创建一个全局 UI_Manager，
// [解析]         在任务循环中 pack() 之后用 data()/size() 送进串口。
// [解析] ==========================================================================

#include "manager.hpp"

// [解析] ↓ 用 CRC 工具算校验：get_crc8() 算帧头校验，get_crc16() 算整帧校验。
// [解析]   注意裁判系统用的是"CRC8 校验前 4 字节 + CRC16 校验整帧"的双校验结构。
#include "tools/crc/crc.hpp"

namespace sp
{
// [解析] DATA_HEAD_LEN：交互数据里"图形数据之前"的那一段头部长度，等于 6 字节。
// [解析] 算法：RobotInteractionData 总长 118，减去 user_data[112]，剩下 6，
// [解析]   也就是 data_cmd_id(2) + sender_id(2) + receiver_id(2)。
// [解析] 小知识：sizeof(类型::成员名) 是 C++11 允许的写法（对非静态成员求大小），
// [解析]   编译器在编译期就能算出 112，没有任何运行时开销。
// [解析] 以后 data_len 的公式都是"DATA_HEAD_LEN + 这一帧要带的图形数据长度"。
constexpr size_t DATA_HEAD_LEN =
  sizeof(referee::RobotInteractionData) - sizeof(referee::RobotInteractionData::user_data);
// [解析] FIG_LEN：一个图形描述占 15 字节（referee_protocol.hpp 里用
// [解析]   static_assert(sizeof(InteractionFigure) == 15) 兜了底，改协议会立刻编译报错）。
// [解析] 帧长里的图形部分就是 FIG_LEN * 图形个数。
constexpr size_t FIG_LEN = sizeof(referee::InteractionFigure);
// [解析] MAX_STR_LEN：字符图形里"字符数据"的最大长度，30 字节。
// [解析] 用它来截断过长的字符串，保证不越界（详见 pack(const ui::String *)）。
constexpr size_t MAX_STR_LEN = sizeof(referee::ExtClientCustomCharacter::data);

// [解析] get_client_id：把"机器人 ID"翻译成"对应选手端的 Client ID"。
// [解析] 为什么需要翻译？因为裁判系统里这是两套编号：
// [解析]   机器人 ID：红方 1~11、蓝方 101~111（见 referee::robot_id）
// [解析]   Client ID：红方 0x0101~0x0106、蓝方 0x0165~0x016A（见 referee::client_id）
// [解析] UI 帧里的 receiver_id 必须填 Client ID，选手端才会处理这一帧。
// [解析] 参数 robot_id：referee::robot_id 里的常量（1 = 红方英雄，101 = 蓝方英雄 …）。
// [解析] 返回值：对应的 Client ID；没有匹配的机器人时返回 0x8080（裁判系统服务器）。
// [解析] 注意：哨兵(7/107)、飞镖(8/108)、雷达(9/109)、基地、前哨站都不在表里，
// [解析]   这些 ID 调用本函数会落到 default 分支拿到 0x8080。
// [解析]   将来的比赛规则若允许哨兵画 UI，需要在这里补 case。
uint16_t get_client_id(uint8_t robot_id)
{
  switch (robot_id) {
    case referee::robot_id::RED_HERO:  // [解析] 红方英雄：robot_id = 1
      return referee::client_id::RED_HERO_CLIENT;  // [解析] -> 0x0101
    case referee::robot_id::RED_ENGINEER:
      return referee::client_id::RED_ENGINEER_CLIENT;
    case referee::robot_id::RED_STANDARD_3:
      return referee::client_id::RED_STANDARD_3_CLIENT;
    case referee::robot_id::RED_STANDARD_4:
      return referee::client_id::RED_STANDARD_4_CLIENT;
    case referee::robot_id::RED_STANDARD_5:
      return referee::client_id::RED_STANDARD_5_CLIENT;
    case referee::robot_id::RED_AERIAL:
      return referee::client_id::RED_AERIAL_CLIENT;
    // [解析] 以上是红方 6 种机器人；下面是蓝方，规律完全相同，
    // [解析]   只是 robot_id 是 101~106、Client ID 是 0x0165~0x016A。
    case referee::robot_id::BLUE_HERO:
      return referee::client_id::BLUE_HERO_CLIENT;
    case referee::robot_id::BLUE_ENGINEER:
      return referee::client_id::BLUE_ENGINEER_CLIENT;
    case referee::robot_id::BLUE_STANDARD_3:
      return referee::client_id::BLUE_STANDARD_3_CLIENT;
    case referee::robot_id::BLUE_STANDARD_4:
      return referee::client_id::BLUE_STANDARD_4_CLIENT;
    case referee::robot_id::BLUE_STANDARD_5:
      return referee::client_id::BLUE_STANDARD_5_CLIENT;
    case referee::robot_id::BLUE_AERIAL:
      return referee::client_id::BLUE_AERIAL_CLIENT;
    default:
      // [解析] 兜底分支：ID 不在上面的表里（哨兵/飞镖/雷达/基地等，或者填错了），
      // [解析] 就发给裁判系统服务器 0x8080，不会崩，但 UI 也不会显示。
      // [解析] 调试时如果画面一直没反应，可以先检查是不是落到了这里。
      return referee::client_id::REFEREE_SERVER;
  }
}

// [解析] 构造函数：把"每一帧都固定不变"的字段先填好，不做任何硬件操作。
// [解析]   frame_.head.sof = referee::SOF：SOF = Start Of Frame（帧起始标志），
// [解析]     裁判系统协议规定固定为 0xA5，接收方靠它找到一帧的开头。
// [解析]   frame_.head.seq = 0：帧序号从 0 开始，之后每打包一次自增（见 apply_crc）。
// [解析]   frame_.cmd_id = 0x0301（ROBOT_INTERACTION_DATA）：UI 绘制就走这个命令，
// [解析]     所有图形、删除图层都通过它下发，具体画什么由子命令 data_cmd_id 决定。
// [解析]   empty_.operate_type = EMPTY(0)：把"占位图形"标记成空操作。
// [解析] 注意：这里没有初始化 head.data_len，也没有完全初始化 empty_ 的其余位域！
// [解析]   如果对象定义在函数栈上，这些位域就是随机值，所以：
// [解析]   1) 第一次 pack()/delete_xxx() 之前不要调 size()/data()；
// [解析]   2) 建议像 readme.md 那样把 UI_Manager 定义成全局变量（全局对象在程序
// [解析]      启动时会先被自动清零，未显式赋值的字段都是 0，行为才稳定）。
UI_Manager::UI_Manager()
{
  frame_.head.sof = referee::SOF;
  frame_.head.seq = 0;
  frame_.cmd_id = referee::cmd_id::ROBOT_INTERACTION_DATA;
  empty_.operate_type = static_cast<uint8_t>(ui::OperateType::EMPTY);
}

// [解析] size()：返回这一帧"真正需要发送"的字节数，交给串口的第二个参数。
// [解析] 公式 = HEAD_LEN(5) + CMD_ID_LEN(2) + data_len + TAIL_LEN(2)。
// [解析] 为什么不用 sizeof(frame_)（=125）？因为 frame_ 里 user_data 固定留了 112 字节，
// [解析]   而一帧最多只用到 107 字节，多发的部分裁判系统不认，还会白白占用串口带宽。
// [解析] 举例：pack 1 个图形时 data_len = 21，size() = 5+2+21+2 = 30 字节。
// [解析] 注意：data_len 是 pack()/delete_xxx() 写进帧头的字段，
// [解析]   所以必须在它们之后调用 size()，否则拿到的是上次遗留的值（或未初始化的值）。
size_t UI_Manager::size() const
{
  return referee::HEAD_LEN + referee::CMD_ID_LEN + frame_.head.data_len + referee::TAIL_LEN;
}

// [解析] data()：返回指向帧缓冲区首个字节的指针。
// [解析] reinterpret_cast<const uint8_t *> 的意思是"把这块内存重新解释成字节数组"，
// [解析]   它不做任何数值转换，只是换个视角看同一块内存（结构体本来就是一串字节）。
// [解析] 典型用法：pm02.send(ui_manager.data(), ui_manager.size());
// [解析] 为什么 const？因为这个函数不会修改缓冲区；也提醒调用者别去改它。
// [解析] 注意：指针指向成员 frame_，下一次 pack() 会直接覆盖里面的内容，
// [解析]   所以发完就作废，不要保存这个指针长期使用。
const uint8_t * UI_Manager::data() const { return reinterpret_cast<const uint8_t *>(&frame_); }

// [解析] set_sender_id：设置发送者，并按发送者推出接收者（选手端）。
// [解析] 两行分别填交互数据里的两个字段：
// [解析]   sender_id   = 调用者给的 robot_id（裁判系统里就是机器人 ID）；
// [解析]   receiver_id = get_client_id(robot_id)，即对应的选手端 Client ID。
// [解析] 举例：robot_id = 1（红方英雄）时，sender_id = 1、receiver_id = 0x0101。
// [解析] 调用时机：readme 的示例在任务开始处调用一次；如果想让代码同时适配红蓝方，
// [解析]   可以在循环里用 pm02.robot_status.robot_id 反复调用它（开销很小）。
void UI_Manager::set_sender_id(uint8_t robot_id)
{
  frame_.data.sender_id = robot_id;
  frame_.data.receiver_id = get_client_id(robot_id);
}

// [解析] delete_all：请求选手端删除"所有图层上的所有图形"。
// [解析] 逐行看：
// [解析]   1) data_len = DATA_HEAD_LEN(6) + sizeof(InteractionLayerDelete)(2) = 8，
// [解析]      表示这一帧的交互数据一共 8 字节（6 字节交互头 + 2 字节删除指令）；
// [解析]   2) data_cmd_id = 0x0100（INTERACTION_LAYER_DELETE），子命令"删图层"；
// [解析]   3) user_data[0] = 2 —— 协议规定 delete_type：0 空操作 / 1 删除指定图层 / 2 删除全部；
// [解析]   4) apply_crc() 收尾：帧序号自增并补上 CRC8/CRC16。
// [解析] 提示：这里的 2 是协议里的"魔法数字"，对照 referee_protocol.hpp 里
// [解析]   InteractionLayerDelete 结构体的注释就能看懂（代码里没有用枚举，属于原作者的写法）。
// [解析] 注意：user_data[1]（本来放图层号）在这里没有赋值，发出去的是缓冲区里的旧值；
// [解析]   因为 delete_type = 2（删除全部）时协议规定接收方忽略该字节，所以不影响功能。
void UI_Manager::delete_all()
{
  frame_.head.data_len = DATA_HEAD_LEN + sizeof(referee::InteractionLayerDelete);
  frame_.data.data_cmd_id = referee::data_cmd_id::INTERACTION_LAYER_DELETE;
  frame_.data.user_data[0] = 2;
  apply_crc();
}

// [解析] delete_layer：只删除指定图层上的图形，其他图层保留。
// [解析] 与 delete_all() 的差别只有两点：
// [解析]   user_data[0] = 1 —— delete_type = 1，表示"删除指定图层"（不是全部）；
// [解析]   user_data[1] = 图层号（0~9），所以必须传 ui::Layer 枚举进来。
// [解析] data_len 仍然是 8：6 字节交互头 + 2 字节（delete_type + layer），长度一样多。
// [解析] 常见用法：把"每帧都在变的数值"放第 1 层，需要重画时先删第 1 层再重新 pack，
// [解析]   这样静态的第 0 层（比如背景框线）不用重复发送，能省不少带宽。
// [解析] 注意：apply_crc() 会把 CRC16 写到 user_data[2]、user_data[3]（见 manager.hpp）。
void UI_Manager::delete_layer(ui::Layer layer)
{
  frame_.head.data_len = DATA_HEAD_LEN + sizeof(referee::InteractionLayerDelete);
  frame_.data.data_cmd_id = referee::data_cmd_id::INTERACTION_LAYER_DELETE;
  frame_.data.user_data[0] = 1;
  frame_.data.user_data[1] = static_cast<uint8_t>(layer);
  apply_crc();
}

// [解析] pack(String)：打包一个"字符图形"，也就是在 UI 上显示一段文字。
// [解析] 这是唯一一个同时发送"图形描述 + 字符串内容"的 pack，所以写得最长。
// [解析] 逐行拆解：
// [解析]   1) data_len = 6 + sizeof(ExtClientCustomCharacter)(45) = 51 字节；
// [解析]      这 45 = 15（图形描述 InteractionFigure）+ 30（字符数据 data[30]）；
// [解析]   2) data_cmd_id = 0x0110（EXT_CLIENT_CUSTOM_CHARACTER），字符图形专用子命令，
// [解析]      和普通图形的 0x0101~0x0104 不是一个；
// [解析]   3) str->str.data()：取出 std::string 内部字符数组的首地址
// [解析]      （C++11 起 std::string 的内存保证连续，所以可以这样按字节拷）；
// [解析]   4) str_len：取"字符串实际长度"和"30"里较小的那个，防止越界；
// [解析]   5) std::fill(...)：把字符区 30 字节先全部清零。这一步很关键！
// [解析]      因为 frame_ 是复用的缓冲区，上一帧的内容还残留在里面，
// [解析]      如果新字符串比上次短，不清零就会在尾巴上看到上一次的文字残影；
// [解析]   6) std::copy(...)：把字符串前 str_len 个字节拷到字符区
// [解析]      （字符区从 user_data 的第 15 字节开始，正好在图形描述之后）；
// [解析]   7) copy(str, 0)：把 15 字节的图形描述拷到 user_data 开头（第 0 槽位）；
// [解析]   8) apply_crc()：更新序号并算 CRC。
// [解析] 注意：局部变量 data 与成员函数 data() 同名，在函数内部会"遮蔽"那个函数名 ——
// [解析]   本函数后面没有再调用 data()，所以能编过；自己写代码时最好避免这种命名。
// [解析] 注意：字符串长度用的是 details_b（在 String::set_value 里设置的），
// [解析]   但这里拷贝时又重新用 std::min 算了一遍，两处都做了限制，双保险。
void UI_Manager::pack(const ui::String * str)
{
  frame_.head.data_len = DATA_HEAD_LEN + sizeof(referee::ExtClientCustomCharacter);
  frame_.data.data_cmd_id = referee::data_cmd_id::EXT_CLIENT_CUSTOM_CHARACTER;

  auto data = str->str.data();
  auto str_len = std::min(str->str.size(), MAX_STR_LEN);
  std::fill(frame_.data.user_data + FIG_LEN, frame_.data.user_data + FIG_LEN + MAX_STR_LEN, 0);
  std::copy(data, data + str_len, frame_.data.user_data + FIG_LEN);

  copy(str, 0);
  apply_crc();
}

// [解析] pack(1)：打包 1 个普通图形，这是最"省"的一帧。
// [解析]   data_len = 6 + 15*1 = 21 字节；子命令 0x0101（INTERACTION_FIGURE，画一个图形）；
// [解析]   copy(e1, 0) 把它写到 user_data 的第 0 个槽位（偏移 0）。
// [解析] 适用场景：只有一个数值需要刷新时用它最省带宽，例如每帧只更新一个 Integer。
void UI_Manager::pack(const ui::ComposableElement * e1)
{
  frame_.head.data_len = DATA_HEAD_LEN + FIG_LEN;
  frame_.data.data_cmd_id = referee::data_cmd_id::INTERACTION_FIGURE;
  copy(e1, 0);
  apply_crc();
}

// [解析] pack(2)：一次打包 2 个普通图形。
// [解析]   data_len = 6 + 15*2 = 36 字节；子命令 0x0102（画两个图形）。
// [解析]   两个图形依次写到 user_data 的第 0、1 个槽位（偏移 0 和 15）。
// [解析] 和"分两帧各发一个"相比，这样能省掉一次帧头/校验的开销，串口压力更小。
void UI_Manager::pack(const ui::ComposableElement * e1, const ui::ComposableElement * e2)
{
  frame_.head.data_len = DATA_HEAD_LEN + FIG_LEN * 2;
  frame_.data.data_cmd_id = referee::data_cmd_id::INTERACTION_FIGURE_2;
  copy(e1, 0);
  copy(e2, 1);
  apply_crc();
}

// [解析] pack(5)：一次打包 5 个普通图形（协议规定的"中等批量"）。
// [解析]   data_len = 6 + 15*5 = 81 字节；子命令 0x0103（INTERACTION_FIGURE_5，画五个图形）。
// [解析]   5 个图形依次放到槽位 0~4（偏移 0、15、30、45、60）。
// [解析] 注意 manager.hpp 里 e4/e5 带默认值 nullptr：
// [解析]   调用 pack(a, b, c) 也能编过，但槽位 3、4 会被 empty_ 占位，
// [解析]   帧长照样是 81 字节 —— 等于白发了 30 字节的空数据，所以建议 5 个都填满。
// [解析] 提示：这也是本工程最常用的重载，一帧画 5 个图形兼顾了带宽和刷新率。
void UI_Manager::pack(
  const ui::ComposableElement * e1, const ui::ComposableElement * e2,
  const ui::ComposableElement * e3, const ui::ComposableElement * e4,
  const ui::ComposableElement * e5)
{
  frame_.head.data_len = DATA_HEAD_LEN + FIG_LEN * 5;
  frame_.data.data_cmd_id = referee::data_cmd_id::INTERACTION_FIGURE_5;
  copy(e1, 0);
  copy(e2, 1);
  copy(e3, 2);
  copy(e4, 3);
  copy(e5, 4);
  apply_crc();
}

// [解析] pack(7)：一次打包 7 个普通图形，这是裁判系统一帧能带的上限。
// [解析]   data_len = 6 + 15*7 = 111 字节；子命令 0x0104（INTERACTION_FIGURE_7）。
// [解析]   7 个图形依次放在槽位 0~6，最后一个槽位 6 的偏移是 90，占到 user_data[104]，
// [解析]   紧接着 user_data[105]、user_data[106] 就是 CRC16 的位置（见 manager.hpp 说明）。
// [解析] 注意：这一帧最容易踩到"user_data 够不够用"的边界，好在协议给 user_data
// [解析]   定义了 112 字节，而绘制 UI 最多只用到 105 字节，多出来的空间正好够放 CRC16。
// [解析] 注意：e7 有默认值，所以最少传 6 个参数；只传 6 个时槽位 6 用空图形占位，
// [解析]   帧长依然是 111 字节，会白白多发 15 字节。
// [解析] 性能提醒：111 字节是 UI 里最长的一帧，串口波特率低（比如 115200）时
// [解析]   一帧就要约 10 ms，7 图形 + 高频刷新很容易把串口占满，按需取舍。
void UI_Manager::pack(
  const ui::ComposableElement * e1, const ui::ComposableElement * e2,
  const ui::ComposableElement * e3, const ui::ComposableElement * e4,
  const ui::ComposableElement * e5, const ui::ComposableElement * e6,
  const ui::ComposableElement * e7)
{
  frame_.head.data_len = DATA_HEAD_LEN + FIG_LEN * 7;
  frame_.data.data_cmd_id = referee::data_cmd_id::INTERACTION_FIGURE_7;
  copy(e1, 0);
  copy(e2, 1);
  copy(e3, 2);
  copy(e4, 3);
  copy(e5, 4);
  copy(e6, 5);
  copy(e7, 6);
  apply_crc();
}

// [解析] copy()：把一个图形描述拷进帧缓冲区的第 i 个槽位。所有 pack() 都靠它填数据。
// [解析] 分成三步看：
// [解析]   1) (e == nullptr) ? &empty_ : &e->data —— 空指针保护。
// [解析]      pack(a, nullptr, c) 这种写法不会崩，而是用一个"空操作"图形占住这个槽位，
// [解析]      这样帧长仍然和图形个数一一对应，选手端解析时不会错位；
// [解析]   2) reinterpret_cast<const uint8_t *>：把图形结构体指针当成字节指针，
// [解析]      因为 std::copy 是按字节拷贝的，而 InteractionFigure 是 packed 的紧凑结构，
// [解析]      整块拷 15 字节正好就是协议要求的字节流；
// [解析]   3) 目标地址 = user_data + FIG_LEN * i：第 i 个图形从第 i*15 字节开始。
// [解析] 注意：&e->data 取的是基类 ui::Element 的 data 成员，所以即使传进来的是
// [解析]   String（多了一个 std::string 成员），也只拷那 15 字节的图形描述 —— 这是有意为之，
// [解析]   字符串内容由 pack(const ui::String *) 单独处理。
// [解析] 注意：参数 i 的合法范围是 0~6（见 manager.cpp 里的调用点，全是写死的常量），
// [解析]   如果哪天有人传入更大的 i，就会写到 user_data 甚至 frame_ 之外（缓冲区溢出）。
// [解析] 为什么形参写成 ui::Element*，调用点却能传 ComposableElement*/String*？
// [解析]   因为是公有继承，派生类指针可以隐式转换成基类指针（向上转型）。
void UI_Manager::copy(const ui::Element * e, size_t i)
{
  auto fig = reinterpret_cast<const uint8_t *>((e == nullptr) ? &empty_ : &e->data);
  std::copy(fig, fig + FIG_LEN, frame_.data.user_data + FIG_LEN * i);
}

// [解析] apply_crc()：给整帧补上校验，是每个打包函数的最后一步（漏调就会发出一帧废数据）。
// [解析]   seq++：帧序号自增。裁判系统（和模拟器）靠它判断有没有丢帧或收到重复帧，
// [解析]     所以每一帧都必须不一样，不能永远停在 0。
// [解析]   CRC8：只算帧头的前 4 个字节（sof、data_len、seq），长度参数是 4 而不是 5 ——
// [解析]     因为第 5 个字节就是 crc8 自己，算校验时必须把它排除在外（crc.hpp 注释也写明
// [解析]     "len 不包括 crc8"）。
// [解析]   CRC16：算"从帧首到 CRC16 之前"的全部内容，所以长度是 size() - TAIL_LEN。
// [解析] 关键两行：
// [解析]   tail = (uint8_t *)&frame_.data + frame_.head.data_len;
// [解析]     指针算术：&frame_.data 是交互数据起点（帧内偏移 7），加上 data_len
// [解析]     就正好落在"这一帧数据最后一个字节的后面"，也就是 CRC16 该放的位置；
// [解析]   crc16 = (uint16_t *)tail; *crc16 = ...;
// [解析]     把这个地址重新解释成 2 字节整数指针，直接写进去（小端：低字节在前）。
// [解析] 为什么不用先清零 CRC16 那两个字节？因为 get_crc16 的长度参数
// [解析]   size() - TAIL_LEN = 7 + data_len，正好等于 CRC16 的起始偏移，
// [解析]   也就是说校验范围天然避开了 CRC16 自己，缓冲区里的旧值不会被算进去。
// [解析] 注意：tail 可能是奇数地址（非对齐）。这里做了 uint8_t* -> uint16_t* 的强制转换，
// [解析]   在 Cortex-M4 上默认允许非对齐的半字访问，所以能正常工作；
// [解析]   如果编译器配置打开了非对齐访问陷阱（或换到某些内核/开启严格别名优化），
// [解析]   这里就可能出问题，属于潜在的可移植性隐患，仅作提示。
void UI_Manager::apply_crc()
{
  frame_.head.seq++;

  // 先计算crc8
  // [解析] ↑ 原注释保留：下面这一行只算帧头的 4 字节校验，写进 head.crc8。
  frame_.head.crc8 = get_crc8(data(), 4);

  // 再计算crc16
  // [解析] ↑ 原注释保留：先定位帧尾，再算整帧校验写进去。
  auto tail = reinterpret_cast<uint8_t *>(&frame_.data) + frame_.head.data_len;
  auto crc16 = reinterpret_cast<uint16_t *>(tail);
  *crc16 = get_crc16(data(), size() - referee::TAIL_LEN);
}

}  // namespace sp
