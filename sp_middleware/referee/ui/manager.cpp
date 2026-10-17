// [解析] UI_Manager 的实现：把图形打包成裁判系统 0x0301 机器人交互数据帧，并支持删除全部/删除一层。
// [解析] 各 pack() 的差别只有 data_len、子命令号和图形槽位数，最后都调 apply_crc() 收尾。
// [解析] 帧长 = 6(交互头) + 15×图形数；CRC16 写在 user_data[] 内部，位置由 data_len 决定。

#include "manager.hpp"

// [解析] 裁判系统是双校验：CRC8 只算帧头前 4 字节，CRC16 算"帧首到 CRC16 之前"的整段。
#include "tools/crc/crc.hpp"

namespace sp
{
// [解析] 交互数据里"图形数据之前"的头部长度：RobotInteractionData 总长减去 user_data 数组长度，等于 6。
constexpr size_t DATA_HEAD_LEN =
  sizeof(referee::RobotInteractionData) - sizeof(referee::RobotInteractionData::user_data);
// [解析] 一个图形描述固定 15 字节（协议里有 static_assert 兜底，改协议会立刻编译报错）。
constexpr size_t FIG_LEN = sizeof(referee::InteractionFigure);
// [解析] 字符图形的文本区最大 30 字节，超出的字符要在拷贝前截断。
constexpr size_t MAX_STR_LEN = sizeof(referee::ExtClientCustomCharacter::data);

// [解析] 机器人 ID（红 1~11 / 蓝 101~111）和选手端 Client ID（0x0101~ / 0x0165~）是两套编号，
// [解析] UI 帧的 receiver_id 必须填 Client ID；表里没有的（哨兵/飞镖/雷达等）落到 default，发给 0x8080。
uint16_t get_client_id(uint8_t robot_id)
{
  switch (robot_id) {
    case referee::robot_id::RED_HERO:
      return referee::client_id::RED_HERO_CLIENT;
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
      return referee::client_id::REFEREE_SERVER;
  }
}

// [解析] 只填每帧都不变的字段（sof/seq/cmd_id）和占位图形的操作类型，data_len 留给各 pack() 去填。
UI_Manager::UI_Manager()
{
  frame_.head.sof = referee::SOF;
  frame_.head.seq = 0;
  frame_.cmd_id = referee::cmd_id::ROBOT_INTERACTION_DATA;
  empty_.operate_type = static_cast<uint8_t>(ui::OperateType::EMPTY);
}

// [解析] 实际帧长 = 5+2+data_len+2，不是 sizeof(frame_)：user_data 多留的空间不该发出去，也不该被算进校验。
size_t UI_Manager::size() const
{
  return referee::HEAD_LEN + referee::CMD_ID_LEN + frame_.head.data_len + referee::TAIL_LEN;
}

const uint8_t * UI_Manager::data() const { return reinterpret_cast<const uint8_t *>(&frame_); }

void UI_Manager::set_sender_id(uint8_t robot_id)
{
  frame_.data.sender_id = robot_id;
  frame_.data.receiver_id = get_client_id(robot_id);
}

// [解析] data_len = 8（6 交互头 + 2 删除指令），user_data[0] = 2 是协议里的"删除全部"；
// [解析] 这个 2 是魔法数字，对应 referee_protocol.hpp 里 InteractionLayerDelete 的 delete_type。
void UI_Manager::delete_all()
{
  frame_.head.data_len = DATA_HEAD_LEN + sizeof(referee::InteractionLayerDelete);
  frame_.data.data_cmd_id = referee::data_cmd_id::INTERACTION_LAYER_DELETE;
  frame_.data.user_data[0] = 2;
  apply_crc();
}

// [解析] 与 delete_all() 只差 user_data[0]=1（删指定层）和多填一个 user_data[1]=图层号，帧长同样是 8。
void UI_Manager::delete_layer(ui::Layer layer)
{
  frame_.head.data_len = DATA_HEAD_LEN + sizeof(referee::InteractionLayerDelete);
  frame_.data.data_cmd_id = referee::data_cmd_id::INTERACTION_LAYER_DELETE;
  frame_.data.user_data[0] = 1;
  frame_.data.user_data[1] = static_cast<uint8_t>(layer);
  apply_crc();
}

// [解析] 唯一同时发"图形描述 + 文本"的 pack：data_len = 6 + 45，其中 45 = 15(图形) + 30(文本)。
// [解析] 文本区必须先整块清零再拷，否则复用缓冲区里上一帧更长的文字会在尾巴上露出残影。
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

void UI_Manager::pack(const ui::ComposableElement * e1)
{
  frame_.head.data_len = DATA_HEAD_LEN + FIG_LEN;
  frame_.data.data_cmd_id = referee::data_cmd_id::INTERACTION_FIGURE;
  copy(e1, 0);
  apply_crc();
}

void UI_Manager::pack(const ui::ComposableElement * e1, const ui::ComposableElement * e2)
{
  frame_.head.data_len = DATA_HEAD_LEN + FIG_LEN * 2;
  frame_.data.data_cmd_id = referee::data_cmd_id::INTERACTION_FIGURE_2;
  copy(e1, 0);
  copy(e2, 1);
  apply_crc();
}

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

// [解析] 最长的一帧：data_len = 111，7 个槽位占 user_data[0..104]，紧接着 [105] 起就是 CRC16，正好放得下。
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

// [解析] 按字节块把 15 字节拷进第 i 个槽位（偏移 i*15）；传 nullptr 时改用 empty_ 占位，帧长不变。
void UI_Manager::copy(const ui::Element * e, size_t i)
{
  auto fig = reinterpret_cast<const uint8_t *>((e == nullptr) ? &empty_ : &e->data);
  std::copy(fig, fig + FIG_LEN, frame_.data.user_data + FIG_LEN * i);
}

// [解析] 每个打包函数的最后一步：seq 自增（选手端靠它判丢帧/重复帧），再补 CRC8 和 CRC16。
// [解析] CRC8 的长度写 4 而不是 5，因为第 5 字节就是 crc8 自己；CRC16 从帧首算到 size()-TAIL_LEN。
// [解析] tail 由 &frame_.data + data_len 定位，正好落在 CRC16 该写的位置；这是非对齐半字访问，M4 允许。
void UI_Manager::apply_crc()
{
  frame_.head.seq++;

  // 先计算crc8
  frame_.head.crc8 = get_crc8(data(), 4);

  // 再计算crc16
  auto tail = reinterpret_cast<uint8_t *>(&frame_.data) + frame_.head.data_len;
  auto crc16 = reinterpret_cast<uint16_t *>(tail);
  *crc16 = get_crc16(data(), size() - referee::TAIL_LEN);
}

}  // namespace sp
