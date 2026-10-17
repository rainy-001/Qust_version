// [解析] UI 发送端：把 element 里的图形打包成裁判系统 0x0301 帧，并算好帧头 CRC8、帧尾 CRC16。
// [解析] 用法：set_sender_id -> (delete_all/delete_layer) -> pack(...) -> send(data(), size())，每 30~40 ms 一次。
// [解析] 最费解的一点：帧尾 CRC16 不占独立成员，而是借用 user_data[] 末尾没被图形占用的字节。

#ifndef SP__UI_MANAGER_HPP
#define SP__UI_MANAGER_HPP

#include "element.hpp"
#include "referee/referee_protocol/referee_protocol.hpp"

namespace sp
{
// [解析] 内部只有一块约 125 字节的复用缓冲区 frame_，pack() 只覆写不申请内存，可在任务循环里反复调用。
class UI_Manager
{
public:
  UI_Manager();

  // [解析] 返回真正要发的字节数 = 5+2+data_len+2，不是 sizeof(frame_)；须在 pack()/delete_xxx() 之后调用。
  size_t size() const;
  const uint8_t * data() const;

  // [解析] sender_id 填 referee::robot_id 常量，receiver_id 由它映射成选手端 Client ID（见 get_client_id）。
  void set_sender_id(uint8_t robot_id);

  // [解析] 两个删除指令：user_data[0]=2 清全部、=1 清指定图层；同样要 send(data(), size()) 才生效。
  void delete_all();
  void delete_layer(ui::Layer layer);

  // [解析] 5 个 pack() 重载按参数个数区分：1 个 String、1/2/5/7 个普通图形；缺省参数用空图形占位，帧长不缩短。
  void pack(const ui::String * str);
  void pack(const ui::ComposableElement * e1);
  void pack(const ui::ComposableElement * e1, const ui::ComposableElement * e2);
  void pack(
    const ui::ComposableElement * e1, const ui::ComposableElement * e2,
    const ui::ComposableElement * e3, const ui::ComposableElement * e4 = nullptr,
    const ui::ComposableElement * e5 = nullptr);
  void pack(
    const ui::ComposableElement * e1, const ui::ComposableElement * e2,
    const ui::ComposableElement * e3, const ui::ComposableElement * e4,
    const ui::ComposableElement * e5, const ui::ComposableElement * e6,
    const ui::ComposableElement * e7 = nullptr);

private:
  // [解析] frame_ 的字节顺序就是协议顺序；packed 禁止编译器插入对齐填充，否则发出去的字节位置全错。
  struct __attribute__((packed))
  {
    referee::FrameHeader head;
    uint16_t cmd_id;
    referee::RobotInteractionData data;
    // 绘制UI所需的user_data最大长度为105, 而其定义长度为112, 有剩余, 因此帧尾的crc16在data内部
  } frame_;

  referee::InteractionFigure empty_;

  void copy(const ui::Element * e, size_t i);
  // [解析] 所有 pack()/delete_xxx() 的最后一步：seq 自增后填 CRC8 与 CRC16，漏调就会发出一帧废数据。
  void apply_crc();
};

}  // namespace sp

#endif  // SP__UI_MANAGER_HPP