// [解析] ==========================================================================
// [解析] 【文件】sp_middleware/referee/vt02/vt02.cpp
// [解析] 【作用】vt02.hpp 里 VT02 类的实现：启动串口接收、拆帧、校验、拷贝数据。
// [解析] 【看代码的重点】三处最容易看不懂，下面都会逐段标注：
// [解析]   1) request() 里为什么用 ReceiveToIdle 而不是普通 Receive，以及那句
// [解析]      __HAL_DMA_DISABLE_IT(..., DMA_IT_HT) 是在关什么；
// [解析]   2) update() 里的 "小端拼字节"：(buff_[2] << 8) | buff_[1] 为什么高低字节是反的；
// [解析]   3) 三道校验门槛（SOF -> CRC8 -> CRC16）分别防的是什么错误。
// [解析] 【协议速查】帧结构见 referee_def.h 顶部的 ASCII 图。
// [解析] ==========================================================================

#include "vt02.hpp"

#include <cstring>

#include "referee_def.h"
#include "tools/crc/crc.hpp"

namespace sp
{
// [解析] 构造函数：初始化列表把 huart 存进 huart_、use_dma 存进 use_dma_。
// [解析] 函数体为空 —— 此时不该动硬件，真正的接收由后面的 request() 启动。
// [解析] 注意 has_read_ / last_read_ms_ / buff_ 没有在这里初始化（buff_ 是裸数组、不是 std::array），
// [解析]   所以它们的初值是"不确定"的，只能靠 is_open() 先判断 has_read_ 来兜住。
VT02::VT02(UART_HandleTypeDef * huart, bool use_dma) : huart_(huart), use_dma_(use_dma) {}

void VT02::request()
{
  // [解析] 两条分支的区别只在"数据搬运工"是谁：DMA 由硬件搬（CPU 空闲），IT 由中断搬（每来一字节进一次中断）。
  if (use_dma_) {
    // dismiss return
    // [解析] ReceiveToIdle = "一直收到串口空闲为止"。
    // [解析] 为什么不用普通的 HAL_UART_Receive_DMA？因为裁判系统每帧长度不固定，
    // [解析]   而 ReceiveToIdle 会在总线静默（一帧发完）时触发回调，天然完成了"分帧"。
    // [解析] 参数：huart=哪路串口，buff_=搬到哪，VT02_BUFF_SIZE=最多搬 50 字节。
    // [解析] 它只是"挂上去"，函数立刻返回，数据在后台由 DMA 慢慢灌进 buff_。
    HAL_UARTEx_ReceiveToIdle_DMA(huart_, buff_, VT02_BUFF_SIZE);

    // ref: basic_framework/bsp/usart/bsp_usart.c at master · HNUYueLuRM/basic_framework
    // [解析] 这句是把 DMA 的"半传输中断"(Half Transfer, HT)关掉。
    // [解析] 缓冲一共 50 字节，DMA 搬满 25 字节时会先来一次 HT 中断，
    // [解析]   但那时帧才收了一半，进去解析只会得到半截数据（纯属浪费 CPU 甚至误判）。
    // [解析] 关掉之后只在"收满"或"串口空闲"时才进回调，语义正好符合需求。
    // [解析] 这也是一句典型踩坑点：不关 HT，串口回调会被无意义地触发两次。
    __HAL_DMA_DISABLE_IT(huart_->hdmarx, DMA_IT_HT);
  }
  else {
    // dismiss return
    // [解析] 非 DMA 版本：改成中断方式收，行为一致（同样是"收到空闲为止"），
    // [解析]   只是每个字节都要进一次中断，波特率高时 CPU 占用明显更大。
    HAL_UARTEx_ReceiveToIdle_IT(huart_, buff_, VT02_BUFF_SIZE);
  }
}

// [解析] update() 是整个文件的核心：把 buff_ 里那串字节翻译成结构体。
// [解析] 参数 stamp_ms：当前毫秒时间戳（HAL_GetTick()），解析成功时记进 last_read_ms_。
// [解析] 整体思路是"漏斗式过滤"，一共 4 道关卡，任意一道不过就 return（直接放弃这一帧）：
// [解析]   关卡1 帧头字节必须是 0xA5        —— 防止把错位/噪声当数据
// [解析]   关卡2 帧头 5 字节的 CRC8 要对     —— 防止帧头被打乱
// [解析]   关卡3 整帧 CRC16 要对            —— 防止数据段被干扰
// [解析]   关卡4 命令码必须是 0x0302        —— 只关心"自定义控制器"这一种数据
// [解析] 为什么帧头单独再加一个 CRC8？因为数据长度、序号都在帧头里，
// [解析]   帧头错了就没法知道这帧有多长，必须先用 CRC8 确认帧头可信。
void VT02::update(uint32_t stamp_ms)
{
  // [解析] 关卡1：SOF（Start Of Frame）检查。REFEREE_SOF = 0xA5。
  // [解析] buff_[0] 不是 0xA5，说明接收错位或者根本是别的设备的数据，直接丢弃。
  if (buff_[0] != REFEREE_SOF) return;
  // [解析] 关卡2：校验帧头。注意 check_crc8 的 len 是"包含 CRC8 字节本身"的，
  // [解析]   所以这里传 REFEREE_HEAD_LEN=5（SOF+len+seq+crc8），正好把最后一字节当校验和。
  if (!check_crc8(buff_, REFEREE_HEAD_LEN)) return;

  // [解析] 从帧头里取出"数据段长度"。关键点：协议是小端(Little Endian)，
  // [解析]   先发低字节、后发高字节，所以 buff_[1] 是低 8 位、buff_[2] 是高 8 位！
  // [解析]   (buff_[2] << 8) | buff_[1] 就是"高字节左移 8 位再和低字节拼起来" = 真实的 16 位长度。
  // [解析]   写成 (buff_[1] << 8) | buff_[2] 会得到一个完全错误的长度（字节序反了）。
  size_t data_len = (buff_[2] << 8) | buff_[1];
  // [解析] 整帧长度 = 帧头5 + 命令码2 + 数据段data_len + 帧尾2。
  // [解析] 帧尾就是 CRC16 那两个字节，它算在 check_crc16 的覆盖范围内。
  size_t len = REFEREE_HEAD_LEN + REFEREE_CMD_ID_LEN + data_len + REFEREE_TAIL_LEN;
  // [解析] 关卡3：整帧 CRC16 校验。len 同样"包含 CRC16 本身"，因为它占据最后 2 字节。
  // [解析] 注意：这里隐含假设 buff_ 里真的收满了 len 个字节。
  // [解析]   如果串口只收到半帧就触发了空闲中断，CRC16 一样会对不上而被拦掉，属于安全的失败方式。
  if (!check_crc16(buff_, len)) return;

  // [解析] 取命令码（cmd_id），同样是"低字节在前"：buff_[5] 是低 8 位，buff_[6] 是高 8 位。
  // [解析] 0x0302 存成两字节就是 低=0x02、高=0x03，所以拼出来才是 0x0302。
  uint16_t cmd_id = (buff_[6] << 8) | buff_[5];
  if (cmd_id == REFEREE_CUSTOM_CMD_ID) {
    // [解析] 关卡4通过：记录"什么时候收到的"和"收到过了"。
    // [解析] last_read_ms_ 会被 is_alive() 用来算超时；has_read_ 会被 is_open() 返回。
    last_read_ms_ = stamp_ms;
    has_read_ = true;
    // [解析] 把数据段原封不动地拷进 custom 结构体。
    // [解析] this-> 是多余的（成员函数里直接写 custom 也行），但写成 this->custom 更醒目。
    // [解析] sizeof(RefereeCustomData) = 34 字节（8 个 float + 1 个保留字节），
    // [解析]   因为结构体带 packed，成员的字节位置和协议里完全一致，这才是能直接 memcpy 的前提。
    // [解析] ⚠ 注意：这里**没有检查 data_len 是否等于 sizeof(RefereeCustomData)**。
    // [解析]   如果上位机发来的数据段不足 34 字节，memcpy 就会读到帧尾甚至缓冲区之外的字节；
    // [解析]   对比例子见 vt03.cpp，那里用 `if (data_len != sizeof(...)) break;` 做了长度校验。
    std::memcpy(&(this->custom), buff_ + REFEREE_DATA_START, sizeof(RefereeCustomData));
  }
}

// [解析] 一行函数：返回"开机以来是否至少收到过一帧合法的 0x0302 数据"。
// [解析] 用途：判断图传链路有没有接上、上位机有没有在发数据。
bool VT02::is_open() const { return has_read_; }

// [解析] 在线判断：必须同时满足 ①曾经收到过 ②距今不到 2000ms。
// [解析] now_ms - last_read_ms_ 依赖 uint32_t 自然回绕（HAL_GetTick 约 49.7 天溢出一次），
// [解析]   回绕后的减法仍然能算对时间差，这是嵌入式里惯用的写法。
// [解析] 阈值 2000ms 比 VT03 的 100ms 宽松得多：因为图传链路本身延迟大、更新率低（约几十 Hz）。
bool VT02::is_alive(uint32_t now_ms) const { return is_open() && (now_ms - last_read_ms_ < 2000); }

}  // namespace sp