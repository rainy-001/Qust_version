// [解析] VT03 类的实现：串口接收、双协议拆帧（遥控器 0xA9 0x53 / 裁判系统 0xA5）、位域换算、组帧发送。
// [解析] 上层在串口空闲回调里调 update(size, HAL_GetTick())，之后每次都要重新 request() 才会继续收。
// [解析] 一个缓冲里可能粘好几帧，所以 update() 解析完一帧会递归处理剩下的字节。
// [解析] 摇杆原始值 0~2047 由 (x - 1024) / 660.0f 归一化，660 是实测标定值，见 update_remote()。

#include "vt03.hpp"

#include "tools/crc/crc.hpp"

namespace sp
{
// [解析] 初始化列表把所有成员给到安全初值（摇杆 0、按键 false），函数体再把 255 字节接收缓冲清零。
// [解析] 这里必须用初始化列表而不是赋值，是因为 use_dma_ 是 const 成员。
VT03::VT03(UART_HandleTypeDef * huart, bool use_dma)
  : huart(huart),
    mode(VT03Mode::C),
    ch_rh(0.0f),
    ch_rv(0.0f),
    ch_lh(0.0f),
    ch_lv(0.0f),
    wheel(0.0f),
    fn_l(false),
    fn_r(false),
    pause(false),
    trigger(false),
    custom(),
    robot(),
    mouse{0.0f, 0.0f, 0.0f, false, false, false},
    keys(),
    custom_client(),
    keyboard_value(0),
    use_dma_(use_dma),
    has_read_(false),
    last_read_ms_(0),
    custom_2_robot_has_read_(false),
    custom_2_robot_last_read_ms_(0),
    seq_(0)
{
  std::fill(buff_.begin(), buff_.end(), 0);
}

bool VT03::is_open() const { return has_read_; }

// [解析] 遥控器在线：收到过 && 距今 < 100ms（上报率 100Hz 以上，连丢约 10 帧就判掉线）。
bool VT03::is_alive(uint32_t now_ms) const { return is_open() && (now_ms - last_read_ms_ < 100); }

bool VT03::custom_2_robot_is_open() const { return custom_2_robot_has_read_; }

bool VT03::custom_2_robotis_alive(uint32_t now_ms) const
{
  return custom_2_robot_is_open() && (now_ms - custom_2_robot_last_read_ms_ < 100);
}

// [解析] 用 ReceiveToIdle 收（总线静默即一帧结束），并关掉 DMA 半传输(HT)中断：搬到一半的回调只会拿到半帧。
void VT03::request()
{
  if (use_dma_) {
    HAL_UARTEx_ReceiveToIdle_DMA(this->huart, buff_.data(), buff_.size());
    // ref: https://github.com/HNUYueLuRM/basic_framework/blob/master/bsp/usart/bsp_usart.c
    __HAL_DMA_DISABLE_IT(this->huart->hdmarx, DMA_IT_HT);
  }
  else {
    HAL_UARTEx_ReceiveToIdle_IT(this->huart, buff_.data(), buff_.size());
  }
}

// [解析] 解析入口：把内部 buff_ 转交给带起始指针的重载版本（递归时要能从缓冲中间继续）。
void VT03::update(uint16_t size, uint32_t stamp_ms) { update(buff_.data(), size, stamp_ms); }

// [解析] 核心函数：先按遥控器私有帧（0xA9 0x53）试，不是再按裁判系统标准帧（0xA5）试。
// [解析] 用两个字节当帧标志而不是一个，是为了降低数据中间正好出现 0xA9 的误判概率。
void VT03::update(uint8_t * frame_start, uint16_t size, uint32_t stamp_ms)
{
  // [解析] ⚠ has_read_/last_read_ms_ 一进函数就置位：即使 CRC 全不过，is_open()/is_alive() 也会返回 true。
  has_read_ = true;
  last_read_ms_ = stamp_ms;
  if (frame_start[0] == 0xA9 && frame_start[1] == 0x53) {
    size_t frame_len = sizeof(VT03RemoteData);

    if (size < frame_len) return;
    if (!check_crc16(frame_start, frame_len)) return;

    update_remote(reinterpret_cast<VT03RemoteData *>(frame_start));

    // 递归解析, 因为缓冲区中可能包含多帧裁判系统的数据
    update(frame_start + frame_len, size - frame_len, stamp_ms);
    return;
  }

  if (size < referee::HEAD_LEN) return;
  if (frame_start[0] != referee::SOF) return;
  if (!check_crc8(frame_start, referee::HEAD_LEN)) return;

  // [解析] 小端：frame_start[1] 是低字节、frame_start[2] 是高字节，写反了整帧长度就全错。
  size_t data_len = (frame_start[2] << 8) | frame_start[1];
  size_t frame_len = referee::HEAD_LEN + referee::CMD_ID_LEN + data_len + referee::TAIL_LEN;

  if (size < frame_len) return;
  if (!check_crc16(frame_start, frame_len)) return;

  uint16_t cmd_id = (frame_start[6] << 8) | frame_start[5];

  switch (cmd_id) {
    // 0x0302 自定义控制器与机器人交互数据
    case referee::cmd_id::CUSTOM_ROBOT_DATA:
      // [解析] 长度必须恰好等于结构体大小才拷贝，避免读越界/写错位（vt02.cpp 少了这一步）。
      if (data_len != sizeof(this->custom)) break;
      std::memcpy(
        reinterpret_cast<uint8_t *>(&this->custom), frame_start + referee::DATA_START,
        sizeof(this->custom));
      custom_2_robot_last_read_ms_ = stamp_ms;
      custom_2_robot_has_read_ = true;
      break;

    // 0x0309 自定义控制器与机器人交互数据
    case referee::cmd_id::ROBOT_CUSTOM_DATA:
      if (data_len != sizeof(this->robot)) break;
      std::memcpy(
        reinterpret_cast<uint8_t *>(&this->robot), frame_start + referee::DATA_START,
        sizeof(this->robot));
      break;

    //0x0311 自定义客户端发送给机器人的自定义指令
    case referee::cmd_id::CLIENT_ROBOT_DATA:
      // [解析] 变长帧：先整体清零避免残留旧数据，再校验 data_len <= 30，超出就标记无效并丢弃。
      std::fill(this->custom_client.data.begin(), this->custom_client.data.end(), 0);

      if (data_len > this->custom_client.data.size()) {
        this->custom_client.size = 0;
        this->custom_client.is_valid = false;
        break;
      }

      this->custom_client.size = static_cast<uint8_t>(data_len);
      this->custom_client.is_valid = true;
      std::memcpy(this->custom_client.data.data(), frame_start + referee::DATA_START, data_len);
      break;

    default:
      break;
  }

  // 递归解析, 因为缓冲区中可能包含多帧裁判系统的数据
  update(frame_start + frame_len, size - frame_len, stamp_ms);
}

// [解析] 把遥控器帧里的位域/原始值翻译成归一化成员；调用前必须已过 CRC16 校验，否则读到的位域是垃圾。
void VT03::update_remote(const VT03RemoteData * data)
{
  // [解析] 摇杆原始值是 11 位（0~2047），减 1024 让中位归 0，再除 660 缩放；
  // [解析] 660 是实测"摇杆有效行程"标定的，所以推到底约 ±1.55，并没有钳制在 [-1,1]。
  // [解析] ch_1 -> ch_rv、ch_2 -> ch_lv 这种通道号和语义的对应是作者按自己遥控器定的，方向反了就改这里。
  this->ch_rh = (data->ch_0 - 1024) / 660.0f;
  this->ch_rv = (data->ch_1 - 1024) / 660.0f;
  this->ch_lv = (data->ch_2 - 1024) / 660.0f;
  this->ch_lh = (data->ch_3 - 1024) / 660.0f;
  this->wheel = (data->wheel - 1024) / 660.0f;

  this->fn_l = data->fn_1;
  this->fn_r = data->fn_2;
  this->pause = data->pause;
  this->trigger = data->trigger;

  // [解析] 鼠标增量是 int16，除以正半量程 32768 归一化到约 [-1,1)，上层就不必关心 DPI。
  this->mouse.vx = data->mouse_x / 32768.0f;
  this->mouse.vy = data->mouse_y / 32768.0f;
  this->mouse.vs = data->mouse_z / 32768.0f;
  this->mouse.left = data->mouse_left;
  this->mouse.middle = data->mouse_middle;
  this->mouse.right = data->mouse_right;

  this->keyboard_value = data->keys;

  // [解析] 16 位掩码逐位拆成 bool：与对应 MASK 相与非 0 即该键按下，任意按键组合可同时生效。
  this->keys.w = ((this->keyboard_value & VT03_KEY_W_MASK) != 0);
  this->keys.s = ((this->keyboard_value & VT03_KEY_S_MASK) != 0);
  this->keys.a = ((this->keyboard_value & VT03_KEY_A_MASK) != 0);
  this->keys.d = ((this->keyboard_value & VT03_KEY_D_MASK) != 0);
  this->keys.shift = ((this->keyboard_value & VT03_KEY_SHIFT_MASK) != 0);
  this->keys.ctrl = ((this->keyboard_value & VT03_KEY_CTRL_MASK) != 0);
  this->keys.q = ((this->keyboard_value & VT03_KEY_Q_MASK) != 0);
  this->keys.e = ((this->keyboard_value & VT03_KEY_E_MASK) != 0);
  this->keys.r = ((this->keyboard_value & VT03_KEY_R_MASK) != 0);
  this->keys.f = ((this->keyboard_value & VT03_KEY_F_MASK) != 0);
  this->keys.g = ((this->keyboard_value & VT03_KEY_G_MASK) != 0);
  this->keys.z = ((this->keyboard_value & VT03_KEY_Z_MASK) != 0);
  this->keys.x = ((this->keyboard_value & VT03_KEY_X_MASK) != 0);
  this->keys.c = ((this->keyboard_value & VT03_KEY_C_MASK) != 0);
  this->keys.v = ((this->keyboard_value & VT03_KEY_V_MASK) != 0);
  this->keys.b = ((this->keyboard_value & VT03_KEY_B_MASK) != 0);

  // [解析] 2 -> S、1 -> N、其它（0 或开关抖动出的 3）-> C，最后那一支是兜底。
  this->mode = (data->mode_sw == 2)   ? VT03Mode::S
               : (data->mode_sw == 1) ? VT03Mode::N
                                      : VT03Mode::C;
}

// [解析] 按裁判系统帧格式打包 0x0310：帧头(5) + 命令码(2) + 300 字节数据 + CRC16(2) = 309 字节。
// [解析] seq_ 每帧自增写进帧头，接收方靠它发现丢包；发送是异步的，上一帧没发完再调会丢帧。
void VT03::send_custom_client_data(const CustomByteBlock & custom_data)
{
  // 静态数组避免频繁在栈上分配内存，320字节通常足够裁判系统单包使用
  // [解析] 必须 static：DMA/中断发送是异步的，函数返回后硬件还在读这块缓冲，放栈上会发乱码（代价是不可重入）。
  static uint8_t tx_buff[320];

  constexpr size_t data_len = sizeof(CustomByteBlock);
  constexpr size_t frame_len =
    referee::HEAD_LEN + referee::CMD_ID_LEN + data_len + referee::TAIL_LEN;

  // 1. 安全防线：防止缓冲区溢出
  if (frame_len > sizeof(tx_buff)) return;

  // 2. 组装帧头 (Frame Header)
  tx_buff[0] = referee::SOF;
  tx_buff[1] = data_len & 0xFF;
  tx_buff[2] = (data_len >> 8) & 0xFF;
  tx_buff[3] = this->seq_++;
  // [解析] CRC8 必须等帧头 5 个字节全填完再算，而且要在数据段之前 —— 它只覆盖帧头这 5 字节。
  sp::append_crc8(tx_buff, referee::HEAD_LEN);  // 一步追加 CRC8

  // 3. 组装命令码 (Cmd ID = 0x0310)
  tx_buff[referee::HEAD_LEN] = 0x10;
  tx_buff[referee::HEAD_LEN + 1] = 0x03;

  // 4. 拷贝数据段 (Data)
  std::memcpy(tx_buff + referee::DATA_START, &custom_data, data_len);

  // 5. 组装帧尾 (Frame Tail / CRC16)
  // [解析] CRC16 必须等数据段拷完再算，它覆盖帧头 + 命令码 + 数据段共 307 字节。
  sp::append_crc16(tx_buff, frame_len);  // 优雅地一步追加 CRC16

  // 6. 物理层发送
  // [解析] 发送是异步的：函数立刻返回；串口正忙时 HAL 返回 HAL_BUSY，这一帧就悄悄丢了。
  if (use_dma_) {
    HAL_UART_Transmit_DMA(this->huart, tx_buff, frame_len);
  }
  else {
    HAL_UART_Transmit_IT(this->huart, tx_buff, frame_len);
  }
}

}  // namespace sp