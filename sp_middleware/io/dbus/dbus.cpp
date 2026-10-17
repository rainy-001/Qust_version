// [解析] DBus 遥控器驱动实现：发起串口接收 + 把 18 字节原始帧解析成摇杆、拨杆、鼠标、键盘。
// [解析] 由 applications/uart_task.cpp 的 HAL_UARTEx_RxEventCallback() 调用，跑在中断上下文。
// [解析] 最难看懂的是 update() 里的位域拼接：4 个摇杆各 11 位，跨字节且不对齐。

#include "dbus.hpp"

#include <cmath>

namespace sp
{
// [解析] 11 位原始值(0~2047) -> [-1,1]：减 1024 去掉机械中位，除 660 是官方给的行程。
// [解析] 推到极限(2047)会算出 1.55，所以 update() 靠 |ch|>1 判断整帧是否错位。
static float get_stick(uint16_t raw) { return (raw - 1024) / 660.0f; }

// [解析] 拨杆原始值 1=上、3=中、2=下，这个顺序是协议规定的，不是代码写反。
// [解析] else 把 0 之类的异常值也兜底成 DOWN，宁可当成最安全的"下"。
static DBusSwitchMode get_switch(uint8_t raw)
{
  if (raw == 1)
    return DBusSwitchMode::UP;
  else if (raw == 3)
    return DBusSwitchMode::MID;
  else
    return DBusSwitchMode::DOWN;
}

DBus::DBus(UART_HandleTypeDef * huart, bool use_dma)
: huart_(huart), use_dma_(use_dma), has_read_(false)
{
}

// [解析] 发起一次异步接收，立刻返回；收满 18 字节或总线空闲时由 HAL 回调，每帧都要再调一次。
void DBus::request()
{
  if (use_dma_) {
    // dismiss return
    HAL_UARTEx_ReceiveToIdle_DMA(huart_, buff_, DBUS_BUFF_SIZE);

    // ref: https://github.com/HNUYueLuRM/basic_framework/blob/master/bsp/usart/bsp_usart.c
    // [解析] 关掉 DMA 半传输中断，否则收到第 9 字节也会回调一次（Size=9，会被 update() 丢掉）。
    __HAL_DMA_DISABLE_IT(huart_->hdmarx, DMA_IT_HT);
  }
  else {
    // dismiss return
    HAL_UARTEx_ReceiveToIdle_IT(huart_, buff_, DBUS_BUFF_SIZE);
  }
}

// [解析] 核心解析：0~5 字节是 4 个 11 位摇杆紧挨着排，6~15 是鼠标/按键（小端），16~17 是拨轮。
// [解析] 摇杆的位跨字节连续，只能"移位 + 按位或"拼回来，再用 & 0x07ff 抹掉别的高位。
void DBus::update(uint16_t size, uint32_t stamp_ms)
{
  // [解析] 长度不对说明串口丢字节或多字节，整帧丢弃，免得错位数据拿去控制机器人。
  if (size != DBUS_BUFF_SIZE) return;

  // [解析] 先记时间戳、再校验：只要还在源源不断来数据就算"接收机活着"，哪怕内容暂时不可信。
  has_read_ = true;
  last_read_ms_ = stamp_ms;

  // 遥控器解析
  // [解析] 下面 5 行同一套路：把跨字节的位拼起来，最后只留 11 位。
  float ch_rh = get_stick((buff_[0] | (buff_[1] << 8)) & 0x07ff);
  float ch_rv = get_stick(((buff_[1] >> 3) | (buff_[2] << 5)) & 0x07ff);
  float ch_lh = get_stick(((buff_[2] >> 6) | (buff_[3] << 2) | (buff_[4] << 10)) & 0x07ff);
  float ch_lv = get_stick(((buff_[4] >> 1) | (buff_[5] << 7)) & 0x07ff);
  float ch_lu = get_stick(((buff_[16] | (buff_[17] << 8)) & 0x07ff));

  DBusSwitchMode sw_r = get_switch((buff_[5] >> 4) & 0x0003);
  // [解析] sw_l 写成 ((b >> 4) & 0x000C) >> 2，等价于更直观的 (b >> 6) & 0x03。
  DBusSwitchMode sw_l = get_switch(((buff_[5] >> 4) & 0x000C) >> 2);

  // 遥控器数据异常
  // [解析] 摇杆超出 ±1 说明整帧错位了；注意拨轮 ch_lu 没有参与这条检查。
  if (std::abs(ch_rh) > 1 || std::abs(ch_rv) > 1 || std::abs(ch_lh) > 1 || std::abs(ch_lv) > 1)
    return;

  // 键鼠解析
  // [解析] 鼠标分量是低字节在前的小端 int16：用 int16_t 才能让"左/上"变负数，再除 32768 归一化。
  int16_t mouse_vx_int = (buff_[7] << 8) | buff_[6];
  int16_t mouse_vy_int = (buff_[9] << 8) | buff_[8];
  int16_t mouse_vs_int = (buff_[11] << 8) | buff_[10];
  float mouse_vx = mouse_vx_int / 32768.0f;
  float mouse_vy = mouse_vy_int / 32768.0f;
  float mouse_vs = mouse_vs_int / 32768.0f;

  uint16_t keyboard_value = (buff_[15] << 8) | buff_[14];

  // 鼠标数据异常
  // [解析] 这条判断其实恒不成立（|int16|/32768 最大就是 1），留着只是求个心安。
  if (std::abs(mouse_vx) > 1 || std::abs(mouse_vy) > 1 || std::abs(mouse_vs) > 1) return;

  /// 更新公有属性

  // [解析] 先全算到局部变量、校验通过再统一提交，坏帧时公有成员仍是上一帧的好数据。
  // 遥控器
  this->ch_rh = ch_rh;
  this->ch_rv = ch_rv;
  this->ch_lh = ch_lh;
  this->ch_lv = ch_lv;
  this->ch_lu = ch_lu;

  this->sw_r = sw_r;
  this->sw_l = sw_l;

  // 键鼠
  this->mouse.vx = mouse_vx;
  this->mouse.vy = mouse_vy;
  this->mouse.vs = mouse_vs;

  this->mouse.left = (buff_[12] == 1);
  this->mouse.right = (buff_[13] == 1);
  this->keyboard_value = keyboard_value;

  // [解析] 16 行同一个套路：(原始值 & 掩码) != 0 就是该键按下，掩码顺序见 dbus.hpp。
  this->keys.w = ((keyboard_value & DBUS_KEY_W_MASK) != 0);
  this->keys.s = ((keyboard_value & DBUS_KEY_S_MASK) != 0);
  this->keys.a = ((keyboard_value & DBUS_KEY_A_MASK) != 0);
  this->keys.d = ((keyboard_value & DBUS_KEY_D_MASK) != 0);
  this->keys.shift = ((keyboard_value & DBUS_KEY_SHIFT_MASK) != 0);
  this->keys.ctrl = ((keyboard_value & DBUS_KEY_CTRL_MASK) != 0);
  this->keys.q = ((keyboard_value & DBUS_KEY_Q_MASK) != 0);
  this->keys.e = ((keyboard_value & DBUS_KEY_E_MASK) != 0);
  this->keys.r = ((keyboard_value & DBUS_KEY_R_MASK) != 0);
  this->keys.f = ((keyboard_value & DBUS_KEY_F_MASK) != 0);
  this->keys.g = ((keyboard_value & DBUS_KEY_G_MASK) != 0);
  this->keys.z = ((keyboard_value & DBUS_KEY_Z_MASK) != 0);
  this->keys.x = ((keyboard_value & DBUS_KEY_X_MASK) != 0);
  this->keys.c = ((keyboard_value & DBUS_KEY_C_MASK) != 0);
  this->keys.v = ((keyboard_value & DBUS_KEY_V_MASK) != 0);
  this->keys.b = ((keyboard_value & DBUS_KEY_B_MASK) != 0);
}

bool DBus::is_open() const { return has_read_; }

// [解析] 100ms 约等于连续丢 7 帧（一帧 14ms）；用无符号相减，毫秒计数绕回也不会误判。
bool DBus::is_alive(uint32_t now_ms) const { return is_open() && (now_ms - last_read_ms_ < 100); }

}  // namespace sp
