// [解析] ==========================================================================
// [解析] 【文件】sp_middleware/io/dbus/dbus.cpp
// [解析] 【作用】dbus.hpp 里那些函数的具体实现：发起串口接收 + 把 18 字节原始数据解析成物理量。
// [解析] 【被谁调用】applications/uart_task.cpp 里的 HAL_UARTEx_RxEventCallback() 回调
// [解析]           （即中断上下文）会调用 update()；上电时任务里调用一次 request()。
// [解析] 【看代码的重点】三处最容易卡住新手，已在下面逐行标注：
// [解析]   1) get_stick() 里为什么减 1024、为什么除 660；
// [解析]   2) update() 前半段那一堆 << 和 >> 到底在干什么（DBus 的 11 位位域拼接）；
// [解析]   3) 为什么 update() 要把解析结果先存进同名的局部变量，最后才 this->xxx = xxx。
// [解析] ==========================================================================

#include "dbus.hpp"

#include <cmath>
// [解析] ↑ 只是为了用 std::abs() 做"数据合法性检查"（判断摇杆值有没有超出 ±1）。

namespace sp
{
// [解析] ★ get_stick()：把遥控器发来的 11 位无符号整数，换算成 [-1, 1] 的摇杆浮点值。
// [解析]   raw 的取值范围是 0~2047（11 位，见下面 update() 里 & 0x07ff 的掩码）。
// [解析]   -1024：遥控器的"机械中位"对应的原始值是 1024，减掉它，摇杆回中时结果就是 0。
// [解析]   / 660.0f：660 是官方文档给的"中位到满量程的偏移量"，也就是推到极限时
// [解析]     原始值大约变成 1024 + 660 = 1684 或 1024 - 660 = 364，除以 660 后正好约等于 ±1。
// [解析]   注意 660 是"典型行程"而不是硬件极限：如果真把摇杆推到 2047，算出来是 (2047-1024)/660 ≈ 1.55，
// [解析]     会超过 1 —— 这正是下面 update() 里要做 |ch| > 1 合法性检查的原因。
// [解析]   写成 660.0f 而不是 660，是为了强制用 float 做除法（否则整数除法会把结果直接变成 0）。
static float get_stick(uint16_t raw) { return (raw - 1024) / 660.0f; }

// [解析] get_switch()：把拨杆的 2 位原始值(0~3)翻译成"上/中/下"三挡。
// [解析] 遥控器实际发出的编码是：1 = 上(UP)，3 = 中(MID)，2 = 下(DOWN)。
// [解析] ★ 这个"1/3/2"的顺序不是笔误，是 DBus 协议的原始定义（上=1、中=3、下=2），照抄官方即可。
// [解析] ★ 注意最后的 else 把"除 1 和 3 以外的一切值"都判成 DOWN，包括合法的 2，
// [解析]   也包括 0 这种不可能出现的异常值。这是一种"兜底"写法：宁可当成最安全的"下"，
// [解析]   也不要让拨杆处于一个未定义状态。
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
  // [解析] 初始化列表里把三个成员初始化好：串口句柄、接收方式、以及"还没收到过数据"。
  // [解析] 函数体是空的 —— 构造函数刻意不做任何硬件动作，真正的启动在 request() 里。
  // [解析] 注意 last_read_ms_ 和 buff_[18] 没有显式初始化：
  // [解析]   buff_ 会被 DMA/中断直接覆盖，不初始化没关系；
  // [解析]   last_read_ms_ 在第一次 update() 里才被赋值，而在此之前 has_read_ 是 false，
  // [解析]   is_alive() 会先判断 has_read_ 再决定要不要用 last_read_ms_，所以不会读到脏值。
}

// [解析] request()：发起一次"接收 18 个字节"的请求。它是异步的，函数返回时数据还没收完。
// [解析] 收满 18 字节（或串口总线空闲）后，HAL 会调用 HAL_UARTEx_RxEventCallback()，
// [解析]   你在那个回调里调用 update() 解析、然后再次 request() 接力，如此周而复始。
void DBus::request()
{
  // [解析] 两种接收方式二选一，由构造函数的 use_dma 决定（对应 C 板 / 达妙板的不同接线）。
  if (use_dma_) {
    // dismiss return
    // [解析] DMA 方式：串口收到的字节由 DMA 搬运到 buff_，CPU 完全不参与，最省中断时间。
    // [解析] ReceiveToIdle 的意思是"收到 18 字节 或 总线空闲一段时间"就认为一帧结束并触发回调，
    // [解析]   这样即使对方发的字节数略少也能及时把已收到的部分交出来，不会死等。
    HAL_UARTEx_ReceiveToIdle_DMA(huart_, buff_, DBUS_BUFF_SIZE);

    // ref: https://github.com/HNUYueLuRM/basic_framework/blob/master/bsp/usart/bsp_usart.c
    // [解析] ★ 这一行是"关掉 DMA 半传输中断(HT)"，是本文件最容易被忽略的一步，值得讲清楚：
    // [解析]   HAL_UARTEx_ReceiveToIdle_DMA 默认还会在"缓冲区收到一半"(即 18/2 = 9 字节)时
    // [解析]     也触发一次 RxEventCallback，此时回调收到的 Size 是 9。
    // [解析]   虽然 update() 里有 size != 18 就丢弃的保护，不会解析出错误数据，
    // [解析]     但白白多进一次中断，而且容易让人在调试时看到"莫名其妙的 9"。
    // [解析]   所以这里直接把半传输中断关掉，只保留"收满/空闲"这一种触发。
    // [解析]   __HAL_DMA_DISABLE_IT 是 HAL 提供的宏，直接操作 DMA 寄存器，等价于
    // [解析]     hdma->Instance->CR &= ~DMA_IT_HT; （即把 CR 寄存器里的 HT 中断使能位清零）。
    __HAL_DMA_DISABLE_IT(huart_->hdmarx, DMA_IT_HT);
  }
  else {
    // dismiss return
    // [解析] 中断方式（不使用 DMA）：每收到一个字节进一次中断，由 HAL 帮你拼成完整的一帧再回调。
    // [解析]   功能上和上面等价，只是 CPU 开销大一些。达妙板上就是这么用的。
    HAL_UARTEx_ReceiveToIdle_IT(huart_, buff_, DBUS_BUFF_SIZE);
  }
}

// [解析] ★ update()：整个驱动最核心也最难读的函数，作用是"把 18 个字节翻译成人话"。
// [解析] 【DBus 的 18 字节是怎么排的】（这是理解下面所有移位的唯一钥匙）
// [解析]   第 0~5 字节：4 个摇杆通道，每个通道占 11 位，紧挨着排，完全不照顾字节边界，
// [解析]     所以一个通道经常横跨 2~3 个字节：想要哪个通道，就得"把属于它的那些位从各个字节里抠出来，
// [解析]     再按正确的位序拼到一起"，最后用 0x07ff（= 11 个二进制 1）把多余的高位抹掉。
// [解析]     具体切分（bit 序号从每个字节的最低位算起）：
// [解析]       ch_rh(右水平) = 字节0 的全部 8 位 + 字节1 的 bit0~2
// [解析]       ch_rv(右垂直) = 字节1 的 bit3~7 + 字节2 的 bit0~5
// [解析]       ch_lh(左水平) = 字节2 的 bit6~7 + 字节3 的全部 8 位 + 字节4 的 bit0
// [解析]       ch_lv(左垂直) = 字节4 的 bit1~7 + 字节5 的 bit0~3
// [解析]       字节5 剩下的 bit4~5 = 右拨杆 sw_r，bit6~7 = 左拨杆 sw_l
// [解析]     第 6~15 字节：鼠标 x/y/滚轮 各 2 字节（小端），左右键各 1 字节，键盘 2 字节
// [解析]     第 16~17 字节：拨轮 ch_lu（11 位，小端）
// [解析]   记住这个排布后，下面每一行的 |= 和 << 就都变成"按图索骥"了。
void DBus::update(uint16_t size, uint32_t stamp_ms)
{
  // [解析] 长度必须正好 18 字节才解析。少一字节后面全错位、多一字节说明串口有噪声，
  // [解析]   两种情况都宁可整帧丢掉，等下一帧好数据，也不要拿脏数据去控制机器人。
  if (size != DBUS_BUFF_SIZE) return;

  // [解析] 这两行放在最前面（在数据合法性检查之前）是有意为之：
  // [解析]   只要串口还在源源不断来数据，就说明"接收机还活着"，即使内容因为干扰暂时不对。
  // [解析]   is_open() / is_alive() 就是靠这两个变量判断的。
  has_read_ = true;
  last_read_ms_ = stamp_ms;

  // 遥控器解析
  // [解析] ↓ 这就是上面说的"位域拼接"，逐行拆开看：
  // [解析]   buff_[0] | (buff_[1] << 8)：先把字节0、字节1 拼成一个 16 位整数（字节1 是高 8 位），
  // [解析]     这时 ch_rh 的 11 位在最右边，但字节1 的高 5 位是"下一个通道"的内容，
  // [解析]     所以最后 & 0x07ff 把这 11 位以外的东西统统抹掉。
  float ch_rh = get_stick((buff_[0] | (buff_[1] << 8)) & 0x07ff);
  // [解析] ch_rv 从字节1 的 bit3 开始：先 >> 3 把它挪到最右边，再把字节2 整体 << 5 接在后面，
  // [解析]   同样 & 0x07ff 只留 11 位。
  float ch_rv = get_stick(((buff_[1] >> 3) | (buff_[2] << 5)) & 0x07ff);
  // [解析] ch_lh 跨了三个字节，所以要拼三段：
  // [解析]   字节2 >> 6 —— 取它的最高 2 位，放在最低位；
  // [解析]   字节3 << 2 —— 全部 8 位接在那 2 位后面；
  // [解析]   字节4 << 10 —— 只需要它的 bit0，接到最上面凑满 11 位。
  float ch_lh = get_stick(((buff_[2] >> 6) | (buff_[3] << 2) | (buff_[4] << 10)) & 0x07ff);
  // [解析] ch_lv：字节4 从 bit1 起（>> 1），字节5 的低 4 位接在后面（<< 7，因为前面已经有 7 位了）。
  float ch_lv = get_stick(((buff_[4] >> 1) | (buff_[5] << 7)) & 0x07ff);
  // [解析] ch_lu 是拨轮，协议把它单独放在最后的第 16、17 字节，所以拼法很规矩（小端 16 位再取 11 位）。
  // [解析]   ⚠ 注意：它没有参与下面的"摇杆越界检查"，见后面那条 注意 说明。
  float ch_lu = get_stick(((buff_[16] | (buff_[17] << 8)) & 0x07ff));

  // [解析] 两个拨杆藏在字节5 的高 4 位里，每个占 2 位。
  // [解析]   右拨杆 = 字节5 的 bit4~5：先 >> 4 挪到最低位，再 & 0x0003 取 2 位。
  DBusSwitchMode sw_r = get_switch((buff_[5] >> 4) & 0x0003);
  // [解析]   左拨杆 = 字节5 的 bit6~7：这里先 >> 4 挪一次，再用 & 0x000C 取"第 2、3 位"，
  // [解析]     最后 >> 2 才落到 0~3。绕了一圈，等价于更直观的写法 (buff_[5] >> 6) & 0x03。
  DBusSwitchMode sw_l = get_switch(((buff_[5] >> 4) & 0x000C) >> 2);

  // 遥控器数据异常
  // [解析] 合法性检查：get_stick() 的换算结果正常应该落在 [-1, 1] 里，
  // [解析]   如果超了，说明收到的 18 字节不是"从帧头开始"的（比如串口中途丢字节导致整帧错位），
  // [解析]   这时解析出来的一切都不可信，直接 return 丢弃这一帧。
  // [解析] 注意：这里只检查了 4 个摇杆，没有检查 ch_lu（拨轮）—— 拨轮如果错位可能出现 1.5 之类的值。
  if (std::abs(ch_rh) > 1 || std::abs(ch_rv) > 1 || std::abs(ch_lh) > 1 || std::abs(ch_lv) > 1)
    return;

  // 键鼠解析
  // [解析] ↓ 鼠标三个分量各占 2 字节，而且是"低字节在前"（小端）。
  // [解析]   写法 (buff_[7] << 8) | buff_[6] 就是把后一个字节抬到高 8 位再和低字节合并 ——
  // [解析]   顺序看起来和内存里相反，但算出来的值是对的。
  // [解析] 类型是 int16_t 而不是 uint16_t：鼠标往左/往上时数值是负数，
  // [解析]   用无符号会得到一个 6 万多的巨大正数。因为 int16_t 是"有符号补码"，
  // [解析]   当高字节 >= 0x80 时自动变成负数（这一步依赖编译器的补码实现，ARM GCC 上没问题）。
  int16_t mouse_vx_int = (buff_[7] << 8) | buff_[6];
  int16_t mouse_vy_int = (buff_[9] << 8) | buff_[8];
  int16_t mouse_vs_int = (buff_[11] << 8) | buff_[10];
  // [解析] 除以 32768.0f 把 int16 归一化到 [-1, 1)：int16 的最小值是 -32768，
  // [解析]   所以 -32768 / 32768 = -1.0，而最大值 32767 / 32768 ≈ 0.99997，永远够不到 1。
  // [解析]   32768 是 2 的 15 次方，就是 int16 的取值范围大小（写 .0f 同样是为了走浮点除法）。
  float mouse_vx = mouse_vx_int / 32768.0f;
  float mouse_vy = mouse_vy_int / 32768.0f;
  float mouse_vs = mouse_vs_int / 32768.0f;

  // [解析] 键盘也是小端两个字节，拼成 16 位整数：16 个位对应 16 个按键（掩码见 dbus.hpp）。
  uint16_t keyboard_value = (buff_[15] << 8) | buff_[14];

  // 鼠标数据异常
  // [解析] 思路和上面的摇杆检查一样：归一化结果理应落在 [-1, 1] 内。
  // [解析] 注意：这条判断实际上永远不会成立 —— 因为分母 32768 恰好等于 |int16 最小值|，
  // [解析]   任何 int16 除以它都不超过 1，所以 std::abs(...) > 1 恒为 false。
  // [解析]   换句话说这是一条"求个心安"的检查，并没有真正的过滤作用（代码没错，只是没用）。
  // [解析]   真正能拦住坏帧的是最上面那句 size != 18。
  if (std::abs(mouse_vx) > 1 || std::abs(mouse_vy) > 1 || std::abs(mouse_vs) > 1) return;

  /// 更新公有属性

  // [解析] ★ 为什么前面解析时要用"和成员变量同名的局部变量"，到这里再 this->xxx = xxx 抄一遍？
  // [解析]   答案就是上面那两道"数据异常"检查：如果解析到一半发现数据不可信，函数会直接 return，
  // [解析]     此时公有成员变量一个都没被改动，上层任务读到的仍然是"上一帧的好数据"，
  // [解析]     而不会出现"摇杆是新的、鼠标还是旧的"这种半新半旧的脏状态。
  // [解析]   这是一步一步"先全部算完、校验通过、再一次性提交"的经典写法，值得学。
  // [解析]   代价就是这里必须显式写 this->，否则赋值的只是那个同名的局部变量，等于白算。

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

  // [解析] 鼠标按键：协议规定 1 = 按下、0 = 松开，所以直接和 1 比较。
  // [解析]   这里不是"位"而是整字节的值，和键盘那种位掩码不是一回事，别混。
  this->mouse.left = (buff_[12] == 1);
  this->mouse.right = (buff_[13] == 1);
  // [解析] 顺手把 16 位原始值也公开出去，方便调试时直接看十六进制。
  this->keyboard_value = keyboard_value;

  // [解析] ↓ 下面 16 行是"把 16 位原始值拆成 16 个 bool"。
  // [解析]   写法固定为 ((keyboard_value & 掩码) != 0)：
  // [解析]     & 掩码 之后，如果那个位是 1，结果就是一个非零值；如果那个位是 0，结果就是 0。
  // [解析]     != 0 把"非零"统一变成 true，就是我们要的"这个键按下去了"。
  // [解析]   也可以写成 (keyboard_value & 掩码) ? true : false，效果一样，只是第一种更省事。
  // [解析]   各掩码对应的按键见 dbus.hpp 里 DBUS_KEY_xxx_MASK 的注释。
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

// [解析] is_open()：是否"曾经成功收到过一帧"。
// [解析]   因为是 const 成员函数，所以只能在后面加 const —— 表示"调用它不会改动对象"。
// [解析]   它读的是 update() 在第一帧时置上的 has_read_。
bool DBus::is_open() const { return has_read_; }

// [解析] is_alive()：判断遥控器现在是否还在正常发数据（掉线保护的关键函数）。
// [解析] 两个条件是"与"关系：
// [解析]   is_open()         —— 必须已经收到过至少一帧，否则谈不上"还活着"。
// [解析]   now_ms - last_read_ms_ < 100 —— 距最后一帧不到 100ms 才算活着。
// [解析] ★ 这里用"无符号减法"判断时间差，是本项目（以及大多数嵌入式工程）的标准写法。
// [解析]   好处：即使 32 位毫秒计数绕回溢出（约 49.7 天一次），相减的结果依然正确，
// [解析]   所以千万不要改写成 last_read_ms_ + 100 > now_ms 这种形式，那样绕回时就会误判。
// [解析] ★ 100ms 这个阈值：DBus 大约每 14ms 来一帧，100ms ≈ 连续丢 7 帧，够宽松又不至于迟钝。
// [解析]   想让"掉线停车"反应更快可以把这个数调小，但注意别小于 14ms 否则会频繁误报。
bool DBus::is_alive(uint32_t now_ms) const { return is_open() && (now_ms - last_read_ms_ < 100); }

}  // namespace sp
