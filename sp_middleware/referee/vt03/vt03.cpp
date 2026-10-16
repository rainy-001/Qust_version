// [解析] ==========================================================================
// [解析] 【文件】sp_middleware/referee/vt03/vt03.cpp
// [解析] 【作用】vt03.hpp 里 VT03 类的实现：接收、双协议拆帧、位域换算、组帧发送。
// [解析] 【看代码的重点】四处最容易看不懂，下面都会逐段标注：
// [解析]   1) update() 为什么要"递归"调用自己 —— 一个缓冲区里可能粘了好几帧；
// [解析]   2) 两种协议的区分方法：先看 0xA9 0x53（遥控器私有帧），再看 0xA5（裁判系统帧）；
// [解析]   3) 摇杆换算 (x - 1024) / 660.0f 里 1024 和 660 是哪来的；
// [解析]   4) send_custom_client_data() 里"先填帧头、算 CRC8、再填数据、最后算 CRC16"的顺序为什么不能乱。
// [解析] 【协议速查】结构体布局见 vt03.hpp；裁判系统通用帧格式见 referee_protocol.hpp。
// [解析] ==========================================================================

#include "vt03.hpp"

#include "tools/crc/crc.hpp"

namespace sp
{
// [解析] 构造函数用了长长的初始化列表，把所有成员一次性"给个安全初值"。
// [解析] 为什么非得用初始化列表？因为 use_dma_ 是 const 成员，只能在初始化列表里赋值。
// [解析] 各项含义：
// [解析]   huart(huart)          —— 保存串口句柄
// [解析]   mode(VT03Mode::C)     —— 默认档位 C
// [解析]   ch_*(0.0f) / wheel   —— 摇杆默认在中位（归一化后 0）
// [解析]   fn_l/fn_r/pause/trigger(false) —— 按键默认松开
// [解析]   custom() / robot() / keys() / custom_client() —— 值初始化，成员全部清零
// [解析]   mouse{0.0f, 0.0f, 0.0f, false, false, false} —— 按成员顺序依次给 vx/vy/vs/left/middle/right
// [解析]   keyboard_value(0)    —— 键盘掩码清零
// [解析]   seq_(0)              —— 发送序号从 0 开始
// [解析] ⚠ 小提示：mouse 用的是花括号逐个列值，一旦将来 VT03MouseData 增加成员，
// [解析]   这里必须同步补上（否则报错或少初始化）；用 mouse{} 会更省事，但那就改了代码。
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
  // [解析] 把 255 字节的接收缓冲全部填 0。
  // [解析] 目的：防止上电瞬间 buff_ 里是随机的栈/堆垃圾，被误当成一帧数据解析。
  // [解析] std::fill(begin(), end(), 0) 就是"从头到尾每个字节写 0"。
  std::fill(buff_.begin(), buff_.end(), 0);
}

// [解析] "开机以来是否收到过合法的遥控器帧"。
bool VT03::is_open() const { return has_read_; }

// [解析] 遥控器是否在线：收到过 && 距今不到 100ms。
// [解析] 100ms 的来历：遥控器上报率在 100Hz 以上（每 10ms 一帧），
// [解析]   允许连丢约 10 帧才判掉线；比这更宽松就会让"失控保护"反应太慢。
bool VT03::is_alive(uint32_t now_ms) const { return is_open() && (now_ms - last_read_ms_ < 100); }

// [解析] "开机以来是否收到过合法的 0x0302 帧"（也就是图传业务数据通道通不通）。
bool VT03::custom_2_robot_is_open() const { return custom_2_robot_has_read_; }

// [解析] 0x0302 通道是否还在刷新：收到过 && 距今不到 100ms。
// [解析] 函数名的拼写（robotis 缺下划线）是原代码就这样，不要顺手"修正"，否则调用方全部编不过。
bool VT03::custom_2_robotis_alive(uint32_t now_ms) const
{
  return custom_2_robot_is_open() && (now_ms - custom_2_robot_last_read_ms_ < 100);
}

// [解析] 启动接收。用法：初始化后调用一次，并且在每次处理完收到的数据后再调用一次。
// [解析] buff_.data() 取首地址，buff_.size() = 255，所以最多一次收 255 字节。
void VT03::request()
{
  // [解析] 两条分支与 VT02::request() 完全同构，区别只是缓冲区换成了 std::array。
  if (use_dma_) {
    // [解析] DMA 模式：硬件自动把收到的字节搬进 buff_，收满 255 或串口空闲时产生事件回调。
    HAL_UARTEx_ReceiveToIdle_DMA(this->huart, buff_.data(), buff_.size());
    // ref: https://github.com/HNUYueLuRM/basic_framework/blob/master/bsp/usart/bsp_usart.c
    // [解析] 关掉"半传输中断"（HT）。255 字节缓冲在收到一半（约 127 字节）时会误触发一次回调，
    // [解析]   而此时往往一帧还没收完，进回调解析只会失败；关掉后只在"收满/空闲"时才回调。
    __HAL_DMA_DISABLE_IT(this->huart->hdmarx, DMA_IT_HT);
  }
  else {
    // [解析] 中断模式：每来一个字节进一次中断，CPU 负担更大但不需要配 DMA。
    HAL_UARTEx_ReceiveToIdle_IT(this->huart, buff_.data(), buff_.size());
  }
}

// [解析] 公开的解析入口：把内部缓冲 buff_ 和长度、时间戳转交给真正的实现。
// [解析] 之所以要这层"转发"，是因为内部实现要支持"从缓冲中间某个位置继续解析"（递归用），
// [解析]   所以它需要一个"起始指针"参数，而外部调用者只知道自己的 buff_。
void VT03::update(uint16_t size, uint32_t stamp_ms) { update(buff_.data(), size, stamp_ms); }

// [解析] ★核心函数★ 真正的解析逻辑。参数：
// [解析]   frame_start —— 本次要解析的这一帧的起始地址（可能不是 buff_ 开头，见递归调用）
// [解析]   size        —— 从 frame_start 开始还有多少字节可用
// [解析]   stamp_ms    —— 当前毫秒时间戳，解析成功时写进 last_read_ms_
// [解析] 整体结构是"先试协议A，再试协议B"，最后无论哪种都递归解析剩下的字节。
void VT03::update(uint8_t * frame_start, uint16_t size, uint32_t stamp_ms)
{
  // [解析] 注意：这两行在所有 return 之前，也就是"只要函数被调用过，就认为链路是活的"。
  // [解析]   含义是 has_read_ 代表"串口收到过东西"，而不是"收到过校验正确的帧"。
  // [解析]   ⚠ 副作用：即使数据全错、CRC 全不过，is_open()/is_alive() 也会返回 true。
  // [解析]   如果希望"只有校验通过才算活着"，就得把这两行挪到校验成功之后（属于逻辑改动，这里不动）。
  has_read_ = true;
  last_read_ms_ = stamp_ms;
  // [解析] 协议 A：遥控器私有帧。用两个字节 0xA9 0x53 当"magic number"（固定的识别标志）。
  // [解析]   之所以要两个字节而不是一个，是为了降低误判概率：单个 0xA9 出现在数据中间的概率不低。
  // [解析] ⚠ 注意下面直接读了 frame_start[1]，如果 size == 0 会越界读一个字节；
  // [解析]   实际调用时由 HAL 保证 size >= 1（有事件才有数据），但严格来说应该先判 size >= 2。
  if (frame_start[0] == 0xA9 && frame_start[1] == 0x53) {
    // [解析] 遥控器帧是定长的，长度直接由结构体大小决定（21 字节），不需要从帧里读长度。
    size_t frame_len = sizeof(VT03RemoteData);

    // [解析] 长度检查：剩余字节不够一帧，说明这帧被截断了，直接放弃（不能硬解析）。
    if (size < frame_len) return;
    // [解析] CRC16 校验：帧头和数据全错都会在这里被拦下来，是保证数据可信的关键一步。
    // [解析] 注意 len 传的是整帧长度（包含 CRC16 自己那两个字节），具体覆盖方式见 check_crc16 实现。
    if (!check_crc16(frame_start, frame_len)) return;

    // [解析] reinterpret_cast：把"字节指针"强行当成"VT03RemoteData 指针"用。
    // [解析]   因为结构体是 packed 且布局与协议一致，所以这样读出来的位域就是正确值。
    // [解析]   这也是为什么 packed 绝对不能删 —— 少了它，位域/成员偏移一变，这里读到的就是错位数据。
    update_remote(reinterpret_cast<VT03RemoteData *>(frame_start));

    // 递归解析, 因为缓冲区中可能包含多帧裁判系统的数据
    // [解析] 关键技巧：frame_start + frame_len 跳到"下一帧的开头"，size - frame_len 是剩余长度。
    // [解析]   一帧处理完就交给"下一轮的自己"，直到剩余数据不够一帧时被上面的长度检查挡回去。
    // [解析]   效果等价于一个 while 循环，只是写成递归更简洁（帧数很少，不必担心栈溢出）。
    update(frame_start + frame_len, size - frame_len, stamp_ms);
    // [解析] 必须 return：这段数据已经按协议 A 处理完了，不能再去按协议 B 解析一遍。
    return;
  }

  // [解析] 到这里说明不是遥控器帧，接着按协议 B（裁判系统标准帧）解析。
  // [解析] 标准帧的识别字节是 0xA5（referee::SOF），帧格式见 referee_protocol.hpp：
  // [解析]   [0]=0xA5 [1..2]=数据长度(小端) [3]=seq [4]=CRC8 [5..6]=cmd_id(小端) [7..]=数据 [末尾2]=CRC16
  // [解析] 这里没有校验 crc16，只是保证"帧头 5 字节够长"。
  // [解析] 先判断剩余长度够不够一个帧头，不够就直接返回（避免越界读 frame_start[2..6]）。
  if (size < referee::HEAD_LEN) return;
  // [解析] 帧头第一字节必须是 0xA5，否则说明这段数据完全不是裁判系统帧，放弃。
  if (frame_start[0] != referee::SOF) return;
  // [解析] CRC8 校验帧头。裁判系统把帧头单独做了 CRC8，因为"数据长度"就在帧头里 ——
  // [解析]   帧头不校验就没法安全地按长度取整帧，所以这一步必须在读 data_len 之前做。
  if (!check_crc8(frame_start, referee::HEAD_LEN)) return;

  // [解析] 取数据段长度。小端序：frame_start[1] 是低字节、frame_start[2] 是高字节，
  // [解析]   所以写成 (高 << 8) | 低，即 (frame_start[2] << 8) | frame_start[1]。
  size_t data_len = (frame_start[2] << 8) | frame_start[1];
  // [解析] 整帧长度 = 帧头5 + 命令码2 + 数据段 + 帧尾2(CRC16)。
  size_t frame_len = referee::HEAD_LEN + referee::CMD_ID_LEN + data_len + referee::TAIL_LEN;

  // [解析] 长度检查：剩余字节不足一整帧就放弃（可能被截断，或只是还没收完）。
  if (size < frame_len) return;
  // [解析] 整帧 CRC16 校验，覆盖"帧头 + 命令码 + 数据段"以及 CRC16 自身两个字节。
  if (!check_crc16(frame_start, frame_len)) return;

  // [解析] 取命令码 cmd_id，同样是小端：(高字节 << 8) | 低字节。
  // [解析] 比如 0x0302 在字节流里是 0x02 0x03，拼起来才是 0x0302。
  uint16_t cmd_id = (frame_start[6] << 8) | frame_start[5];

  // [解析] switch 分发：只处理本项目关心的三种命令码，其他一律忽略（default 什么都不做）。
  switch (cmd_id) {
    // 0x0302 自定义控制器与机器人交互数据
    case referee::cmd_id::CUSTOM_ROBOT_DATA:
      // [解析] ★关键的长度校验★：数据段长度必须和结构体大小完全相等（30 字节）才拷贝。
      // [解析]   如果不等就 break（不拷贝），这样既不会读越界，也不会写坏结构体。
      // [解析]   对比 vt02.cpp 里没有这个判断，属于本文件的更严谨之处。
      if (data_len != sizeof(this->custom)) break;
      // [解析] 把数据段原样搬进 custom 结构体。
      // [解析] reinterpret_cast<uint8_t*>(&custom)：把结构体地址转成"字节指针"，
      // [解析]   因为 memcpy 是按字节拷贝的，必须给字节指针。
      // [解析] 源地址 frame_start + referee::DATA_START 正好跳过帧头+命令码，指向数据段首字节。
      // [解析] 长度用 sizeof(this->custom) 而不是 data_len —— 两者相等（上一行已校验），
      // [解析]   但用 sizeof 更能表达"拷满整个结构体"，也更抗改动。
      std::memcpy(
        reinterpret_cast<uint8_t *>(&this->custom), frame_start + referee::DATA_START,
        sizeof(this->custom));
      // [解析] 这一路数据校验通过且拷贝完成，才刷新"自定义控制器通道"的在线时间戳。
      // [解析]   注意：刷新的是 custom_2_robot_* 这组变量，不是上面的 has_read_ / last_read_ms_，
      // [解析]   所以判断 0x0302 通道是否在线要用 custom_2_robotis_alive()。
      custom_2_robot_last_read_ms_ = stamp_ms;
      custom_2_robot_has_read_ = true;
      break;

    // 0x0309 自定义控制器与机器人交互数据
    // [解析] 这是"机器人 -> 自定义控制器"方向的数据（机器人上报自己的状态），
    // [解析]   解完放在 this->robot 里，供上层转发给图传上位机显示。
    case referee::cmd_id::ROBOT_CUSTOM_DATA:
      // [解析] 同样先校验长度（应为 30 字节），不一致就丢弃，防止越界/错位。
      if (data_len != sizeof(this->robot)) break;
      // [解析] 拷贝数据段到 robot 结构体（和上面 0x0302 的处理方式完全一致）。
      std::memcpy(
        reinterpret_cast<uint8_t *>(&this->robot), frame_start + referee::DATA_START,
        sizeof(this->robot));
      break;

    //0x0311 自定义客户端发送给机器人的自定义指令
    // [解析] 这条是"变长"的（0~30 字节），所以不能用固定长度 memcpy，
    // [解析]   必须借助 custom_client.size 记录实际长度，并先用 is_valid 标出有效性。
    case referee::cmd_id::CLIENT_ROBOT_DATA:
      // [解析] 先把 data 数组整体清 0，再决定填什么。
      // [解析]   目的：如果本次数据比上次短，残留的旧字节会变成"幽灵数据"，清零可彻底避免。
      std::fill(this->custom_client.data.begin(), this->custom_client.data.end(), 0);

      // [解析] 溢出保护：协议规定最大 30 字节，万一收到超过 30 字节的帧，
      // [解析]   就标记"无效、长度 0"并退出，绝不去写超出 30 字节的数组（防缓冲区溢出）。
      // [解析]   这是本文件里最值得学的安全写法。
      if (data_len > this->custom_client.data.size()) {
        this->custom_client.size = 0;
        this->custom_client.is_valid = false;
        break;
      }

      // [解析] 长度合法：记录实际字节数并置有效标志。
      // [解析]   static_cast<uint8_t> 是显式窄化转换：data_len 是 size_t(32/64 位)，
      // [解析]   这里已经确认它 <= 30，所以转成 uint8_t 不会丢数据（编译器看到强转就不会警告）。
      this->custom_client.size = static_cast<uint8_t>(data_len);
      this->custom_client.is_valid = true;
      // [解析] 拷贝实际长度（data_len）而不是整个 30 字节：这样 size 之外的字节保持为 0，
      // [解析]   上层按 custom_client.size 读多少就是多少，不会读到无意义的填充。
      std::memcpy(this->custom_client.data.data(), frame_start + referee::DATA_START, data_len);
      break;

    default:
      // [解析] 不认识的命令码：什么都不做（既不报错也不清数据），
      // [解析]   这样即使裁判系统新增了命令码，也不会影响已有功能。
      break;
  }

  // 递归解析, 因为缓冲区中可能包含多帧裁判系统的数据
  // [解析] 和协议 A 分支一样：跳过这一帧，把剩下的字节继续交给下一轮解析，
  // [解析]   直到剩余长度不足一帧（被前面的 size 检查挡下）。
  // [解析]   这一个设计让"一次串口事件收到多帧"也能被全部处理，不会白白丢掉后面的数据。
  update(frame_start + frame_len, size - frame_len, stamp_ms);
}

// [解析] ★换算函数★ 把遥控器私有帧里的原始值翻译成"人能直接用的物理量/布尔量"。
// [解析] 前置条件：调用者必须已经做过 CRC16 校验（update() 里保证了这一点），否则读到的位域是错的。
// [解析] 参数 data：指向已经通过校验的 VT03RemoteData（就是那 21 字节原始帧）。
void VT03::update_remote(const VT03RemoteData * data)
{
  // [解析] ★摇杆换算公式★ (原始值 - 1024) / 660.0f，逐项拆开讲：
  // [解析]   1) 原始值：ADC 量化后的 11 位数据，范围 0 ~ 2047（2^11 - 1）。
  // [解析]   2) 减 1024：机械中位大约在量程中点（2047/2 ≈ 1024）。
  // [解析]      减完之后，中位变 0、往一边最多 +1023、往另一边最多 -1024，正负对称。
  // [解析]   3) 除以 660：把 ±1023 大约压到 ±1.55 的范围，而不是 ±1。
  // [解析]      ⚠ 注意：660 意味着满打满算时值会到约 ±1.55，**并没有被钳制在 [-1,1]**！
  // [解析]      这样做的好处是"摇杆推到底还能再大一点"，代价是上层若要严格 [-1,1] 得自己限幅。
  // [解析]      660 这个数是通过实测"摇杆正常行程大约只能到 ±660"标定出来的，属于经验值。
  // [解析]   4) 除以 660.0f 而不是 660：必须用浮点数，否则整数除法会把结果全部截断成 0 或 ±1。
  // [解析] 通道映射注意：ch_1 -> ch_rv，ch_2 -> ch_lv，ch_3 -> ch_lh，
  // [解析]   也就是"物理通道号"和"左右/上下语义"的对应关系是作者按自己遥控器定的，
  // [解析]   如果换遥控器发现前后左右反了，改的就是这里（而不是改公式）。
  this->ch_rh = (data->ch_0 - 1024) / 660.0f;
  this->ch_rv = (data->ch_1 - 1024) / 660.0f;
  this->ch_lv = (data->ch_2 - 1024) / 660.0f;
  this->ch_lh = (data->ch_3 - 1024) / 660.0f;
  // [解析] 拨轮与摇杆同量纲（也是 0~2047 的 11 位数据），所以用同一个公式。
  this->wheel = (data->wheel - 1024) / 660.0f;

  // [解析] 按键是 1 bit 的位域，直接赋给 bool 即可（0 -> false，1 -> true），不需要换算。
  this->fn_l = data->fn_1;
  this->fn_r = data->fn_2;
  this->pause = data->pause;
  this->trigger = data->trigger;

  // [解析] ★鼠标换算★ 除数是 32768.0f，也就是 int16 的正半量程（2^15）。
  // [解析]   原始值范围 -32768 ~ 32767，除以 32768 后得到约 [-1, 1) 的比例值。
  // [解析]   为什么要归一化？因为鼠标每次上报的是"位移增量"，绝对值取决于 DPI，
  // [解析]   归一化之后，上层的速度环参数就不用跟着鼠标型号改（乘个最大速度就行）。
  // [解析]   注意 -32768 / 32768.0f 正好是 -1.0，而 +32767 / 32768.0f 只有约 0.99997，
  // [解析]   所以严格说是 [-1, 1) 而不是 [-1, 1]，实际使用完全无影响。
  this->mouse.vx = data->mouse_x / 32768.0f;
  this->mouse.vy = data->mouse_y / 32768.0f;
  this->mouse.vs = data->mouse_z / 32768.0f;
  // [解析] 鼠标键是 2 bit 位域，这里直接当 bool 用：只要非 0 就算按下。
  // [解析]   当前协议实际只用 0（松开）/1（按下），所以这样赋值没问题。
  this->mouse.left = data->mouse_left;
  this->mouse.middle = data->mouse_middle;
  this->mouse.right = data->mouse_right;

  // [解析] 先把 16 位原始掩码原封不动存下来，方便上层自己做扩展判断。
  this->keyboard_value = data->keys;

  // [解析] 下面 16 行是"位掩码 -> 布尔量"的标准写法，逐个解释：
  // [解析]   keyboard_value & VT03_KEY_W_MASK —— 按位与，只保留 W 那一位，其他位清 0。
  // [解析]     例如 keyboard_value = 0b...0000_0101（W 和 A 按下），
  // [解析]     与 0x0001 得 0x0001（非 0 -> W 按下）；与 0x0002 得 0x0000（S 没按）。
  // [解析]   != 0 —— 把"非零值"显式转成 true，写成 bool 更清晰；顺便避免编译器警告。
  // [解析]   整体等价于"这一位是 1 就按下，是 0 就松开"，16 个键互不影响，可以同时按下任意组合。
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

  // [解析] 档位开关映射：把 2 bit 的 mode_sw 数字翻译成 VT03Mode 枚举。
  // [解析] 这是嵌套三目运算符，读法是从左往右按条件拆：
  // [解析]   mode_sw == 2 ? S : (mode_sw == 1 ? N : C)
  // [解析]   即 2 -> S，1 -> N，其他（0）-> C。
  // [解析] ⚠ 注意 C 那一支用的是"兜底"而不是 == 0：如果开关接触不良读到 3，也会被当成 C 档。
  // [解析]   好还是坏取决于需求 —— 好处是永远不会落到非法值，坏处是掩盖了硬件异常。
  this->mode = (data->mode_sw == 2)   ? VT03Mode::S
               : (data->mode_sw == 1) ? VT03Mode::N
                                      : VT03Mode::C;
}

// [解析] ★组帧发送★ 把 300 字节的自定义数据按裁判系统格式打包后从串口发出（命令码 0x0310）。
// [解析] 参数 custom_data：要发送的 300 字节数据块（引用传参，避免拷贝 300 字节的开销）。
// [解析] 这一帧的总长度 = 5(帧头) + 2(命令码) + 300(数据) + 2(CRC16) = 309 字节。
// [解析] 副作用：① 内部 seq_ 自增；② 占用串口发送（DMA/中断是异步的，注意别连续高频调用）。
void VT03::send_custom_client_data(const CustomByteBlock & custom_data)
{
  // 静态数组避免频繁在栈上分配内存，320字节通常足够裁判系统单包使用
  // [解析] static 的含义：这块 320 字节只在第一次执行到这里时分配一次（放在静态存储区），
  // [解析]   以后每次调用都复用同一块，不会反复压栈。
  // [解析]   为什么必须 static？因为下面用的是"中断/DMA 发送"：函数返回后数据还得继续被硬件读，
  // [解析]   如果放在栈上（普通局部数组），函数一返回栈就被覆盖，发出去的就是乱码。
  // [解析]   ⚠ 代价：这个函数不可重入（DMA 还在发的时候又调一次会踩同一块缓冲）。
  // [解析]   320 的来历：309 字节的帧 + 一点余量，凑个整数。
  static uint8_t tx_buff[320];

  // [解析] constexpr 是"编译期常量"，编译器会直接在编译时算好，运行时没有任何开销。
  // [解析] data_len = sizeof(CustomByteBlock) = 300（有 static_assert 保证）。
  constexpr size_t data_len = sizeof(CustomByteBlock);
  // [解析] frame_len = 5 + 2 + 300 + 2 = 309，也就是这一帧要发多少字节。
  constexpr size_t frame_len =
    referee::HEAD_LEN + referee::CMD_ID_LEN + data_len + referee::TAIL_LEN;

  // 1. 安全防线：防止缓冲区溢出
  // [解析] 编译期已知 309 <= 320，所以这个判断实际上永远不会成立（编译器甚至可能优化掉）。
  // [解析]   它的价值在于"防御未来"：哪天 CustomByteBlock 被改成 400 字节，这一行会拦住越界写。
  // [解析]   这种"看起来多余但很便宜"的检查，在嵌入式里是鼓励保留的。
  if (frame_len > sizeof(tx_buff)) return;

  // 2. 组装帧头 (Frame Header)
  // [解析] 帧头 5 字节的填写顺序：SOF -> 长度低字节 -> 长度高字节 -> 序号 -> CRC8。
  tx_buff[0] = referee::SOF;
  // [解析] ↑ 起始字节，固定 0xA5。
  // [解析] 长度按"小端"拆成两个字节：& 0xFF 取低 8 位，>> 8 取高 8 位。
  // [解析]   300 = 0x012C，所以 tx_buff[1] = 0x2C，tx_buff[2] = 0x01。
  tx_buff[1] = data_len & 0xFF;
  tx_buff[2] = (data_len >> 8) & 0xFF;
  // [解析] seq_ 是"后置自增"：先把当前值写进帧头，再把自己加 1。
  // [解析]   这样上一行写的是"本次的序号"，seq_ 指向"下次要用的序号"。
  // [解析]   它溢出时（255 -> 0）自然回绕，接收方按"是否连续"判断丢包。
  tx_buff[3] = this->seq_++;
  // [解析] ★顺序很重要★ 必须在"帧头 5 个字节全部填完"之后才算 CRC8，
  // [解析]   而且必须在填数据段之前算 —— 因为裁判系统的 CRC8 只覆盖帧头这 5 个字节。
  // [解析]   append_crc8 会把结果直接写进 tx_buff[4]（帧头最后一个字节）。
  sp::append_crc8(tx_buff, referee::HEAD_LEN);  // 一步追加 CRC8

  // 3. 组装命令码 (Cmd ID = 0x0310)
  // [解析] 0x0310 同样按小端拆：低字节 0x10 在前，高字节 0x03 在后。
  // [解析]   referee::HEAD_LEN = 5，所以下标 5、6 正好紧接帧头。
  // [解析]   这里没有用 cmd_id::ROBOT_CLIENT_DATA 常量而是直接写字面量，效果一样。
  tx_buff[referee::HEAD_LEN] = 0x10;
  tx_buff[referee::HEAD_LEN + 1] = 0x03;

  // 4. 拷贝数据段 (Data)
  // [解析] 把调用者给的 300 字节原样拷到"帧头+命令码"之后。
  // [解析]   &custom_data 取的是结构体地址，而结构体里只有一个 uint8_t[300]，
  // [解析]   所以这里拷贝的就是那 300 个原始字节，不涉及任何字节序转换。
  std::memcpy(tx_buff + referee::DATA_START, &custom_data, data_len);

  // 5. 组装帧尾 (Frame Tail / CRC16)
  // [解析] ★必须在数据段拷完之后才算 CRC16★，因为它要覆盖帧头 + 命令码 + 数据段的所有字节，
  // [解析]   算早了就会漏掉数据段，接收方一定校验失败。
  // [解析]   append_crc16(tx_buff, frame_len) 计算并写入最后 2 个字节（下标 307、308）。
  sp::append_crc16(tx_buff, frame_len);  // 优雅地一步追加 CRC16

  // 6. 物理层发送
  // [解析] 两种发送方式：DMA 由硬件搬运（CPU 占用低、适合长帧），IT 由中断逐字节发。
  // [解析]   两者都是"异步"的：函数立刻返回，数据在后台继续发。
  // [解析]   所以必须用上面那个 static 缓冲，且不能在这个函数里再改 tx_buff 的内容。
  // [解析] ⚠ 这里忽略了 HAL 的返回值（没有 "dismiss return" 注释，属于风格不一致），
  // [解析]   若串口正忙（上一帧还没发完）会直接返回 HAL_BUSY，本帧就丢了。
  // [解析]   如果上层调用频率可能超过串口带宽，建议自行加"发送忙"判断。
  if (use_dma_) {
    HAL_UART_Transmit_DMA(this->huart, tx_buff, frame_len);
  }
  else {
    HAL_UART_Transmit_IT(this->huart, tx_buff, frame_len);
  }
}

}  // namespace sp