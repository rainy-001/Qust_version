// [解析] ==========================================================================
// [解析] 【文件】sp_middleware/io/laser/laser.cpp
// [解析] 【作用】laser.hpp 里声明的那些函数的具体实现：串口发命令 + 逐字节拼帧 + 解析距离。
// [解析] 【阅读顺序】建议按数据流读：
// [解析]   receive_complete()  收到一批字节 —— 入口
// [解析]     -> update()       逐字节维护 8 字节滑动窗口（本文件最烧脑的地方）
// [解析]       -> frame_valid() 帧头 + 累加和校验
// [解析]         -> parse_frame() 把字节拆成距离/模式，并盖时间戳
// [解析] 【三个最容易踩的坑，已在下面逐处标注】
// [解析]   1) update() 里滑动窗口"满了还要继续挪"是在干什么；
// [解析]   2) parse_frame() 里距离是两个字节拼出来的大端数，不是直接赋值；
// [解析]   3) parse_frame() 里状态字节 frame_[6] 同时塞了"故障码"和"模式"，要用位运算分别取。
// [解析] ==========================================================================

#include "laser.hpp"

namespace sp
{
// [解析] 构造函数：": 成员(初值)" 这种写法叫"初始化列表"。
// [解析] 真正执行顺序是"按成员在类里的声明顺序"，不是按这里书写的顺序，
// [解析] 所以这里特意也按声明顺序排，免得看代码时误判。
// [解析] rx_buff_{0} / frame_{0} 是把整个数组所有元素都置 0 的惯用简写。
LaserGY56::LaserGY56(UART_HandleTypeDef * huart)
: huart_(huart),                // [解析] 记住用哪一路串口，后面所有 HAL 调用都靠它
  rx_buff_{0},                  // [解析] DMA 接收缓冲区清零（DMA 会直接往它里面写）
  frame_{0},                    // [解析] 滑动窗口清零
  frame_size_(0),               // [解析] 窗口里目前 0 个字节
  distance_cm_(0),              // [解析] 距离初值 0（只是占位，必须配合 is_valid() 才有意义）
  last_frame_tick_(0),          // [解析] 还没收到过帧，时间戳为 0
  mode_(LaserMode::UNKNOWN),    // [解析] 模式未知
  has_reliable_distance_(false) // [解析] 还没有任何可信数据
{
  // [解析] 函数体空：这里不碰硬件。"开始接收"是后面 start_receive() 的活。
}

// [解析] 最"便宜"的取数函数：直接返回缓存的距离值，不校验新鲜度。
// [解析] 所以调用方必须自己先问一句 is_valid()。
uint16_t LaserGY56::get_distance() const { return distance_cm_; }

// [解析] 数据新鲜度判断。三步逻辑：
// [解析]   1) 一帧有效数据都没收到过 -> 直接不算数；
// [解析]   2) 用当前毫秒数减去最后一帧的时间戳，得到"数据放了多久"；
// [解析]   3) 小于等于容忍值 timeout_ms 才算新鲜。
// [解析] HAL_GetTick() 从 FreeRTOS 启动开始按 1 ms 递增，约 49.7 天后回绕归零；
// [解析] 两个 uint32_t 相减即使发生回绕，结果在"小时间差"场景下依然正确，所以安全。
bool LaserGY56::is_valid(uint32_t timeout_ms) const
{
  if (!has_reliable_distance_) {
    return false;
  }

  return HAL_GetTick() - last_frame_tick_ <= timeout_ms;
}

LaserMode LaserGY56::get_mode() const { return mode_; }

// [解析] 把"人类看的模式枚举"翻译成"模块说明书里的命令字节"，再交给 send_command()。
// [解析] 0x51 / 0x52 / 0x53 依次对应短 / 中 / 长距离模式。
// [解析] ★注意：不同厂家（甚至同厂家不同批次的）说明书中，这三个命令字的语义可能不同，
// [解析]   有的手册里 0x51/0x52/0x53 是"单次测量 / 连续测量 / 停止测量"。
// [解析]   如果实测发现距离不刷新或模式不对，务必先翻本车模块附带的手册核对。
// [解析] default 分支兜住 LaserMode::UNKNOWN 和非法值，返回 false 表示"没发任何东西"。
bool LaserGY56::send_mode(LaserMode mode)
{
  switch (mode) {
    case LaserMode::SHORT:
      return send_command(0x51);
    case LaserMode::MEDIUM:
      return send_command(0x52);
    case LaserMode::LONG:
      return send_command(0x53);
    default:
      return false;
  }
}

// [解析] 改成 9600 波特率（命令字 0xAE）。改完记得再调 save_config() 保存，以及同步改 CubeMX 配置。
bool LaserGY56::set_baud_rate_9600() { return send_command(0xAE); }

// [解析] 改成 115200 波特率（命令字 0xAF）。115200 速度更快，距离帧刷新率能更高。
bool LaserGY56::set_baud_rate_115200() { return send_command(0xAF); }

// [解析] 把当前设置写入模块的掉电不丢失存储区（命令字 0x25），否则断电后设置会丢。
bool LaserGY56::save_config() { return send_command(0x25); }

// [解析] 启动接收。返回 true = 已经在接收（或本次启动成功），false = 启动失败。
// [解析] 三个前置检查分别解释了"为什么可能失败"：
// [解析]   1) huart_ 为空，说明创建对象时传了个空指针，纯属编程错误；
// [解析]      hdmarx 为空，说明 CubeMX 里这个串口没有开 DMA 接收通道 —— 这是最常见的失败原因；
// [解析]   2) RxState == BUSY_RX：说明已经在接收了，直接返回 true，避免重复启动；
// [解析]   3) RxState 既不是 READY 也不是 BUSY_RX（例如正处于 BUSY_TX 发送中），
// [解析]      这时不能强行启动，返回 false 让上层稍后重试。
bool LaserGY56::start_receive()
{
  if (huart_ == nullptr || huart_->hdmarx == nullptr) {
    return false;
  }
  if (huart_->RxState == HAL_UART_STATE_BUSY_RX) {
    return true;
  }
  if (huart_->RxState != HAL_UART_STATE_READY) {
    return false;
  }

  // [解析] HAL_UARTEx_ReceiveToIdle_DMA：DMA 负责把字节往 rx_buff_ 里搬（不占 CPU），
  // [解析] 直到"串口空闲"（一帧结束后的静默）或缓冲区满 64 字节，才触发一次回调，
  // [解析] 回调里会告诉你"这次收了多少字节"。这正是解析变长/不定长串口帧的正确姿势。
  if (HAL_UARTEx_ReceiveToIdle_DMA(huart_, rx_buff_, RX_BUFFER_SIZE) != HAL_OK) {
    return false;
  }

  // 仅在空闲或缓冲区满时处理数据, 避免半满回调重复解析
  // [解析] 上面那行 HAL 调用默认会同时打开"半传输(HT)中断"——收满一半(32 字节)就回调一次。
  // [解析] 这里把 HT 中断关掉，只保留"传输完成 + 空闲"的那次回调，
  // [解析] 否则同一批数据会被回调两次，update() 就会把半截数据当完整数据解析，出现脏帧。
  // [解析] __HAL_DMA_DISABLE_IT 是 HAL 提供的宏，直接改 DMA 寄存器，不返回错误码。
  __HAL_DMA_DISABLE_IT(huart_->hdmarx, DMA_IT_HT);
  return true;
}

// [解析] 回调入口函数：由 HAL 在"串口空闲 / 缓冲区满"时调用（通常运行在中断上下文里）。
// [解析] 参数 size：本次 DMA 实际收到的字节数，单位"字节"，理论范围 0~64。
// [解析] 注意它运行在中断里，所以里面的操作要尽量短、不能阻塞（不能 osDelay、不能等锁）。
void LaserGY56::receive_complete(uint16_t size)
{
  // [解析] 防御性裁剪：正常情况下 size 不会超过 64，但万一句柄被别处改过导致异常，
  // [解析] 这里夹一下，避免下面 for 循环越界读 rx_buff_（数组越界会读到别的变量）。
  if (size > RX_BUFFER_SIZE) {
    size = RX_BUFFER_SIZE;
  }

  // [解析] 把刚收到的这一批字节按"先来后到"顺序逐个交给 update() 拼帧。
  for (uint16_t i = 0; i < size; i++) {
    update(rx_buff_[i]);
  }
  // [解析] 处理完立刻重新挂起 DMA 接收，等待下一批数据。
  // [解析] ★注意：这里没有检查 start_receive() 的返回值。如果此时串口正好在发送
  // [解析]   （RxState 不是 READY），重启会失败，接收就此静默中断，直到有人再调一次。
  // [解析]   稳妥的写法是把返回值向上报，或失败时打个标志让任务里重试。
  start_receive();
}

// [解析] 【本文件最重要的函数】滑动窗口拼帧。
// [解析] 为什么要滑动？串口是一串连续的字节流，模块可能连续吐好几帧，
// [解析] 也可能你启动接收时刚好从某一帧的中间开始 —— 直接按"每 8 字节切一刀"去解析，
// [解析] 只要错位一字节，后面所有帧全错。滑动窗口的做法是：
// [解析] 永远只看"最近 8 个字节"，每来一个新字节就整体左移一格，
// [解析] 只要数据里出现过完整的合法帧，就一定能被对齐抓到，而且能自动重新同步。
void LaserGY56::update(uint8_t byte)
{
  // 维护8字节窗口, 使帧头错位或分段接收时仍能重新找到完整数据帧
  // [解析] 阶段一：窗口还没填满，就顺次往后放（frame_[0]、frame_[1]……）。
  if (frame_size_ < FRAME_SIZE) {
    frame_[frame_size_] = byte;
    frame_size_++;
  }
  // [解析] 阶段二：窗口已经满了，丢掉最老的 frame_[0]，整体左移一格，新字节放末尾。
  // [解析] 这里的 for 循环相当于"把 frame_[1..7] 搬到 frame_[0..6]"。
  else {
    for (uint8_t i = 0; i < FRAME_SIZE - 1; i++) {
      frame_[i] = frame_[i + 1];
    }
    frame_[FRAME_SIZE - 1] = byte;
  }

  // [解析] 窗口没满（还不到 8 字节）就没法校验，直接返回等下一个字节。
  // [解析] !frame_valid() 表示"这 8 个字节不像一帧"（帧头不对或校验和不对），
  // [解析] 也直接返回 —— 反正下一个字节进来会再滑一格，继续尝试对齐。
  if (frame_size_ < FRAME_SIZE || !frame_valid()) {
    return;
  }

  // [解析] 走到这里说明"这 8 个字节是一帧合法数据"，可以解析了。
  parse_frame();
  // [解析] 解析完把 frame_size_ 清 0，让窗口回到"阶段一"。
  // [解析] 为什么？因为帧已经完整消费掉了，下一帧从头开始收集能少做 7 次移位。
  // [解析] 但这也意味着：如果两帧数据粘在同一个 DMA 缓冲里，第二帧要靠"重新填满 8 字节"
  // [解析] 走一遍，仍然是正确的，只是多绕了几次。
  frame_size_ = 0;
}

// [解析] 向模块发送一条命令，返回 true = 发送成功，false = 失败。
// [解析] 本模块的命令格式固定 3 字节：0xA5 + 命令字 + 校验和，
// [解析] 校验和就是前两个字节的"逐字节累加"（0xA5 + command）。
// [解析] ★注意：static_cast<uint8_t>(0xA5 + command) 这一步是必须的。
// [解析]   因为 0xA5 + command 在 C++ 里会先提升成 int 再相加，结果可能超过 255（比如 0xA5+0xAF=0x154），
// [解析]   强制转回 uint8_t 相当于"只保留低 8 位"，也就是自然溢出取模 256（0x154 -> 0x54），
// [解析]   这正是模块期望的校验和算法。少了这个转换，编译器的类型窄化警告可能报错。
bool LaserGY56::send_command(uint8_t command)
{
  if (huart_ == nullptr) {
    return false;
  }

  uint8_t data[3] = {0xA5, command, static_cast<uint8_t>(0xA5 + command)};
  // [解析] HAL_UART_Transmit 是"阻塞式"发送：函数要等 3 个字节全发完才返回。
  // [解析] 最后一个参数 10 是超时时间，单位 ms —— 115200 波特率下发 3 字节不到 0.3 ms，
  // [解析] 给 10 ms 绰绰有余。返回 HAL_OK 才认为成功。
  return HAL_UART_Transmit(huart_, data, sizeof(data), 10) == HAL_OK;
}

// [解析] 校验"frame_ 里的 8 个字节是不是一帧合法数据"。返回 true = 通过。
// [解析] 检查两件事：
// [解析]   1) 帧头：frame_[0]==0x5A、frame_[1]==0x5A（帧起始标志，两个 0x5A 便于对齐）；
// [解析]      frame_[2]==0x15 是模块地址/保留位，frame_[3]==0x03 表示"后面带 3 个数据字节"；
// [解析]   2) 校验和：把前 7 个字节逐个累加（自然溢出到 8 位），结果应等于第 8 个字节。
// [解析] 累加和只能发现"大部分"传输错误（比如两处错误互相抵消就查不出来），
// [解析] 好处是计算极快，适合在中断里跑；想更可靠得用 CRC。
// [解析] 调用前提：必须已经在 update() 里确认 frame_size_ == FRAME_SIZE，
// [解析] 否则 frame_ 后半部分还是初始值 0，会读到无意义的数据。
bool LaserGY56::frame_valid() const
{
  // 检查帧头、数据类型和逐字节累加校验
  if (frame_[0] != 0x5A || frame_[1] != 0x5A || frame_[2] != 0x15 || frame_[3] != 0x03) {
    return false;
  }

  uint8_t checksum = 0;
  // [解析] 只累加前 7 个字节（下标 0~6），第 8 个字节(下标 7)本身就是校验和，不参与累加。
  for (uint8_t i = 0; i < FRAME_SIZE - 1; i++) {
    checksum = static_cast<uint8_t>(checksum + frame_[i]);
  }
  return checksum == frame_[FRAME_SIZE - 1];
}

// [解析] 把已经校验通过的 8 字节解析成"距离 + 模式 + 时间戳"。
// [解析] 帧内布局（下标）：
// [解析]   [0][1] 帧头 0x5A 0x5A | [2] 0x15 | [3] 0x03
// [解析]   [4][5] 距离值（大端：高字节在前）| [6] 状态(高4位)+模式(低2位) | [7] 累加和
void LaserGY56::parse_frame()
{
  // 状态不为0时不更新距离、模式和有效时间
  // [解析] frame_[6] 的高 4 位是"状态/故障码"：0 表示本次测量正常。
  // [解析] (frame_[6] >> 4) & 0x0F 意思是：先右移 4 位把高 4 位挪到低位，再用 0x0F(二进制 1111)
  // [解析] 把低 4 位之外的东西全部屏蔽掉，只留这 4 位。这是读"位域"的标准写法。
  // [解析] ★这样设计的原因：一个字节只有 8 位，模块为了省字节，把"状态"和"模式"挤在同一个字节里。
  // [解析] 遇到异常状态（测量失败、信号太弱等）时整帧作废 —— 不刷新距离和时间戳，
  // [解析] 于是 is_valid() 会因为时间戳变旧而把数据判为"过期"，上层就知道这次数据不能用了。
  uint8_t status = static_cast<uint8_t>((frame_[6] >> 4) & 0x0F);
  if (status != 0) {
    return;
  }

  // 实机8字节协议的距离单位为cm, 直接保存帧中的数值
  // [解析] 距离占 2 个字节，是"大端"(Big-Endian)存放：frame_[4] 是高 8 位，frame_[5] 是低 8 位。
  // [解析] ★而 STM32 本身是"小端"机器，所以绝不能把这两个字节直接强转成 uint16_t 读
  // [解析]   （那样高低字节会反，比如实际 100 cm 会读成 25600），必须手动拼：
  // [解析]   先把高字节转成 16 位再左移 8 位，然后和低字节做"按位或"。
  // [解析] 范围：0 ~ 65535 cm，本模块实际量程远小于这个值。
  // [解析] 内层那个 static_cast<uint16_t>(frame_[4]) 是必须的：
  // [解析]   如果直接写 frame_[4] << 8，它会先提升成 int 再移位；虽然在 32 位 int 上结果一样，
  // [解析]   但显式转换能表达"我要的是 16 位结果"的意图，也避免了很多编译器告警。
  distance_cm_ = static_cast<uint16_t>((static_cast<uint16_t>(frame_[4]) << 8) | frame_[5]);
  // [解析] 模式在 frame_[6] 的低 2 位：& 0x03（二进制 11）只保留最低两位，取值 0~3，
  // [解析]   正好和 LaserMode 枚举的 UNKNOWN/SHORT/MEDIUM/LONG 一一对应。
  mode_ = static_cast<LaserMode>(frame_[6] & 0x03);
  // [解析] 盖时间戳：记下"这一刻"的毫秒数，is_valid() 就靠它算数据有没有过期。
  last_frame_tick_ = HAL_GetTick();
  // [解析] 置位"曾经收到过可信数据"，这是 is_valid() 的第一道门槛。
  has_reliable_distance_ = true;
}

}  // namespace sp
