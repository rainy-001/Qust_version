// [解析] ==========================================================================
// [解析] 【文件】sp_middleware/io/vision/vision.cpp
// [解析] 【作用】Vision 类的实现：从 USB 虚拟串口收到的字节流里"切"出合法帧并解析，
// [解析]         以及把本车状态打包成帧发回给上位机。
// [解析] 【被谁调用】收：applications/vision_task.cpp 里的 vision_callback()（USB 驱动回调）。
// [解析]             发：vision_task() 任务里每 1 ms 调一次 vis.send(...)。
// [解析] 【本文件三个难点】
// [解析]   1) 最上面的 CDC_Transmit_Select()：为什么要"挑一个函数来调用"，
// [解析]      以及 `if (CDC_Transmit_HS)` 这种写法为什么能判断"函数有没有被实现"；
// [解析]   2) update() 里的收包循环：帧头匹配 + 长度检查 + CRC 校验 + 失败时只丢 1 个字节
// [解析]      重新对齐，这是所有串口协议解析的标准套路，务必看懂；
// [解析]   3) update() 在中断里跑，而 send() 在任务里跑 —— 这两者用到的成员变量不重叠，
// [解析]      所以这里没有加锁，也暂时没有竞争问题（改代码时要注意保持这个前提）。
// [解析] ==========================================================================

#include "vision.hpp"

#include <cstring>
// [解析] 用 std::copy / std::memmove 做内存搬运，比手写 for 循环更不容易越界写错。

#include "cmsis_os.h"
// [解析] 提供 osKernelSysTick()（FreeRTOS 的毫秒计数）等 CMSIS-RTOS 接口。
#include "tools/crc/crc.hpp"
// [解析] CRC16 计算与校验：get_crc16() 算校验值，check_crc16() 直接校验整帧。
#include "tools/math_tools/math_tools.hpp"
// [解析] 数学工具。★注意：本文件里其实没有用到它的任何函数，属于多余的包含（无害）。
#include "usbd_cdc_if.h"
// [解析] CubeMX 生成的 USB CDC 中间件头文件，里面声明了 CDC_Transmit_FS()。
// [解析] ★如果你的工程没启用 USB，这个头文件会找不到 → 编译报错，必须先配好 USB。

extern "C" {
// [解析] extern "C" 表示"这些函数按 C 语言的规则链接（函数名不做 C++ 名称修饰 mangling）"。
// [解析] 因为 CDC_Transmit_HS/FS 是 C 文件（usbd_cdc_if.c）里定义的，
// [解析] 不套 extern "C" 的话链接时会找不到符号。
#if defined(__GNUC__)
// [解析] __GNUC__ 只有 GCC/arm-none-eabi-gcc 这类编译器才定义，所以这是在区分"用 GNU 还是别的工具链"。
uint8_t CDC_Transmit_HS(uint8_t * Buf, uint16_t Len) __attribute__((weak));
uint8_t CDC_Transmit_FS(uint8_t * Buf, uint16_t Len) __attribute__((weak));
// [解析] ↑ 只做"弱声明"：告诉编译器"这两个函数应该存在，但也可能没有"。
// [解析]   weak（弱符号）的含义：如果整个工程里没人定义它，链接时不会报错，
// [解析]   该符号的地址就是 0（空指针）。★这正是下面 CDC_Transmit_Select() 能"探测"它的依据：
// [解析]   本工程当前还没启用 USB（连 usbd_cdc_if.c 都还没生成），
// [解析]   所以这两个函数都没人实现、地址都是 0，本函数目前实际会返回 0xFF；
// [解析]   代码就能自动跳过它去用 FS —— 一份代码同时兼容两种 USB 配置。
// [解析]   等按 docs/C板引脚配置说明.md 把 USB 中间件配好之后，
// [解析]   它会自动挑中真正实现了的那一个（FS 或 HS），上层代码一个字都不用改。
#else
/* For non-GNU toolchains, declare without weak attribute; the project
  should ensure one of these is provided. */
uint8_t CDC_Transmit_HS(uint8_t * Buf, uint16_t Len);
uint8_t CDC_Transmit_FS(uint8_t * Buf, uint16_t Len);
// [解析] ↑ 非 GNU 工具链（例如 Keil AC5、IAR）不支持 __attribute__((weak))，
// [解析]   只能做普通声明，并指望工程里确实提供了其中一个实现，否则链接报错。
#endif
}

// [解析] 一个"适配层"函数：自动挑选能用的那个 USB 发送函数。
// [解析] 为什么需要它？因为 C 板可能用 USB_OTG_FS（全速），也可能用 USB_OTG_HS（高速），
// [解析] 两者生成的发送函数名不同（CDC_Transmit_FS / CDC_Transmit_HS）。
// [解析] 这个函数让上层只写 CDC_Transmit_Select(...)，不用关心底层用的是哪一种。
// [解析] 返回值：0(USBD_OK) = 发送成功；1(USBD_BUSY) = 上一次还没发完，本次数据被丢弃；
// [解析]   2(USBD_FAIL) = 失败；0xFF = 一个发送函数都没实现（本函数自己约定的错误码）。
static inline uint8_t CDC_Transmit_Select(uint8_t * Buf, uint16_t Len)
{
#if defined(__GNUC__)
  // [解析] ★关键一行：`if (CDC_Transmit_HS)` 不是在调用函数，而是在判断"函数地址是不是 0"。
  // [解析]   函数名单独出现在表达式里会退化成函数指针；配合上面的 weak 声明，
  // [解析]   没被实现的函数地址恒为 0，于是就能当成"有没有这个功能"来用。
  // [解析]   先试高速，再试全速；两个都没有（地址都是 0）就返回 0xFF。
  if (CDC_Transmit_HS) return CDC_Transmit_HS(Buf, Len);
  if (CDC_Transmit_FS) return CDC_Transmit_FS(Buf, Len);
  return 0xFF; /* no implementation available */
#else
  /* Prefer HS when both declared; fall back to FS. Linker will error
    if neither is provided for non-GNU toolchains. */
  extern uint8_t CDC_Transmit_HS(uint8_t *, uint16_t);
  extern uint8_t CDC_Transmit_FS(uint8_t *, uint16_t);
  /* Try HS first */
  return CDC_Transmit_HS ? CDC_Transmit_HS(Buf, Len) : CDC_Transmit_FS(Buf, Len);
  // [解析] ★注意：这个非 GNU 分支和上面的语义不一样 —— 上面的写法在"两个函数都没实现"时
  // [解析]   会安全返回 0xFF；而这里对普通（非弱）声明的函数取地址永远不为 0，
  // [解析]   所以它一定会去调用 CDC_Transmit_HS，缺实现时是链接错误而不是优雅降级。
  // [解析]   本工程用 GCC 编译，走的是上面那个分支，这段基本不会被编译到，看看即可。
#endif
}

namespace sp
{

// [解析] 【收包主函数】把 USB 收到的一批字节拼成完整帧并解析。
// [解析] 参数 buf：数据首地址（由 USB 驱动提供）；len：本次字节数。
// [解析] 返回值：无。解析结果直接写进 control / fire / yaw / ... 这些 public 成员。
// [解析] 运行上下文：由 vision_callback() 调用，也就是在 USB 中断里，
// [解析]   所以这里没有打印、没有延时、没有内存分配 —— 中断里必须快进快出。
void Vision::update(uint8_t * buf, uint32_t len)
{
  // [解析] 入参防御：长度 0 或空指针直接返回，避免后面解引用空指针崩溃。
  if (len == 0 || buf == nullptr) return;

  // Append new data to the buffer, discard all if overflow
  // [解析] 溢出策略：如果"已有的 + 新来的"超过 512 字节缓冲区容量，
  // [解析] 就干脆把已缓存的内容全部丢掉（rx_len_ = 0），再从零开始攒。
  // [解析] 为什么不去掉最老的一部分、保留最新数据？因为那样可能把一帧切成两半，
  // [解析] 反而更难对齐；整段丢弃后靠后面的字节重新找帧头更简单可靠。
  if (rx_len_ + len > sizeof(rx_buffer_)) {
    rx_len_ = 0;
  }
  // [解析] std::copy(源首, 源尾, 目标首)：把 buf 开头的 len 个字节搬到
  // [解析] rx_buffer_ 的末尾（rx_len_ 处），也就是"追加"到已有数据后面。
  // [解析] ★注意：如果某一次 len 本身就大于 512（USB 回调理论上不会这么干，
  // [解析]   但换驱动/改调用方就有可能），上面的清零点也救不了，这里会越界写内存。
  // [解析]   更稳妥的写法是判断 len > sizeof(rx_buffer_) 时直接 return。
  std::copy(buf, buf + len, rx_buffer_ + rx_len_);
  // [解析] 更新"已缓存字节数"。
  rx_len_ += len;

  // [解析] 主循环：手里至少要有 2 个字节，才能判断帧头是哪一种。
  // [解析] 每轮循环只处理"缓冲区最前面的一帧"：要么成功取走一整帧，
  // [解析] 要么丢掉最前面的 1 个字节重新对齐；数据不够一帧就 break 出去等下一批。
  // [解析] 注意这里用的是 while 而不是 if —— 一次 update 可能收到好几帧，要一次全处理完。
  while (rx_len_ >= 2) {
    if (rx_buffer_[0] == 'S' && rx_buffer_[1] == 'P') {
      // [解析] 帧头是 'S''P'（字符常量 'S'=0x53、'P'=0x50），对应上位机发来的控制指令。
      // [解析] sizeof(VisionToGimbal) 就是这一帧的固定长度（packed 保证没有填充字节）= 30。
      if (rx_len_ >= sizeof(VisionToGimbal)) {
        // [解析] check_crc16(数据首地址, 整帧长度)：内部会把"最后两个字节"当成 CRC 值
        // [解析] （小端：低字节在前），对前面 len-2 个字节重算 CRC16 再比对。
        // [解析] 校验不过说明这包在传输中出错了，绝不能拿它当控制指令用
        // [解析] （错误的 yaw 会让云台突然抽一下，非常危险）。
        if (check_crc16(rx_buffer_, sizeof(VisionToGimbal))) {
          // [解析] 校验通过：把这一整帧的原始字节"整块"拷进结构体 rx_data_。
          // [解析] reinterpret_cast<uint8_t *>(&rx_data_) 是把结构体地址强制看成字节数组 ——
          // [解析] 因为结构体是 packed 的，内存布局和线上字节流一一对应，所以这样拷贝是安全的。
          std::copy(
            rx_buffer_, rx_buffer_ + sizeof(VisionToGimbal),
            reinterpret_cast<uint8_t *>(&rx_data_));

          // [解析] ★注意这一行的顺序：它是用"旧的"时间戳来判断心跳的，
          // [解析]   也就是说 autoaim_alive 反映的是"上一帧和上上帧之间的间隔"，
          // [解析]   而不是"当前这一帧新不新鲜"。所以上电后的第一帧一定是 false。
          // [解析]   100 是超时阈值，单位 ms：两次数据间隔超过 100 ms 就认为上位机掉线了。
          this->autoaim_alive = (osKernelSysTick() - this->autoaim_last_read_ms_ < 100);
          // [解析] mode 只要不是 0，就说明上位机要求接管云台。
          this->control = (rx_data_.mode != 0);
          // [解析] mode == 2 是唯一"允许开火"的模式（0 不接管、1 接管但不开火）。
          this->fire = (rx_data_.mode == 2);
          // [解析] ↓ 把结构体里的目标角度搬出来，单位仍是 rad / rad/s / rad/s²。
          this->yaw = rx_data_.yaw;
          this->yaw_vel = rx_data_.yaw_vel;
          this->yaw_acc = rx_data_.yaw_acc;
          this->pitch = rx_data_.pitch;
          this->pitch_vel = rx_data_.pitch_vel;
          this->pitch_acc = rx_data_.pitch_acc;
          // [解析] 目标编号，8 表示无目标。
          this->auto_aim_target = rx_data_.target;
          // [解析] 刷新心跳时间戳，供"下一次"判断 autoaim_alive 用。
          this->autoaim_last_read_ms_ = osKernelSysTick();

          // Shift remaining data forward
          // [解析] 这一帧已经消费掉了，把缓冲区长度减去一帧的长度。
          rx_len_ -= sizeof(VisionToGimbal);
          // [解析] 如果后面还粘着别的数据（可能还有半帧），就把剩余部分整体前移到缓冲区开头，
          // [解析] 让下一轮 while 继续从偏移 0 开始判断帧头。
          // [解析] memmove 和 memcpy 的区别：memmove 允许源和目的区域重叠，这里正是重叠的情况。
          if (rx_len_ > 0) std::memmove(rx_buffer_, rx_buffer_ + sizeof(VisionToGimbal), rx_len_);
        }
        else {
          // CRC failed, invalid frame, shift 1 byte forward to re-sync
          // [解析] CRC 没过：不信任这一帧。★但也不能一次扔掉 30 个字节，
          // [解析] 因为很可能"帧头"是巧合撞上的（比如上一帧的尾巴刚好是 'S''P'），
          // [解析] 真正的帧其实从后面某个位置才开始。所以只丢 1 个字节，
          // [解析] 让帧头往后挪一位重新对齐 —— 这是串口协议解析的标准重同步手法。
          rx_len_--;
          if (rx_len_ > 0) std::memmove(rx_buffer_, rx_buffer_ + 1, rx_len_);
        }
      }
      else {
        break;  // Wait for more data
        // [解析] 帧头对上了，但攒的字节还不够一整帧（不到 30 字节）—— 什么都不做，
        // [解析] 直接退出 while，保留缓冲区内容，等下一次 update 送更多字节来再拼。
        // [解析] ★这里必须 break（跳出整个 while）而不是 continue：
        // [解析] 缓冲区最前面就是这半帧，再循环也凑不出来，continue 会变成死循环。
      }
    }
    else if (rx_buffer_[0] == 'S' && rx_buffer_[1] == 'C') {
      // [解析] 帧头是 'S''C'（'C'=0x43），对应上位机发来的 288 字节大包。
      // [解析] 处理套路和上面完全一样，只是帧长从 30 变成了 293。
      if (rx_len_ >= sizeof(VisionToHanging)) {
        if (check_crc16(rx_buffer_, sizeof(VisionToHanging))) {
          // [解析] 整帧拷进 rx_data_hanging_（含 head / seq / data / crc16）。
          std::copy(
            rx_buffer_, rx_buffer_ + sizeof(VisionToHanging),
            reinterpret_cast<uint8_t *>(&rx_data_hanging_));

          // [解析] 取出包序号，上层可以用它发现丢包（序号不连续就说明中间掉了包）。
          this->seq = rx_data_hanging_.seq;
          // [解析] 再把这 288 字节的负载逐字节拷到 public 的 data[] 里方便使用。
          // [解析] ★注意：这里 288 是写死的数字，没有用 sizeof(rx_data_hanging_.data)。
          // [解析]   如果以后改了 VisionToHanging::data 的长度，这两处必须同步改，
          // [解析]   否则要么漏拷数据，要么越界写内存。可以用 sizeof 或 std::copy 更安全。
          for (int i = 0; i < 288; i++) this->data[i] = rx_data_hanging_.data[i];

          // [解析] 大包也算"上位机还活着"的证据，同样刷新心跳时间戳。
          // [解析] ★注意：这里没有像 'S''P' 分支那样重新计算 autoaim_alive，
          // [解析]   所以如果只来大包、不来控制包，autoaim_alive 不会被更新。
          this->autoaim_last_read_ms_ = osKernelSysTick();

          // Shift remaining data forward
          rx_len_ -= sizeof(VisionToHanging);
          if (rx_len_ > 0) std::memmove(rx_buffer_, rx_buffer_ + sizeof(VisionToHanging), rx_len_);
        }
        else {
          // CRC failed, invalid frame, shift 1 byte forward to re-sync
          // [解析] 校验失败，同样只丢 1 个字节重新对齐（理由见上面 'S''P' 分支的说明）。
          rx_len_--;
          if (rx_len_ > 0) std::memmove(rx_buffer_, rx_buffer_ + 1, rx_len_);
        }
      }
      else {
        break;  // Wait for more data
        // [解析] 大包有 293 字节，需要好几次 USB 回调才能攒齐，所以走到这个分支很正常。
      }
    }
    else {
      // Invalid head, shift 1 byte forward to find next 'S'
      // [解析] 最前面两个字节既不是 'S''P' 也不是 'S''C' —— 说明当前位置不是帧头
      // [解析] （可能是残留的半个包，也可能是被干扰的垃圾数据）。
      // [解析] 丢掉最前面 1 个字节，整体前移 1 位，继续找下一个 'S'。
      // [解析] 注意：既然首字节都不是 'S'，其实可以优化成"一次跳过多个字节"，
      // [解析] 但逐字节丢最简单、也不会漏掉任何可能的帧头，性能在这个数据量下完全够用。
      rx_len_--;
      if (rx_len_ > 0) std::memmove(rx_buffer_, rx_buffer_ + 1, rx_len_);
    }
  }
}

// [解析] 【发包函数 1/2】通用版：把本车状态填进 GimbalToVision（57 字节）并发给上位机。
// [解析] 参数含义、单位见 vision.hpp 里对应声明的注释，这里是实现。
// [解析] 返回值：无。★注意发送结果被丢掉了，原因见函数最后一行附近的说明。
void Vision::send(
  uint8_t mode, float q[4], float yaw, float yaw_vel, float pitch, float pitch_vel,
  float bullet_speed, uint16_t bullet_count, float supercap_power_in, float supercap_power_out,
  float supercap_voltage, uint8_t supercap_temperature, uint8_t supercap_status)
{
  // [解析] ↓ 逐个字段赋值。★这里故意没有给 head 赋值：
  // [解析]   tx_data_ 是成员变量，靠结构体里的默认初始化器 head[2] = {'S','P'} 自动填好，
  // [解析]   而且这个值一旦初始化就再也不会变，所以每帧发出去的帧头都是对的。
  tx_data_.mode = mode;
  // [解析] q 是外部传进来的 float 数组，逐个元素拷贝（不能直接写 tx_data_.q = q，
  // [解析] C++ 里数组不能整体赋值）。
  tx_data_.q[0] = q[0];
  tx_data_.q[1] = q[1];
  tx_data_.q[2] = q[2];
  tx_data_.q[3] = q[3];
  tx_data_.yaw = yaw;
  tx_data_.yaw_vel = yaw_vel;
  tx_data_.pitch = pitch;
  tx_data_.pitch_vel = pitch_vel;
  tx_data_.bullet_speed = bullet_speed;
  tx_data_.bullet_count = bullet_count;
  tx_data_.supercap_power_in = supercap_power_in;
  tx_data_.supercap_power_out = supercap_power_out;
  tx_data_.supercap_voltage = supercap_voltage;
  tx_data_.supercap_temperature = supercap_temperature;
  tx_data_.supercap_status = supercap_status;
  // [解析] 计算 CRC16：范围是"整帧字节数 - 最后那个 crc16 字段的 2 字节"= 57-2 = 55 字节。
  // [解析] ★为什么减去 sizeof(crc16) 而不是写死 2？写成 sizeof 以后即使结构体加字段也不会漏；
  // [解析]   这也解释了为什么 crc16 必须放在结构体最后 —— 算法就依赖"校验值在末尾"这个约定，
  // [解析]   而且要和上位机实现的覆盖范围完全一致，差一个字节就永远校验不过。
  tx_data_.crc16 =
    get_crc16(reinterpret_cast<uint8_t *>(&tx_data_), sizeof(tx_data_) - sizeof(tx_data_.crc16));

  // [解析] 把整个结构体的字节原样交给 USB 发出（长度 = sizeof = 57）。
  // [解析] ★注意：返回值没有接收。CDC_Transmit 在"上一次数据还没发完"时会返回 USBD_BUSY，
  // [解析]   此时这一帧就被静默丢掉了 —— 表现为上位机偶尔少一帧（1 kHz 发送时很常见）。
  // [解析]   对本用途（发状态给上位机）影响不大；但如果你要在关键逻辑里依赖"每帧必达"，
  // [解析]   就得检查返回值并在忙时跳过或缓存重发。
  CDC_Transmit_Select(reinterpret_cast<uint8_t *>(&tx_data_), sizeof(tx_data_));
}

// [解析] 【发包函数 2/2】英雄专用版：和上一个函数逻辑完全相同，
// [解析] 只是把对象换成 tx_data_with_game_progress_（GimbalToVisionHero，58 字节），
// [解析] 并且多填一个 game_progress 字段。字段含义见 vision.hpp，这里不再重复解释。
void Vision::send(
  uint8_t mode, float q[4], float yaw, float yaw_vel, float pitch, float pitch_vel,
  float bullet_speed, uint16_t bullet_count, float supercap_power_in, float supercap_power_out,
  float supercap_voltage, uint8_t supercap_temperature, uint8_t supercap_status,
  uint8_t game_progress)
{
  tx_data_with_game_progress_.mode = mode;
  tx_data_with_game_progress_.q[0] = q[0];
  tx_data_with_game_progress_.q[1] = q[1];
  tx_data_with_game_progress_.q[2] = q[2];
  tx_data_with_game_progress_.q[3] = q[3];
  tx_data_with_game_progress_.yaw = yaw;
  tx_data_with_game_progress_.yaw_vel = yaw_vel;
  tx_data_with_game_progress_.pitch = pitch;
  tx_data_with_game_progress_.pitch_vel = pitch_vel;
  tx_data_with_game_progress_.bullet_speed = bullet_speed;
  tx_data_with_game_progress_.bullet_count = bullet_count;
  tx_data_with_game_progress_.supercap_power_in = supercap_power_in;
  tx_data_with_game_progress_.supercap_power_out = supercap_power_out;
  tx_data_with_game_progress_.supercap_voltage = supercap_voltage;
  tx_data_with_game_progress_.supercap_temperature = supercap_temperature;
  tx_data_with_game_progress_.supercap_status = supercap_status;
  // [解析] 本版本独有的字段：比赛进程标志。
  tx_data_with_game_progress_.game_progress = game_progress;
  // [解析] CRC16 覆盖前面 58-2 = 56 字节，算法与通用版一致。
  tx_data_with_game_progress_.crc16 = get_crc16(
    reinterpret_cast<uint8_t *>(&tx_data_with_game_progress_),
    sizeof(tx_data_with_game_progress_) - sizeof(tx_data_with_game_progress_.crc16));

  // [解析] 发出整帧（58 字节），返回值同样没有检查，理由同上。
  CDC_Transmit_Select(
    reinterpret_cast<uint8_t *>(&tx_data_with_game_progress_), sizeof(tx_data_with_game_progress_));
}
}  // namespace sp