// [解析] ==========================================================================
// [解析] 【文件】sp_middleware/io/laser/laser.hpp
// [解析] 【作用】把 GY-56 激光测距模块（串口输出 8 字节小帧的那种）封装成 LaserGY56 类。
// [解析]         上层只要调 get_distance()，就能知道"正前方障碍物还有多少厘米"。
// [解析] 【被谁用】本工程目前还没有任务真正使用它（CMakeLists.txt 里也没把它加入编译），
// [解析]         属于"预留但没接线"的中间件。典型用法是：
// [解析]           sp::LaserGY56 laser(&huart3);   // 创建对象，绑定串口
// [解析]           laser.start_receive();          // 启动 DMA 接收
// [解析]           然后在 HAL_UARTEx_RxEventCallback() 里调 laser.receive_complete(Size);
// [解析]           主循环里读 laser.is_valid() / laser.get_distance()。
// [解析] 【关键点】1) 数据帧固定 8 字节，帧头 0x5A 0x5A；因为串口可能从半截开始收，
// [解析]             代码用"滑动窗口"逐字节对齐，凑满 8 字节再校验，这点最容易看不懂；
// [解析]           2) 距离单位是 cm，且模块会偶尔吐脏数据，所以必须用 is_valid() 判断数据是否过期；
// [解析]           3) 几个成员带 volatile，因为它们会被串口中断改写，
// [解析]              主循环里读它们时，volatile 能禁止编译器"只读一次"的优化。
// [解析] ==========================================================================

#ifndef SP__LASER_GY56_HPP
#define SP__LASER_GY56_HPP

#include <cstdint>
// [解析] cstdint 提供 uint8_t / uint16_t / uint32_t 这些"定长整数"类型。
// [解析] 嵌入式代码里尽量不用 int，因为 int 的位数随平台而变，
// [解析] 而定长类型能保证协议里的字节数永远一致。

#include "usart.h"
// [解析] usart.h 由 CubeMX 生成，里面有 UART_HandleTypeDef 结构体的定义，
// [解析] 以及 huart1 / huart3 等串口句柄的 extern 声明。

namespace sp
{
// [解析] 中间件所有代码都放在 sp 命名空间里，避免与 HAL 库或其它库重名。
// [解析] 模块有三种测距量程。这里用 enum class（强类型枚举）：
// [解析] 使用时必须写 LaserMode::SHORT，不能像老式枚举那样直接写 SHORT，
// [解析] 好处是不会和普通整数、别的枚举值混淆，编译器能帮你查错。
// [解析] ": uint8_t" 指定底层存储类型为 1 字节，因为协议里模式只占 2 个 bit（取值 0~3）。
enum class LaserMode : uint8_t
{
  UNKNOWN = 0,  // 尚未从有效数据帧获取模式
  SHORT = 1,    // 短距离模式
  MEDIUM = 2,   // 中距离模式
  LONG = 3      // 长距离模式
};

// [解析] LaserGY56 类：一个对象 = 一路串口上的一只激光模块。
// [解析] 它的工作方式以"被动接收"为主：模块自己按设定周期往串口吐帧，
// [解析] 每来一帧就刷新一次内部数据；主循环只负责读，不用发查询。
// [解析] 只有改模式 / 改波特率 / 存配置时才需要主动发命令。
class LaserGY56
{
public:
  // [解析] 构造函数只记住"用哪一路串口"，不碰硬件，也不会自动开始接收。
  // [解析] explicit 表示禁止隐式类型转换，防止写出 LaserGY56 l = &huart3; 这种含糊写法。
  explicit LaserGY56(UART_HandleTypeDef * huart);

  // 返回最近一帧有效距离, 单位: cm; 使用前通过is_valid()判断是否过期
  uint16_t get_distance() const;
  // [解析] 返回值：距离值，单位 cm（厘米）。具体量程/精度取决于当前模式，通常几十米以内。
  // [解析] ★坑：如果从没收到过有效帧，返回的是初始值 0。
  // [解析]   所以读到 0 既可能是"真的贴脸了"，也可能是"根本没有数据"，
  // [解析]   必须先确认 is_valid() 为 true 才能相信这个数字。
  // [解析] const 修饰在函数末尾，表示本函数不会修改对象的成员变量。
  // 收到过有效帧且距上一帧不超过timeout_ms时返回true, 单位: ms
  bool is_valid(uint32_t timeout_ms = 500) const;
  // [解析] 判断"手里这份数据还算不算新鲜"。
  // [解析] 参数 timeout_ms：容忍的过期时间，单位 ms，默认 500 ms（即 0.5 秒没更新就认为失效）。
  // [解析] 返回值：true = 曾经收到过有效帧，且距最后一帧不超过 timeout_ms。
  // [解析] 内部用 HAL_GetTick()（FreeRTOS 启动后按 1 ms 递增的毫秒计数）做差；
  // [解析] 无符号整数相减天然能处理计数器回绕（溢出归零），不用额外写防溢出的判断。
  // 返回最近一帧有效数据中报告的模式
  LaserMode get_mode() const;
  // [解析] 返回模块在最近一帧里自报的量程模式（SHORT / MEDIUM / LONG）。
  // [解析] 若从未收到有效帧，返回 LaserMode::UNKNOWN。

  // 返回命令是否通过串口发送成功
  bool send_mode(LaserMode mode);
  bool set_baud_rate_9600();
  bool set_baud_rate_115200();
  bool save_config();
  // [解析] 上面这 4 个函数都是"往模块发一条命令"，返回值含义相同：
  // [解析]   true  = 这几个字节确实通过串口发出去了；
  // [解析]   false = 串口句柄为空，或发送超时失败。
  // [解析] 注意：true 只代表"发成功"，不代表模块执行成功 —— 模块不会回 ACK 应答。
  // [解析] ★坑：改完波特率（9600 / 115200）必须再调 save_config() 保存，
  // [解析]   否则模块掉电重启后又会变回原来的波特率，而上位机代码是按新波特率配的，会收不到数据。
  // [解析]   另外：发完改波特率的命令后，要记得同步把 CubeMX 里该串口的波特率也改成一样的。

  // 启动USART的DMA空闲接收; 回调传入本次收到的字节数
  bool start_receive();
  void receive_complete(uint16_t size);
  void update(uint8_t byte);
  // [解析] start_receive()：启动" DMA + 空闲中断"接收，返回值 true = 启动成功。
  // [解析]   它内部用的是 HAL_UARTEx_ReceiveToIdle_DMA()：串口总线一旦空闲（一帧发完，
  // [解析]   一段时间没有新字节），或者缓冲区收满，HAL 就会自动调用你写的
  // [解析]   HAL_UARTEx_RxEventCallback()；在回调里转手调用本类的 receive_complete(Size)。
  // [解析] receive_complete(size)：size = 本次 DMA 收到的字节数（单位：字节，最大 64）。
  // [解析]   把这段数据逐个喂给 update()，喂完立刻重新启动接收，准备下一批。
  // [解析] update(byte)：本文件的核心 —— 每来 1 个字节就维护一次 8 字节滑动窗口，
  // [解析]   凑满一帧且校验通过就解析出距离和模式。参数是 1 个字节，取值 0x00~0xFF。

private:
  // [解析] 本模块一帧固定 8 字节，所以窗口长度就是 8：
  // [解析] [0][1] 帧头 0x5A 0x5A | [2] 设备/保留 | [3] 数据长度 0x03
  // [解析] [4][5] 距离（大端：frame_[4] 是高 8 位）| [6] 状态+模式 | [7] 累加校验和
  static constexpr uint8_t FRAME_SIZE = 8;
  // [解析] DMA 接收缓冲区长度取 64：一次空闲中断里可能收到好几帧拼在一起，
  // [解析] 缓冲区大一点能少丢数据；但也意味着"一次最多处理 64 字节"。
  // [解析] static constexpr 表示"编译期常量"，不占用对象的存储空间。
  static constexpr uint16_t RX_BUFFER_SIZE = 64;

  bool send_command(uint8_t command);
  bool frame_valid() const;
  void parse_frame();
  // [解析] 上面 3 个是内部函数，外部调用不到：
  // [解析]   send_command()：发送 A5 + 命令字 + 累加和 共 3 字节（阻塞式，超时 10 ms）。
  // [解析]   frame_valid()：检查帧头 4 个字节和最后 1 字节的"逐字节累加和"校验。
  // [解析]   parse_frame()：校验通过后，把 8 字节拆成距离 / 模式，并刷新时间戳。

  UART_HandleTypeDef * huart_;
  uint8_t rx_buff_[RX_BUFFER_SIZE];
  uint8_t frame_[FRAME_SIZE];
  uint8_t frame_size_;
  // [解析] huart_：指向 CubeMX 生成的串口句柄（例如 &huart3）；所有 HAL 调用都拿它当第一个参数。
  // [解析] rx_buff_：DMA 直接往里写数据的缓冲区，必须一直有效（所以是成员变量而不是局部变量）。
  // [解析] frame_：滑动窗口，始终保存"最近收到的 8 个字节"。
  // [解析] frame_size_：窗口里已经填了多少字节，从 0 涨到 8；到 8 之后就一直是 8。

  volatile uint16_t distance_cm_;
  volatile uint32_t last_frame_tick_;
  volatile LaserMode mode_;
  volatile bool has_reliable_distance_;
  // [解析] ↓ 这 4 个成员都加了 volatile，原因：它们会在串口中断里被改写，
  // [解析]   而主循环在别的地方读它们。volatile 告诉编译器"这个变量随时可能被别人改"，
  // [解析]   禁止把多次读取优化成"只读一次"，否则主循环可能永远看到旧值。
  // [解析] distance_cm_：最近一帧的有效距离，单位 cm。
  // [解析] last_frame_tick_：收到最后一帧时的 HAL_GetTick() 毫秒数，is_valid() 用它判断过期。
  // [解析] mode_：最近一帧自报的量程模式。
  // [解析] has_reliable_distance_：是否"曾经"收到过至少一帧通过校验的数据。
  // [解析] 注意：frame_ / frame_size_ 没加 volatile，因为 update() 只在接收回调这一条
  // [解析]   路径上跑，外部不会直接读它们；但如果以后要在别的任务里读 frame_，就得补上。
};

}  // namespace sp

#endif  // SP__LASER_GY56_HPP
