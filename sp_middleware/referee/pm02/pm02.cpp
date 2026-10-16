// [解析] ==========================================================================
// [解析] 【文件】sp_middleware/referee/pm02/pm02.cpp
// [解析] 【作用】pm02.hpp 里 PM02 类的实现：拆帧 → 校验 → 按命令码分派 → 存进对应结构体。
// [解析] 【看代码的重点】下面几处最容易看不懂，都在相应位置逐段标注了：
// [解析]   1) 匿名 namespace 里那堆小工具（copy_fixed / read_u16_le / value_or_default）
// [解析]      和 7 个 default_radar_xxx()：它们负责"缺省值"和"无效字段清洗"；
// [解析]   2) update() 的 4 道校验关卡，以及为什么必须先算出长度才能做 CRC16；
// [解析]   3) 0x0301"交互数据"要先剥掉 6 字节头部，再按子编码 0x0210~0x0213 分派；
// [解析]   4) 结尾那句"递归解析"是怎么把缓冲区里叠着的多帧一次吃完的。
// [解析] 【协议速查】帧结构（多字节字段都是小端，低字节在前）：
// [解析]   SOF(1) | data_len(2) | seq(1) | CRC8(1) | cmd_id(2) | data(data_len) | CRC16(2)
// [解析] ==========================================================================

#include "pm02.hpp"

// [解析] <cstring>：提供 std::memcpy / std::memset（按字节拷贝、按字节清零）。
#include <cstring>

// [解析] crc.hpp：提供 check_crc8 / check_crc16，用来验证帧头和整帧有没有被干扰。
#include "tools/crc/crc.hpp"

// [解析] 匿名 namespace（没有名字的 namespace）：里面定义的常量和函数"只在本 .cpp 内可见"，
// [解析]   不会和别的文件重名冲突，作用相当于 C 语言里的 static。
// [解析]   本文件把解析用的私有小工具都放在这里，避免污染 sp 命名空间。
namespace
{
// [解析] INTERACTION_HEADER_LEN = 6：0x0301 交互数据的数据段开头那 6 个字节的头部长度，
// [解析]   组成 = 内容ID(2) + 发送者ID(2) + 接收者ID(2)；解析用户载荷之前必须先跳过它。
constexpr size_t INTERACTION_HEADER_LEN = 6;

// [解析] copy_fixed()：把协议数据段整体拷进目标结构体（模板函数，T 由实参自动推导）。
// [解析]   模板 = "同一段代码适配多种类型"，这里 T 可以是 GameStatus、PowerHeatData……
// [解析] 参数：dst 目标结构体（引用，会被改写）；data 数据段首地址；size 数据段字节数。
// [解析] 返回值：无。副作用：只有长度"正好等于"结构体大小时才拷贝，否则**静默丢弃**。
// [解析] 为什么要判长度：这些结构体都是 __attribute__((packed)) 的（紧凑排列、不含填充字节），
// [解析]   字节布局和协议字段一一对应（referee_protocol.hpp 里有 static_assert 兜底），
// [解析]   长度一致才能直接 memcpy；否则可能读到帧尾之外的内存（越界）或拷进错位的数据。
// [解析] 注意：长度不对时函数直接 return，上层收不到任何错误提示 ——
// [解析]   如果你发现某个成员一直是 0，先怀疑"收到的帧长度和结构体大小对不上"。
template <typename T>
void copy_fixed(T & dst, const uint8_t * data, size_t size)
{
  // 固定长度协议帧必须和结构体大小一致，避免异常长度导致 memcpy 越界。
  // [解析] 关键的一票否决：size 只要不等于 sizeof(T) 就什么都不做。
  if (size != sizeof(T)) return;
  // [解析] 第三个参数用 sizeof(T) 而不是 size：既然上面已确认二者相等，写 sizeof(T) 更直观。
  std::memcpy(&dst, data, sizeof(T));
}

// [解析] read_u16_le()：从字节流里按**小端**（Little Endian）读出 16 位整数，_le 即 little endian。
// [解析] 裁判系统协议规定多字节字段"低字节在前"：data[0] 是低 8 位、data[1] 是高 8 位，
// [解析]   所以拼法是 data[0] | (data[1] << 8)。
// [解析] 如果写成 (data[0] << 8) | data[1]，就会把 0x0212 读成 0x1202 —— 字节序反了，非常隐蔽。
// [解析] 参数 data：指向待读的 2 个字节（越界检查由调用者负责）；返回拼好的 uint16_t。
// [解析] 两处 static_cast<uint16_t> 是防止 uint8_t 参与移位时发生整型提升带来的意外。
uint16_t read_u16_le(const uint8_t * data)
{
  return static_cast<uint16_t>(data[0]) | (static_cast<uint16_t>(data[1]) << 8U);
}

// [解析] value_or_default()：协议里"全 1"表示"这个字段无效/雷达没测到"
// [解析]   （uint8_t 是 0xFF，uint16_t 是 0xFFFF，int16_t 是 -1），接收方应把它当成"没有数据"。
// [解析] 本函数把"全 1"换成调用者给的 default_value，其余值原样返回。
// [解析] 关键写法 static_cast<T>(-1) 就是"该类型全 1 的位模式"，比分别写 0xFF/0xFFFF/-1 更通用。
// [解析] 参数：value 原始字段值；default_value 默认值（类型是 uint32_t，足够容纳各种默认值）。
// [解析] 返回：T 类型的最终值（例如 int16 的坐标字段传默认值 0，就得到 0）。
template <typename T>
T value_or_default(T value, uint32_t default_value)
{
  return value == static_cast<T>(-1) ? static_cast<T>(default_value) : value;
}

// [解析] ↓ 下面 7 个 default_radar_xxx()：给出各组雷达数据的"安全默认值"。
// [解析]   两个地方会用到：① 构造函数初始化；② 数据超时或帧长度非法时把数据恢复成这些值。
// [解析]   设计原则：默认值要"一眼就能看出是无效数据"，不能和真实数值混淆（例如血量给 999）。
// [解析]   函数体里的 return {} 表示"值初始化"，即把结构体的所有成员清零。

// [解析] 敌方飞镖预警默认全 0：0 表示"没有预警"，是最安全的状态。
sp::referee::RadarEnemyDartWarning default_radar_enemy_dart_warning()
{
  return {};
}

// [解析] 对方空中机器人反制状态默认 0：0 = "没有反制/没有反制信息"，按无效数据处理。
sp::referee::RadarAerialCountered default_radar_aerial_countered()
{
  return {};
}

// [解析] 坐标默认全 0：{} 会把 source 和全部 x/y 清零，表示"未知/无效坐标"。
// [解析] 注意坐标是 int16_t，单位由雷达侧约定，中间件不做任何换算（见 referee_protocol.hpp 注释）。
sp::referee::RadarSentryPosition default_radar_position()
{
  return {};
}

// [解析] 血量默认 999：真实血量最大也就几百，999 是明显超出正常范围的"哨兵值（sentinel）"，
// [解析]   上层看到 999 就知道"这份数据不可信"，对应 readme 里 0x0212 A02 条的约定。
// [解析] 返回一个填好默认值的 RadarAllyHp（5 台己方机器人的血量，单位：点）。
sp::referee::RadarAllyHp default_radar_hp()
{
  sp::referee::RadarAllyHp data{};
  // [解析] 英雄、工程、3 号步兵、4 号步兵、哨兵，逐个给成 999（下面 data{} 已先把它们清 0）。
  data.hero_hp = 999U;
  data.engineer_hp = 999U;
  data.infantry3_hp = 999U;
  data.infantry4_hp = 999U;
  data.sentry_hp = 999U;
  return data;
}

// [解析] 允许发弹量默认 999：同样是超出正常范围的哨兵值（真实发弹量最多几百），
// [解析]   对应 readme 里 0x0212 A03 条的约定；单位是"发"。
sp::referee::RadarAllyAmmo default_radar_ammo()
{
  sp::referee::RadarAllyAmmo data{};
  // [解析] 英雄、3 号步兵、4 号步兵、空中、哨兵，逐个给成 999。
  data.hero_ammo = 999U;
  data.infantry3_ammo = 999U;
  data.infantry4_ammo = 999U;
  data.aerial_ammo = 999U;
  data.sentry_ammo = 999U;
  return data;
}

// [解析] 场地信息默认值：剩余/总金币数给 9999（远超真实金币数，一眼就是无效数据），
// [解析]   占领状态位掩码 status_flags 给 0（表示"什么都没被占领"）。
sp::referee::RadarAllyField default_radar_field()
{
  sp::referee::RadarAllyField data{};
  data.remain_coins = 9999U;
  data.total_coins = 9999U;
  data.status_flags = 0U;
  return data;
}

// [解析] 增益状态默认值：先 {} 把 5 台机器人 × 5 类增益（heal/cool/def/vuln/atk）
// [解析]   和 6 个 status 字段全部清 0（0 = 无增益、机器人存活），再单独把哨兵姿态设为 3。
// [解析] sentry_posture 为什么取 3：协议里哨兵姿态 1=进攻、2=防御、3=移动，
// [解析]   取 3（移动）是最"中性"的默认值，不会被上层误当成进攻或防御姿态。
sp::referee::RadarBuffStatus default_radar_buff_status()
{
  sp::referee::RadarBuffStatus data{};
  data.sentry_posture = 3U;
  return data;
}

// [解析] ↓ 下面 6 个 normalize_radar_xxx()：对**已经拷进来**的结构体做"字段级清洗"。
// [解析]   为什么只替换单个字段、而不是把整帧丢掉：一帧 0x0212 里可能只有某台机器人
// [解析]   雷达没看到（对应字段是全 1），其余字段仍然是好数据，不能因为一个坏字段丢掉整帧。
// [解析]   每个字段使用的默认值都和上面 default_radar_xxx() 保持一致，避免两处对不上。

// [解析] 坐标清洗：source 和 12 个坐标（6 台机器人 × x/y）逐个把"全 1(-1)"换成 0。
// [解析] 参数 data 是引用，函数就地修改它、没有返回值（副作用型函数）。
void normalize_radar_position(sp::referee::RadarSentryPosition & data)
{
  data.source = value_or_default(data.source, 0U);
  data.hero_x = value_or_default(data.hero_x, 0);
  data.hero_y = value_or_default(data.hero_y, 0);
  data.engineer_x = value_or_default(data.engineer_x, 0);
  data.engineer_y = value_or_default(data.engineer_y, 0);
  data.infantry3_x = value_or_default(data.infantry3_x, 0);
  data.infantry3_y = value_or_default(data.infantry3_y, 0);
  data.infantry4_x = value_or_default(data.infantry4_x, 0);
  data.infantry4_y = value_or_default(data.infantry4_y, 0);
  data.aerial_x = value_or_default(data.aerial_x, 0);
  data.aerial_y = value_or_default(data.aerial_y, 0);
  data.sentry_x = value_or_default(data.sentry_x, 0);
  data.sentry_y = value_or_default(data.sentry_y, 0);
}

// [解析] 血量清洗：5 个血量字段逐个把 0xFFFF 换成 999。
void normalize_radar_hp(sp::referee::RadarAllyHp & data)
{
  data.hero_hp = value_or_default(data.hero_hp, 999U);
  data.engineer_hp = value_or_default(data.engineer_hp, 999U);
  data.infantry3_hp = value_or_default(data.infantry3_hp, 999U);
  data.infantry4_hp = value_or_default(data.infantry4_hp, 999U);
  data.sentry_hp = value_or_default(data.sentry_hp, 999U);
}

// [解析] 弹量清洗：5 个允许发弹量字段逐个把 0xFFFF 换成 999。
void normalize_radar_ammo(sp::referee::RadarAllyAmmo & data)
{
  data.hero_ammo = value_or_default(data.hero_ammo, 999U);
  data.infantry3_ammo = value_or_default(data.infantry3_ammo, 999U);
  data.infantry4_ammo = value_or_default(data.infantry4_ammo, 999U);
  data.aerial_ammo = value_or_default(data.aerial_ammo, 999U);
  data.sentry_ammo = value_or_default(data.sentry_ammo, 999U);
}

// [解析] 场地清洗：剩余/总金币的 0xFFFF 换成 9999，status_flags 换成 0。
void normalize_radar_field(sp::referee::RadarAllyField & data)
{
  data.remain_coins = value_or_default(data.remain_coins, 9999U);
  data.total_coins = value_or_default(data.total_coins, 9999U);
  data.status_flags = value_or_default(data.status_flags, 0U);
}

// [解析] 增益清洗：5 台机器人 × 5 类增益共 25 个字段全部把 0xFF 换成 0（无增益），
// [解析]   最后 6 个机器人主状态字段也换成 0（协议里 0 = 存活），哨兵姿态换成 3（移动）。
void normalize_radar_buff_status(sp::referee::RadarBuffStatus & data)
{
  data.hero_heal = value_or_default(data.hero_heal, 0U);
  data.hero_cool = value_or_default(data.hero_cool, 0U);
  data.hero_def = value_or_default(data.hero_def, 0U);
  data.hero_vuln = value_or_default(data.hero_vuln, 0U);
  data.hero_atk = value_or_default(data.hero_atk, 0U);
  data.engineer_heal = value_or_default(data.engineer_heal, 0U);
  data.engineer_cool = value_or_default(data.engineer_cool, 0U);
  data.engineer_def = value_or_default(data.engineer_def, 0U);
  data.engineer_vuln = value_or_default(data.engineer_vuln, 0U);
  data.engineer_atk = value_or_default(data.engineer_atk, 0U);
  data.infantry3_heal = value_or_default(data.infantry3_heal, 0U);
  data.infantry3_cool = value_or_default(data.infantry3_cool, 0U);
  data.infantry3_def = value_or_default(data.infantry3_def, 0U);
  data.infantry3_vuln = value_or_default(data.infantry3_vuln, 0U);
  data.infantry3_atk = value_or_default(data.infantry3_atk, 0U);
  data.infantry4_heal = value_or_default(data.infantry4_heal, 0U);
  data.infantry4_cool = value_or_default(data.infantry4_cool, 0U);
  data.infantry4_def = value_or_default(data.infantry4_def, 0U);
  data.infantry4_vuln = value_or_default(data.infantry4_vuln, 0U);
  data.infantry4_atk = value_or_default(data.infantry4_atk, 0U);
  data.sentry_heal = value_or_default(data.sentry_heal, 0U);
  data.sentry_cool = value_or_default(data.sentry_cool, 0U);
  data.sentry_def = value_or_default(data.sentry_def, 0U);
  data.sentry_vuln = value_or_default(data.sentry_vuln, 0U);
  data.sentry_atk = value_or_default(data.sentry_atk, 0U);
  data.sentry_posture = value_or_default(data.sentry_posture, 3U);
  data.hero_status = value_or_default(data.hero_status, 0U);
  data.engineer_status = value_or_default(data.engineer_status, 0U);
  data.infantry3_status = value_or_default(data.infantry3_status, 0U);
  data.infantry4_status = value_or_default(data.infantry4_status, 0U);
  data.sentry_status = value_or_default(data.sentry_status, 0U);
}

}  // namespace

namespace sp
{
// [解析] 构造函数：初始化列表 : huart(huart), use_dma_(use_dma) 把两个参数存进同名成员。
// [解析] 函数体做三件事：① 把"数据结果箱"一次性清零；② 清接收缓冲；③ 给 7 组雷达数据设默认值。
// [解析] 注意：这里完全不碰硬件，所以"构造对象"这一步是安全的；
// [解析]   什么时候真正开始收数据，由后面的 request() 决定。
PM02::PM02(UART_HandleTypeDef * huart, bool use_dma) : huart(huart), use_dma_(use_dma)
{
  // [解析] 下面 5 行是"一次性清零"的技巧：以第一个成员 game_status 的地址作为起点，
  // [解析]   以最后一个成员 custom_info 的末尾作为终点，中间整块内存直接 memset 成 0。
  // [解析] 前提是这些成员在类里是**连续声明**的（game_status … custom_info，见 pm02.hpp 那块"数据结果箱"）。
  // [解析]   如果以后有人调整了成员声明顺序、或往中间插了新成员，清零范围就会出错，
  // [解析]   而且编译器不会报错 —— 这是本文件一个隐藏的坑，改动成员顺序时必须回来检查这里。
  // [解析] reinterpret_cast<uint8_t *> 是把"结构体指针"当成"字节指针"来用，
  // [解析]   这样才能做以字节为单位的地址加减法（两个字节指针相减得到的就是字节数）。
  uint8_t * start = reinterpret_cast<uint8_t *>(&this->game_status);
  // [解析] end 指向 custom_info 之后的下一个字节（首地址 + 它的大小），即清零区间的上界。
  uint8_t * end = reinterpret_cast<uint8_t *>(&this->custom_info) + sizeof(this->custom_info);
  // [解析] 兜底判断：正常情况下终点的地址一定大于起点，这里只是防御性写法。
  if (end > start) {
    // [解析] end - start 就是这整块区域的字节数；强制转成 size_t 是为了匹配 memset 第三个参数的类型。
    std::memset(start, 0, static_cast<size_t>(end - start));
  }

  // [解析] 清接收缓冲 buff_（255 字节）。用 sizeof(this->buff_) 而不是写死 255，
  // [解析]   以后改 PM02_BUFF_SIZE 时这里不用跟着改，也就不会漏改。
  std::memset(this->buff_, 0, sizeof(this->buff_));
  // [解析] 清双缓冲 multi_buff_：范围 for 遍历 2 行，每行 data() 取首地址、size() 取长度（255）。
  for (auto & mb : this->multi_buff_) {
    std::memset(mb.data(), 0, mb.size());
  }

  // [解析] 7 组雷达数据不在上面那段 memset 的范围里（它们声明在 custom_info 之后），
  // [解析]   所以必须在这里显式赋默认值；而 _valid / _last_update_ms 不在这里赋值，
  // [解析]   它们靠 pm02.hpp 里的类内初始化 "= false" / "= 0" 保证初值。
  this->radar_enemy_dart_warning = default_radar_enemy_dart_warning();
  this->radar_aerial_countered = default_radar_aerial_countered();
  this->enemy_robot_position = default_radar_position();
  this->radar_ally_hp = default_radar_hp();
  this->radar_ally_ammo = default_radar_ammo();
  this->radar_ally_field = default_radar_field();
  this->radar_buff_status = default_radar_buff_status();
}

// [解析] request()：把串口接收"挂上去"。调用一次只负责接收**一包**，
// [解析]   所以每次 update() 处理完之后都要再调一次，否则收完这一包就再也不进回调了。
// [解析] 为什么用 ReceiveToIdle（收到空闲为止）而不是普通 Receive：
// [解析]   裁判系统每帧长度不固定，ReceiveToIdle 会在总线静默（一帧发完）时触发
// [解析]   HAL_UARTEx_RxEventCallback，天然完成了"分帧"，不用自己数长度。
void PM02::request()
{
  // [解析] 两条分支的区别只在"谁负责搬数据"：DMA 由硬件搬（CPU 空闲），IT 由中断搬（每字节进一次中断）。
  if (use_dma_) {
    // dismiss return
    // [解析] "dismiss return" 是本项目的写法约定：故意忽略 HAL 的返回值，表示"这里不检查成功失败"，
    // [解析]   看到这个注释就知道不是漏写了判断。
    // [解析] 参数：huart=哪路串口，buff_=收到的原始字节存哪，PM02_BUFF_SIZE=最多收 255 字节。
    // [解析] 它是异步的：函数立刻返回，数据由 DMA 在后台搬进 buff_，搬满或串口空闲时才进回调。
    HAL_UARTEx_ReceiveToIdle_DMA(this->huart, buff_, PM02_BUFF_SIZE);

    // ref: https://github.com/HNUYueLuRM/basic_framework/blob/master/bsp/usart/bsp_usart.c
    // [解析] 关掉 DMA 的"半传输中断"(Half Transfer, HT)：
    // [解析]   缓冲有 255 字节，DMA 搬到一半（约 127 字节）时本来也会触发一次回调，
    // [解析]   但那时一帧数据往往还没收完，进去解析只能拿到半截，纯属浪费 CPU 甚至误判。
    // [解析] hdmarx 就是"本串口对应的 DMA 接收流"句柄，借它拿到要关闭中断的那条流。
    __HAL_DMA_DISABLE_IT(this->huart->hdmarx, DMA_IT_HT);
  }
  else {
    // dismiss return
    // [解析] 非 DMA 版本：改用中断方式接收（每来一个字节进一次中断），
    // [解析]   语义和 DMA 版一致（同样是"收到空闲为止"），只是波特率高时 CPU 占用明显更大。
    HAL_UARTEx_ReceiveToIdle_IT(this->huart, buff_, PM02_BUFF_SIZE);
  }
}

// [解析] update(size)：串口事件回调的入口（HAL_UARTEx_RxEventCallback 把收到的字节数传进来）。
// [解析]   先做雷达数据超时兜底（HAL_GetTick() 取当前毫秒时间），再从 buff_ 解析这 size 个字节。
// [解析] 副作用：会改写各数据成员、_valid 和 _last_update_ms；不会清缓冲（下一包会整体覆盖）。
void PM02::update(uint16_t size)
{
  // [解析] 每一次都顺手检查雷达数据是否过期 —— 即使裁判系统完全断流、这个函数不再被调用，
  // [解析]   任务里周期调用的 update_radar_data_timeout() 依然能让超时兜底正常工作。
  update_radar_data_timeout(HAL_GetTick());
  // [解析] 转调私有重载：从缓冲头开始，一共 size 字节（里面可能叠了好几帧）。
  update(buff_, size);
}

// [解析] 私有 update()：真正的拆帧函数，对缓冲做"漏斗式过滤"，共 4 道关卡，
// [解析]   任意一道不过就 return（放弃从这个位置开始的数据）。关卡顺序不能乱：
// [解析]     关卡1 剩余长度够不够一个帧头   —— 防止越界读
// [解析]     关卡2 首字节必须是 SOF(0xA5)   —— 防止错位/噪声被当成数据
// [解析]     关卡3 帧头 5 字节的 CRC8 对不对 —— 确认帧头可信，才能相信里面的长度字段
// [解析]     关卡4 整帧 CRC16 对不对         —— 确认数据段没被干扰
// [解析] 为什么帧头要单独再加一个 CRC8：数据长度就写在帧头里，帧头不可信就不知道这帧有多长，
// [解析]   必须先验证帧头，才能安全地算出整帧长度去做 CRC16。
// [解析] 参数：frame_start 本帧的起点（递归时会往后移动）；size 从该点起还有多少字节可用。
void PM02::update(uint8_t * frame_start, uint16_t size)
{
  // [解析] 关卡1：连一个帧头都凑不齐（HEAD_LEN = sizeof(FrameHeader) = 5 字节），
  // [解析]   说明后面不可能再有完整帧了，结束解析。
  if (size < referee::HEAD_LEN) return;
  // [解析] 关卡2：SOF（Start Of Frame）固定是 0xA5；首字节不是它，
  // [解析]   说明数据错位了、或者这段根本不是裁判系统发的数据。
  if (frame_start[0] != referee::SOF) return;
  // [解析] 关卡3：帧头 CRC8。注意 check_crc8 的 len 是"包含 CRC8 字节本身"的，
  // [解析]   所以传 HEAD_LEN=5（SOF 1 + 长度 2 + 序号 1 + CRC8 1）刚好。
  if (!check_crc8(frame_start, referee::HEAD_LEN)) return;

  // [解析] 取出"数据段长度"。协议是小端：frame_start[1] 是低字节、frame_start[2] 是高字节，
  // [解析]   所以 [1] 放低 8 位、[2] 左移 8 位放高位（写反了长度就完全错了）。
  size_t data_len = (frame_start[2] << 8) | frame_start[1];
  // [解析] 整帧长度 = 帧头5 + 命令码2 + 数据段 data_len + 帧尾2；
  // [解析]   帧尾就是最后那 2 字节 CRC16，它也计入下面 check_crc16 的校验范围。
  size_t frame_len = referee::HEAD_LEN + referee::CMD_ID_LEN + data_len + referee::TAIL_LEN;

  // [解析] 关卡4 的前置保护：如果长度字段被干扰成一个大数，算出的 frame_len 可能超过缓冲，
  // [解析]   此时直接 return，避免接下来的 CRC16 和 memcpy 越界访问缓冲区之外的内存。
  // [解析] 注意：这里一 return，同一缓冲里排在后面的其它帧也会被一起丢掉（偏安全侧的选择）。
  if (frame_len > PM02_BUFF_SIZE) return;
  // [解析] 剩余字节数不够一整帧（半包），放弃这一轮；等下一包数据补齐或对方重发。
  if (size < frame_len) return;
  // [解析] 关卡4：整帧 CRC16，覆盖从 SOF 到数据段结束的全部字节；
  // [解析]   len 同样"包含 CRC16 本身"，因为 CRC16 就占整帧最后 2 字节。
  if (!check_crc16(frame_start, frame_len)) return;

  // [解析] 取命令码 cmd_id，同样是小端：frame_start[5] 是低字节、[6] 是高字节
  // [解析]   （帧头正好占 5 字节，所以命令码从下标 5 开始）。0x0202 就是 低=0x02、高=0x02 拼出来的。
  uint16_t cmd_id = (frame_start[6] << 8) | frame_start[5];
  // [解析] 数据段首地址 = HEAD_LEN(5) + CMD_ID_LEN(2) = 7，即常量 referee::DATA_START。
  const uint8_t * data = frame_start + referee::DATA_START;

  // [解析] 按命令码分派：每个 case 对应一种裁判系统数据，做法都是把数据段整段拷进对应成员。
  // [解析] copy_fixed 内部会核对"数据段长度 == 结构体大小"，长度不符就静默跳过（该成员保持旧值）。
  // [解析] 没有列在这里的命令码会落到 default 被忽略，这是正常设计（协议的命令码非常多）。
  switch (cmd_id) {
    // 0x0001 比赛状态数据
    case referee::cmd_id::GAME_STATUS:
      copy_fixed(this->game_status, data, data_len);
      break;
    // 0x0002 比赛结果数据
    case referee::cmd_id::GAME_RESULT:
      copy_fixed(this->game_result, data, data_len);
      break;
    // 0x0003 比赛机器人血量数据
    case referee::cmd_id::GAME_ROBOT_HP:
      copy_fixed(this->game_robot_hp, data, data_len);
      break;
    // 0x0101 场地事件数据
    case referee::cmd_id::EVENT_DATA:
      copy_fixed(this->event, data, data_len);
      break;
    // 0x0104 裁判警告数据
    case referee::cmd_id::REFEREE_WARNING:
      copy_fixed(this->referee_warning, data, data_len);
      break;
    // 0x0105 飞镖发射相关数据
    case referee::cmd_id::DART_INFO:
      copy_fixed(this->dart_info, data, data_len);
      break;
    // 0x0201 机器人状态数据
    case referee::cmd_id::ROBOT_STATUS:
      copy_fixed(this->robot_status, data, data_len);
      break;
    // 0x0202 实时底盘缓冲能量和射击热量数据
    case referee::cmd_id::POWER_HEAT_DATA:
      copy_fixed(this->power_heat, data, data_len);
      break;
    // 0x0203 机器人位置数据
    case referee::cmd_id::ROBOT_POS:
      copy_fixed(this->robot_pos, data, data_len);
      break;
    // 0x0204 机器人增益数据
    case referee::cmd_id::BUFF:
      copy_fixed(this->buff, data, data_len);
      break;
    // 没有 0x0205
    // 0x0206 伤害状态数据
    case referee::cmd_id::HURT_DATA:
      copy_fixed(this->hurt, data, data_len);
      break;
    // 0x0207 实时射击数据
    case referee::cmd_id::SHOOT_DATA:
      copy_fixed(this->shoot, data, data_len);
      break;
    // 0x0208 允许发弹量数据
    case referee::cmd_id::PROJECTILE_ALLOWANCE:
      copy_fixed(this->projectile_allowance, data, data_len);
      break;
    // 0x0209 机器人 RFID 模块状态
    case referee::cmd_id::RFID_STATUS:
      copy_fixed(this->rfid_status, data, data_len);
      break;
    // 0x020A 飞镖选手端指令数据
    case referee::cmd_id::DART_CLIENT_CMD:
      copy_fixed(this->dart_client_cmd, data, data_len);
      break;
    // 0x020B 地面机器人位置数据
    case referee::cmd_id::GROUND_ROBOT_POSITION:
      copy_fixed(this->ground_robot_pos, data, data_len);
      break;
    // 0x020C 雷达标记进度数据
    case referee::cmd_id::RADAR_MARK_DATA:
      copy_fixed(this->radar_mark, data, data_len);
      break;
    // 0x020D 哨兵自主决策信息同步
    case referee::cmd_id::SENTRY_INFO:
      copy_fixed(this->sentry_info, data, data_len);
      break;
    // 0x020E 雷达自主决策信息同步
    case referee::cmd_id::RADAR_INFO:
      copy_fixed(this->radar_info, data, data_len);
      break;
    // 0x0A05 雷达站增益点状态数据（雷达侧可据此生成无敌掩码）
    // [解析] 这是"裁判系统直接下发"的路径（无线链路 0x0A05）；它和 0x0301/0x0212 里的 Buff
    // [解析]   写的是同一个成员 radar_buff_status，两条路谁后到就以谁为准。
    case referee::cmd_id::RADAR_BUFF_STATUS:
      // [解析] 长度必须和结构体大小完全一样才敢 memcpy：结构体是 packed 的，字节布局和协议一一对应。
      if (data_len == sizeof(this->radar_buff_status)) {
        // [解析] 先拷进临时变量 received，清洗掉"全 1"的无效字段之后再赋给成员，
        // [解析]   这样上层永远不会看到半成品/脏数据。
        referee::RadarBuffStatus received{};
        std::memcpy(&received, data, sizeof(received));
        normalize_radar_buff_status(received);
        this->radar_buff_status = received;
        this->radar_buff_status_valid = true;
        // [解析] 记录更新时刻（HAL_GetTick()，单位 ms），供 _fresh() 判断数据有没有过期。
        this->radar_buff_status_last_update_ms = HAL_GetTick();
      }
      else {
        // [解析] 长度不对：当作"收到一包废数据"，内容恢复默认值并标记无效（不清 _last_update_ms）。
        this->radar_buff_status = default_radar_buff_status();
        this->radar_buff_status_valid = false;
      }
      break;
    // 0x0301 雷达自定义子编码：0x0210~0x0213。
    // [解析] 0x0301 是"机器人交互数据"的总命令码，它的数据段开头还有 6 字节交互头：
    // [解析]   内容ID(data_cmd_id, 2) + 发送者ID(sender_id, 2) + 接收者ID(receiver_id, 2)，
    // [解析]   真正的用户载荷从数据段的第 7 个字节开始（结构体见 referee_protocol.hpp 的 RobotInteractionData）。
    // [解析] 整个 case 用一对 {} 包起来，是因为里面要定义局部变量，作用域限定在这段里。
    case referee::cmd_id::ROBOT_INTERACTION_DATA: {
      // [解析] 数据段连 6 字节交互头都不够，没法解析，直接跳出这个 case（去执行后面的递归）。
      if (data_len < INTERACTION_HEADER_LEN) break;

      // [解析] 读子编码（小端 16 位）：它决定这包到底是哪一种雷达数据。
      const uint16_t data_cmd_id = read_u16_le(data);
      // [解析] 读接收者 ID：它位于交互头第 5、6 字节（data+4 开始），含义是"这包是发给谁的"。
      // [解析] 中间的发送者 ID（data+2）本模块用不到，所以直接跳过没读。
      const uint16_t receiver_id = read_u16_le(data + 4);

      // [解析] 只处理"发给本机"的包：本机 ID 取自裁判系统 0x0201 下发的 robot_status.robot_id。
      // [解析] 注意：如果 0x0201 还没收到过，robot_id 就是 0，只有 receiver_id 也为 0 的包才会通过，
      // [解析]   所以上电早期拿不到雷达数据属于正常现象（等 0x0201 到了就好了）。
      if (receiver_id != this->robot_status.robot_id) break;

      // [解析] payload 指向 6 字节交互头之后的真正数据；payload_len 是这块数据的长度。
      const uint8_t * payload = data + INTERACTION_HEADER_LEN;
      const size_t payload_len = data_len - INTERACTION_HEADER_LEN;
      // [解析] 当前时间只取一次，同一帧里所有 _last_update_ms 都用它，保证几组数据的时间戳一致。
      const uint32_t now_ms = HAL_GetTick();

      // [解析] 第二层 switch：按子编码分派到 4 种雷达数据（0x0210~0x0213）。
      switch (data_cmd_id) {
        // [解析] 0x0210 敌方飞镖预警。先把 _valid 清掉，只有"长度正确并成功拷贝"才重新置 true，
        // [解析]   这样后面任何一步出错都不会留下"数据仍然有效"的假象。
        case referee::data_cmd_id::RADAR_ENEMY_DART_WARNING_CMD:
          this->radar_enemy_dart_warning_valid = false;
          // [解析] 载荷长度必须等于结构体大小（RadarEnemyDartWarning 只有 1 字节）才敢 memcpy。
          if (payload_len == sizeof(referee::RadarEnemyDartWarning)) {
            referee::RadarEnemyDartWarning received{};
            std::memcpy(&received, payload, sizeof(received));
            // [解析] 该字段若是 0xFF（雷达没有数据），清洗成 0（= 无预警）。
            received.dart_gate_status = value_or_default(received.dart_gate_status, 0U);
            this->radar_enemy_dart_warning = received;
            this->radar_enemy_dart_warning_valid = true;
            this->radar_enemy_dart_warning_last_update_ms = now_ms;
          }
          else {
            this->radar_enemy_dart_warning = default_radar_enemy_dart_warning();
          }
          break;

        // [解析] 0x0211 机器人坐标（结构体里是 source + 6 台机器人 × x/y，共 25 字节）。
        // [解析]   坐标是 int16_t 且单位由雷达侧约定，中间件只做"无效值清洗"，不做单位换算。
        case referee::data_cmd_id::RADAR_SENTRY_POSITION_CMD:
          this->enemy_robot_position_valid = false;
          if (payload_len == sizeof(referee::RadarSentryPosition)) {
            referee::RadarSentryPosition received{};
            std::memcpy(&received, payload, sizeof(received));
            normalize_radar_position(received);
            this->enemy_robot_position = received;
            this->enemy_robot_position_valid = true;
            this->enemy_robot_position_last_update_ms = now_ms;
          }
          else {
            this->enemy_robot_position = default_radar_position();
          }
          break;

        // [解析] 0x0212 合并数据：一帧里同时包含 HP、弹药、场地、Buff 四块（共 69 字节）。
        // [解析]   四组一起置 true / 取 now_ms，长度不对则四组一起恢复默认值 —— 它们是"同生共死"的。
        case referee::data_cmd_id::RADAR_COMBINED_DATA_CMD:
          this->radar_ally_hp_valid = false;
          this->radar_ally_ammo_valid = false;
          this->radar_ally_field_valid = false;
          this->radar_buff_status_valid = false;
          if (payload_len == sizeof(referee::RadarCombinedData)) {
            referee::RadarCombinedData received{};
            std::memcpy(&received, payload, sizeof(received));

            // 0x0212 的四组数据分别清洗。某组或某字段为全 1 时，
            // 只替换对应字段，不影响同一帧中其他正确数据。
            normalize_radar_hp(received.hp);
            normalize_radar_ammo(received.ammo);
            normalize_radar_field(received.field);
            normalize_radar_buff_status(received.buff);

            this->radar_ally_hp = received.hp;
            this->radar_ally_ammo = received.ammo;
            this->radar_ally_field = received.field;
            this->radar_buff_status = received.buff;

            this->radar_ally_hp_valid = true;
            this->radar_ally_ammo_valid = true;
            this->radar_ally_field_valid = true;
            this->radar_buff_status_valid = true;

            this->radar_ally_hp_last_update_ms = now_ms;
            this->radar_ally_ammo_last_update_ms = now_ms;
            this->radar_ally_field_last_update_ms = now_ms;
            this->radar_buff_status_last_update_ms = now_ms;
          }
          else {
            this->radar_ally_hp = default_radar_hp();
            this->radar_ally_ammo = default_radar_ammo();
            this->radar_ally_field = default_radar_field();
            this->radar_buff_status = default_radar_buff_status();
          }
          break;

        // [解析] 0x0213 对方空中机器人被反制状态（结构体只有 1 字节，同样是 0xFF 清洗成 0）。
        case referee::data_cmd_id::RADAR_AERIAL_COUNTERED_CMD:
          this->radar_aerial_countered_valid = false;
          if (payload_len == sizeof(referee::RadarAerialCountered)) {
            referee::RadarAerialCountered received{};
            std::memcpy(&received, payload, sizeof(received));
            received.aerial_countered = value_or_default(received.aerial_countered, 0U);
            this->radar_aerial_countered = received;
            this->radar_aerial_countered_valid = true;
            this->radar_aerial_countered_last_update_ms = now_ms;
          }
          else {
            this->radar_aerial_countered = default_radar_aerial_countered();
          }
          break;

        // [解析] 其它子编码（例如选手端 UI 绘制 0x0100~0x0104）本模块不处理，直接忽略。
        default:
          break;
      }
      break;
    }
    // 0x0303 选手端小地图交互数据
    case referee::cmd_id::MAP_COMMAND:
      copy_fixed(this->map_command, data, data_len);
      break;
    // 0x0305 选手端小地图接收雷达数据
    case referee::cmd_id::MAP_ROBOT_DATA:
      copy_fixed(this->map_robot_data, data, data_len);
      break;
    // 0x0306 自定义控制器与选手端交互数据
    // case referee::cmd_id::CUSTOM_CLIENT_DATA:
    //   copy_fixed(this->custom_client_data, data, data_len);
    //   break;
    // [解析] ↑ 这三行被作者注释掉了（对应的成员 custom_client_data 在 pm02.hpp 里也被注释掉），
    // [解析]   属于"暂不支持/待办"的功能，不是漏写，也不要以为这里能收到 0x0306 的数据。
    // 0x0307 选手端小地图接收哨兵数据
    case referee::cmd_id::MAP_DATA:
      copy_fixed(this->map_data, data, data_len);
      break;
    // 0x0308 选手端小地图接收机器人数据
    case referee::cmd_id::CUSTOM_INFO:
      copy_fixed(this->custom_info, data, data_len);
      break;
    // [解析] 其余命令码（例如本模块未实现的 0x0302 / 0x0306 / 0x0309）落到这里，
    // [解析]   直接忽略：对应成员保持上一次的值不变（不清零、也不报错）。
    default:
      break;
  }

  // 递归解析，因为缓冲区中可能包含多帧裁判系统的数据。
  // [解析] 这是本函数最巧妙、也最容易看晕的一行：一包串口数据里可能"叠"了好几帧，
  // [解析]   处理完当前帧后，把起点后移 frame_len、剩余长度减去 frame_len，再调用自己继续处理下一帧。
  // [解析] 递归的终止条件就是函数开头那几道关卡：剩余长度 < 5（连帧头都凑不齐）时 return。
  // [解析] 递归深度取决于"一包里叠了几帧"；单帧最长 255 字节、缓冲也是 255 字节，
  // [解析]   因此深度有天然上限，不会把中断里的栈压爆。
  update(frame_start + frame_len, size - frame_len);
}

// [解析] send()：把 data 指向的 size 字节发给裁判系统（典型用途是 UI 图形包、自定义交互包）。
// [解析]   返回值是 HAL 状态码，调用者可以据此判断有没有发出去（HAL_OK 表示成功）。
// [解析]   DMA 版是"非阻塞"的：函数立刻返回，数据由 DMA 在后台慢慢发；
// [解析]     因此发送期间 data 指向的缓冲区不能改写，否则发出去的内容会被改坏。
HAL_StatusTypeDef PM02::send(const uint8_t * data, size_t size)
{
  if (use_dma_) {
    return HAL_UART_Transmit_DMA(this->huart, data, size);
  }

  // [解析] 非 DMA 版是"阻塞发送"：CPU 要一直等到发完或者超时才返回。
  // [解析] 最后那个 0xFF 是超时时间，单位 ms（0xFF = 255ms）。
  // [解析] 注意：HAL 里表示"无限等待"的常量是 HAL_MAX_DELAY(0xFFFFFFFF)，
  // [解析]   这里写 0xFF 意味着低速波特率下发大包（例如整屏 UI 包）可能因超时而只发出去一部分。
  return HAL_UART_Transmit(this->huart, data, size, 0xFF);
}

// [解析] update_radar_data_timeout()：周期调用的"看门狗"。
// [解析] 逻辑：对 7 组雷达数据逐个问"你还新鲜吗"（调用 xxx_fresh()）；
// [解析]   只要有一组不新鲜（从没收到过，或者超过 timeout_ms 没更新），
// [解析]   就把它恢复成默认值并清掉 _valid —— 宁可对外表现为"数据无效"，
// [解析]   也不用过期数据去做决策（例如拿 10 秒前的坐标去瞄准）。
// [解析] 参数：now_ms 当前毫秒时间（任务里传 HAL_GetTick()）；timeout_ms 超时阈值，默认 1500ms。
// [解析] 副作用：会重置数据；如果链路一直正常，本函数什么也不改（只是反复判断一遍）。
// [解析] 提示：PM02 的数据是在串口中断里被改写的，任务里调用本函数若要求和中断互斥，
// [解析]   需要放进 taskENTER_CRITICAL()/taskEXIT_CRITICAL()（readme.md 里演示了这个用法）。
void PM02::update_radar_data_timeout(uint32_t now_ms, uint32_t timeout_ms)
{
  // [解析] 飞镖预警超时 → 恢复默认（预警 0）并标记数据无效。
  if (!radar_enemy_dart_warning_fresh(now_ms, timeout_ms)) {
    this->radar_enemy_dart_warning = default_radar_enemy_dart_warning();
    this->radar_enemy_dart_warning_valid = false;
  }

  // [解析] 对方空中机器人反制状态超时 → 恢复默认（无反制）并标记无效。
  if (!radar_aerial_countered_fresh(now_ms, timeout_ms)) {
    this->radar_aerial_countered = default_radar_aerial_countered();
    this->radar_aerial_countered_valid = false;
  }

  // [解析] 机器人坐标超时 → 全部坐标和 source 清 0 并标记无效。
  if (!enemy_robot_position_fresh(now_ms, timeout_ms)) {
    this->enemy_robot_position = default_radar_position();
    this->enemy_robot_position_valid = false;
  }

  // [解析] 己方血量超时 → 5 个血量都给 999 并标记无效。
  if (!radar_ally_hp_fresh(now_ms, timeout_ms)) {
    this->radar_ally_hp = default_radar_hp();
    this->radar_ally_hp_valid = false;
  }

  // [解析] 己方弹量超时 → 5 个发弹量都给 999 并标记无效。
  if (!radar_ally_ammo_fresh(now_ms, timeout_ms)) {
    this->radar_ally_ammo = default_radar_ammo();
    this->radar_ally_ammo_valid = false;
  }

  // [解析] 场地信息超时 → 金币 9999、占领状态 0，并标记无效。
  if (!radar_ally_field_fresh(now_ms, timeout_ms)) {
    this->radar_ally_field = default_radar_field();
    this->radar_ally_field_valid = false;
  }

  // [解析] 增益状态超时 → 全部增益 0、哨兵姿态 3，并标记无效。
  if (!radar_buff_status_fresh(now_ms, timeout_ms)) {
    this->radar_buff_status = default_radar_buff_status();
    this->radar_buff_status_valid = false;
  }
}

// [解析] ↓ 下面 7 个 xxx_fresh()：数据"新鲜度"判断，返回 true 表示这组数据现在可以信任。
// [解析]   条件：① _valid 为 true（收到过合法帧）② 距上次更新不超过 timeout_ms。
// [解析] 关键点：now_ms - last_update_ms 用的是 uint32_t 的"自然回绕"减法，
// [解析]   HAL_GetTick() 约 49.7 天会溢出回绕一次，回绕之后这个减法依然能得到正确的时间差，
// [解析]   所以不需要额外写溢出判断 —— 这是嵌入式里判断超时的通用写法。
// [解析]   参数：now_ms 当前毫秒时间；timeout_ms 允许的最大数据年龄，默认 1500ms。

// [解析] 敌方飞镖预警是否新鲜。
bool PM02::radar_enemy_dart_warning_fresh(uint32_t now_ms, uint32_t timeout_ms) const
{
  // [解析] && 是短路求值：如果 _valid 已经是 false，右边的时间比较根本不会执行。
  return this->radar_enemy_dart_warning_valid &&
         static_cast<uint32_t>(now_ms - this->radar_enemy_dart_warning_last_update_ms) <=
           timeout_ms;
}

// [解析] 对方空中机器人反制状态是否新鲜（判断方式与上面完全同理）。
bool PM02::radar_aerial_countered_fresh(uint32_t now_ms, uint32_t timeout_ms) const
{
  return this->radar_aerial_countered_valid &&
         static_cast<uint32_t>(now_ms - this->radar_aerial_countered_last_update_ms) <= timeout_ms;
}

// [解析] 机器人坐标（0x0211）是否新鲜。
bool PM02::enemy_robot_position_fresh(uint32_t now_ms, uint32_t timeout_ms) const
{
  return this->enemy_robot_position_valid &&
         static_cast<uint32_t>(now_ms - this->enemy_robot_position_last_update_ms) <= timeout_ms;
}

// [解析] 己方血量（0x0212 的 HP 块）是否新鲜。
bool PM02::radar_ally_hp_fresh(uint32_t now_ms, uint32_t timeout_ms) const
{
  return this->radar_ally_hp_valid &&
         static_cast<uint32_t>(now_ms - this->radar_ally_hp_last_update_ms) <= timeout_ms;
}

// [解析] 己方允许发弹量（0x0212 的弹药块）是否新鲜。
bool PM02::radar_ally_ammo_fresh(uint32_t now_ms, uint32_t timeout_ms) const
{
  return this->radar_ally_ammo_valid &&
         static_cast<uint32_t>(now_ms - this->radar_ally_ammo_last_update_ms) <= timeout_ms;
}

// [解析] 场地信息（0x0212 的场地块）是否新鲜。
bool PM02::radar_ally_field_fresh(uint32_t now_ms, uint32_t timeout_ms) const
{
  return this->radar_ally_field_valid &&
         static_cast<uint32_t>(now_ms - this->radar_ally_field_last_update_ms) <= timeout_ms;
}

// [解析] 增益状态（0x0212 的 Buff 块，或直接下发的 0x0A05）是否新鲜 —— 本文件最后一个判新鲜函数。
bool PM02::radar_buff_status_fresh(uint32_t now_ms, uint32_t timeout_ms) const
{
  return this->radar_buff_status_valid &&
         static_cast<uint32_t>(now_ms - this->radar_buff_status_last_update_ms) <= timeout_ms;
}

}  // namespace sp
