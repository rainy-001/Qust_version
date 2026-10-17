// [解析] PM02 类实现：串口缓冲 -> 4 道校验关卡 -> 按 cmd_id 分派 -> 拷进对应结构体成员。
// [解析] 入口是 uart_task.cpp 的 HAL_UARTEx_RxEventCallback：update(size) 解析，处理完再 request() 重新挂接收。
// [解析] 帧结构：SOF(0xA5) | data_len(2) | seq(1) | CRC8(1) | cmd_id(2) | data | CRC16(2)，多字节字段都是小端。
// [解析] 最容易踩的坑：必须先过帧头 CRC8 才能相信帧头里的 data_len 去算整帧长度，否则长度一错就连 CRC16 都没法做。
// [解析] 0x0301 交互数据要先剥 6 字节交互头、再按子编码 0x0210~0x0213 分派；函数末尾递归把缓冲里叠着的多帧吃完。

#include "pm02.hpp"

#include <cstring>

#include "tools/crc/crc.hpp"

namespace
{
// [解析] 0x0301 数据段开头固定 6 字节交互头（内容ID + 发送者ID + 接收者ID，各 2 字节），用户载荷从第 7 字节起。
constexpr size_t INTERACTION_HEADER_LEN = 6;

// [解析] 把协议数据段整段拷进目标结构体；T 由实参推导，可以是 GameStatus、PowerHeatData 等任意协议结构体。
// [解析] 结构体都是 packed 的，字节布局与协议一一对应，所以长度"正好相等"才能直接 memcpy。
// [解析] 长度不符时静默 return、不给任何错误提示 —— 某个成员一直是 0，先怀疑帧长和结构体大小对不上。
template <typename T>
void copy_fixed(T & dst, const uint8_t * data, size_t size)
{
  // 固定长度协议帧必须和结构体大小一致，避免异常长度导致 memcpy 越界。
  if (size != sizeof(T)) return;
  std::memcpy(&dst, data, sizeof(T));
}

// [解析] 按小端读出 16 位：data[0] 是低 8 位、data[1] 是高 8 位。
// [解析] 写成 (data[0] << 8) | data[1] 会把 0x0212 读成 0x1202，字节序反了非常难查。
uint16_t read_u16_le(const uint8_t * data)
{
  return static_cast<uint16_t>(data[0]) | (static_cast<uint16_t>(data[1]) << 8U);
}

// [解析] 协议约定"全 1"表示字段无效（uint8_t 0xFF / uint16_t 0xFFFF / int16_t -1），接收方要当成没有数据。
// [解析] static_cast<T>(-1) 正是"该类型全 1 的位模式"，命中就换成调用者给的默认值，其余原样返回。
template <typename T>
T value_or_default(T value, uint32_t default_value)
{
  return value == static_cast<T>(-1) ? static_cast<T>(default_value) : value;
}

// [解析] ↓ 7 个 default_radar_xxx()：各组雷达数据的安全默认值，构造函数清零和超时/长度非法时都会用到。
// [解析] 原则是默认值要一眼看出是无效数据（血量 999、金币 9999），不能和真实数值混淆。
// [解析] 敌方飞镖预警默认全 0：0 表示没有预警，是最安全的状态。
sp::referee::RadarEnemyDartWarning default_radar_enemy_dart_warning()
{
  return {};
}

// [解析] 对方空中机器人反制状态默认全 0：按"没有反制信息"处理。
sp::referee::RadarAerialCountered default_radar_aerial_countered()
{
  return {};
}

// [解析] 坐标默认全 0：表示未知/无效坐标；字段是 int16_t，单位由雷达侧约定，中间件不做换算。
sp::referee::RadarSentryPosition default_radar_position()
{
  return {};
}

// [解析] 血量默认 999：真实血量最多几百，这是明显越界的哨兵值，对应 readme 里 0x0212 A02 的约定。
sp::referee::RadarAllyHp default_radar_hp()
{
  sp::referee::RadarAllyHp data{};
  data.hero_hp = 999U;
  data.engineer_hp = 999U;
  data.infantry3_hp = 999U;
  data.infantry4_hp = 999U;
  data.sentry_hp = 999U;
  return data;
}

// [解析] 允许发弹量默认 999：同样是越界的哨兵值，单位是"发"，对应 readme 里 0x0212 A03 的约定。
sp::referee::RadarAllyAmmo default_radar_ammo()
{
  sp::referee::RadarAllyAmmo data{};
  data.hero_ammo = 999U;
  data.infantry3_ammo = 999U;
  data.infantry4_ammo = 999U;
  data.aerial_ammo = 999U;
  data.sentry_ammo = 999U;
  return data;
}

// [解析] 场地默认：剩余/总金币给 9999（远超真实金币数），占领状态掩码 status_flags 给 0（什么都没被占领）。
sp::referee::RadarAllyField default_radar_field()
{
  sp::referee::RadarAllyField data{};
  data.remain_coins = 9999U;
  data.total_coins = 9999U;
  data.status_flags = 0U;
  return data;
}

// [解析] 增益默认：先 {} 全部清 0（0 = 无增益、机器人存活），再把 sentry_posture 设为 3（移动，最中性的姿态）。
sp::referee::RadarBuffStatus default_radar_buff_status()
{
  sp::referee::RadarBuffStatus data{};
  data.sentry_posture = 3U;
  return data;
}

// [解析] ↓ 6 个 normalize_radar_xxx()：对已拷进来的结构体做字段级清洗，把"全 1"换成默认值。
// [解析] 只换坏字段、不丢整帧：一帧里可能只有某台机器人没被雷达看到，其余字段仍是好数据。
// [解析] 坐标清洗：source 和 6 台机器人 × x/y 逐个把"全 1"（int16_t 的 -1）换成 0。
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

// [解析] 血量清洗：5 个血量字段的 0xFFFF 逐个换成 999，与上面的默认值保持一致。
void normalize_radar_hp(sp::referee::RadarAllyHp & data)
{
  data.hero_hp = value_or_default(data.hero_hp, 999U);
  data.engineer_hp = value_or_default(data.engineer_hp, 999U);
  data.infantry3_hp = value_or_default(data.infantry3_hp, 999U);
  data.infantry4_hp = value_or_default(data.infantry4_hp, 999U);
  data.sentry_hp = value_or_default(data.sentry_hp, 999U);
}

// [解析] 弹量清洗：5 个允许发弹量字段的 0xFFFF 逐个换成 999。
void normalize_radar_ammo(sp::referee::RadarAllyAmmo & data)
{
  data.hero_ammo = value_or_default(data.hero_ammo, 999U);
  data.infantry3_ammo = value_or_default(data.infantry3_ammo, 999U);
  data.infantry4_ammo = value_or_default(data.infantry4_ammo, 999U);
  data.aerial_ammo = value_or_default(data.aerial_ammo, 999U);
  data.sentry_ammo = value_or_default(data.sentry_ammo, 999U);
}

// [解析] 场地清洗：两个金币字段的 0xFFFF 换成 9999，status_flags 换成 0。
void normalize_radar_field(sp::referee::RadarAllyField & data)
{
  data.remain_coins = value_or_default(data.remain_coins, 9999U);
  data.total_coins = value_or_default(data.total_coins, 9999U);
  data.status_flags = value_or_default(data.status_flags, 0U);
}

// [解析] 增益清洗：5 台机器人 × 5 类增益 + 6 个状态字段的 0xFF 换成 0，哨兵姿态换成 3。
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
// [解析] 构造函数只清内存、设默认值，完全不碰硬件；真正开始收数据必须显式调 request()。
PM02::PM02(UART_HandleTypeDef * huart, bool use_dma) : huart(huart), use_dma_(use_dma)
{
  // [解析] 以第一个成员和最后一个成员的地址框出清零区间：中间整块内存一次 memset 成 0。
  // [解析] 前提是这些成员连续声明；以后调整声明顺序或往中间插新成员，清零范围就会静默出错。
  uint8_t * start = reinterpret_cast<uint8_t *>(&this->game_status);
  uint8_t * end = reinterpret_cast<uint8_t *>(&this->custom_info) + sizeof(this->custom_info);
  if (end > start) {
    std::memset(start, 0, static_cast<size_t>(end - start));
  }

  std::memset(this->buff_, 0, sizeof(this->buff_));
  for (auto & mb : this->multi_buff_) {
    std::memset(mb.data(), 0, mb.size());
  }

  // [解析] 7 组雷达数据声明在 custom_info 之后，不在上面那段 memset 范围内，这里显式赋默认值。
  this->radar_enemy_dart_warning = default_radar_enemy_dart_warning();
  this->radar_aerial_countered = default_radar_aerial_countered();
  this->enemy_robot_position = default_radar_position();
  this->radar_ally_hp = default_radar_hp();
  this->radar_ally_ammo = default_radar_ammo();
  this->radar_ally_field = default_radar_field();
  this->radar_buff_status = default_radar_buff_status();
}

// [解析] 挂上一次串口接收，一次只收一包：每次 update() 处理完都必须再调一次，否则收一包就断流。
// [解析] 用 ReceiveToIdle 而不是普通 Receive：帧长不固定，总线静默（一帧发完）就触发回调，天然完成分帧。
void PM02::request()
{
  if (use_dma_) {
    // dismiss return
    HAL_UARTEx_ReceiveToIdle_DMA(this->huart, buff_, PM02_BUFF_SIZE);
    // [解析] 异步：函数立刻返回，数据由 DMA 在后台搬进 buff_，搬满或串口空闲时才进回调。

    // ref: https://github.com/HNUYueLuRM/basic_framework/blob/master/bsp/usart/bsp_usart.c
    __HAL_DMA_DISABLE_IT(this->huart->hdmarx, DMA_IT_HT);
    // [解析] 关掉 DMA 半传输中断：缓冲 255 字节，搬到一半（约 127 字节）也会触发一次回调，那时一帧往往还没收完。
    // [解析] hdmarx 是本串口对应的 DMA 接收流句柄。
  }
  else {
    // dismiss return
    HAL_UARTEx_ReceiveToIdle_IT(this->huart, buff_, PM02_BUFF_SIZE);
  }
}

// [解析] 串口事件回调的入口：先做一次雷达数据超时兜底（HAL_GetTick() 取当前毫秒），再解析这 size 个字节。
// [解析] 会改写各数据成员、_valid 和 _last_update_ms；不清缓冲，下一包整体覆盖。
void PM02::update(uint16_t size)
{
  // [解析] 即使裁判系统断流、本函数不再被调用，任务里周期调用的 update_radar_data_timeout() 仍能兜住超时。
  update_radar_data_timeout(HAL_GetTick());
  update(buff_, size);
}

// [解析] 真正的拆帧函数，4 道关卡依次否决：剩余长度够帧头 -> 首字节是 SOF(0xA5) -> 帧头 CRC8 -> 整帧 CRC16。
// [解析] 关卡顺序不能乱：数据长度就写在帧头里，帧头不可信就算不出整帧长度，也就做不了 CRC16。
// [解析] frame_start 是本帧起点（递归时后移），size 是从该点起还剩多少字节。
void PM02::update(uint8_t * frame_start, uint16_t size)
{
  if (size < referee::HEAD_LEN) return;
  if (frame_start[0] != referee::SOF) return;
  // [解析] check_crc8 的 len 包含 CRC8 本身，所以传 HEAD_LEN = 5（SOF 1 + 长度 2 + 序号 1 + CRC8 1）。
  if (!check_crc8(frame_start, referee::HEAD_LEN)) return;

  // [解析] 数据段长度也是小端：[1] 是低字节、[2] 是高字节，写反了整帧长度就全错了。
  size_t data_len = (frame_start[2] << 8) | frame_start[1];
  // [解析] 整帧 = 帧头 5 + 命令码 2 + 数据段 data_len + 帧尾 CRC16 2；CRC16 本身也计入校验范围。
  size_t frame_len = referee::HEAD_LEN + referee::CMD_ID_LEN + data_len + referee::TAIL_LEN;

  // [解析] 长度字段被干扰成大数时 frame_len 会超过缓冲，先挡住，避免 CRC16 和 memcpy 越界。
  // [解析] 这里一 return，同一缓冲里排在后面的帧也会一起丢掉 —— 偏安全侧的选择。
  if (frame_len > PM02_BUFF_SIZE) return;
  // [解析] 半包：剩余字节不够一整帧，等下一包补齐或对方重发。
  if (size < frame_len) return;
  if (!check_crc16(frame_start, frame_len)) return;

  // [解析] 命令码从下标 5 开始（帧头正好 5 字节），同样小端；数据段首地址就是 HEAD_LEN + CMD_ID_LEN。
  uint16_t cmd_id = (frame_start[6] << 8) | frame_start[5];
  const uint8_t * data = frame_start + referee::DATA_START;

  // [解析] 按 cmd_id 分派，每个 case 都是把数据段整段拷进对应成员。
  // [解析] 没有列在这里的命令码落到 default 被忽略，这是正常设计（协议的命令码非常多）。
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
    // [解析] 这是裁判系统直发路径（0x0A05 无线链路）；它和 0x0301/0x0212 的 Buff 块写同一个成员，谁后到以谁为准。
    case referee::cmd_id::RADAR_BUFF_STATUS:
      // [解析] 长度必须和 packed 结构体大小完全一样才敢 memcpy。
      if (data_len == sizeof(this->radar_buff_status)) {
        // [解析] 先拷进临时变量、清洗掉"全 1"字段再赋给成员，上层就不会看到半成品。
        referee::RadarBuffStatus received{};
        std::memcpy(&received, data, sizeof(received));
        normalize_radar_buff_status(received);
        this->radar_buff_status = received;
        this->radar_buff_status_valid = true;
        this->radar_buff_status_last_update_ms = HAL_GetTick();
      }
      else {
        this->radar_buff_status = default_radar_buff_status();
        this->radar_buff_status_valid = false;
      }
      break;
    // 0x0301 雷达自定义子编码：0x0210~0x0213。
    // [解析] 0x0301 是"机器人交互数据"总命令码，数据段开头还有 6 字节交互头：内容ID + 发送者ID + 接收者ID。
    // [解析] 用户载荷从第 7 字节起，按子编码 0x0210~0x0213 分派；外面这对 {} 只是给局部变量限定作用域。
    case referee::cmd_id::ROBOT_INTERACTION_DATA: {
      if (data_len < INTERACTION_HEADER_LEN) break;

      const uint16_t data_cmd_id = read_u16_le(data);
      const uint16_t receiver_id = read_u16_le(data + 4);

      // [解析] 只处理"发给本机"的包，本机 ID 取自 0x0201 下发的 robot_status.robot_id。
      // [解析] 0x0201 还没到时 robot_id 为 0，上电早期几乎全被过滤 —— 拿不到雷达数据属于正常现象。
      if (receiver_id != this->robot_status.robot_id) break;

      const uint8_t * payload = data + INTERACTION_HEADER_LEN;
      const size_t payload_len = data_len - INTERACTION_HEADER_LEN;
      // [解析] 时间只取一次，同一帧里所有 _last_update_ms 都用它，保证几组数据的时间戳一致。
      const uint32_t now_ms = HAL_GetTick();

      switch (data_cmd_id) {
        // [解析] 0x0210 敌方飞镖预警：先清 _valid，只有长度正确且拷贝成功才重新置 true，不会留下"数据仍有效"的假象。
        case referee::data_cmd_id::RADAR_ENEMY_DART_WARNING_CMD:
          this->radar_enemy_dart_warning_valid = false;
          if (payload_len == sizeof(referee::RadarEnemyDartWarning)) {
            referee::RadarEnemyDartWarning received{};
            std::memcpy(&received, payload, sizeof(received));
            received.dart_gate_status = value_or_default(received.dart_gate_status, 0U);
            this->radar_enemy_dart_warning = received;
            this->radar_enemy_dart_warning_valid = true;
            this->radar_enemy_dart_warning_last_update_ms = now_ms;
          }
          else {
            this->radar_enemy_dart_warning = default_radar_enemy_dart_warning();
          }
          break;

        // [解析] 0x0211 机器人坐标（source + 6 台机器人 × x/y）：int16_t，单位由雷达侧约定，中间件只清洗无效值不换算。
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

        // [解析] 0x0212 一帧内含 HP/弹药/场地/Buff 四块（69 字节），四组同生共死：一起置 true，长度不对一起恢复默认。
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

        // [解析] 0x0213 对方空中机器人被反制状态（1 字节，0xFF 同样清洗成 0）。
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

        // [解析] 其它子编码（例如选手端 UI 绘制 0x0100~0x0104）本模块不处理。
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
    // [解析] 下面三行是作者有意注释掉的（pm02.hpp 里对应成员也被注释），不是漏写，这里收不到 0x0306 的数据。
    // case referee::cmd_id::CUSTOM_CLIENT_DATA:
    //   copy_fixed(this->custom_client_data, data, data_len);
    //   break;
    // 0x0307 选手端小地图接收哨兵数据
    case referee::cmd_id::MAP_DATA:
      copy_fixed(this->map_data, data, data_len);
      break;
    // 0x0308 选手端小地图接收机器人数据
    case referee::cmd_id::CUSTOM_INFO:
      copy_fixed(this->custom_info, data, data_len);
      break;
    // [解析] 其余命令码（含本模块未实现的 0x0302/0x0306/0x0309）落到这里直接忽略，对应成员保持上一次的值。
    default:
      break;
  }

  // 递归解析，因为缓冲区中可能包含多帧裁判系统的数据。
  update(frame_start + frame_len, size - frame_len);
  // [解析] 一包串口数据里可能叠了好几帧：当前帧处理完后起点后移 frame_len、剩余长度减去 frame_len，再调自己解析下一帧。
  // [解析] 递归终止条件就是开头的关卡（剩余长度连帧头都不够）；单帧最长 255、缓冲也是 255，深度有天然上限，压不爆中断栈。
}

// [解析] 往裁判系统发数据（UI 图形包、自定义交互包），返回 HAL 状态码，HAL_OK 表示成功。
// [解析] use_dma_ 为 true 时走 DMA、立刻返回，data 指向的缓冲区在整包发完前不能被改写。
HAL_StatusTypeDef PM02::send(const uint8_t * data, size_t size)
{
  if (use_dma_) {
    return HAL_UART_Transmit_DMA(this->huart, data, size);
  }

  // [解析] 非 DMA 版阻塞发送：0xFF 是 255ms 超时（不是 HAL_MAX_DELAY），低速波特率下发整屏 UI 包可能只发出去一部分。
  return HAL_UART_Transmit(this->huart, data, size, 0xFF);
}

// [解析] 周期调用的看门狗：对 7 组雷达数据逐个判新鲜，不新鲜就恢复默认值并清掉 _valid。
// [解析] 宁可对外表现为"数据无效"，也不让上层拿过期数据做决策（例如用 10 秒前的坐标去瞄准）。
// [解析] 链路一直正常时本函数什么也不改，只是反复判断一遍。
void PM02::update_radar_data_timeout(uint32_t now_ms, uint32_t timeout_ms)
{
  if (!radar_enemy_dart_warning_fresh(now_ms, timeout_ms)) {
    this->radar_enemy_dart_warning = default_radar_enemy_dart_warning();
    this->radar_enemy_dart_warning_valid = false;
  }

  if (!radar_aerial_countered_fresh(now_ms, timeout_ms)) {
    this->radar_aerial_countered = default_radar_aerial_countered();
    this->radar_aerial_countered_valid = false;
  }

  if (!enemy_robot_position_fresh(now_ms, timeout_ms)) {
    this->enemy_robot_position = default_radar_position();
    this->enemy_robot_position_valid = false;
  }

  if (!radar_ally_hp_fresh(now_ms, timeout_ms)) {
    this->radar_ally_hp = default_radar_hp();
    this->radar_ally_hp_valid = false;
  }

  if (!radar_ally_ammo_fresh(now_ms, timeout_ms)) {
    this->radar_ally_ammo = default_radar_ammo();
    this->radar_ally_ammo_valid = false;
  }

  if (!radar_ally_field_fresh(now_ms, timeout_ms)) {
    this->radar_ally_field = default_radar_field();
    this->radar_ally_field_valid = false;
  }

  if (!radar_buff_status_fresh(now_ms, timeout_ms)) {
    this->radar_buff_status = default_radar_buff_status();
    this->radar_buff_status_valid = false;
  }
}

// [解析] ↓ 7 个 xxx_fresh()：_valid 为 true 且距上次更新不超过 timeout_ms，返回 true 表示这组数据现在可以信任。
// [解析] now_ms - _last_update_ms 靠 uint32_t 自然回绕：HAL_GetTick() 约 49.7 天溢出一次，回绕之后照样算得对。
// [解析] 因此不需要额外的溢出判断，这是嵌入式里判超时的通用写法。
bool PM02::radar_enemy_dart_warning_fresh(uint32_t now_ms, uint32_t timeout_ms) const
{
  return this->radar_enemy_dart_warning_valid &&
         static_cast<uint32_t>(now_ms - this->radar_enemy_dart_warning_last_update_ms) <=
           timeout_ms;
}

bool PM02::radar_aerial_countered_fresh(uint32_t now_ms, uint32_t timeout_ms) const
{
  return this->radar_aerial_countered_valid &&
         static_cast<uint32_t>(now_ms - this->radar_aerial_countered_last_update_ms) <= timeout_ms;
}

bool PM02::enemy_robot_position_fresh(uint32_t now_ms, uint32_t timeout_ms) const
{
  return this->enemy_robot_position_valid &&
         static_cast<uint32_t>(now_ms - this->enemy_robot_position_last_update_ms) <= timeout_ms;
}

bool PM02::radar_ally_hp_fresh(uint32_t now_ms, uint32_t timeout_ms) const
{
  return this->radar_ally_hp_valid &&
         static_cast<uint32_t>(now_ms - this->radar_ally_hp_last_update_ms) <= timeout_ms;
}

bool PM02::radar_ally_ammo_fresh(uint32_t now_ms, uint32_t timeout_ms) const
{
  return this->radar_ally_ammo_valid &&
         static_cast<uint32_t>(now_ms - this->radar_ally_ammo_last_update_ms) <= timeout_ms;
}

bool PM02::radar_ally_field_fresh(uint32_t now_ms, uint32_t timeout_ms) const
{
  return this->radar_ally_field_valid &&
         static_cast<uint32_t>(now_ms - this->radar_ally_field_last_update_ms) <= timeout_ms;
}

bool PM02::radar_buff_status_fresh(uint32_t now_ms, uint32_t timeout_ms) const
{
  return this->radar_buff_status_valid &&
         static_cast<uint32_t>(now_ms - this->radar_buff_status_last_update_ms) <= timeout_ms;
}

}  // namespace sp
