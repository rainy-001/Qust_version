// [解析] 裁判系统串口协议字典：把官方手册里的命令码和数据包结构体，逐条翻译成常量与 struct。
// [解析] referee/pm02（主串口）、vt03（图传链路）、ui/manager、ui/element 都引用它；本文件没有可执行代码。
// [解析] 真正"怎么收、怎么校验、怎么分发"在 pm02.cpp 与 vt03.cpp 里，这里只描述"数据长什么样"。
// [解析] 一条主线：SOF/FrameHeader 定义一帧 -> cmd_id 说明这帧是什么事 -> 后面的结构体是这件事的内容。
// [解析] 所有数据包都带 packed，长度和偏移由文件末尾那批 static_assert 在编译期钉死。

#ifndef SP__REFEREE_PROTOCOL_HPP
#define SP__REFEREE_PROTOCOL_HPP

#include <cstddef>
#include <cstdint>

// [解析] robot_id：裁判系统给每个兵种单位分配的编号，红方 1~11，蓝方在红方基础上 +100。
// [解析] 10/11 是基地、110/111 是前哨站等固定建筑；同一套编号也出现在判罚包的违规者字段里。
namespace sp::referee::robot_id
{
constexpr uint8_t RED_HERO = 1;
constexpr uint8_t RED_ENGINEER = 2;
constexpr uint8_t RED_STANDARD_3 = 3;
constexpr uint8_t RED_STANDARD_4 = 4;
constexpr uint8_t RED_STANDARD_5 = 5;
constexpr uint8_t RED_AERIAL = 6;
constexpr uint8_t RED_SENTRY = 7;
constexpr uint8_t RED_DART = 8;
constexpr uint8_t RED_RADAR = 9;
constexpr uint8_t RED_BASE = 10;
constexpr uint8_t RED_OUTPOST = 11;
constexpr uint8_t BLUE_HERO = 101;
constexpr uint8_t BLUE_ENGINEER = 102;
constexpr uint8_t BLUE_STANDARD_3 = 103;
constexpr uint8_t BLUE_STANDARD_4 = 104;
constexpr uint8_t BLUE_STANDARD_5 = 105;
constexpr uint8_t BLUE_AERIAL = 106;
constexpr uint8_t BLUE_SENTRY = 107;
constexpr uint8_t BLUE_DART = 108;
constexpr uint8_t BLUE_RADAR = 109;
constexpr uint8_t BLUE_BASE = 110;
constexpr uint8_t BLUE_OUTPOST = 111;
}  // namespace sp::referee::robot_id

// [解析] client_id：选手端/图传客户端/自定义控制器的身份编号，出现在 0x0301 的 sender_id 与 receiver_id。
// [解析] 换算关系 client_id = 0x0100 + robot_id，红蓝两侧取值不相交，收到一帧就能判断来源。
namespace sp::referee::client_id
{
constexpr uint16_t RED_HERO_CLIENT = 0x0101;
constexpr uint16_t RED_ENGINEER_CLIENT = 0x0102;
constexpr uint16_t RED_STANDARD_3_CLIENT = 0x0103;
constexpr uint16_t RED_STANDARD_4_CLIENT = 0x0104;
constexpr uint16_t RED_STANDARD_5_CLIENT = 0x0105;
constexpr uint16_t RED_AERIAL_CLIENT = 0x0106;
constexpr uint16_t BLUE_HERO_CLIENT = 0x0165;
constexpr uint16_t BLUE_ENGINEER_CLIENT = 0x0166;
constexpr uint16_t BLUE_STANDARD_3_CLIENT = 0x0167;
constexpr uint16_t BLUE_STANDARD_4_CLIENT = 0x0168;
constexpr uint16_t BLUE_STANDARD_5_CLIENT = 0x0169;
constexpr uint16_t BLUE_AERIAL_CLIENT = 0x016A;
constexpr uint16_t REFEREE_SERVER = 0x8080;
}  // namespace sp::referee::client_id

// [解析] cmd_id：命令码，位于帧头之后 2 字节（小端），决定数据段该按哪个结构体解释。
// [解析] 按高位分段：0x000x 全局比赛信息、0x010x 场地事件、0x020x 自身状态、0x030x 交互数据、
// [解析]   0x0A0x 雷达站、0x0F0x 图传信道；新码值先看落在哪一段，就知道内容归谁管。
namespace sp::referee::cmd_id
{
constexpr uint16_t GAME_STATUS = 0x0001;    // 比赛状态数据
constexpr uint16_t GAME_RESULT = 0x0002;    // 比赛结果数据
constexpr uint16_t GAME_ROBOT_HP = 0x0003;  // 比赛机器人血量数据

constexpr uint16_t EVENT_DATA = 0x0101;       // 场地事件数据
constexpr uint16_t REFEREE_WARNING = 0x0104;  // 裁判警告数据
constexpr uint16_t DART_INFO = 0x0105;        // 飞镖发射相关数据

constexpr uint16_t ROBOT_STATUS = 0x0201;           // 机器人状态数据
constexpr uint16_t POWER_HEAT_DATA = 0x0202;        // 实时底盘缓冲能量和射击热量数据
constexpr uint16_t ROBOT_POS = 0x0203;              // 机器人位置数据
constexpr uint16_t BUFF = 0x0204;                   // 机器人增益数据
                                                    // 没有 0x0205
constexpr uint16_t HURT_DATA = 0x0206;              // 伤害状态数据
constexpr uint16_t SHOOT_DATA = 0x0207;             // 实时射击数据
constexpr uint16_t PROJECTILE_ALLOWANCE = 0x0208;   // 允许发弹量数据
constexpr uint16_t RFID_STATUS = 0x0209;            // 机器人 RFID 模块状态
constexpr uint16_t DART_CLIENT_CMD = 0x020A;        // 飞镖选手端指令数据
constexpr uint16_t GROUND_ROBOT_POSITION = 0x020B;  // 地面机器人位置数据
constexpr uint16_t RADAR_MARK_DATA = 0x020C;        // 雷达标记进度数据
constexpr uint16_t SENTRY_INFO = 0x020D;            // 哨兵自主决策信息同步
constexpr uint16_t RADAR_INFO = 0x020E;             // 雷达自主决策信息同步

// [解析] 0x0301 是双向的"信封"，真正的类型在包内 data_cmd_id；它走图传链路的串口，收错口就一直接不到数据。
constexpr uint16_t ROBOT_INTERACTION_DATA = 0x0301;  // 机器人交互数据
constexpr uint16_t CUSTOM_ROBOT_DATA = 0x0302;       // 自定义控制器与机器人交互数据 图传链路
constexpr uint16_t MAP_COMMAND = 0x0303;             // 选手端小地图交互数据
constexpr uint16_t MAP_ROBOT_DATA = 0x0305;          // 选手端小地图接收雷达数据
constexpr uint16_t CUSTOM_CLIENT_DATA = 0x0306;      // 自定义控制器与选手端交互数据
constexpr uint16_t MAP_DATA = 0x0307;                // 选手端小地图接收哨兵数据
constexpr uint16_t CUSTOM_INFO = 0x0308;             // 选手端小地图接收机器人数据
constexpr uint16_t ROBOT_CUSTOM_DATA = 0x0309;       // 自定义控制器接收机器人数据 图传链路
constexpr uint16_t ROBOT_CLIENT_DATA =
  0x0310;  // 机器人发送给自定义客户端的数据 图传链路，最大 300 字节
constexpr uint16_t CLIENT_ROBOT_DATA = 0x0311;  // 自定义客户端发送给机器人的自定义指令，最大 30 字节

constexpr uint16_t RADAR_ROBOT_POS = 0x0A01;         // 雷达站发送机器人坐标数据 无线链路
constexpr uint16_t RADAR_ROBOT_HP = 0x0A02;          // 雷达站发送机器人血量数据 无线链路
constexpr uint16_t RADAR_SENTRY_ALLOWANCE = 0x0A03;  // 雷达站发送哨兵自主兑换允许发弹量 无线链路
constexpr uint16_t RADAR_ROBOT_STATUS = 0x0A04;      // 雷达站发送机器人状态数据 无线链路
constexpr uint16_t RADAR_BUFF_STATUS = 0x0A05;       // 雷达站发送增益点状态数据 无线链路
constexpr uint16_t RADAR_MARK_INFO = 0x0A06;         // 雷达站发送敌方机器人标记进度 无线链路
constexpr uint16_t SET_IMAGE_TRANSFER = 0x0F01;      // 设置图传出图信道 图传链路
constexpr uint16_t INQUIRY_IMAGE_TRANSFER = 0x0F02;  // 查询图传出图信道 图传链路
}  // namespace sp::referee::cmd_id

// [解析] data_cmd_id：二级命令码，只有 0x0301 的包内头 2 字节 —— 外层是信封，它是信纸类型。
// [解析] 判断流程恒为"cmd_id == 0x0301 -> 读 data_cmd_id -> 再选结构体"。
// [解析] 两个命名空间的数值会撞车（0x0101 在 cmd_id 里是场地事件，在这里是画图形），不能只看数值。
namespace sp::referee::data_cmd_id
{
// 雷达通过 0x0301 发送的自定义子编码。
constexpr uint16_t RADAR_ENEMY_DART_WARNING_CMD = 0x0210;
constexpr uint16_t RADAR_SENTRY_POSITION_CMD = 0x0211;
constexpr uint16_t RADAR_COMBINED_DATA_CMD = 0x0212;
constexpr uint16_t RADAR_AERIAL_COUNTERED_CMD = 0x0213;
// 0x0200~0x02FF 机器人之间通信 TODO
constexpr uint16_t INTERACTION_LAYER_DELETE = 0x0100;     // 选手端删除图层
// [解析] 0x0101~0x0104 是"一次画 1/2/5/7 个图形"的四个版本，包的数据区长度固定；
// [解析] 要画几个就挑数量匹配的那条发，不足的位置必须补零，否则选手端会把残留字节当成图形。
constexpr uint16_t INTERACTION_FIGURE = 0x0101;           // 选手端绘制一个图形
constexpr uint16_t INTERACTION_FIGURE_2 = 0x0102;         // 选手端绘制两个图形
constexpr uint16_t INTERACTION_FIGURE_5 = 0x0103;         // 选手端绘制五个图形
constexpr uint16_t INTERACTION_FIGURE_7 = 0x0104;         // 选手端绘制七个图形
constexpr uint16_t EXT_CLIENT_CUSTOM_CHARACTER = 0x0110;  // 选手端绘制字符图形
constexpr uint16_t SENTRY_CMD = 0x0120;                   // 哨兵自主决策指令 TODO
constexpr uint16_t RADAR_CMD = 0x0121;                    // 雷达自主决策指令 TODO
}  // namespace sp::referee::data_cmd_id

namespace sp::referee
{
// [解析] SOF = 0xA5（0/1 交替，连续 0 或连续 1 时不易混淆），但数据段里也可能出现同样的字节，
// [解析] 所以收帧要靠 data_len + CRC8 + CRC16 三道关卡一起判，不能一见 0xA5 就当新帧。
constexpr uint8_t SOF = 0xA5;

// [解析] FrameHeader：固定 5 字节帧头，字段顺序必须与手册一致；整帧长度 = 5 + 2 + data_len + 2。
// [解析] packed 禁止编译器插对齐空隙，代价是多字节成员可能落在奇数地址，不要取地址做强转。
struct __attribute__((packed)) FrameHeader
{
  uint8_t sof;
  uint16_t data_len;
  uint8_t seq;
  uint8_t crc8;
};

// [解析] DATA_START = 帧头 5 + 命令码 2 = 7，即数据段在整帧里的起始偏移。
// [解析] 拷数据前必须先过 CRC 并确认 cmd_id 匹配，否则 memcpy 进来的是垃圾。
constexpr size_t HEAD_LEN = sizeof(FrameHeader);
constexpr size_t CMD_ID_LEN = 2;
constexpr size_t TAIL_LEN = 2;
constexpr size_t DATA_START = HEAD_LEN + CMD_ID_LEN;

// 0x0001 比赛状态数据
// [解析] GameStatus（0x0001）：比赛类型、当前阶段、阶段剩余秒数、NTP 时间戳，packed 后 11 字节。
// [解析] game_type 占低 4 位、game_progress 占高 4 位，共用一个字节；判断"在比赛中"看 game_progress == 4。
// [解析] sync_timestamp 要等机器人连上裁判系统 NTP 服务器才有意义，没连上时别拿它做判断。
struct __attribute__((packed)) GameStatus
{
  uint8_t game_type : 4;
  // bit 0-3: 比赛类型
  //       1: RoboMaster机甲大师超级对抗赛
  //       2: RoboMaster机甲大师高校单项赛
  //       3: ICRA RoboMaster高校人工智能挑战赛
  //       4: RoboMaster机甲大师高校联盟赛3V3对抗
  //       5: RoboMaster机甲大师高校联盟赛步兵对抗
  uint8_t game_progress : 4;
  // bit 4-7: 当前比赛阶段
  //       0: 未开始比赛
  //       1: 准备阶段
  //       2: 十五秒裁判系统自检阶段
  //       3: 五秒倒计时
  //       4: 比赛中
  //       5: 比赛结算中
  uint16_t stage_remain_time;  // 当前阶段剩余时间，单位：s
  uint64_t sync_timestamp;     // UNIX 时间，当机器人正确连接到裁判系统 NTP 服务器后生效
};

// 0x0002 比赛结果数据
// [解析] GameResult（0x0002）：比赛结束后才下发的胜负结果，只有 1 字节。
struct __attribute__((packed)) GameResult
{
  uint8_t winner;  // 0: 平局, 1: 红方胜利, 2: 蓝方胜利
};

// 0x0003 比赛机器人血量数据
// [解析] GameRobotHP（0x0003）：场上各单位血量与己方伤害差值，packed 后 20 字节。
// [解析] 字段名里的数字是兵种序号（协议没有 5、6 号，所以跳号）；damage_difference 有符号，落后时为负。
struct __attribute__((packed)) GameRobotHP
{
  uint16_t ally_1_robot_hp;  // 己方 1 英雄机器人血量
  uint16_t ally_2_robot_hp;  // 己方 2 工程机器人血量
  uint16_t ally_3_robot_hp;  // 己方 3 步兵机器人血量
  uint16_t ally_4_robot_hp;  // 己方 4 步兵机器人血量
  int16_t damage_difference;  // 己方伤害量相对对方伤害量的差值
  uint16_t ally_7_robot_hp;   // 己方 7 哨兵机器人血量
  uint16_t ally_outpost_hp;   // 己方前哨站血量
  uint16_t ally_base_hp;      // 己方基地血量
  uint16_t enemy_outpost_hp;  // 对方前哨站血量
  uint16_t enemy_base_hp;     // 对方基地血量
};

// 0x0101 场地事件数据
// [解析] EventData（0x0101）：4 字节里塞了 11 段位域，描述补给区/能量机关/高地的占领状态和飞镖命中。
// [解析] 位域从低位向高位连续排，位数相加 3+4+2+2+9+3+2+2+2+1+2 = 32，多一位少一位后面全错位。
// [解析] 取值含义见下方原注释（0 未占领 / 1 己方 / 2 对方），不要从字段名猜。
struct __attribute__((packed)) EventData
{
  // 0: 未占领/未激活
  // 1: 已占领/已激活
  uint32_t supply_status : 3;  // bit 0-2：己方补给区占领状态
  //       bit 0：己方补给区的占领状态，1 为已占领
  //       bit 1：保留
  //       bit 2：己方补给区的占领状态，1 为已占领（仅 RMUL 适用）
  uint32_t energy_status : 4;  // bit 3-6：己方小/大能量机关激活状态
  //       bit 3-4：己方小能量机关状态，0 未激活，1 已激活，2 正在激活
  //       bit 5-6：己方大能量机关状态，0 未激活，1 已激活，2 正在激活
  uint32_t central_highground_status : 2;      // bit 7-8：己方中央高地占领状态
  //       0：未被占领 1：被己方占领 2：被对方占领
  uint32_t trapezoidal_highground_status : 2;  // bit 9-10：己方梯形高地占领状态
  //       0：未被占领 1：被己方占领 2：被对方占领
  uint32_t last_hit_time : 9;                  // bit 11-19：对方飞镖最后一次命中时间
  //       0-420，单位：s，开局默认为 0
  uint32_t last_hit_target : 3;                // bit 20-22：对方飞镖最后一次命中目标
  //       0：开局默认值
  //       1：前哨站
  //       2：基地固定目标
  //       3：基地随机固定目标
  //       4：基地随机移动目标
  //       5：基地末端移动目标
  uint32_t center_boost_status : 2;            // bit 23-24：中心增益点占领状态
  //       0：未被占领 1：被己方占领 2：被对方占领 3：被双方占领（仅 RMUL 适用）
  uint32_t fortess_boost_status : 2;           // bit 25-26：己方堡垒增益点占领状态
  //       0：未被占领 1：被己方占领 2：被对方占领 3：被双方占领（仅 RMUL 适用）
  uint32_t outpost_boost_status : 2;           // bit 27-28：己方前哨站增益点占领状态
  //       0：未被占领 1：被己方占领 2：被对方占领
  uint32_t base_boost_status : 1;              // bit 29：己方基地增益点占领状态
  //       0：未被占领 1：已占领
  uint32_t reserved : 2;                       // bit 30-31：保留
};

// 0x0104 裁判警告数据
// [解析] RefereeWarning（0x0104）：3 字节，描述"己方最后一次受到的判罚"，不是累加统计。
// [解析] level 是判罚等级（1 双方黄牌 / 2 双方红牌 / 3 判负，以当季手册为准）。
// [解析] offending_robot_id 用的就是上面 robot_id 那套编号，收到后可直接在 UI 上报警。
struct __attribute__((packed)) RefereeWarning
{
  uint8_t level;               // 己方最后一次受到判罚的等级
  uint8_t offending_robot_id;  // 己方最后一次受到判罚的违规机器人 ID
  uint8_t count;               // 己方最后一次受到判罚的违规次数
};

// 0x0105 飞镖发射相关数据
// [解析] DartInfo（0x0105）：1 字节剩余时间 + 16 位位域包，共 3 字节；不加 packed 会被撑成 4 字节。
// [解析] 位域是"命中目标(bit 0-2) / 累计命中次数(bit 3-5) / 当前选中目标(bit 6-8)"，其余保留。
struct __attribute__((packed)) DartInfo
{
  uint8_t dart_remaining_time;  // 己方飞镖发射站剩余发射时间，单位：s
  uint16_t dart_info;
  // bit 0-2：对方飞镖最后一次击中目标
  //       0：开局默认值
  //       1：前哨站
  //       2：基地固定目标
  //       3：基地随机固定目标
  //       4：基地随机移动目标
  //       5：基地末端移动目标
  // bit 3-5：对方最近被击中目标的累计次数
  //       开局默认为 0，至多为 4
  // bit 6-8：当前飞镖选择的击打目标
  //       0：开局默认值
  //       1：基地固定目标
  //       2：基地随机固定目标
  //       3：基地随机移动目标
  //       4：基地末端移动目标
  // bit 9-15：保留
};

// 0x0201 机器人状态数据
// [解析] RobotStatus（0x0201）：本机等级、当前/最大血量、枪口热量上限与冷却、底盘功率上限、弹速上限，17 字节。
// [解析] 做发弹限制、热量限制、超电功率控制都要用它；两处热量单位是"热量值"不是摄氏度。
// [解析] bullet_speed_limit 是 float，前面若不加 packed 编译器会插空隙，长度立刻对不上。
struct __attribute__((packed)) RobotStatus
{
  uint8_t robot_id;                       // 机器人 ID
  uint8_t robot_level;                    // 机器人等级
  uint16_t current_hp;                    // 当前血量
  uint16_t maximum_hp;                    // 血量上限
  uint16_t shooter_barrel_cooling_value;  // 枪口热量每秒冷却值
  uint16_t shooter_barrel_heat_limit;     // 枪口热量上限
  uint16_t chassis_power_limit;           // 底盘功率上限，单位：W
  float bullet_speed_limit;               // 弹丸初速度上限，单位：m/s
  uint8_t power_management_gimbal_output : 1;   // gimbal 口输出: 0 为无输出, 1 为 24V 输出
  uint8_t power_management_chassis_output : 1;  // chassis 口输出: 0 为无输出, 1 为 24V 输出
  uint8_t power_management_shooter_output : 1;  // shooter 口输出: 0 为无输出, 1 为 24V 输出
  //       bit 0：gimbal 口输出状态
  //       bit 1：chassis 口输出状态
  //       bit 2：shooter 口输出状态
  uint8_t reserved : 5;                         // bit 3-7：保留
};

// 0x0202 实时底盘缓冲能量和射击热量数据
// [解析] PowerHeatData（0x0202）：缓冲能量(J) 与两个发射机构的当前热量，超电功率控制的主要输入。
// [解析] 前三个 reserved_* 是协议废弃字段，但仍占 8 字节，删掉会让后面字段全部前移。
struct __attribute__((packed)) PowerHeatData
{
  uint16_t reserved_1;
  uint16_t reserved_2;
  float reserved_3;
  uint16_t buffer_energy;               // 缓冲能量，单位：J
  uint16_t shooter_17mm_1_barrel_heat;  // 第 1 个 17mm 发射机构的射击热量
  uint16_t shooter_42mm_barrel_heat;    // 42mm 发射机构的射击热量
};

// 0x0203 机器人位置数据
// [解析] RobotPos（0x0203）：本机 x/y 坐标(m) 与朝向(度)，3 个 float 共 12 字节。
// [解析] 朝向正北为 0、顺时针增大，单位是度不是弧度；没装定位模块时字段可能是 0，别当真实位置。
struct __attribute__((packed)) RobotPos
{
  float x;      // 本机器人位置 x 坐标，单位：m
  float y;      // 本机器人位置 y 坐标，单位：m
  float angle;  // 本机器人测速模块朝向，单位：度，正北为 0 度
};

// 0x0204 机器人增益数据
// [解析] Buff（0x0204）：回血/冷却/防御/负防御/攻击五类增益，8 字节。
// [解析] 除 cooling_buff 是"每秒多冷却几点的直接值"，其余四项都是百分比（值 10 表示 10%）。
// [解析] remaining_energy 是位掩码不是数值，直接当数字读会拿到 0x80 这类和能量比例无关的数。
struct __attribute__((packed)) Buff
{
  uint8_t recovery_y_buff;  // 机器人回血增益（百分比，值为 10 表示每秒恢复血量上限的 10%）
  uint16_t cooling_buff;    // 机器人射击热量冷却增益具体值（直接值，值为 x 表示热量冷却增加 x/s）
  uint8_t defence_buff;     // 机器人防御增益（百分比，值为 50 表示 50%防御增益）
  uint8_t vulnerability_buff;  // 机器人负防御增益（百分比，值为 30 表示-30%防御增益）
  uint16_t attack_buff;        // 机器人攻击增益（百分比，值为 50 表示 50%攻击增益）
  uint8_t remaining_energy;
  // bit  0-6：机器人剩余能量值反馈，
  // 以 16 进制标识机器人剩余能量值比例，
  // 仅在机器人剩余能量小于 50%时反馈，其余默认反馈 0x80。机器人初始能量视为100%
  // bit:0: 在剩余能量≥125%时为 1，其余情况为 0
  // bit:1: 在剩余能量≥100%时为 1，其余情况为 0
  // bit 2：在剩余能量≥50%时为 1，其余情况为 0
  // bit 3：在剩余能量≥30%时为 1，其余情况为 0
  // bit 4：在剩余能量≥15%时为 1，其余情况为 0
  // bit 5：在剩余能量≥5%时为 1，其余情况为 0
  // bit 6：在剩余能量≥1%时为 1，其余情况为 0
};

// 0x0206 伤害状态数据
// [解析] HurtData（0x0206）：1 字节，低 4 位是装甲板/测速模块编号，高 4 位是扣血原因。
// [解析] 把原因打出来能区分"被打掉的"和"自己超热量/超功率被扣的"，后者说明控制参数要改。
struct __attribute__((packed)) HurtData
{
  uint8_t armor_id : 4;
  // bit 0-3：当扣血原因为装甲模块被弹丸攻击、受撞击、离线或测速模块离线时，
  // 该 4 bit 组成的数值为装甲模块或测速模块的 ID 编号；
  // 当其他原因导致扣血时，该数值为 0
  uint8_t HP_deduction_reason : 4;
  // bit 4-7：血量变化类型
  //       0：装甲模块被弹丸攻击导致扣血
  //       1：装甲模块或超级电容管理模块离线导致扣血
  //       2：射速超限导致扣血
  //       3：枪口热量超限导致扣血
  //       4：底盘功率超限导致扣血
  //       5：装甲模块受到撞击导致扣血
  //       6：测速模块离线导致扣血
};

// 0x0207 实时射击数据
// [解析] ShootData（0x0207）：弹种、发射机构编号、射频(Hz)、实测初速(m/s)，7 字节。
// [解析] 1+1+1+4 = 7；不加 packed 时 float 前会补 1 字节空隙，接收长度就对不上。
struct __attribute__((packed)) ShootData
{
  uint8_t bullet_type;          // 1: 17mm 弹丸, 2: 42mm 弹丸
  uint8_t shooter_number;       // 1: 第1个17mm发射机构, 2: 第2个17mm发射机构, 3: 42mm发射机构
  uint8_t launching_frequency;  // 射频，单位：Hz
  float initial_speed;          // 弹丸初速度，单位：m/s
};

// 0x0208 允许发弹量数据
// [解析] ProjectileAllowance（0x0208）：17mm/42mm 允许发弹量、金币、堡垒储备弹量，8 字节。
struct __attribute__((packed)) ProjectileAllowance
{
  uint16_t projectile_allowance_17mm;      // 17mm 弹丸允许发弹量
  uint16_t projectile_allowance_42mm;      // 42mm 弹丸允许发弹量
  uint16_t remaining_gold_coin;            // 剩余金币数量
  uint16_t projectile_allowance_fortress;  // 堡垒增益点提供的储备 17mm 弹丸允许发弹量
};

// 0x0209 机器人 RFID 模块状态
// [解析] RFID_Status（0x0209）：一个 bit 对应一个场地增益点，命名了 32 + 6 个，共 5 字节。
// [解析] 后 6 位必须用 uint8_t 当底层类型：换回 uint32_t 会另起存储单元，长度从 5 变 8、整包错位。
// [解析] 用法同 EventData，不必自己移位，例如 rfid.friendly_base == 1 就是"己方基地增益点已踩到"。
struct __attribute__((packed)) RFID_Status
{
  uint32_t friendly_base : 1;             // bit 0：己方基地增益点
  uint32_t friendly_central : 1;          // bit 1：己方中央高地增益点
  uint32_t enemy_central : 1;             // bit 2：对方中央高地增益点
  uint32_t friendly_trapezoid : 1;        // bit 3：己方梯形高地增益点
  uint32_t enemy_trapezoid : 1;           // bit 4：对方梯形高地增益点
  uint32_t friendly_flying_slope_f : 1;   // bit 5：己方飞坡前增益点
  uint32_t friendly_flying_slope_b : 1;   // bit 6：己方飞坡后增益点
  uint32_t enemy_flying_slope_f : 1;      // bit 7：对方飞坡前增益点
  uint32_t enemy_flying_slope_b : 1;      // bit 8：对方飞坡后增益点
  uint32_t friendly_ch_down : 1;          // bit 9：己方中央高地下方增益点
  uint32_t friendly_ch_up : 1;            // bit 10：己方中央高地上方增益点
  uint32_t enemy_ch_down : 1;             // bit 11：对方中央高地下方增益点
  uint32_t enemy_ch_up : 1;               // bit 12：对方中央高地上方增益点
  uint32_t friendly_road_down : 1;        // bit 13：己方公路下方增益点
  uint32_t friendly_road_up : 1;          // bit 14：己方公路上方增益点
  uint32_t enemy_road_down : 1;           // bit 15：对方公路下方增益点
  uint32_t enemy_road_up : 1;             // bit 16：对方公路上方增益点
  uint32_t friendly_fort : 1;             // bit 17：己方堡垒增益点
  uint32_t friendly_outpost : 1;          // bit 18：己方前哨站增益点
  uint32_t friendly_supply_no_trade : 1;  // bit 19：己方与兑换区不重叠的补给区
  uint32_t friendly_supply_trade : 1;     // bit 20：己方与兑换区重叠的补给区
  uint32_t friendly_big_resource : 1;     // bit 21：己方大资源岛增益点
  uint32_t enemy_big_resource : 1;        // bit 22：对方大资源岛增益点
  uint32_t center_point : 1;              // bit 23：中心增益点
  uint32_t enemy_fort : 1;                // bit 24：对方堡垒增益点
  uint32_t enemy_outpost : 1;             // bit 25：对方前哨站增益点
  uint32_t friendly_tunnel_road_down : 1;       // bit 26：己方隧道公路区下方
  uint32_t friendly_tunnel_road_mid : 1;        // bit 27：己方隧道公路区中间
  uint32_t friendly_tunnel_road_up : 1;         // bit 28：己方隧道公路区上方
  uint32_t friendly_tunnel_trapezoid_low : 1;   // bit 29：己方隧道梯形高地较低处
  uint32_t friendly_tunnel_trapezoid_mid : 1;   // bit 30：己方隧道梯形高地中间处
  uint32_t friendly_tunnel_trapezoid_high : 1;  // bit 31：己方隧道梯形高地较高处
  uint8_t enemy_tunnel_road_down : 1;           // bit 0：对方隧道公路区下方
  uint8_t enemy_tunnel_road_mid : 1;            // bit 1：对方隧道公路区中间
  uint8_t enemy_tunnel_road_up : 1;             // bit 2：对方隧道公路区上方
  uint8_t enemy_tunnel_trapezoid_low : 1;       // bit 3：对方隧道梯形高地较低处
  uint8_t enemy_tunnel_trapezoid_mid : 1;       // bit 4：对方隧道梯形高地中间处
  uint8_t enemy_tunnel_trapezoid_high : 1;      // bit 5：对方隧道梯形高地较高处
  uint8_t reserved : 2;                         // bit 6-7：保留
};

// 0x020A 飞镖选手端指令数据
// [解析] DartClientCmd（0x020A）：飞镖发射站开合状态与两个"比赛剩余时间"，6 字节。
// [解析] 开合状态是三态（0 已开启 / 1 关闭 / 2 动作中），判断能否发射要写 == 0，不能只判非零。
struct __attribute__((packed)) DartClientCmd
{
  uint8_t dart_launch_opening_status;  // 当前飞镖发射站状态：0 已开启，1 关闭，2 正在开启或关闭
  uint8_t reserved;                    // 保留
  uint16_t target_change_time;  // 切换击打目标时的比赛剩余时间，单位：秒，无/未切换动作，默认为 0。
  uint16_t
    latest_launch_cmd_time;  // 发送最新飞镖发射指令的比赛剩余时间，单位：秒，无/未发射，默认为 0。
};

// 0x020B 地面机器人位置数据
// [解析] GroundRobotPosition（0x020B）：己方英雄/工程/3 号/4 号步兵的坐标(m)，10 个 float 共 40 字节。
// [解析] 末尾两个 reserved 不能删，协议预留了位置，删了后面的数据会对不上。
struct __attribute__((packed)) GroundRobotPosition
{
  float hero_x;        // 己方英雄机器人位置 x 坐标，单位：m
  float hero_y;        // 己方英雄机器人位置 y 坐标，单位：m
  float engineer_x;    // 己方工程机器人位置 x 坐标，单位：m
  float engineer_y;    // 己方工程机器人位置 y 坐标，单位：m
  float standard_3_x;  // 己方 3 号步兵机器人位置 x 坐标，单位：m
  float standard_3_y;  // 己方 3 号步兵机器人位置 y 坐标，单位：m
  float standard_4_x;  // 己方 4 号步兵机器人位置 x 坐标，单位：m
  float standard_4_y;  // 己方 4 号步兵机器人位置 y 坐标，单位：m
  float reserved1;     // 保留
  float reserved2;     // 保留
};

// 0x020C 雷达标记进度数据
// [解析] RadarMarkData（0x020C）：16 个 bit 的标记/易伤状态，读法与 RFID 相同。
// [解析] 置 1 门槛两队不同：对方标记进度 ≥100、己方 ≥50；易伤期间受伤加倍，是雷达最核心的输出。
struct __attribute__((packed)) RadarMarkData
{
  // 对方机器人：在对应机器人被标记进度≥100 时发送 1，被标记进度<100 时发送 0。
  // 己方机器人：在对应机器人被标记进度≥50 时发送 1，被标记进度<50 时发送 0。
  // 空中机器人特殊标识和被压制标识由雷达/裁判系统根据当前比赛状态同步。
  uint16_t enemy_hero_1_vulnerable : 1;      // bit 0：对方 1 号英雄机器人易伤情况
  uint16_t enemy_engineer_2_vulnerable : 1;  // bit 1：对方 2 号工程机器人易伤情况
  uint16_t enemy_infantry_3_vulnerable : 1;  // bit 2：对方 3 号步兵机器人易伤情况
  uint16_t enemy_infantry_4_vulnerable : 1;  // bit 3：对方 4 号步兵机器人易伤情况
  uint16_t enemy_uav_special : 1;            // bit 4：对方空中机器人特殊标识情况
  uint16_t enemy_sentry_vulnerable : 1;      // bit 5：对方哨兵机器人易伤情况
  uint16_t friendly_hero_1_special : 1;      // bit 6：己方 1 号英雄机器人特殊标识情况
  uint16_t friendly_engineer_2_special : 1;  // bit 7：己方 2 号工程机器人特殊标识情况
  uint16_t friendly_infantry_3_special : 1;  // bit 8：己方 3 号步兵机器人特殊标识情况
  uint16_t friendly_infantry_4_special : 1;  // bit 9：己方 4 号步兵机器人特殊标识情况
  uint16_t friendly_uav_special : 1;         // bit 10：己方空中机器人特殊标识情况
  uint16_t friendly_sentry_special : 1;      // bit 11：己方哨兵机器人特殊标识情况
  uint16_t enemy_uav_radar_hit : 1;          // bit 12：对方空中机器人被雷达命中标识
  uint16_t enemy_uav_suppressed : 1;         // bit 13：对方空中机器人被压制标识
  uint16_t friendly_uav_radar_hit : 1;       // bit 14：己方空中机器人被雷达命中标识
  uint16_t friendly_uav_suppressed : 1;      // bit 15：己方空中机器人被压制标识
};

// 0x020D 哨兵自主决策信息同步
// [解析] SentryInfo（0x020D）：哨兵自主决策状态，14 字节 = uint32 段 + uint16 段 + uint64 段。
// [解析] 注释里的 bit 0 指"该段内部的 bit 0"，不是整包的 bit 0，跨段读最容易看错。
// [解析] 末尾 8 字节是 2026 协议新增，旧赛季不发，解析前必须确认 data_len 够 14 再 memcpy。
struct __attribute__((packed)) SentryInfo
{
  uint32_t allowed_fire_amount : 11;         // bit 0-10：除远程兑换外，哨兵成功兑换的允许发弹量
  uint32_t remote_exchange_fire_count : 4;   // bit 11-14：哨兵远程兑换允许发弹量次数
  //       开局为 0，单调递增，每成功兑换一次加 1
  uint32_t remote_exchange_health_count : 4;  // bit 15-18：哨兵远程兑换血量次数
  //       开局为 0，单调递增，每成功兑换一次加 1
  uint32_t can_confirm_free_resurrect : 1;    // bit 19：当前是否可以确认免费复活
  //       0：不可以 1：可以
  uint32_t can_exchange_immediate_resurrect : 1;  // bit 20：当前是否可以兑换立即复活
  //       0：不可以 1：可以
  uint32_t immediate_resurrect_cost : 10;  // bit 21-30：兑换立即复活需要花费的金币数
  uint32_t reserved1 : 1;                  // bit 31：保留
  uint16_t out_of_combat : 1;              // bit 0：哨兵是否处于脱战状态
  //       0：在战 1：脱战
  uint16_t ammo_exchange_allowance : 11;   // bit 1-11：队伍 17mm 允许发弹量的剩余可兑换数
  uint16_t sentry_pose : 2;                // bit 12-13：哨兵当前姿态，1 进攻，2 防御，3 移动

  uint16_t energy_mechanism_status : 1;    // bit 14：己方能量机关是否可进入激活状态
  //       0：不可激活 1：可激活
  uint16_t sentry_pose_boosted : 1;        // bit 15：哨兵当前姿态是否存在增益
  //       0：无增益 1：有增益
  // ===== 2026 协议新增 sentry_info_3 =====
  uint64_t attack_pose_duration : 8;                // bit 0-7：进攻姿态剩余持续时间

  uint64_t defense_pose_duration : 8;               // bit 8-15：防御姿态剩余持续时间

  uint64_t move_pose_duration : 8;                  // bit 16-23：移动姿态剩余持续时间

  uint64_t reserved2 : 8;                           // bit 24-31：保留

  uint64_t enhanced_attack_pose_duration : 8;       // bit 32-39：强化进攻姿态剩余持续时间

  uint64_t enhanced_defense_pose_duration : 8;      // bit 40-47：强化防御姿态剩余持续时间

  uint64_t enhanced_move_pose_duration : 8;         // bit 48-55：强化移动姿态剩余持续时间

  uint64_t reserved3 : 8;                           // bit 56-63：保留
};

// 0x020E 雷达自主决策信息同步
// [解析] RadarInfo（0x020E）：雷达自主决策状态，1 字节 8 个 bit，取值见下方原注释。
// [解析] can_ket_be_modified 的 ket 是协议原文的笔误（应为 key），属公开接口名，不能改。
struct __attribute__((packed)) RadarInfo
{
  uint8_t double_vul_trigger_chance : 2;       // bit 0-1：雷达拥有触发双倍易伤的机会
  //       0-2，开局为 0，最大可达 2
  uint8_t opponent_in_double_vulnerability : 1;  // bit 2：对方是否正在被触发双倍易伤
  //       0：未被触发双倍易伤 1：正在被触发双倍易伤
  uint8_t friendly_encryption_level : 2;       // bit 3-4：己方加密等级
  //       开局为 1，最高为 3
  uint8_t can_ket_be_modified : 1;             // bit 5：当前是否可以修改密钥
  //       0：不可修改 1：可修改
  uint8_t reserved : 2;                        // bit 6-7：保留
};

// 0x0A05 雷达站发送增益点状态数据（无线链路）
// [解析] RadarBuffStatus（0x0A05）：5 个兵种各 7 字节增益 + 末尾 6 个状态字节，共 41 字节。
// [解析] 增益字段含义与 0x0204(Buff) 完全一致，同样是"百分比/直接值"混用，换算时回看 Buff 的说明。
struct __attribute__((packed)) RadarBuffStatus
{
  uint8_t hero_heal;
  uint16_t hero_cool;
  uint8_t hero_def;
  uint8_t hero_vuln;
  uint16_t hero_atk;

  uint8_t engineer_heal;
  uint16_t engineer_cool;
  uint8_t engineer_def;
  uint8_t engineer_vuln;
  uint16_t engineer_atk;

  uint8_t infantry3_heal;
  uint16_t infantry3_cool;
  uint8_t infantry3_def;
  uint8_t infantry3_vuln;
  uint16_t infantry3_atk;

  uint8_t infantry4_heal;
  uint16_t infantry4_cool;
  uint8_t infantry4_def;
  uint8_t infantry4_vuln;
  uint16_t infantry4_atk;

  uint8_t sentry_heal;
  uint16_t sentry_cool;
  uint8_t sentry_def;
  uint8_t sentry_vuln;
  uint16_t sentry_atk;

  uint8_t sentry_posture;
  uint8_t hero_status;
  uint8_t engineer_status;
  uint8_t infantry3_status;
  uint8_t infantry4_status;
  uint8_t sentry_status;
};

// 自定义 0x0301/0x0212 合并数据中的机器人状态保持 0x0A05 原值：
// 0=存活，1=战亡，2=无敌但不虚弱，3=无敌且虚弱。
// [解析] robot_main_status：上面 6 个 *_status 的取值，用常量代替裸数字，便于阅读和全局搜索。
// [解析] 3 是"无敌且虚弱"，和 2（无敌且不虚弱）是两回事，写伤害/命中判断时别混。
namespace robot_main_status
{
constexpr uint8_t ALIVE = 0;
constexpr uint8_t DEAD = 1;
constexpr uint8_t INVINCIBLE = 2;
constexpr uint8_t INVINCIBLE_AND_VULNERABLE = 3;
}  // namespace robot_main_status

// 自定义 0x0301/0x0210 用户数据（不含 6 字节交互头）。
// [解析] 以下结构体是战队自定义格式，走 0x0301 交互数据，码值落在 data_cmd_id 里。
// [解析] "不含 6 字节交互头"指已跳过 data_cmd_id + sender_id + receiver_id，拷贝起点是 DATA_START + 6。
struct __attribute__((packed)) RadarEnemyDartWarning
{
  uint8_t dart_gate_status;
};

// 自定义 0x0301/0x0213 用户数据（不含 6 字节交互头）。
struct __attribute__((packed)) RadarAerialCountered
{
  uint8_t aerial_countered;
};

// 自定义 0x0301/0x0211 用户数据（不含 6 字节交互头）。
// 坐标保持雷达发送的 int16 原值，不在中间件内进行单位换算。
// [解析] RadarSentryPosition（0x0211）：雷达算出的 12 个坐标，1 + 12×2 = 25 字节。
// [解析] int16 是有符号原值（原点左下方为负），中间件不做单位换算，1 个刻度代表多少米需与雷达端约定。
struct __attribute__((packed)) RadarSentryPosition
{
  uint8_t source;
  int16_t hero_x;
  int16_t hero_y;
  int16_t engineer_x;
  int16_t engineer_y;
  int16_t infantry3_x;
  int16_t infantry3_y;
  int16_t infantry4_x;
  int16_t infantry4_y;
  int16_t aerial_x;
  int16_t aerial_y;
  int16_t sentry_x;
  int16_t sentry_y;
};

// 自定义 0x0301/0x0212 合并数据中的 HP 部分。
// [解析] 0x0212 那个大包被拆成 HP / 弹药 / 场地 / Buff 四个小结构体，再用 RadarCombinedData 串起来；
// [解析] 拆开写的好处是每段能单独用 static_assert 量长度，出问题时定位更快。这里是 HP 部分，5×2 = 10 字节。
struct __attribute__((packed)) RadarAllyHp
{
  uint16_t hero_hp;
  uint16_t engineer_hp;
  uint16_t infantry3_hp;
  uint16_t infantry4_hp;
  uint16_t sentry_hp;
};

// 自定义 0x0301/0x0212 合并数据中的弹药部分。
// [解析] 弹药部分：5 个兵种的剩余弹量，5×2 = 10 字节。
// [解析] 顺序与 HP 段不同：这里是英雄/3 号/4 号/空中/哨兵，中间没有工程，照习惯写会串位。
struct __attribute__((packed)) RadarAllyAmmo
{
  uint16_t hero_ammo;
  uint16_t infantry3_ammo;
  uint16_t infantry4_ammo;
  uint16_t aerial_ammo;
  uint16_t sentry_ammo;
};

// 自定义 0x0301/0x0212 合并数据中的场地部分。
// [解析] 场地部分：剩余/总金币 + 32 位自定义状态标志，2+2+4 = 8 字节；status_flags 每一位的含义由雷达端定义。
struct __attribute__((packed)) RadarAllyField
{
  uint16_t remain_coins;
  uint16_t total_coins;
  uint32_t status_flags;
};

// 自定义 0x0301/0x0212 合并数据（不含 6 字节交互头）。
// 线序固定为 HP、弹药、场地、Buff；用户载荷 69 字节，连同交互头共 75 字节。
// [解析] RadarCombinedData（0x0212）：四段按 HP、弹药、场地、Buff 的固定线序拼成 69 字节，加交互头共 75 字节。
// [解析] 各段偏移由下面几条 offsetof 断言钉死（0 / 10 / 20 / 28），调换顺序长度不变但数据全串位。
struct __attribute__((packed)) RadarCombinedData
{
  RadarAllyHp hp;
  RadarAllyAmmo ammo;
  RadarAllyField field;
  RadarBuffStatus buff;
};

static_assert(sizeof(RadarEnemyDartWarning) == 1U);
static_assert(sizeof(RadarAerialCountered) == 1U);
static_assert(sizeof(RadarSentryPosition) == 25U);
static_assert(sizeof(RadarAllyHp) == 10U);
static_assert(sizeof(RadarAllyAmmo) == 10U);
static_assert(sizeof(RadarAllyField) == 8U);
static_assert(sizeof(RadarBuffStatus) == 41U);
static_assert(sizeof(RadarCombinedData) == 69U);
static_assert(offsetof(RadarCombinedData, hp) == 0U);
static_assert(offsetof(RadarCombinedData, ammo) == 10U);
static_assert(offsetof(RadarCombinedData, field) == 20U);
static_assert(offsetof(RadarCombinedData, buff) == 28U);

// [解析] radar_main_status_valid()：校验收到的状态字节是否在 0~3 内，串口受扰时可能读到 200 这类非法值。
// [解析] 不拦下来，后面按状态分支的逻辑就会走进没有定义的情形。
constexpr bool radar_main_status_valid(uint8_t status)
{
  return status <= robot_main_status::INVINCIBLE_AND_VULNERABLE;
}

// [解析] radar_buff_status_valid()：5 个状态字节逐个校验，有一个非法就整包丢弃 —— 处理整包数据时
// [解析] "要么全信、要么全丢"比半信半疑安全；它只查状态字节，不校验血量/增益等数值字段。
inline bool radar_buff_status_valid(const RadarBuffStatus & status)
{
  return radar_main_status_valid(status.hero_status) &&
         radar_main_status_valid(status.engineer_status) &&
         radar_main_status_valid(status.infantry3_status) &&
         radar_main_status_valid(status.infantry4_status) &&
         radar_main_status_valid(status.sentry_status);
}

// 0x0301 机器人交互数据
// [解析] RobotInteractionData（0x0301）：交互数据的信封，2+2+2+112 = 118 字节，实际可用长度 = data_len - 6。
// [解析] 按 data_cmd_id 解释 user_data 前必须确认本次长度 ≥ 子结构体大小，否则是越界读（STM32 上直接 HardFault）。
struct __attribute__((packed)) RobotInteractionData
{
  uint16_t data_cmd_id;    // 内容 ID
  uint16_t sender_id;      // 发送者 ID
  uint16_t receiver_id;    // 接收者 ID
  uint8_t user_data[112];  // 最大为 112 字节
};

// 0x0301 0x0100 选手端删除图层
// [解析] InteractionLayerDelete（0x0301 + 0x0100）：删图层，图层 0~9，delete_type = 2 表示清空全部。
// [解析] 实战习惯：重画 UI 前先发一条清空，否则上一帧的旧图形会和新图形叠在一起。
struct __attribute__((packed)) InteractionLayerDelete
{
  uint8_t delete_type;  // 0：空操作 1：删除图层 2：删除所有
  uint8_t layer;        // 图层数：0~9
};

// 0x0301 0x0101 选手端绘制一个图形
// [解析] InteractionFigure（0x0301 + 0x0101）：画一个图形的完整描述，3 + 4×3 = 15 字节。
// [解析] 后三个 uint32_t 各切成几段位域，分别填满 32 位（3+3+4+4+9+9、10+11+11、10+11+11）。
// [解析] details_a~details_e 的含义随 figure_type 变化，必须查手册对应表，不能凭字段名猜。
struct __attribute__((packed)) InteractionFigure
{
  uint8_t figure_name[3];  // 在图形删除、修改等操作中，作为索引

  // bit 0-2：图形操作 0：空操作 1：增加 2：修改 3：删除
  uint32_t operate_type : 3;
  // bit 3-5：图形类型 0：直线 1：矩形 2：正圆 3：椭圆 4：圆弧 5：浮点数 6：整型数 7：字符
  uint32_t figure_type : 3;
  // bit 6-9：图层数（0~9）
  uint32_t layer : 4;
  // bit 10-13：颜色 0：红/蓝（己方颜色） 1：黄色 2：绿色 3：橙色 4：紫红色 5：粉色 6：青色 7：黑色 8：白色
  uint32_t color : 4;
  // bit 14-22：根据绘制的图形不同，含义不同
  uint32_t details_a : 9;
  // bit 23-31：根据绘制的图形不同，含义不同
  uint32_t details_b : 9;

  // bit 0-9：线宽，建议字体大小与线宽比例为 10：1
  uint32_t width : 10;
  // bit 10-20：起点/圆心 x 坐标
  uint32_t start_x : 11;
  // bit 21-31：起点/圆心 y 坐标
  uint32_t start_y : 11;

  // bit 0-9：根据绘制的图形不同，含义不同
  uint32_t details_c : 10;
  // bit 10-20：根据绘制的图形不同，含义不同
  uint32_t details_d : 11;
  // bit 21-31：根据绘制的图形不同，含义不同
  uint32_t details_e : 11;
};

// 0x0301 0x0102 选手端绘制两个图形
// [解析] 一次画 2/5/7 个图形：把 InteractionFigure 拼成数组，长度分别为 30 / 75 / 105 字节。
struct __attribute__((packed)) InteractionFigure2
{
  InteractionFigure interaction_figures[2];
};

// 0x0301 0x0103 选手端绘制五个图形
struct __attribute__((packed)) InteractionFigure5
{
  InteractionFigure interaction_figures[5];
};

// 0x0301 0x0104 选手端绘制七个图形
struct __attribute__((packed)) InteractionFigure7
{
  InteractionFigure interaction_figures[7];
};

// 0x0301 0x0110 选手端绘制字符图形
// [解析] ExtClientCustomCharacter（0x0301 + 0x0110）：图形描述 + 30 字节字符数据，15+30 = 45 字节。
struct __attribute__((packed)) ExtClientCustomCharacter
{
  InteractionFigure interaction_figure;
  uint8_t data[30];  // 字符数据
};

// 0x0301 0x0120 哨兵自主决策指令 TODO
// [解析] SentryCmd（0x0301 + 0x0120）：哨兵自主决策指令，8 段位域刚好填满一个 uint32_t。
// [解析] remote_exchange_* 是开局以来单调递增的累计次数，不是"还剩几次"，要用得自己记上一次做差。
struct __attribute__((packed)) SentryCmd
{
  uint32_t resurrect : 1;             // bit 0：是否确认复活
  //       0：否 1：是
  uint32_t immediate_resurrect : 1;   // bit 1：是否消耗金币兑换立即复活
  //       0：否 1：是
  uint32_t exchange_17mm_value : 11;  // bit 2-12：发弹量兑换值
  //       递增有效
  uint32_t remote_exchange_17mm_count : 4;  // bit 13-16：远程兑换发弹量次数
  //       开局为 0，单调递增，每次加 1
  uint32_t remote_exchange_blood_count : 4;  // bit 17-20：远程兑换血量次数
  //       开局为 0，单调递增，每次加 1
  uint32_t sentry_change_pose : 3;       // bit 21-23：哨兵修改当前姿态指令
  //       1：进攻姿态 2：防御姿态 3：移动姿态，默认为 3
  uint32_t confirm_energy_activated : 1;  // bit 24：确认使能量机关进入正在激活状态
  //       0：不确认 1：确认
  uint32_t reserved : 7;                  // bit 25-31：保留
};

// 0x0301 0x0121 雷达自主决策指令 TODO
// [解析] RadarCmd（0x0301 + 0x0121）：雷达自主决策指令，8 字节，后 6 字节是配套密钥。
struct __attribute__((packed)) RadarCmd
{
  uint8_t radar_cmd;     // 触发双倍易伤次数，开局为 0，单调递增，每次加 1，最大为 2
  uint8_t password_cmd;  // 密钥更新或验证指令
  uint8_t password_1;
  uint8_t password_2;
  uint8_t password_3;
  uint8_t password_4;
  uint8_t password_5;
  uint8_t password_6;
};

// 0x0302 自定义控制器与机器人交互数据 图传链路
// [解析] CustomRobotData（0x0302）：自定义控制器经图传链路发来的 30 字节裸数据，含义由收发双方约定。
struct __attribute__((packed)) CustomRobotData
{
  uint8_t data[30];
};

// 0x0303 选手端小地图交互数据
// [解析] MapCommand（0x0303）：云台手在选手端小地图上点选的目标，12 字节。
// [解析] 坐标单位是 m，而 MapRobotData 是 cm —— 两套单位并存，换算时看字段注释。
struct __attribute__((packed)) MapCommand
{
  float target_position_x;  // 目标位置 x 坐标，单位：m
  float target_position_y;  // 目标位置 y 坐标，单位：m
  uint8_t cmd_keyboard;     // 云台手按下的键盘按键通用键值
  uint8_t target_robot_id;  // 对方机器人 ID
  uint16_t cmd_source;      // 信息来源 ID
};

// 0x0305 选手端小地图接收雷达数据
// [解析] MapRobotData（0x0305）：小地图上显示的双方坐标，24 个 uint16 共 48 字节，单位统一为 cm。
// [解析] 这些是给界面显示用的整数坐标，和 RobotPos 的浮点米是两套东西。
struct __attribute__((packed)) MapRobotData
{
  uint16_t opponent_hero_position_x;        // 对方英雄机器人 x 位置坐标，单位：cm
  uint16_t opponent_hero_position_y;        // 对方英雄机器人 y 位置坐标，单位：cm
  uint16_t opponent_engineer_position_x;    // 对方工程机器人 x 位置坐标，单位：cm
  uint16_t opponent_engineer_position_y;    // 对方工程机器人 y 位置坐标，单位：cm
  uint16_t opponent_infantry_3_position_x;  // 对方 3 号步兵机器人 x 位置坐标，单位：cm
  uint16_t opponent_infantry_3_position_y;  // 对方 3 号步兵机器人 y 位置坐标，单位：cm
  uint16_t opponent_infantry_4_position_x;  // 对方 4 号步兵机器人 x 位置坐标，单位：cm
  uint16_t opponent_infantry_4_position_y;  // 对方 4 号步兵机器人 y 位置坐标，单位：cm
  uint16_t opponent_aerial_position_x;      // 对方空中机器人 x 位置坐标，单位：cm
  uint16_t opponent_aerial_position_y;      // 对方空中机器人 y 位置坐标，单位：cm
  uint16_t opponent_sentry_position_x;      // 对方哨兵机器人 x 位置坐标，单位：cm
  uint16_t opponent_sentry_position_y;      // 对方哨兵机器人 y 位置坐标，单位：cm
  uint16_t ally_hero_position_x;            // 己方英雄机器人 x 位置坐标，单位：cm
  uint16_t ally_hero_position_y;            // 己方英雄机器人 y 位置坐标，单位：cm
  uint16_t ally_engineer_position_x;        // 己方工程机器人 x 位置坐标，单位：cm
  uint16_t ally_engineer_position_y;        // 己方工程机器人 y 位置坐标，单位：cm
  uint16_t ally_infantry_3_position_x;      // 己方 3 号步兵机器人 x 位置坐标，单位：cm
  uint16_t ally_infantry_3_position_y;      // 己方 3 号步兵机器人 y 位置坐标，单位：cm
  uint16_t ally_infantry_4_position_x;      // 己方 4 号步兵机器人 x 位置坐标，单位：cm
  uint16_t ally_infantry_4_position_y;      // 己方 4 号步兵机器人 y 位置坐标，单位：cm
  uint16_t ally_aerial_position_x;          // 己方空中机器人 x 位置坐标，单位：cm
  uint16_t ally_aerial_position_y;          // 己方空中机器人 y 位置坐标，单位：cm
  uint16_t ally_sentry_position_x;          // 己方哨兵机器人 x 位置坐标，单位：cm
  uint16_t ally_sentry_position_y;          // 己方哨兵机器人 y 位置坐标，单位：cm
};

// 0x0306 自定义控制器与选手端交互数据
// [解析] CustomClientData（0x0306）：自定义控制器与选手端的键鼠数据，8 字节。
// [解析] 鼠标坐标只占 12 位（0~4095 够用），省下的 4 位放按键，于是一组"坐标+按键"只花 2 字节。
struct __attribute__((packed)) CustomClientData
{
  uint16_t key_value_1 : 8;  // bit 0-7：按键 1 键值
  uint16_t key_value_2 : 8;  // bit 8-15：按键 2 键值
  uint16_t x_position : 12;  // bit 0-11：鼠标 X 轴像素位置
  uint16_t mouse_left : 4;   // bit 12-15：鼠标左键状态
  //       0：未按下 1：按下
  uint16_t y_position : 12;  // bit 0-11：鼠标 Y 轴像素位置
  uint16_t mouse_right : 4;  // bit 12-15：鼠标右键状态
  //       0：未按下 1：按下
  uint16_t reserved;         // 保留
};

// 0x0307 选手端小地图接收哨兵数据
// [解析] MapData（0x0307）：哨兵发给选手端的路径规划，1+2+2+49+49+2 = 105 字节。
// [解析] delta_x/delta_y 是相对上一个点的增量（dm、int8，可为负），不是绝对坐标；sender_id 在最后 2 字节。
struct __attribute__((packed)) MapData
{
  uint8_t intention;          // 1：到目标点攻击 2：到目标点防守 3：移动到目标点
  uint16_t start_position_x;  // 路径起点 x 坐标，单位：dm
  uint16_t start_position_y;  // 路径起点 y 坐标，单位：dm
  int8_t delta_x[49];         // 路径点 x 坐标增量数组，单位：dm
  int8_t delta_y[49];         // 路径点 y 坐标增量数组，单位：dm
  uint16_t sender_id;         // 发送方 ID
};

// 0x0308 选手端小地图接收机器人数据
// [解析] CustomInfo（0x0308）：发送者 + 接收者 + 30 字节自定义数据，共 34 字节。
struct __attribute__((packed)) CustomInfo
{
  uint16_t sender_id;     // 发送者 ID
  uint16_t receiver_id;   // 接收者 ID
  uint8_t user_data[30];  // 自定义数据
};

// 0x0309 自定义控制器接收机器人数据 图传链路
// [解析] RobotCustomData（0x0309）：机器人回给自定义控制器的 30 字节裸数据，内容由业务代码约定。
struct __attribute__((packed)) RobotCustomData
{
  uint8_t data[30];
};

// 0x0310 机器人发送给自定义客户端的数据
// [解析] RobotToClientData（0x0310）：机器人发给选手端自定义客户端的数据，最长 300 字节。
// [解析] 发之前先确认串口/DMA 缓冲放得下，装不下会被分片（半包会被 CRC16 拦掉，不至于用错数据）。
struct __attribute__((packed)) RobotToClientData
{
  uint8_t data[300];  // 根据实际发送长度灵活处理，最大 300 字节
};

// 0x0311 自定义客户端发送给机器人的自定义指令
// [解析] ClientToRobotData（0x0311）：自定义客户端发给机器人的自定义指令，最长 30 字节。
struct __attribute__((packed)) ClientToRobotData
{
  uint8_t data[30];  // 包含自定义控制逻辑或键鼠数据，最大 30 字节
};

// 防止协议结构体长度变化后 memcpy 目标悄悄失配。
// [解析] 以下 static_assert 是编译期量尺：长度或偏移与协议对不上就编译失败，而不是等上车收到乱码。
// [解析] 也可以当协议速查表用：左边结构体名，右边是它该有的字节数。
static_assert(sizeof(FrameHeader) == 5);
static_assert(sizeof(GameStatus) == 11);
static_assert(sizeof(GameResult) == 1);
static_assert(sizeof(GameRobotHP) == 20);
static_assert(sizeof(EventData) == 4);
static_assert(sizeof(RefereeWarning) == 3);
static_assert(sizeof(DartInfo) == 3);
static_assert(sizeof(RobotStatus) == 17);
static_assert(sizeof(PowerHeatData) == 14);
static_assert(sizeof(RobotPos) == 12);
static_assert(sizeof(Buff) == 8);
static_assert(sizeof(HurtData) == 1);
static_assert(sizeof(ShootData) == 7);
static_assert(sizeof(ProjectileAllowance) == 8);
static_assert(sizeof(RFID_Status) == 5);
static_assert(sizeof(DartClientCmd) == 6);
static_assert(sizeof(GroundRobotPosition) == 40);
static_assert(sizeof(RadarMarkData) == 2);
static_assert(sizeof(SentryInfo) == 14);
static_assert(sizeof(RadarInfo) == 1);
static_assert(sizeof(RadarBuffStatus) == 41);
static_assert(sizeof(RadarSentryPosition) == 25);
static_assert(sizeof(RobotInteractionData) == 118);
static_assert(sizeof(InteractionLayerDelete) == 2);
static_assert(sizeof(InteractionFigure) == 15);
static_assert(sizeof(InteractionFigure2) == 30);
static_assert(sizeof(InteractionFigure5) == 75);
static_assert(sizeof(InteractionFigure7) == 105);
static_assert(sizeof(ExtClientCustomCharacter) == 45);
static_assert(sizeof(SentryCmd) == 4);
static_assert(sizeof(RadarCmd) == 8);
static_assert(sizeof(CustomRobotData) == 30);
static_assert(sizeof(MapCommand) == 12);
static_assert(sizeof(MapRobotData) == 48);
static_assert(sizeof(CustomClientData) == 8);
static_assert(sizeof(MapData) == 105);
static_assert(sizeof(CustomInfo) == 34);
static_assert(sizeof(RobotCustomData) == 30);
static_assert(sizeof(RobotToClientData) == 300);
static_assert(sizeof(ClientToRobotData) == 30);

}  // namespace sp::referee

#endif  // SP__REFEREE_PROTOCOL_HPP
