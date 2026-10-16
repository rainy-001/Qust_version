// [解析] ==========================================================================
// [解析] 【文件】sp_middleware/referee/referee_protocol/referee_protocol.hpp
// [解析] 【作用】裁判系统（Referee System）串口协议的"字典"：把官方协议手册里的
// [解析]         每个命令码、每个数据包结构体，原样翻译成 C++ 常量和 struct。
// [解析]         本文件只有定义、没有可执行代码，也不碰硬件，是纯头文件。
// [解析] 【被谁用】直接 #include 本文件的只有 4 处：
// [解析]           referee/pm02/pm02.hpp（裁判系统主串口，解析 0x0201 这类状态包）、
// [解析]           referee/vt03/vt03.hpp（图传链路的自定义数据）、
// [解析]           referee/ui/manager.hpp 与 referee/ui/element.hpp（画 UI 的 0x0301 交互数据）；
// [解析]           另外 referee/vt02/referee_def.h 只在注释里对照本文件的帧格式，并未 include。
// [解析] 【怎么读】三条线，建议按顺序看：
// [解析]           1) FrameHeader + SOF + HEAD_LEN/DATA_START —— 一帧数据长什么样；
// [解析]           2) cmd_id —— 这帧讲的是什么事（这就是"命令码"）；
// [解析]           3) 后面一堆结构体 —— 这件事的具体内容。
// [解析] 【关键点】1) 所有数据包结构体都带 __attribute__((packed))。串口送来的字节是
// [解析]           "一个挨一个"紧凑排列的，而编译器默认会为了访问速度在成员之间插入
// [解析]           看不见的空隙（padding）。一旦多出空隙，结构体长度就和字节流对不上，
// [解析]           后面所有字段全部错位 —— packed 就是强制"不许插空隙"。
// [解析]           2) 文件末尾有几十个 static_assert，作用相当于"编译期量尺子"：
// [解析]           谁改动了结构体导致长度/偏移不等于官方协议规定的值，编译就直接报错，
// [解析]           而不是等上车收到一屏乱码才去查。
// [解析]           3) 位域（如 `uint8_t game_type : 4;`）里的数字是"占几个二进制位"，
// [解析]           同一个底层类型下所有位域加起来必须刚好填满它，少一位多一位都会让
// [解析]           后面所有字段整体错位。位域按"从低位到高位"依次填充（小端机器）。
// [解析] ==========================================================================

#ifndef SP__REFEREE_PROTOCOL_HPP
#define SP__REFEREE_PROTOCOL_HPP

// [解析] cstddef 提供 size_t 和 offsetof（文件末尾用 offsetof 量成员偏移）。
// [解析] cstdint 提供 uint8_t/uint16_t/uint32_t/uint64_t/int16_t 这类"定长整数"。
// [解析] 为什么不用 int、unsigned？因为这里所有数字都要和串口字节一一对应，
// [解析] 必须保证"每个字段占几字节"在编译器和平台上完全一致，定长类型才有这个保证。
#include <cstddef>
#include <cstdint>

// [解析] robot_id：裁判系统给场上每个"兵种单位"分配的编号。
// [解析] 规律很好记：红方是 1~11，蓝方就是在红方编号上 +100（101~111）。
// [解析] 这些编号会出现在很多数据包的 robot_id 字段里（如 RobotStatus::robot_id——
// [解析] "我是谁"，RefereeWarning::offending_robot_id——"是谁违规了"），
// [解析] 用来回答"这条数据说的是哪个兵种"。
// [解析] 注意：10/11（红/蓝基地）和 110/111（红/蓝前哨站）是固定建筑，不会移动；
// [解析] 其余编号才对应能跑的机器人。
// [解析] constexpr 意思是"编译期常量"：写在只读区，不占 RAM，等价于 #define 但有类型检查。
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

// [解析] client_id：选手端（操作手电脑 / 图传客户端 / 自定义控制器）的身份编号，
// [解析] 出现在 0x0301 机器人交互数据的 sender_id（发送者）和 receiver_id（接收者）字段里。
// [解析] 换算规律：client_id = 0x0100 + robot_id。
// [解析]   例：红方 3 号步兵 robot_id=3 -> 0x0103；蓝方 3 号步兵 103 -> 0x0167。
// [解析] 所以红方是 0x0101~0x0106，蓝方是 0x0165~0x016A，两边不相交，
// [解析] 收到一帧就能直接判断"是哪个队、哪个兵种发来的"。
// [解析] 用 uint16_t 而不是 uint8_t：0x0165 已经超过 255 了。
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
// [解析] REFEREE_SERVER = 0x8080：裁判系统服务器自己的"身份编号"。
// [解析] 机器人主动往服务器发数据时（例如画 UI、自定义上报），receiver_id 填它；
// [解析] 反过来，服务器广播下来的数据包里 sender_id 也是 0x8080。
constexpr uint16_t REFEREE_SERVER = 0x8080;
}  // namespace sp::referee::client_id

// [解析] cmd_id（命令码）：一帧裁判系统数据的"标题"，告诉接收方"这包讲的是什么事"。
// [解析] 它在帧里的位置是第 6~7 字节（帧头 5 字节之后），小端存放（低字节在前）。
// [解析] 文档里习惯写成 16 进制，例如 0x0201。为了好记，官方按高位分成几大段：
// [解析]   0x000x 比赛全局信息   —— 广播给所有机器人（谁都能收）
// [解析]   0x010x 场地事件/判罚/飞镖
// [解析]   0x020x 机器人自身状态 —— 主要发给自己
// [解析]   0x030x 交互数据       —— 机器人<->选手端/自定义控制器，最复杂的一类
// [解析]   0x0A0x 雷达站专用（走无线链路）
// [解析]   0x0F0x 图传信道设置
// [解析] 看到一个新 cmd_id，先看它属于哪一段，就能大致猜到内容归谁管。
namespace sp::referee::cmd_id
{
constexpr uint16_t GAME_STATUS = 0x0001;    // 比赛状态数据
// [解析] ↑ 比赛类型、当前阶段、阶段剩余时间、NTP 同步时间戳，对应下面的 GameStatus。
constexpr uint16_t GAME_RESULT = 0x0002;    // 比赛结果数据
// [解析] ↑ 只有一字节的胜负结果，对应 GameResult。
constexpr uint16_t GAME_ROBOT_HP = 0x0003;  // 比赛机器人血量数据
// [解析] ↑ 场上各机器人血量和己方伤害差值，对应 GameRobotHP。
// [解析] 这三个包是整场比赛级别的信息，服务器广播给所有人，不分红蓝方。

// [解析] ---- 0x010x：与"场地里的东西"有关的事件 ----
// [解析] 注意这里没有 0x0102、0x0103：说明本项目用不到它们（手册里是补给站动作提示等），
// [解析] 真要用的时候得先查当季协议手册确认码值，不能凭感觉往上加。
constexpr uint16_t EVENT_DATA = 0x0101;       // 场地事件数据
// [解析] ↑ 补给区/能量机关/各种高地的占领状态 + 对方飞镖命中情况，对应 EventData。
constexpr uint16_t REFEREE_WARNING = 0x0104;  // 裁判警告数据
// [解析] ↑ 被判罚的等级、违规机器人 ID、违规次数，对应 RefereeWarning。
constexpr uint16_t DART_INFO = 0x0105;        // 飞镖发射相关数据
// [解析] ↑ 飞镖发射站剩余时间与击打目标信息，对应 DartInfo。

// [解析] ---- 0x020x：发给自己机器人的状态数据（比赛中最常用、也最该先看懂的一组）----
// [解析] 这些包由裁判系统针对"你这台机器人"下发，所以不需要再判断是敌是友。
constexpr uint16_t ROBOT_STATUS = 0x0201;           // 机器人状态数据
// [解析] ↑ 等级、当前/最大血量、枪口热量上限与冷却、底盘功率上限、弹速上限、电源输出 -> RobotStatus
constexpr uint16_t POWER_HEAT_DATA = 0x0202;        // 实时底盘缓冲能量和射击热量数据
// [解析] ↑ 缓冲能量(J) 和 17mm/42mm 枪口热量 -> PowerHeatData，做功率控制和热量限制要用
constexpr uint16_t ROBOT_POS = 0x0203;              // 机器人位置数据
// [解析] ↑ 本机 x/y 坐标(m) 与朝向(度，正北为 0) -> RobotPos
// [解析] BUFF：回血/冷却/防御/攻击增益，外加"剩余能量"的位域反馈 -> Buff
// [解析] 下一行原有的空注释写着"没有 0x0205"：官方命令码表里这一项是空缺的，
// [解析] 所以这里直接跳到 0x0206，属于正常，不是漏写。
constexpr uint16_t BUFF = 0x0204;                   // 机器人增益数据
                                                    // 没有 0x0205
// [解析] HURT_DATA：一次扣血的"部位 + 原因"，装甲板 ID 和扣血原因各占 4 bit -> HurtData
constexpr uint16_t HURT_DATA = 0x0206;              // 伤害状态数据
// [解析] SHOOT_DATA：刚打出去那一发的弹种/发射机构编号/射频(Hz)/初速(m/s) -> ShootData
constexpr uint16_t SHOOT_DATA = 0x0207;             // 实时射击数据
// [解析] PROJECTILE_ALLOWANCE：还能发多少 17mm/42mm 弹、剩多少金币 -> ProjectileAllowance
constexpr uint16_t PROJECTILE_ALLOWANCE = 0x0208;   // 允许发弹量数据
// [解析] RFID_STATUS：车上 RFID 读卡器读到的场地增益点，一个 bit 对应一个点 -> RFID_Status
constexpr uint16_t RFID_STATUS = 0x0209;            // 机器人 RFID 模块状态
// [解析] DART_CLIENT_CMD：飞镖发射站开合状态、切换目标/发射指令时的比赛剩余时间 -> DartClientCmd
constexpr uint16_t DART_CLIENT_CMD = 0x020A;        // 飞镖选手端指令数据
// [解析] GROUND_ROBOT_POSITION：己方英雄/工程/3 号、4 号步兵的坐标(m) -> GroundRobotPosition
constexpr uint16_t GROUND_ROBOT_POSITION = 0x020B;  // 地面机器人位置数据
// [解析] RADAR_MARK_DATA：哪些敌方机器人已经被标记到"易伤"（进度满）-> RadarMarkData
constexpr uint16_t RADAR_MARK_DATA = 0x020C;        // 雷达标记进度数据
// [解析] SENTRY_INFO：哨兵专属——已兑换发弹量、能否复活、当前姿态等 -> SentryInfo
constexpr uint16_t SENTRY_INFO = 0x020D;            // 哨兵自主决策信息同步
// [解析] RADAR_INFO：雷达专属——触发双倍易伤的机会次数、己方加密等级等 -> RadarInfo
constexpr uint16_t RADAR_INFO = 0x020E;             // 雷达自主决策信息同步

// [解析] ---- 0x030x：交互数据（机器人 <-> 选手端 / 自定义控制器）----
// [解析] 这一组是双向的，也最复杂：本文件既要用它解析"别人发给我"的数据，
// [解析] 也要用它组装"我发给别人"的数据（见 referee/ui/manager.cpp）。
// [解析] 特别注意 0x0301：它是一个"信封"，真正的内容类型还要再看包里前 2 字节的
// [解析] data_cmd_id（下面紧跟着的命名空间），等于命令码套命令码，别只看外层。
// [解析] "图传链路"意味着这些包走的是图传模块的串口，而不是裁判系统主串口，
// [解析] 实际项目里两者可能是不同的 UART，收错了口就一直接不到数据。
constexpr uint16_t ROBOT_INTERACTION_DATA = 0x0301;  // 机器人交互数据
// [解析] 自定义控制器（操作手自己焊的控制板）发给机器人的 30 字节 -> CustomRobotData
constexpr uint16_t CUSTOM_ROBOT_DATA = 0x0302;       // 自定义控制器与机器人交互数据 图传链路
// [解析] 云台手在选手端小地图上点了目标点 -> MapCommand（坐标单位 m）
constexpr uint16_t MAP_COMMAND = 0x0303;             // 选手端小地图交互数据
// [解析] 雷达站提供、显示在选手端小地图上的坐标 -> MapRobotData
// [解析] 注意它里面所有坐标单位是 cm，而 RobotPos/MapCommand 是 m，换算时别搞混。
constexpr uint16_t MAP_ROBOT_DATA = 0x0305;          // 选手端小地图接收雷达数据
// [解析] 自定义控制器与选手端之间的键鼠/像素坐标数据 -> CustomClientData
constexpr uint16_t CUSTOM_CLIENT_DATA = 0x0306;      // 自定义控制器与选手端交互数据
// [解析] 哨兵发给选手端的路径规划：起点 + 49 组坐标增量(dm) -> MapData
constexpr uint16_t MAP_DATA = 0x0307;                // 选手端小地图接收哨兵数据
// [解析] 通用的"发送者 + 接收者 + 30 字节自定义数据" -> CustomInfo
constexpr uint16_t CUSTOM_INFO = 0x0308;             // 选手端小地图接收机器人数据
// [解析] 机器人回给自定义控制器的 30 字节 -> RobotCustomData
constexpr uint16_t ROBOT_CUSTOM_DATA = 0x0309;       // 自定义控制器接收机器人数据 图传链路
// [解析] 机器人发给选手端自定义客户端的长数据，最长 300 字节 -> RobotToClientData
// [解析] 这条语句被作者换行书写（0x0310 单独一行），只是排版，值本身没有区别。
constexpr uint16_t ROBOT_CLIENT_DATA =
  0x0310;  // 机器人发送给自定义客户端的数据 图传链路，最大 300 字节
// [解析] 选手端自定义客户端发给机器人的自定义指令，最长 30 字节 -> ClientToRobotData
constexpr uint16_t CLIENT_ROBOT_DATA = 0x0311;  // 自定义客户端发送给机器人的自定义指令，最大 30 字节

// [解析] ---- 0x0A0x：雷达站专用数据（走无线链路，只有雷达站会发）----
// [解析] 普通步兵/英雄车一般收不到这一组；只有在雷达站上，或做雷达联调时才会有意义。
// [解析] 这些包的接收者通常是"另一台哨兵/雷达"，parser 里要用 sender_id 再筛一遍。
constexpr uint16_t RADAR_ROBOT_POS = 0x0A01;         // 雷达站发送机器人坐标数据 无线链路
constexpr uint16_t RADAR_ROBOT_HP = 0x0A02;          // 雷达站发送机器人血量数据 无线链路
constexpr uint16_t RADAR_SENTRY_ALLOWANCE = 0x0A03;  // 雷达站发送哨兵自主兑换允许发弹量 无线链路
constexpr uint16_t RADAR_ROBOT_STATUS = 0x0A04;      // 雷达站发送机器人状态数据 无线链路
// [解析] 5 个兵种的"回血/冷却/防御/负防御/攻击"五件套 + 6 个存活状态 -> RadarBuffStatus
constexpr uint16_t RADAR_BUFF_STATUS = 0x0A05;       // 雷达站发送增益点状态数据 无线链路
constexpr uint16_t RADAR_MARK_INFO = 0x0A06;         // 雷达站发送敌方机器人标记进度 无线链路
// [解析] ---- 0x0F0x：图传信道设置（走图传链路），与比赛数据无关，属于链路维护 ----
constexpr uint16_t SET_IMAGE_TRANSFER = 0x0F01;      // 设置图传出图信道 图传链路
constexpr uint16_t INQUIRY_IMAGE_TRANSFER = 0x0F02;  // 查询图传出图信道 图传链路
// [解析] 命令码表到此结束。最后提醒一个非常容易混淆的点：
// [解析] 两个命名空间里的数值会"撞车"！例如 0x0101 在 cmd_id 里是"场地事件数据"，
// [解析] 而在下一个命名空间 data_cmd_id 里是"选手端画一个图形"。
// [解析] 判断时必须先看外层 cmd_id，再看内层 data_cmd_id，不能只凭数值下结论。
}  // namespace sp::referee::cmd_id

// [解析] data_cmd_id：二级命令码。只有 0x0301（机器人交互数据）这个包里才带它，
// [解析] 位置是包内头 2 字节。可以把 0x0301 理解成"信封"，data_cmd_id 是"信纸类型"。
// [解析] 大致分两片：
// [解析]   0x02xx 雷达站通过 0x0301 发来的自定义数据（对应下面 RadarEnemyDartWarning 等结构体）
// [解析]   0x01xx 选手端画 UI 的指令（删图层/画图形/画字符）以及自主决策指令
// [解析] 判断流程永远是"先看外层 cmd_id == 0x0301，再把内层 data_cmd_id 读出来做 switch"。
namespace sp::referee::data_cmd_id
{
// 雷达通过 0x0301 发送的自定义子编码。
// [解析] 0x0210：对方飞镖闸门状态（1 字节） -> RadarEnemyDartWarning
constexpr uint16_t RADAR_ENEMY_DART_WARNING_CMD = 0x0210;
// [解析] 0x0211：雷达算出的各机器人坐标（25 字节，坐标为 int16 原值） -> RadarSentryPosition
constexpr uint16_t RADAR_SENTRY_POSITION_CMD = 0x0211;
// [解析] 0x0212：雷达合并下发的大包（血量+弹药+场地+Buff，共 69 字节） -> RadarCombinedData
constexpr uint16_t RADAR_COMBINED_DATA_CMD = 0x0212;
// [解析] 0x0213：对方空中机器人被压制的标识（1 字节） -> RadarAerialCountered
constexpr uint16_t RADAR_AERIAL_COUNTERED_CMD = 0x0213;
// 0x0200~0x02FF 机器人之间通信 TODO
// [解析] ---- 0x01xx：选手端 UI 绘制与自主决策指令（本项目最常发出去的一类）----
// [解析] 这些是"机器人 -> 选手端"的指令：告诉操作手界面上画什么、删什么。
constexpr uint16_t INTERACTION_LAYER_DELETE = 0x0100;     // 选手端删除图层
// [解析] 0x0101~0x0104 是"一次画几个图形"的四个版本（1/2/5/7 个）：
// [解析] 一个包的数据区能塞几个图形是固定的，要画几个就挑数量匹配的那条发，
// [解析] 数量不足时必须在末尾补零占位，否则选手端会把垃圾数据当成图形解析。
constexpr uint16_t INTERACTION_FIGURE = 0x0101;           // 选手端绘制一个图形
constexpr uint16_t INTERACTION_FIGURE_2 = 0x0102;         // 选手端绘制两个图形
constexpr uint16_t INTERACTION_FIGURE_5 = 0x0103;         // 选手端绘制五个图形
constexpr uint16_t INTERACTION_FIGURE_7 = 0x0104;         // 选手端绘制七个图形
// [解析] 0x0110：先描述一个图形，再跟 30 字节字符数据，用来在界面上显示文字。
constexpr uint16_t EXT_CLIENT_CUSTOM_CHARACTER = 0x0110;  // 选手端绘制字符图形
// [解析] 0x0120/0x0121 后面标着 TODO：协议手册里已经定义，但本项目还没真正用起来
// [解析] （哨兵/雷达的自主决策没有实现）。读代码时知道它们在哪即可，不必深挖。
constexpr uint16_t SENTRY_CMD = 0x0120;                   // 哨兵自主决策指令 TODO
constexpr uint16_t RADAR_CMD = 0x0121;                    // 雷达自主决策指令 TODO
// [解析] 命令码部分到此结束。下面开始是"每条命令对应的数据长什么样"——
// [解析] 从 FrameHeader（帧长什么样）开始，然后是各个数据包结构体，最后是编译期尺寸检查。
}  // namespace sp::referee::data_cmd_id

// [解析] 下面是裁判系统"通用帧"的定义，也是整个协议的骨架。整帧的字节布局（偏移 0 起算）：
// [解析]   偏移 0        : SOF = 0xA5      —— 帧起始标志，收数据时先找到它才能对齐
// [解析]   偏移 1~2      : data_len (小端) —— 数据段的长度（不含帧头、命令码和帧尾）
// [解析]   偏移 3        : seq            —— 帧序号，用来发现丢帧（累加到自己回绕）
// [解析]   偏移 4        : crc8           —— 对前面 4 字节算 CRC8，专门校验帧头
// [解析]   偏移 5~6      : cmd_id (小端)   —— 命令码，决定数据段该按哪个结构体解释
// [解析]   偏移 7~n-3    : 数据段         —— 对应本文件后面某个结构体
// [解析]   偏移 n-2~n-1  : crc16 (小端)    —— 整帧校验，帧头或数据出错都能发现
// [解析] 校验算法实现在 sp_middleware/tools/crc/crc.cpp：CRC8 初值 0xFF、CRC16 初值 0xFFFF，
// [解析] 都是查表实现（比逐位算快很多，代价是各占 256 字节的常量表）。
// [解析] 为什么"帧头已有 CRC8，整帧还要再来个 CRC16"？因为 data_len 就在帧头里，
// [解析] 帧头一错就不知道这帧多长，没法用整帧校验去救，必须先单独把关帧头。
namespace sp::referee
{
// [解析] SOF（Start Of Frame）就是上面说的 0xA5。选它的原因：二进制 1010_0101，
// [解析] 0 和 1 交替出现，即使串口上连续来一堆 0 或一堆 1 也不容易把它误认成数据，
// [解析] 是嵌入式协议里常见的起始标志取值。
// [解析] 但要注意：数据段里同样可能出现 0xA5 这个字节，所以不能"一见 0xA5 就当新帧"，
// [解析] 正确做法是靠 data_len + CRC8 + CRC16 三道关卡一起判（见 referee/vt03/vt03.cpp）。
constexpr uint8_t SOF = 0xA5;

// [解析] FrameHeader：帧头，固定 5 字节。字段顺序必须和官方手册完全一致，一个字节都不能挪。
// [解析] __attribute__((packed)) 是 GCC 的扩展属性，写在 struct 关键字后面表示
// [解析] "禁止编译器在成员之间插入对齐空隙"。这里不加也恰好是 5 字节，
// [解析] 但一旦成员类型变了（比如加个 uint32_t），不加 packed 就会多出 padding 而错位。
// [解析] 代价：packed 结构体里的多字节成员可能落在奇数地址上，ARM 访问非对齐地址虽然
// [解析] 多数情况能用，但会慢一些，绝不能对这类成员取地址去做 4 字节强转。
struct __attribute__((packed)) FrameHeader
{
  uint8_t sof;      // [解析] 起始字节，固定 0xA5，收到它就说明对齐上了
  uint16_t data_len;  // [解析] 数据段的长度；不含帧头、命令码和帧尾 CRC16；小端存放
  // [解析] ↑ 注意"数据段"从 DATA_START(偏移 7) 开始：整帧长度 = 5 + 2 + data_len + 2，
  // [解析]   项目里的算法见 referee/pm02/pm02.cpp 与 referee/vt03/vt03.cpp，那里就是这么加的。
  uint8_t seq;      // [解析] 帧序号，发送方每发一帧 +1；收方用它判断有没有丢帧
  uint8_t crc8;     // [解析] 对前 4 个字节（sof/data_len/seq）算出的 CRC8，用来确认帧头没被干扰
};

// [解析] HEAD_LEN = sizeof(FrameHeader) = 5，即帧头字节数。
// [解析] 用 sizeof 而不是直接写 5：以后万一帧头改字段，长度会自动跟着变，少一处维护点。
// [解析] CMD_ID_LEN = 2：命令码占 2 字节。
// [解析] TAIL_LEN = 2：帧尾的 CRC16 占 2 字节（放在整帧最后，小端）。
// [解析] DATA_START = 5 + 2 = 7：整帧里第 7 个字节开始才是数据段。
// [解析] 典型用法：memcpy(&my_struct, frame + DATA_START, sizeof(my_struct));
// [解析] 前提是已经校验过 CRC 且确认 cmd_id 匹配，否则拷进来的是垃圾。
constexpr size_t HEAD_LEN = sizeof(FrameHeader);
constexpr size_t CMD_ID_LEN = 2;
constexpr size_t TAIL_LEN = 2;
constexpr size_t DATA_START = HEAD_LEN + CMD_ID_LEN;

// [解析] GameStatus（0x0001）：整场比赛的"大局"状态——比赛类型、进行到哪一步、还剩多少秒。
// [解析] 新手最常用的就是 game_progress（判断"是不是在比赛中"，比赛中该值为 4）和
// [解析] stage_remain_time（还剩多少秒，可以做倒计时逻辑）。
// [解析] 坑点：前两个字段合起来才占 1 个字节 —— game_type 占低 4 位(bit0-3)，
// [解析] game_progress 占高 4 位(bit4-7)。用位域时编译器已经帮你做了"取位"，
// [解析] 直接写 game_status.game_progress 就能拿到那 4 位；但如果哪天改成手工解析
// [解析] 字节数组，就必须自己写 (byte >> 4) & 0x0F，忘记移位是最常见的错误。
// [解析] 本结构体 packed 后 1+2+8 = 11 字节，与文件末尾 static_assert 对应。
// 0x0001 比赛状态数据
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
  // [解析] 当前阶段（例如准备阶段/比赛中）的剩余秒数，比赛结束会变成 0
  uint16_t stage_remain_time;  // 当前阶段剩余时间，单位：s
  // [解析] 裁判系统的 UNIX 时间戳（64 位，防止 2038 溢出问题），只有机器人连上了裁判系统
// [解析] 的 NTP 服务器之后这个值才有意义；没连上时不要去用它做判断。
  uint64_t sync_timestamp;     // UNIX 时间，当机器人正确连接到裁判系统 NTP 服务器后生效
};

// [解析] GameResult（0x0002）：比赛结果，只有 1 个字节，比赛结束后服务器才会下发。
// [解析] winner 取值：0 平局、1 红方胜利、2 蓝方胜利。
// 0x0002 比赛结果数据
struct __attribute__((packed)) GameResult
{
  uint8_t winner;  // 0: 平局, 1: 红方胜利, 2: 蓝方胜利
};

// [解析] GameRobotHP（0x0003）：场上各单位血量 + 伤害差值，packed 后 20 字节。
// [解析] 名字里的数字是"兵种序号"而不是 robot_id：1=英雄、2=工程、3/4=步兵、7=哨兵；
// [解析] 协议这一版没有 5、6 号对应的血量字段，所以看起来"跳号"了，属于正常现象。
// [解析] 另外注意：字段名虽然写成 ally_/enemy_，但裁判系统是按"你这台车属于哪一方"
// [解析] 自动填好再发给你的，所以同一份代码在红蓝两方都能直接用，不用自己判断阵营。
// 0x0003 比赛机器人血量数据
struct __attribute__((packed)) GameRobotHP
{
  uint16_t ally_1_robot_hp;  // 己方 1 英雄机器人血量
  uint16_t ally_2_robot_hp;  // 己方 2 工程机器人血量
  uint16_t ally_3_robot_hp;  // 己方 3 步兵机器人血量
  uint16_t ally_4_robot_hp;  // 己方 4 步兵机器人血量
  // [解析] 有符号数！正数=己方总伤害高于对方，负数=落后。若误当成 uint16_t，落后时会读成一个巨大的正数
  int16_t damage_difference;  // 己方伤害量相对对方伤害量的差值
  uint16_t ally_7_robot_hp;   // 己方 7 哨兵机器人血量
  uint16_t ally_outpost_hp;   // 己方前哨站血量
  uint16_t ally_base_hp;      // 己方基地血量
  uint16_t enemy_outpost_hp;  // 对方前哨站血量
  uint16_t enemy_base_hp;     // 对方基地血量
};

// [解析] EventData（0x0101）：场地事件。整个包只有 4 字节（一个 uint32_t），却被切成 11 段
// [解析] 位域，把"哪块地是谁占着、对方飞镖打到过哪里"全塞了进去——本文件里最需要对照
// [解析] 手册来读的结构体。
// [解析] 读法：所有字段的 bit 编号是"从最低位向最高位"连续排队的 ——
// [解析] supply_status 占 bit0-2，energy_status 紧接占 bit3-6 …… 一直到 bit31 的 reserved。
// [解析] 数目相加验算：3+4+2+2+9+3+2+2+2+1+2 = 32，正好填满一个 uint32_t，
// [解析] 少写一位多写一位都会连带后面全部错位（好在文件末尾有 sizeof==4 的 static_assert 兜底）。
// [解析] 用的时候不需要自己移位，直接读 event.central_highground_status 就是那 2 个 bit 的值；
// [解析] 但取值含义必须查注释/手册，例如"2 表示被对方占领"。
// [解析] 另外：注释里写"仅 RMUL 适用"的取值 3（被双方占领）在超级对抗赛规则下不会出现，
// [解析] 做判断时可以把它当异常值忽略。
// 0x0101 场地事件数据
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

// [解析] RefereeWarning（0x0104）：被判罚的提示，3 个字节，比赛里通常拿来在 UI 上报警。
// [解析] level 是判罚等级（1 双方黄牌 / 2 双方红牌 / 3 判负，具体以当季手册为准），
// [解析] offending_robot_id 是"是谁违规了"（用的就是上面 robot_id 那套编号），
// [解析] count 是违规次数。三个值描述的都是"己方最后一次受到的判罚"，不是累加统计。
// 0x0104 裁判警告数据
struct __attribute__((packed)) RefereeWarning
{
  uint8_t level;               // 己方最后一次受到判罚的等级
  uint8_t offending_robot_id;  // 己方最后一次受到判罚的违规机器人 ID
  uint8_t count;               // 己方最后一次受到判罚的违规次数
};

// [解析] DartInfo（0x0105）：飞镖发射站状态。它同时是"为什么一定要 packed"的最好例子：
// [解析] uint8_t + uint16_t 若不加 packed，编译器为了对齐会插 1 字节空隙变成 4 字节，
// [解析] 而协议里它就是 3 字节 —— 长度对不上，后面所有字段全部读错。
// [解析] 末尾的 dart_info 是个 16 位位域包：低 3 位=对方飞镖最后命中的目标，
// [解析] 中间 3 位=该目标被累计命中的次数，再上面 3 位=当前选中的击打目标，最高 7 位保留。
// 0x0105 飞镖发射相关数据
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

// [解析] RobotStatus（0x0201）：机器人自己的"身份证 + 能力上限"，做发弹限制、超电控功率都要用它。
// [解析] 常用字段：current_hp/maximum_hp 算血量百分比；shooter_barrel_heat_limit 是枪口热量
// [解析] 上限、shooter_barrel_cooling_value 是每秒冷却多少热量（两处单位都是"热量值"，不是摄氏度）；
// [解析] chassis_power_limit 是底盘功率上限，单位 W。
// [解析] 注意 bullet_speed_limit 是 float（4 字节），前面的成员却都是 1/2 字节 ——
// [解析] 这正是必须 packed 的另一个理由：不加 packed，编译器会在 float 之前插入空隙，
// [解析] 而协议里它是紧挨着的，错一个字节后面全废。
// [解析] 最后 1 个字节被拆成 3 个 1 bit 的电源输出状态 + 5 bit 保留，
// [解析] 整个结构体 packed 后是 1+1+2+2+2+2+2+4+1 = 17 字节。
// [解析] 别搞混：这里只有"热量上限"，当前热量在 0x0202（PowerHeatData）里。
// 0x0201 机器人状态数据
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

// [解析] PowerHeatData（0x0202）：缓冲能量 + 枪口热量，是超级电容（超电）功率决策的主要输入。
// [解析] 前 3 个成员都叫 reserved_*，说明协议早期这里是别的量、后来废弃了，
// [解析] 但它们依然占着 2+2+4 = 8 字节，解析时绝对不能删，删了后面字段全部前移。
// [解析] 单位：buffer_energy 是 J（焦耳），表示超电"现在还能给你多少能量"，功率控制算法
// [解析] 把它当余额用；两个 heat 是当前已积累的热量值，和 0x0201 里的 heat_limit 比较，
// [解析] 就能判断"这个发射机构还能不能再打"。
// [解析] 结构体共 2+2+4+2+2+2 = 14 字节。
// 0x0202 实时底盘缓冲能量和射击热量数据
struct __attribute__((packed)) PowerHeatData
{
  uint16_t reserved_1;
  uint16_t reserved_2;
  float reserved_3;
  uint16_t buffer_energy;               // 缓冲能量，单位：J
  uint16_t shooter_17mm_1_barrel_heat;  // 第 1 个 17mm 发射机构的射击热量
  uint16_t shooter_42mm_barrel_heat;    // 42mm 发射机构的射击热量
};

// [解析] RobotPos（0x0203）：本机器人的绝对坐标，坐标系/原点由官方场地规定，共 12 字节（3 个 float）。
// [解析] angle 是测速模块朝向，正北为 0 度、顺时针增大，单位"度"而不是弧度 ——
// [解析] 如果后面要拿去做三角函数计算，别忘了乘 π/180 转成弧度。
// [解析] 只有装了对应定位模块的机器人才能收到有效坐标，没装时字段可能是 0，别拿 0 当真实位置。
// 0x0203 机器人位置数据
struct __attribute__((packed)) RobotPos
{
  float x;      // 本机器人位置 x 坐标，单位：m
  float y;      // 本机器人位置 y 坐标，单位：m
  float angle;  // 本机器人测速模块朝向，单位：度，正北为 0 度
};

// [解析] Buff（0x0204）：当前各类增益，8 字节。这一组字段"单位不统一"，非常容易算错：
// [解析]   recovery_y_buff / defence_buff / vulnerability_buff / attack_buff 是"百分比"，
// [解析]     值为 10 就代表 10%（不是 0.1），参与计算时要自己 /100；
// [解析]   cooling_buff 却是"直接值"：值为 x 表示每秒多冷却 x 点热量，含义和前四个不同；
// [解析]   remaining_energy 是一堆 bit 标志，不是普通数值，见下面紧跟的说明。
// [解析] 结构体共 1+2+1+1+2+1 = 8 字节。
// 0x0204 机器人增益数据
struct __attribute__((packed)) Buff
{
  uint8_t recovery_y_buff;  // 机器人回血增益（百分比，值为 10 表示每秒恢复血量上限的 10%）
  uint16_t cooling_buff;    // 机器人射击热量冷却增益具体值（直接值，值为 x 表示热量冷却增加 x/s）
  uint8_t defence_buff;     // 机器人防御增益（百分比，值为 50 表示 50%防御增益）
  uint8_t vulnerability_buff;  // 机器人负防御增益（百分比，值为 30 表示-30%防御增益）
  uint16_t attack_buff;        // 机器人攻击增益（百分比，值为 50 表示 50%攻击增益）
// [解析] remaining_energy 必须当"位掩码"看，不能当数值读：
// [解析]   直接读会拿到 0x80 这种和能量比例毫无关系的数；
// [解析]   正确姿势是逐位判断，例如 (remaining_energy >> 2) & 1 表示"剩余能量是否 ≥50%"。
// [解析] 注意：原注释这里看起来略自相矛盾 —— 既给出了 bit0/bit1/bit2
// [解析]   （对应 ≥125%/≥100%/≥50%）的含义，又写"仅在剩余能量小于 50% 时反馈，
// [解析]   其余默认反馈 0x80"。使用时请以当季官方手册为准，并用实车数据核对一遍再写逻辑。
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

// [解析] HurtData（0x0206）：一次扣血的"部位 + 原因"，整个包只有 1 个字节。
// [解析] 低 4 位(bit0-3) 是装甲板/测速模块编号，高 4 位(bit4-7) 是扣血原因（0~6，
// [解析] 含义见下面紧跟的注释：0 被弹丸打、2 射速超限、3 热量超限、4 功率超限……）。
// [解析] 实用价值：把 HP_deduction_reason 打到日志/UI 上，就能判断这次掉血是"被打的"
// [解析] 还是"自己超功率/超热量被裁判扣的"——后者说明你的控制参数需要改。
// 0x0206 伤害状态数据
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

// [解析] ShootData（0x0207）：最近一次射击的详细信息，共 7 字节。
// [解析] 又是 packed 的功劳：1+1+1+4 = 7，正好等于协议规定的长度；
// [解析] 若不写 packed，float 需要按 4 字节对齐，编译器会在它前面补 1 字节空隙变成 8 字节，
// [解析] 这样接收时长度就对不上、后面所有数据都读错。
// [解析] initial_speed 是实测弹丸初速(m/s)，可以用来检查发射机构是否正常。
// 0x0207 实时射击数据
struct __attribute__((packed)) ShootData
{
  uint8_t bullet_type;          // 1: 17mm 弹丸, 2: 42mm 弹丸
  uint8_t shooter_number;       // 1: 第1个17mm发射机构, 2: 第2个17mm发射机构, 3: 42mm发射机构
  uint8_t launching_frequency;  // 射频，单位：Hz
  float initial_speed;          // 弹丸初速度，单位：m/s
};

// [解析] ProjectileAllowance（0x0208）：还剩多少发弹、多少金币、堡垒储备弹量，8 字节。
// [解析] 做"没弹了就别再扣扳机"这类保护时，用 projectile_allowance_17mm 判断即可；
// [解析] remaining_gold_coin 只在工程/哨兵兑换时有意义。
// 0x0208 允许发弹量数据
struct __attribute__((packed)) ProjectileAllowance
{
  uint16_t projectile_allowance_17mm;      // 17mm 弹丸允许发弹量
  uint16_t projectile_allowance_42mm;      // 42mm 弹丸允许发弹量
  uint16_t remaining_gold_coin;            // 剩余金币数量
  uint16_t projectile_allowance_fortress;  // 堡垒增益点提供的储备 17mm 弹丸允许发弹量
};

// [解析] RFID_Status（0x0209）：车上 RFID 读卡器读到的"踩点"情况。一个 bit 对应一个增益点，
// [解析] 命名了 32 + 6 个点，另有 2 位保留，packed 后共 4+1 = 5 字节。
// [解析] 用法和 EventData 一样，不需要自己移位，例如 rfid.friendly_base == 1
// [解析] 就表示"己方基地增益点已被踩到"。
// [解析] 易错点：后半段刻意用 uint8_t 而不是 uint32_t 当底层类型 —— 位域换底层类型会
// [解析] "另起一个存储单元"。如果这里也写成 uint32_t，后面那几个 bit 会各自再占 4 字节，
// [解析] 结构体就会从 5 字节胖到 8 字节，接收时立刻错位。
// 0x0209 机器人 RFID 模块状态
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

// [解析] DartClientCmd（0x020A）：飞镖选手端的指令回执，6 字节。
// [解析] 两个时间字段都是"比赛剩余时间(秒)"，为 0 表示从未切换过目标 / 从未发射过，
// [解析] 可以用它们判断操作手到底有没有真的下过指令。
// [解析] dart_launch_opening_status 是"三态"：0 已开启、1 关闭、2 正在开启或关闭中，
// [解析] 所以判断"现在能不能发射"必须写 == 0，不能只判断"非零"。
// 0x020A 飞镖选手端指令数据
struct __attribute__((packed)) DartClientCmd
{
  uint8_t dart_launch_opening_status;  // 当前飞镖发射站状态：0 已开启，1 关闭，2 正在开启或关闭
  uint8_t reserved;                    // 保留
  uint16_t target_change_time;  // 切换击打目标时的比赛剩余时间，单位：秒，无/未切换动作，默认为 0。
  uint16_t
    latest_launch_cmd_time;  // 发送最新飞镖发射指令的比赛剩余时间，单位：秒，无/未发射，默认为 0。
};

// [解析] GroundRobotPosition（0x020B）：己方地面机器人的坐标，10 个 float 共 40 字节。
// [解析] 最后两个 reserved1/reserved2 是协议预留（将来可能给别的兵种），解析时忽略即可，
// [解析] 但一样不能从结构体里删掉，否则后面的数据（如果有）会对不上。
// 0x020B 地面机器人位置数据
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

// [解析] RadarMarkData（0x020C）：雷达标记/易伤情况，2 字节共 16 个 bit，读法与 RFID 相同。
// [解析] "易伤"指被雷达标记进度打满，此时该机器人受伤加倍 —— 这是雷达车最核心的输出信息。
// [解析] 注意置 1 的门槛两队不同：对方机器人标记进度 ≥100 才置 1，己方机器人 ≥50 就置 1。
// 0x020C 雷达标记进度数据
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

// [解析] SentryInfo（0x020D）：哨兵自主决策相关状态，14 字节。这是本文件最容易看懵的结构体，
// [解析] 因为它把三种不同宽度的底层类型拼在一起：uint32_t(4B) + uint16_t(2B) + uint64_t(8B)。
// [解析] 读法其实一样：注释里的 "bit 0" 指的是"自己那一段内部的 bit 0"，不是整个包的 bit 0。
// [解析] 例：sentry_pose 占第二个 uint16_t 的 bit12-13，取值 1 进攻 / 2 防御 / 3 移动。
// [解析] 后半段标着"2026 协议新增 sentry_info_3"，是本赛季追加的 8 字节姿态剩余时间：
// [解析] 旧赛季的裁判系统根本不会发这 8 字节，所以解析时得先确认 data_len 够 14 字节
// [解析] （pm02.cpp 里的 copy_fixed 就做了"长度必须等于 sizeof"的检查，不符就整包跳过），
// [解析] 不能无脑 memcpy 14 字节 —— 这是本文件里唯一需要注意"协议版本兼容"的地方。
// [解析] 另外提醒：位域跨不同底层类型时的排布规则，各编译器并不完全一致，
// [解析] 本项目用 GCC（arm-none-eabi）；文件末尾 static_assert(sizeof(SentryInfo) == 14)
// [解析] 就是在编译期替我们盯住这个假设。
// 0x020D 哨兵自主决策信息同步
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
  
  // [解析] 这两个字段（bit14、bit15）把第二个 uint16_t 的最后 2 位填满，本段到此结束。
  uint16_t energy_mechanism_status : 1;    // bit 14：己方能量机关是否可进入激活状态
  //       0：不可激活 1：可激活
  uint16_t sentry_pose_boosted : 1;        // bit 15：哨兵当前姿态是否存在增益
  //       0：无增益 1：有增益
  // ===== 2026 协议新增 sentry_info_3 =====
  // [解析] uint64_t 段共 64 位，由 8 个 8 位字段依次填满：三种普通姿态的剩余持续时间、
  // [解析] 三种强化姿态的剩余持续时间，中间还夹着两组各 8 位的保留字段。
  // [解析] 每个 8 位字段能表示 0~255，够不够用要看协议手册里"持续时间"的单位定义。
  uint64_t attack_pose_duration : 8;                // bit 0-7：进攻姿态剩余持续时间

  uint64_t defense_pose_duration : 8;               // bit 8-15：防御姿态剩余持续时间

  uint64_t move_pose_duration : 8;                  // bit 16-23：移动姿态剩余持续时间

  uint64_t reserved2 : 8;                           // bit 24-31：保留

  uint64_t enhanced_attack_pose_duration : 8;       // bit 32-39：强化进攻姿态剩余持续时间

  uint64_t enhanced_defense_pose_duration : 8;      // bit 40-47：强化防御姿态剩余持续时间

  uint64_t enhanced_move_pose_duration : 8;         // bit 48-55：强化移动姿态剩余持续时间

  uint64_t reserved3 : 8;                           // bit 56-63：保留
};

// [解析] RadarInfo（0x020E）：雷达的自主决策状态，只有 1 个字节、8 个 bit。
// [解析] double_vul_trigger_chance：还剩几次触发"双倍易伤"的机会（0~2，单调递增）。
// [解析] friendly_encryption_level：己方加密等级，开局 1、最高 3。
// [解析] 注意 can_ket_be_modified 这个名字看着像笔误（"ket" 大概是 "key"，
// [解析] 即"密钥"），但它是公开接口名，不能随手改；使用时按"当前能否修改密钥"理解即可。
// 0x020E 雷达自主决策信息同步
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

// [解析] RadarBuffStatus（0x0A05）：雷达站下发的各机器人增益点状态，41 字节。
// [解析] 布局：5 个兵种 × 7 字节（回血1 + 冷却2 + 防御1 + 负防御1 + 攻击2 = 7）
// [解析]       再加上末尾 6 个状态字节，35 + 6 = 41，和文件末尾的 static_assert 对得上。
// [解析] 每个兵种那 5 个字段的含义与 0x0204(Buff) 完全一致，同样是"百分比/直接值"混用，
// [解析] 换算时请回看上面 Buff 的说明，别把百分比当小数用。
// [解析] 末尾 6 个 *_status 用后面 robot_main_status 里的取值（0 存活/1 战亡/2 无敌/3 无敌且虚弱）。
// [解析] 本结构体没有位域，41 字节纯粹来自"成员顺序紧密排列 + packed"。
// [解析] 同样因为 packed，里面的 uint16_t 可能落在奇数地址上：正常读写没问题，
// [解析] 但千万不要取它的地址再强转成 uint32_t* 之类，那会触发非对齐访问问题。
// 0x0A05 雷达站发送增益点状态数据（无线链路）
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
// [解析] robot_main_status：上面 6 个 *_status 字段的取值含义。用有名字的常量代替裸数字
// [解析] 0/1/2/3，好处是 `if (s == robot_main_status::DEAD)` 一眼能读懂，也方便全局搜索。
// [解析] 注意 3 这个名字容易被误读：它是"无敌但依然虚弱"（类似基地被打掉后仍会被打的状态），
// [解析] 和 2（无敌且不虚弱）完全是两回事，写伤害/命中判断时别弄混。
namespace robot_main_status
{
constexpr uint8_t ALIVE = 0;
constexpr uint8_t DEAD = 1;
constexpr uint8_t INVINCIBLE = 2;
constexpr uint8_t INVINCIBLE_AND_VULNERABLE = 3;
}  // namespace robot_main_status

// [解析] 下面这一组是"自定义协议"结构体：不是官方裁判系统直接定义的，而是同济 SP 战队
// [解析] 在 0x0301（机器人交互数据）里自己约定的数据格式，所以码值都落在 data_cmd_id 里。
// [解析] 原注释里的"不含 6 字节交互头"意思是：解析时已经先跳过了
// [解析] data_cmd_id(2) + sender_id(2) + receiver_id(2) = 6 字节的包头，
// [解析] 也就是真正要拷贝的起始位置是 DATA_START + 6；直接 memcpy 整帧一定会错。
// [解析] 提醒：自定义部分一旦改了字段，必须保证收发的两端（机器人代码和雷达端代码）
// [解析] 同时更新，否则会出现"结构体长度一样但含义不同"的隐蔽 bug。
// 自定义 0x0301/0x0210 用户数据（不含 6 字节交互头）。
struct __attribute__((packed)) RadarEnemyDartWarning
{
  uint8_t dart_gate_status;  // [解析] 对方飞镖闸门状态（具体取值需与雷达端约定，一般为 0 关 / 1 开）
};

// [解析] 0x0213 用户数据：对方空中机器人是否已被压制，只有 1 个字节。
// 自定义 0x0301/0x0213 用户数据（不含 6 字节交互头）。
struct __attribute__((packed)) RadarAerialCountered
{
  uint8_t aerial_countered;  // [解析] 0/1 标志位，具体取值含义与雷达端约定
};

// [解析] 0x0211 用户数据：雷达算出的各方坐标，25 字节 = 1(source) + 12×2(int16 坐标)。
// [解析] source 表示这批坐标的来源/坐标系类型（哪一位代表什么要和雷达端约定）。
// [解析] 用 int16_t 而不是 uint16_t：坐标是相对原点的，原点左下方会是负数。
// [解析] 注意这里刻意保留"雷达发来的原始刻度值"，单位换算留给上层做 ——
// [解析] 好处是中间件不掺和业务单位，坏处是使用方必须自己弄清楚 1 个刻度代表多少米，
// [解析] 而结构体注释里并没有写明单位，这是使用时最容易出错的一点。
// 自定义 0x0301/0x0211 用户数据（不含 6 字节交互头）。
// 坐标保持雷达发送的 int16 原值，不在中间件内进行单位换算。
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

// [解析] 0x0212 这个大包被作者拆成 4 个小结构体（HP / 弹药 / 场地 / Buff），
// [解析] 再用下面那个 RadarCombinedData 串起来。拆开写的好处是以后改某一段只需动对应的小结构体，
// [解析] 而且 static_assert 可以对每一段单独量长度，出问题时定位更快。
// [解析] 这里定义的是 HP 部分：5 个兵种的血量，5×2 = 10 字节。
// 自定义 0x0301/0x0212 合并数据中的 HP 部分。
struct __attribute__((packed)) RadarAllyHp
{
  uint16_t hero_hp;
  uint16_t engineer_hp;
  uint16_t infantry3_hp;
  uint16_t infantry4_hp;
  uint16_t sentry_hp;
};

// [解析] 弹药部分：5 个兵种的剩余弹量，5×2 = 10 字节。
// 自定义 0x0301/0x0212 合并数据中的弹药部分。
struct __attribute__((packed)) RadarAllyAmmo
{
  uint16_t hero_ammo;
  uint16_t infantry3_ammo;
  uint16_t infantry4_ammo;
  uint16_t aerial_ammo;
  uint16_t sentry_ammo;
};

// [解析] 场地部分：剩余金币、总金币和一个 32 位状态标志集合，2+2+4 = 8 字节。
// [解析] status_flags 的每一位代表什么由雷达端定义（属于自定义协议的一部分），
// [解析] 想解析它必须去翻雷达端文档，这里只有容器、没有含义。
// 自定义 0x0301/0x0212 合并数据中的场地部分。
struct __attribute__((packed)) RadarAllyField
{
  uint16_t remain_coins;
  uint16_t total_coins;
  uint32_t status_flags;
};

// [解析] RadarCombinedData：把上面四段拼成完整载荷，10+10+8+41 = 69 字节；
// [解析] 再加上 6 字节交互头，整包就是 75 字节 —— 原注释里"69 / 75"两个数字就是这么来的。
// [解析] 由于每个小结构体都带 packed，嵌套进来后仍然是紧挨着排列的，
// [解析] 所以各段偏移固定为 hp=0、ammo=10、field=20、buff=28（下面的 offsetof 断言在盯这个）。
// [解析] 顺序就是协议规定的"线序"，谁也不能调换 —— 调换后长度一样但数据全串位，
// [解析] 这种 bug 不会报错、只会算出错误结果，最难查。
// 自定义 0x0301/0x0212 合并数据（不含 6 字节交互头）。
// 线序固定为 HP、弹药、场地、Buff；用户载荷 69 字节，连同交互头共 75 字节。
struct __attribute__((packed)) RadarCombinedData
{
  RadarAllyHp hp;
  RadarAllyAmmo ammo;
  RadarAllyField field;
  RadarBuffStatus buff;
};

// [解析] 上面那行原有注释"防止协议结构体长度变化后 memcpy 目标悄悄失配"说的就是下面这些
// [解析] static_assert 的用途：它是"编译期断言"，条件不成立时编译当场报错，程序根本编不出来。
// [解析]   sizeof(X) == 协议字节数    —— 数字对不上，说明有人改了字段或编译器插入了对齐空隙；
// [解析]   offsetof(X, 成员) == 偏移  —— 量成员在结构体内部的位置，专门抓"顺序被调换"
// [解析]     或者"前面被插了 padding"这类光看长度发现不了的问题；
// [解析]   （offsetof 来自 <cstddef>，这就是文件开头 include 它的原因。）
// [解析] 数字后面的 U 后缀（1U、25U）表示无符号常量，只是习惯写法，顺便避免有符号/无符号比较告警。
// [解析] 小技巧：想加一个协议结构体时，抄着这里的写法给自己也加一条断言，
// [解析] 比自己数字节靠谱得多。
static_assert(sizeof(RadarEnemyDartWarning) == 1U);
static_assert(sizeof(RadarAerialCountered) == 1U);
static_assert(sizeof(RadarSentryPosition) == 25U);
static_assert(sizeof(RadarAllyHp) == 10U);
static_assert(sizeof(RadarAllyAmmo) == 10U);
static_assert(sizeof(RadarAllyField) == 8U);
static_assert(sizeof(RadarBuffStatus) == 41U);
static_assert(sizeof(RadarCombinedData) == 69U);
// [解析] 下面 4 行量的是嵌套成员的偏移，正好对应协议里的"线序"：
// [解析] hp 在 0；ammo 在 10（跨过 hp 的 10 字节）；field 在 20；buff 在 28（跨过 field 的 8 字节）。
// [解析] 一旦有人调换这四段的顺序，长度断言还能过，但这 4 行会立刻编译失败 —— 这就是它们的价值。
static_assert(offsetof(RadarCombinedData, hp) == 0U);
static_assert(offsetof(RadarCombinedData, ammo) == 10U);
static_assert(offsetof(RadarCombinedData, field) == 20U);
static_assert(offsetof(RadarCombinedData, buff) == 28U);

// [解析] radar_main_status_valid()：判断从串口收到的"状态字节"是不是合法取值（0~3）。
// [解析] 为什么要专门校验？串口被干扰、或包长和结构体对不上时，可能读到 200 这种非法值；
// [解析] 不拦下来，后面按状态分支的逻辑就会走进没有定义的角落，行为不可预测。
// [解析] constexpr 表示"传常量时能在编译期算出结果"，这里主要起"声明它是纯函数"的作用。
constexpr bool radar_main_status_valid(uint8_t status)
{
  return status <= robot_main_status::INVINCIBLE_AND_VULNERABLE;
}

// [解析] radar_buff_status_valid()：把 5 个兵种的状态逐个过一遍，只要有一个非法就整体返回 false ——
// [解析] "要么全信、要么全丢"是处理一整包数据的常见策略，比半信半疑更安全。
// [解析] 参数写成 const RadarBuffStatus &：用引用避免复制 41 字节结构体，const 保证不改调用方的数据。
// [解析] inline 建议编译器把函数体就地展开，省掉函数调用开销（这里更多是习惯写法）。
// [解析] 注意 && 的短路特性：前面某个状态一旦非法，后面的函数调用就不会再执行。
// [解析] 另外注意：它只检查 5 个 *_status 字节，并不检查 hero_heal 等数值字段是否合理。
inline bool radar_buff_status_valid(const RadarBuffStatus & status)
{
  return radar_main_status_valid(status.hero_status) &&
         radar_main_status_valid(status.engineer_status) &&
         radar_main_status_valid(status.infantry3_status) &&
         radar_main_status_valid(status.infantry4_status) &&
         radar_main_status_valid(status.sentry_status);
}

// [解析] RobotInteractionData（0x0301）：交互数据的"信封"，118 字节 = 2+2+2+112。
// [解析] 头部三个字段各 2 字节（data_cmd_id 说明内容类型，sender/receiver 说明谁发给谁），
// [解析] 后面 112 字节是用户数据区，实际用了多少由帧头的 data_len 决定（data_len 是整段数据段
// [解析] 的长度，所以真正可用的载荷 = data_len - 6），不是每次都填满。
// [解析] 典型用法分两步：先按本结构体读出头部，再根据 data_cmd_id 把 user_data
// [解析] 解释成对应的子结构体（例如下面那些 Interaction* / Radar*）。
// [解析] 安全提醒：解释 user_data 之前一定要确认"本次收到的数据长度 >= 子结构体大小"，
// [解析] 否则就是典型的越界读 —— 在 STM32 上很可能直接把程序打成 HardFault。
// 0x0301 机器人交互数据
struct __attribute__((packed)) RobotInteractionData
{
  uint16_t data_cmd_id;    // 内容 ID
  uint16_t sender_id;      // 发送者 ID
  uint16_t receiver_id;    // 接收者 ID
  uint8_t user_data[112];  // 最大为 112 字节
};

// [解析] InteractionLayerDelete（0x0301 + 0x0100）：让选手端删图层的指令，2 字节。
// [解析] layer 取 0~9，对应选手端界面的 10 个图层；delete_type=2 是"一键清空"。
// [解析] 实战习惯：每帧重画 UI 之前先发一条清空，否则上一帧的旧图形会和新图形叠在一起。
// 0x0301 0x0100 选手端删除图层
struct __attribute__((packed)) InteractionLayerDelete
{
  uint8_t delete_type;  // 0：空操作 1：删除图层 2：删除所有
  uint8_t layer;        // 图层数：0~9
};

// [解析] InteractionFigure（0x0301 + 0x0101）：画一个图形的完整描述，15 字节，
// [解析] 是"位域用来省字节"的极端例子，第一次看会觉得像天书，拆开就不难：
// [解析]   figure_name[3]：3 字节名字，占偏移 0~2，增/删/改时靠它找图形；
// [解析]   后面 3 个 uint32_t 各被切成若干段位域，分别描述"操作/类型/图层/颜色"、
// [解析]   "线宽/起点坐标"、"更多细节"，三段分别填满 32 位：
// [解析]   3+3+4+4+9+9 = 32，10+11+11 = 32，10+11+11 = 32，3 + 12 = 15 字节。
// [解析] 坑点：3 字节数组后面紧跟 uint32_t 位域，packed 下这个 32 位单元落在偏移 3 的奇数地址上，
// [解析] 所以绝不能对这个结构体取地址再做 32 位强转，只能老老实实按成员访问。
// [解析] 另外 details_a~details_e 的含义随 figure_type 变化（直线/矩形/圆/浮点数/字符各不同），
// [解析] 用之前必须查手册里的对应表，不能凭字段名猜。
// 0x0301 0x0101 选手端绘制一个图形
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

// [解析] "一次画 N 个图形"的三个版本：只是把 InteractionFigure 拼成数组，
// [解析] 长度分别是 15×2 = 30、15×5 = 75、15×7 = 105 字节。
// [解析] 为什么要做成"数量固定"的多个命令？因为交互数据段长度是死的，
// [解析] 想一次画几个就得挑数量匹配的那条命令发，图形不够时要把剩余位置补零，
// [解析] 否则选手端会把内存里的垃圾当成图形解析（画出一堆莫名其妙的线条）。
// 0x0301 0x0102 选手端绘制两个图形
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

// [解析] 0x0110：一个图形描述 + 30 字节字符数据（15+30 = 45 字节），用来在界面上显示文字。
// [解析] data[30] 里到底放什么编码（ASCII、点阵还是位图）取决于选手端的实现，本中间件不做解释。
// 0x0301 0x0110 选手端绘制字符图形
struct __attribute__((packed)) ExtClientCustomCharacter
{
  InteractionFigure interaction_figure;
  uint8_t data[30];  // 字符数据
};

// [解析] SentryCmd（0x0301 + 0x0120）：哨兵自主决策指令，4 个字节全塞进一个 uint32_t。
// [解析] 位域分配：resurrect(1) + immediate_resurrect(1) + exchange_17mm_value(11) +
// [解析] remote_exchange_17mm_count(4) + remote_exchange_blood_count(4) + sentry_change_pose(3) +
// [解析] confirm_energy_activated(1) + reserved(7) = 32 位，一个 uint32_t 刚好填满。
// [解析] 注意"次数"类字段（remote_exchange_*）是开局以来单调递增的累计值，不是"还剩几次"，
// [解析] 想判断"这次要不要兑换"得自己记住上一次的值再做差。
// [解析] 原注释标着 TODO：协议手册已经定义，但项目里还没启用，当作字典了解即可。
// 0x0301 0x0120 哨兵自主决策指令 TODO
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

// [解析] RadarCmd（0x0301 + 0x0121）：雷达自主决策指令，8 字节。
// [解析] radar_cmd 是要触发的双倍易伤次数（0~2，单调递增）；
// [解析] password_cmd 是密钥操作指令，后面的 password_1~6 是配套的 6 字节密钥。
// [解析] 同样标着 TODO：项目里还没用起来。真要实现时必须先查手册确认 password_cmd 的取值含义。
// 0x0301 0x0121 雷达自主决策指令 TODO
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

// [解析] CustomRobotData（0x0302）：自定义控制器经图传链路发来的 30 字节裸数据。
// [解析] 中间件只负责"原样搬运"，里面怎么解释由业务代码和自定义控制器双方约定。
// [解析] 因为没有字段名，对接时最容易出现"两边理解不一致"，建议在代码里写清自定义的字段表。
// 0x0302 自定义控制器与机器人交互数据 图传链路
struct __attribute__((packed)) CustomRobotData
{
  uint8_t data[30];
};

// [解析] MapCommand（0x0303）：云台手在选手端小地图上点选目标的信息，12 字节 = 4+4+1+1+2。
// [解析] target_position_x/y 单位是 m（注意和 MapRobotData 的 cm 不是一个单位！）；
// [解析] cmd_keyboard 是键盘通用键值，target_robot_id 用的还是 robot_id 那套编号，
// [解析] cmd_source 表示这条指令由哪个客户端发起。
// [解析] 顺带一提：这个结构体即使不写 packed，长度也刚好是 12（成员正好没有产生空隙），
// [解析] 加 packed 是为了全文件风格统一，同时防止将来加字段时悄悄冒出 padding。
// 0x0303 选手端小地图交互数据
struct __attribute__((packed)) MapCommand
{
  float target_position_x;  // 目标位置 x 坐标，单位：m
  float target_position_y;  // 目标位置 y 坐标，单位：m
  uint8_t cmd_keyboard;     // 云台手按下的键盘按键通用键值
  uint8_t target_robot_id;  // 对方机器人 ID
  uint16_t cmd_source;      // 信息来源 ID
};

// [解析] MapRobotData（0x0305）：小地图上显示的双方坐标，24 个 uint16 = 48 字节。
// [解析] 单位统一是 cm（厘米），而同一个协议里 MapCommand 用的是 m ——
// [解析] 两种单位并存，换算时务必看字段注释，不要凭"感觉"。
// [解析] 这些是"给界面显示用"的整数坐标（精度到 cm），和 RobotPos 的浮点米是两套东西。
// 0x0305 选手端小地图接收雷达数据
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

// [解析] CustomClientData（0x0306）：自定义控制器与选手端交互的键鼠数据，8 字节。
// [解析] 它把 4 个 uint16 拆成 8 段位域来省字节：键盘两个键各占 1 字节；
// [解析] 鼠标是"X 像素位置 12 位 + 左键 4 位""Y 像素位置 12 位 + 右键 4 位"，最后 2 字节保留。
// [解析] 为什么位置只用 12 位？因为屏幕像素坐标范围有限（0~4095 足够用），
// [解析] 省下的 4 位刚好塞按键状态，于是一组"坐标 + 按键"只花 2 字节而不是 4 字节。
// 0x0306 自定义控制器与选手端交互数据
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

// [解析] MapData（0x0307）：哨兵发给选手端的路径规划，105 字节 = 1+2+2+49+49+2。
// [解析] 结构是"起点坐标 + 49 个相对增量"：delta_x[i]/delta_y[i] 是相对上一个点的偏移量，
// [解析] 不是绝对坐标 —— 用增量能用 1 字节表示 ±127，从而在有限长度里描述更长的路径。
// [解析] 单位是 dm（分米），int8_t 有符号，所以增量可以为负（往反方向走）。
// [解析] 注意 sender_id 放在结构体最后 2 字节，别按"发送者一定在开头"的直觉去读。
// 0x0307 选手端小地图接收哨兵数据
struct __attribute__((packed)) MapData
{
  uint8_t intention;          // 1：到目标点攻击 2：到目标点防守 3：移动到目标点
  uint16_t start_position_x;  // 路径起点 x 坐标，单位：dm
  uint16_t start_position_y;  // 路径起点 y 坐标，单位：dm
  int8_t delta_x[49];         // 路径点 x 坐标增量数组，单位：dm
  int8_t delta_y[49];         // 路径点 y 坐标增量数组，单位：dm
  uint16_t sender_id;         // 发送方 ID
};

// [解析] CustomInfo（0x0308）：通用的自定义数据包，34 字节 = 2+2+30，
// [解析] 结构与 RobotInteractionData 类似但更短（没有外层命令码那一套）；
// [解析] user_data[30] 怎么解释同样由业务代码与对方约定。
// 0x0308 选手端小地图接收机器人数据
struct __attribute__((packed)) CustomInfo
{
  uint16_t sender_id;     // 发送者 ID
  uint16_t receiver_id;   // 接收者 ID
  uint8_t user_data[30];  // 自定义数据
};

// [解析] RobotCustomData（0x0309）：机器人回给自定义控制器的 30 字节裸数据，同样由双方约定内容。
// 0x0309 自定义控制器接收机器人数据 图传链路
struct __attribute__((packed)) RobotCustomData
{
  uint8_t data[30];  // [解析] 30 字节自定义区，含义由业务代码约定
};

// [解析] RobotToClientData（0x0310）：机器人发给选手端自定义客户端的数据，最长 300 字节。
// [解析] 这是本文件里最大的结构体。发送时要注意串口/DMA 缓冲够不够放下 300 字节，
// [解析] 装不下就会被分片，选手端可能收到半包（好在有 CRC16 会把它拦掉，不至于用错数据）。
// 0x0310 机器人发送给自定义客户端的数据
struct __attribute__((packed)) RobotToClientData
{
  uint8_t data[300];  // 根据实际发送长度灵活处理，最大 300 字节
};

// [解析] ClientToRobotData（0x0311）：自定义客户端发给机器人的自定义指令，最长 30 字节。
// [解析] 典型内容是自定义控制器/键鼠的操作数据，由业务代码自己解释。
// 0x0311 自定义客户端发送给机器人的自定义指令
struct __attribute__((packed)) ClientToRobotData
{
  uint8_t data[30];  // 包含自定义控制逻辑或键鼠数据，最大 30 字节
};

// [解析] 下面是整份文件的"编译期体检表"：每条断言都把结构体长度和官方协议规定的字节数钉死。
// [解析] 只要有人改错字段类型、删错成员，或者编译器换了排布规则，编译就会在这里当场失败 ——
// [解析] 这是本项目非常值得学习的一个习惯：凡是"要按字节流解析"的结构体，都配一条 sizeof 断言。
// [解析] 读的时候可以把它当成"协议速查表"：左边是结构体名，右边是它该有的字节数。
// 防止协议结构体长度变化后 memcpy 目标悄悄失配。
// [解析] 帧头 5 字节：sof(1) + data_len(2) + seq(1) + crc8(1)，是后面一切解析的前提。
static_assert(sizeof(FrameHeader) == 5);
static_assert(sizeof(GameStatus) == 11);
static_assert(sizeof(GameResult) == 1);
static_assert(sizeof(GameRobotHP) == 20);
// [解析] EventData 只有 4 字节，但里面 11 段位域的位数加起来必须正好 32 ——
// [解析] 这条断言就是在替我们验算 3+4+2+2+9+3+2+2+2+1+2 == 32。
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
// [解析] SentryInfo = 4(uint32 段) + 2(uint16 段) + 8(uint64 段) = 14 字节。
// [解析] 这条断言还顺带保证了"跨底层类型的位域在 GCC 下确实紧挨着排"，非常关键：
// [解析] 如果换成别的编译器排布不同，这里会立刻报错，而不是上车后收到乱码。
static_assert(sizeof(SentryInfo) == 14);
static_assert(sizeof(RadarInfo) == 1);
static_assert(sizeof(RadarBuffStatus) == 41);
static_assert(sizeof(RadarSentryPosition) == 25);
// [解析] RobotInteractionData = 2(data_cmd_id) + 2(sender) + 2(receiver) + 112(user_data) = 118。
// [解析] 这 6 字节头部就是前面注释里反复提到的"交互头"。
static_assert(sizeof(RobotInteractionData) == 118);
static_assert(sizeof(InteractionLayerDelete) == 2);
// [解析] InteractionFigure = 3(figure_name) + 4×3(三个 uint32_t 位域段) = 15 字节。
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
// [解析] MapData = 1(intention) + 2 + 2(起点坐标) + 49 + 49(两组增量) + 2(sender_id) = 105。
// [解析] 49 这个数字来自协议规定的最多路径点数，不是随便定的。
static_assert(sizeof(MapData) == 105);
static_assert(sizeof(CustomInfo) == 34);
static_assert(sizeof(RobotCustomData) == 30);
static_assert(sizeof(RobotToClientData) == 300);
// [解析] 体检表到此结束。给自己加新协议结构体时，请照抄这里的写法补一条 sizeof 断言，
// [解析] 这是花 10 秒钟就能省下几小时上车调试时间的投资。
static_assert(sizeof(ClientToRobotData) == 30);

}  // namespace sp::referee

// [解析] 头文件到此结束。回顾一下建议的阅读顺序：
// [解析]   1) FrameHeader —— 一帧数据长什么样（帧头 5 字节 + 命令码 2 + 数据 + CRC16 2）；
// [解析]   2) cmd_id / data_cmd_id —— 这帧讲的是哪件事；
// [解析]   3) 各数据包结构体 —— 这件事的具体内容（按需查，不必全背）；
// [解析]   4) static_assert —— 编译期确认布局没写错。
// [解析] 真正"怎么收、怎么校验、怎么分发"的逻辑在 referee/pm02/pm02.cpp（裁判系统主串口）
// [解析] 和 referee/vt03/vt03.cpp（图传链路）里，本文件只负责"数据长什么样"。
#endif  // SP__REFEREE_PROTOCOL_HPP
