// [解析] ==========================================================================
// [解析] 【文件】sp_middleware/referee/pm02/pm02.hpp
// [解析] 【作用】裁判系统（Referee System）串口接收模块 PM02 的类声明。
// [解析]         一个 PM02 对象 = 一路串口 + 一大堆"只读"数据成员；
// [解析]         上层不用管字节怎么拆，直接读 pm02.robot_status / pm02.power_heat 等成员即可。
// [解析] 【被谁用】applications/uart_task.cpp（按同目录 readme.md 新建）：
// [解析]             sp::PM02 pm02(&huart6);        // C 板接裁判系统的串口是 huart6
// [解析]             任务里先 pm02.request();        // 启动接收
// [解析]             中断回调 HAL_UARTEx_RxEventCallback() 里：
// [解析]             pm02.update(Size); pm02.request();   // 解析这一包，再重新挂上接收
// [解析] 【背景知识】"裁判系统"是官方装在车上的黑盒子，它通过串口不停地下发
// [解析]             比赛状态、血量、热量、弹量、机器人坐标、雷达数据等；
// [解析]             这些数据是做功率控制、热量限制、小地图显示的依据。
// [解析] 【关键点】1) 绝大部分数据成员没有"是否收到过"的标志：收到才会被覆盖，
// [解析]             没收到就一直是构造函数里清零的 0，所以不能只凭数值判断"有没有数据"；
// [解析]             只有 radar_ 开头的那几组额外带 _valid + _last_update_ms，能判断新鲜度。
// [解析]           2) 解析动作全部发生在串口中断回调里；任务里读这些成员时若要保证
// [解析]             一组数据的一致性，需要自己进临界区（readme.md 里有示范）。
// [解析] 【阅读顺序】先看本文件了解"有哪些数据、用哪些函数"，再看 pm02.cpp 看"怎么拆帧"。
// [解析] ==========================================================================

#ifndef SP__PM02_HPP
#define SP__PM02_HPP

// [解析] <array>：标准库的定长数组，private 里的 multi_buff_（2×255 字节双缓冲）用它。
#include <array>

// [解析] 下面两个 include 分别提供：
// [解析]   referee_protocol.hpp —— 裁判系统协议"字典"：所有命令码（0x0001、0x0201…）
// [解析]                            和所有数据结构体（GameStatus、PowerHeatData…）都定义在这里，
// [解析]                            本类下面的成员变量类型全部来自它。
// [解析]   usart.h             —— CubeMX 生成的串口句柄声明（huart1/huart6…），以及
// [解析]                            HAL_UARTEx_ReceiveToIdle_DMA 等 HAL 函数的声明。
#include "referee/referee_protocol/referee_protocol.hpp"
#include "usart.h"

namespace sp
{
// ref: https://github.com/HNUYueLuRM/basic_framework/blob/master/modules/referee/rm_referee.c#L21
// [解析] PM02_BUFF_SIZE = 255：串口接收缓冲的大小，也是"允许的最大单帧长度"（单位：字节）。
// [解析] 为什么取 255：这是社区通用实现（basic_framework 的 rm_referee.c）沿用的保守上限，
// [解析]   一次接收就能装下裁判系统实际会下发的最长一帧，同时它也被当作拆帧时的越界保护上限。
constexpr size_t PM02_BUFF_SIZE = 255;

// [解析] PM02 类：一个对象负责一路串口上的全部裁判系统数据。
// [解析] 设计风格：不靠函数返回值传数据，而是把解析结果直接放进 public 成员变量，
// [解析]   上层"读成员"即可；所有成员都标了"只读!"，意思是不要在上层改写它们，
// [解析]   因为它们只是中断里解析结果的"输出箱"，乱写会污染数据。
class PM02
{
public:
  // [解析] 构造函数。参数：
  // [解析]   huart   —— 接裁判系统的那路串口句柄：C 板是 &huart6，达妙板用 &huart1。
  // [解析]   use_dma —— true（默认）用 DMA 接收，CPU 负担小；
  // [解析]              false 改用中断方式接收（DMA 不可用或调试时用）。
  // [解析] 注意：构造函数只清缓冲、设默认值，**不启动接收**；
  // [解析]   真正开始收数据必须显式调用 request()（通常在任务开头调一次）。
  PM02(UART_HandleTypeDef * huart, bool use_dma = true);

  // [解析] 保存构造时传入的串口句柄。它之所以是 public，是为了让中断回调里能写
  // [解析]   if (huart == pm02.huart) { ... } 来判断"这包数据是不是我这路串口的"。
  UART_HandleTypeDef * huart;  // 只读!

  // [解析] ↓↓↓ 从这里到下面的 custom_info，是一整块"数据结果箱"。
  // [解析]   每个成员对应裁判系统的一种命令码（cmd_id），成员后面原注释里的 0x???? 就是命令码，
  // [解析]     例如 game_status 对应 0x0001、power_heat 对应 0x0202。
  // [解析]   这些类型都定义在 referee/referee_protocol/referee_protocol.hpp 里，
  // [解析]     想知道某个字段的单位、取值范围、位域含义，去那个文件看结构体上的注释。
  // [解析]   注意：这批成员**没有** valid 标志，只在"收到对应帧"时被整体覆盖，
  // [解析]     上电到收到第一帧之间它们全是 0（构造函数统一清零），不能据此判断"收到没收到"。
  referee::GameStatus game_status;     // 只读! 0x0001 比赛状态数据
  referee::GameResult game_result;     // 只读! 0x0002 比赛结果数据
  referee::GameRobotHP game_robot_hp;  // 只读! 0x0003 比赛机器人血量数据

  referee::EventData event;                 // 只读! 0x0101 机器人事件数据
  referee::RefereeWarning referee_warning;  // 只读! 0x0104 裁判警告数据
  referee::DartInfo dart_info;              // 只读! 0x0105 飞镖发射相关数据

  referee::RobotStatus robot_status;                  // 只读! 0x0201 机器人状态数据
  referee::PowerHeatData power_heat;                  // 只读! 0x0202 实时底盘缓冲能量和射击热量数据
  referee::RobotPos robot_pos;                        // 只读! 0x0203 机器人位置数据
  referee::Buff buff;                                 // 只读! 0x0204 机器人增益数据
                                                      // 没有0x0205
  referee::HurtData hurt;                             // 只读! 0x0206 伤害状态数据
  referee::ShootData shoot;                           // 只读! 0x0207 实时射击数据
  referee::ProjectileAllowance projectile_allowance;  // 只读! 0x0208 允许发弹量
  referee::RFID_Status rfid_status;                   // 只读! 0x0209 机器人RFID模块状态
  referee::DartClientCmd dart_client_cmd;             // 只读! 0x020A 飞镖选手端指令数据
  referee::GroundRobotPosition ground_robot_pos;      // 只读! 0x020B 地面机器人位置数据
  referee::RadarMarkData radar_mark;                  // 只读! 0x020C 雷达标记进度数据
  referee::SentryInfo sentry_info;                    // 只读! 0x020D 哨兵自主决策信息同步
  referee::RadarInfo radar_info;                      // 只读! 0x020E 雷达自主决策信息同步
  //todo
  // referee::RobotInteractionData robot_interaction;  // 只读! 0x0301 机器人交互数据
  // [解析] ↑ 通用的 0x0301 结构体被作者注释掉了：因为本项目对 0x0301 走的是
  // [解析]   "雷达自定义子编码"分支（见 pm02.cpp 里 ROBOT_INTERACTION_DATA 那个 case），
  // [解析]   子编码 0x0210~0x0213 的结果放在下面 radar_ 开头的那几组成员里。
  referee::MapCommand map_command;       // 只读! 0x0303 选手端小地图交互数据
  referee::MapRobotData map_robot_data;  // 只读! 0x0305 选手端小地图接收雷达数据
  // referee::CustomClientData custom_client_data;     // 只读! 0x0306 自定义控制器与选手端交互数据
  referee::MapData map_data;        // 只读! 0x0307 选手端小地图接收哨兵数据
  referee::CustomInfo custom_info;  // 只读! 0x0308 选手端小地图接收机器人数据

  // [解析] ↓↓↓ 以下 7 组数据来自裁判系统的 0x0301"机器人交互数据"，是由雷达站（或空地链路）
  // [解析]   转发过来的，不是裁判系统原生字段，因此每一组都是"三件套"：
  // [解析]     xxx{}               数据本体（{} 保证做值初始化，不会留下随机值）
  // [解析]     xxx_valid           是否收到过至少一帧合法数据（只有成功解析才置 true）
  // [解析]     xxx_last_update_ms  最近一次成功更新的时刻（HAL_GetTick()，单位 ms）
  // [解析]   判断"数据现在还能不能用"请调用 xxx_fresh(now_ms, timeout_ms)，
  // [解析]     不要只看 _valid：链路断了以后 _valid 仍然是 true，
  // [解析]     只有周期调用 update_radar_data_timeout() 才会把过期数据恢复成默认值。
  // 只读! 0x0301/0x0210 敌方飞镖预警。
  referee::RadarEnemyDartWarning radar_enemy_dart_warning{};
  bool radar_enemy_dart_warning_valid = false;
  uint32_t radar_enemy_dart_warning_last_update_ms = 0;

  // 只读! 0x0301/0x0213 对方空中机器人反制状态。
  referee::RadarAerialCountered radar_aerial_countered{};
  bool radar_aerial_countered_valid = false;
  uint32_t radar_aerial_countered_last_update_ms = 0;

  // 只读! 0x0301/0x0211
  referee::RadarSentryPosition enemy_robot_position{};
  bool enemy_robot_position_valid = false;
  uint32_t enemy_robot_position_last_update_ms = 0;

  // [解析] 下面 4 组（HP / 弹药 / 场地 / Buff）其实来自**同一个**子编码 0x0212：
  // [解析]   雷达把四块数据打包成一帧（用户载荷共 69 字节）发过来，这里拆成 4 个成员方便按需读取，
  // [解析]   它们的 _last_update_ms 会在同一帧里被同时刷新，所以时效性是一致的。
  // 只读! 0x0301/0x0212 合并数据中的 HP。
  referee::RadarAllyHp radar_ally_hp{};
  bool radar_ally_hp_valid = false;
  uint32_t radar_ally_hp_last_update_ms = 0;

  // 只读! 0x0301/0x0212 合并数据中的弹药。
  referee::RadarAllyAmmo radar_ally_ammo{};
  bool radar_ally_ammo_valid = false;
  uint32_t radar_ally_ammo_last_update_ms = 0;

  // 只读! 0x0301/0x0212 合并数据中的场地信息。
  referee::RadarAllyField radar_ally_field{};
  bool radar_ally_field_valid = false;
  uint32_t radar_ally_field_last_update_ms = 0;

  // [解析] 注意：这一组有两条数据来源 —— 0x0212 的合并数据，以及裁判系统直接下发的
  // [解析]   0x0A05"雷达站增益点状态数据"（无线链路）；两条路径写的是同一个成员，谁后到以谁为准。
  // 只读! 0x0301/0x0212 合并数据中的 Buff，也兼容裁判系统直接下发的 0x0A05。
  referee::RadarBuffStatus radar_buff_status{};
  bool radar_buff_status_valid = false;
  uint32_t radar_buff_status_last_update_ms = 0;

  // [解析] request()：启动（或重新挂上）一次串口接收。必须在任务入口调用一次，
  // [解析]   并且每次收到一包、update() 处理完之后都要再调一次，否则链路只收一包就停了。
  // [解析]   DMA 模式下它内部会顺手关掉 DMA 的半传输中断（避免收到一半就触发回调）。
  void request();
  // [解析] update(size)：解析一包数据，**在中断回调里调用**（HAL_UARTEx_RxEventCallback）。
  // [解析]   size 是本次实际收到的字节数（由 DMA 剩余计数换算而来），不是命令码。
  // [解析]   内部先做一次雷达数据超时检查，再逐个拆帧（一个缓冲里可能叠了好几帧）。
  void update(uint16_t size);
  // 周期调用。对应 0x0301 子命令超过 timeout_ms 未更新时，将数据恢复为安全默认值。
  // [解析] update_radar_data_timeout(now_ms, timeout_ms)：给上面 7 组雷达数据做"超时兜底"。
  // [解析]   建议每 10ms 在任务里调一次，now_ms 传 HAL_GetTick()。
  // [解析]   timeout_ms 默认 1500U（1.5 秒）：超过这么久没更新，就把该组数据恢复成安全默认值
  // [解析]     （血量 999、金币 9999 等）并清掉 _valid，避免上层拿过期数据做决策
  // [解析]     （例如以为对面还是残血）；1.5s 比数据实际的更新周期宽裕得多，足以容忍偶尔丢包。
  // [解析]   注意：数据是在串口中断里被改写的，任务里调用本函数前后若要求一致性，
  // [解析]     应放进 taskENTER_CRITICAL()/taskEXIT_CRITICAL() 之间（readme.md 有示范）。
  void update_radar_data_timeout(uint32_t now_ms, uint32_t timeout_ms = 1500U);
  // [解析] send(data, size)：往裁判系统**发**数据（例如 UI 图形包、自定义交互包）。
  // [解析]   返回 HAL 状态码：HAL_OK 成功 / HAL_BUSY 外设忙 / HAL_ERROR 出错 / HAL_TIMEOUT 超时。
  // [解析]   use_dma_ 为 true 时走 DMA，函数立刻返回，所以 data 指向的缓冲区
  // [解析]     在整包发完之前不能被改写，否则发出去的内容会被改坏。
  HAL_StatusTypeDef send(const uint8_t * data, size_t size);

  // [解析] 下面 7 个 xxx_fresh()：判断"这组雷达数据现在还能不能用"。
  // [解析]   返回 true 的条件：收到过（_valid 为 true）**且**距上次更新不超过 timeout_ms。
  // [解析]   参数 now_ms 传 HAL_GetTick()；timeout_ms 默认 1500U，与上面的超时兜底保持一致。
  // [解析]   函数末尾的 const 表示"只读成员、不改对象状态"，可以放心在别处查询。
  bool radar_enemy_dart_warning_fresh(uint32_t now_ms, uint32_t timeout_ms = 1500U) const;
  bool radar_aerial_countered_fresh(uint32_t now_ms, uint32_t timeout_ms = 1500U) const;
  bool enemy_robot_position_fresh(uint32_t now_ms, uint32_t timeout_ms = 1500U) const;
  bool radar_ally_hp_fresh(uint32_t now_ms, uint32_t timeout_ms = 1500U) const;
  bool radar_ally_ammo_fresh(uint32_t now_ms, uint32_t timeout_ms = 1500U) const;
  bool radar_ally_field_fresh(uint32_t now_ms, uint32_t timeout_ms = 1500U) const;
  bool radar_buff_status_fresh(uint32_t now_ms, uint32_t timeout_ms = 1500U) const;

  // TODO UI
  // [解析] ↑ 作者预留的待办：UI 绘制（用 send() 把图形包发给选手端）还没有封装到本类里，
  // [解析]   现有的 UI 功能在 sp_middleware/referee/ui/ 目录里单独实现。

private:
  // [解析] 构造时定下来就再也不变，所以是 const：决定收数据走 DMA 还是走中断。
  const bool use_dma_;

  // [解析] 接收缓冲：DMA（或中断）把收到的原始字节直接灌进这里，拆帧也从这里读。
  // [解析] 它是"裸数组"而不是 std::array，长度靠常量 PM02_BUFF_SIZE 记住。
  uint8_t buff_[PM02_BUFF_SIZE];
  // [解析] 双缓冲（2 份 × 255 字节），看名字是为"DMA 收一份、CPU 解析另一份"准备的。
  // [解析] 注意：当前 pm02.cpp 里没有任何地方用到它（疑似从其它模块版本抄来的残留代码），
  // [解析]   真正在用的只有上面的单缓冲 buff_。改代码时不要被它误导。
  std::array<std::array<uint8_t, PM02_BUFF_SIZE>, 2> multi_buff_;

  // [解析] DMA_NDTR_SIZE = 255 × 2 = 510，正好等于上面双缓冲的总字节数，同样没有被引用（残留）。
  // [解析] 它保留了串口 DMA 计数的语义：NDTR 是 DMA 的"剩余待传输字节数"寄存器，
  // [解析]   传输过程中从总长度往下递减，所以"已收字节数 = 总长度 - NDTR"。
  static constexpr uint32_t DMA_NDTR_SIZE = PM02_BUFF_SIZE * 2;

  // [解析] 真正干活的私有重载：从 frame_start 开始、在 size 个字节里逐帧解析。
  // [解析] 它和上面 public 的 update(uint16_t) 同名但参数不同（这是 C++ 的函数重载），
  // [解析]   这样递归处理"一个缓冲里叠了好几帧"时不必再写一遍同样的逻辑。
  void update(uint8_t * frame_start, uint16_t size);
};

}  // namespace sp

#endif  // SP__PM02_HPP
