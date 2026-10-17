// [解析] 裁判系统（官方装在车上的黑盒子，串口下发比赛状态/血量/热量/坐标等）的串口接收模块：一个对象 = 一路串口 + 一堆只读数据成员。
// [解析] 用在 applications/uart_task.cpp：任务里 pm02.request() 启动接收，中断回调里 update(Size) 解析后再 request() 重新挂上。
// [解析] 最容易踩的坑：除 radar_* 那几组，其它成员没有 valid 标志，没收到就是 0，不能凭数值判断有没有数据。
// [解析] 解析全部发生在串口中断里；任务里要保证多个字段的一致性，需要自己进临界区（readme.md 有示范）。

#ifndef SP__PM02_HPP
#define SP__PM02_HPP

#include <array>

#include "referee/referee_protocol/referee_protocol.hpp"
#include "usart.h"

namespace sp
{
// ref: https://github.com/HNUYueLuRM/basic_framework/blob/master/modules/referee/rm_referee.c#L21
// [解析] 255 是社区实现（basic_framework 的 rm_referee.c）沿用的保守上限：一次能装下最长一帧，同时用作拆帧的越界保护上限。
constexpr size_t PM02_BUFF_SIZE = 255;

// [解析] 数据不靠返回值传，解析结果直接写进 public 成员；标"只读!"的成员是中断里解析结果的输出箱，上层不要改写。
// [解析] 成员名对应一种命令码（原注释里的 0x????），字段单位/位域含义见 referee_protocol.hpp 的结构体注释。
class PM02
{
public:
  PM02(UART_HandleTypeDef * huart, bool use_dma = true);

  // [解析] 它之所以是 public，是为了让中断回调能写 if (huart == pm02.huart) 判断这包是不是本路串口的。
  UART_HandleTypeDef * huart;  // 只读!

  // [解析] 下面这一整块是"数据结果箱"，每个成员对应一种 cmd_id，只在收到对应帧时才被整体覆盖。
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
  referee::MapCommand map_command;       // 只读! 0x0303 选手端小地图交互数据
  referee::MapRobotData map_robot_data;  // 只读! 0x0305 选手端小地图接收雷达数据
  // referee::CustomClientData custom_client_data;     // 只读! 0x0306 自定义控制器与选手端交互数据
  referee::MapData map_data;        // 只读! 0x0307 选手端小地图接收哨兵数据
  referee::CustomInfo custom_info;  // 只读! 0x0308 选手端小地图接收机器人数据

  // [解析] 以下 7 组来自 0x0301 机器人交互数据（雷达站/空地链路转发），每组都是"数据 + _valid + _last_update_ms"三件套。
  // [解析] 判断能不能用请调 xxx_fresh()，别只看 _valid：链路断了 _valid 仍是 true，要靠 update_radar_data_timeout() 兜底。
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

  // [解析] 下面 4 组（HP / 弹药 / 场地 / Buff）其实来自同一个子编码 0x0212，一帧里同时刷新，时效性一致。
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

  // [解析] 这个成员有两条来源（0x0212 的 Buff 块、裁判系统直发的 0x0A05），谁后到以谁为准。
  // 只读! 0x0301/0x0212 合并数据中的 Buff，也兼容裁判系统直接下发的 0x0A05。
  referee::RadarBuffStatus radar_buff_status{};
  bool radar_buff_status_valid = false;
  uint32_t radar_buff_status_last_update_ms = 0;

  // [解析] 一次只挂一包：任务开头调一次，之后每次 update() 处理完都要再调，否则收一包就断流。
  void request();
  // [解析] 在中断回调（HAL_UARTEx_RxEventCallback）里调，size 是本次实收字节数；内部先做雷达超时兜底再逐帧拆。
  void update(uint16_t size);
  // 周期调用。对应 0x0301 子命令超过 timeout_ms 未更新时，将数据恢复为安全默认值。
  // [解析] 任务里建议每 10ms 调一次；它和串口中断抢这几组成员，要求一致性时要进临界区。
  void update_radar_data_timeout(uint32_t now_ms, uint32_t timeout_ms = 1500U);
  // [解析] use_dma_ 为 true 时走 DMA 立刻返回，整包发完前 data 指向的缓冲区不能被改写。
  HAL_StatusTypeDef send(const uint8_t * data, size_t size);

  // [解析] 这 7 个 xxx_fresh()：_valid 为真且距上次更新不超过 timeout_ms 才算可用，now_ms 传 HAL_GetTick()。
  bool radar_enemy_dart_warning_fresh(uint32_t now_ms, uint32_t timeout_ms = 1500U) const;
  bool radar_aerial_countered_fresh(uint32_t now_ms, uint32_t timeout_ms = 1500U) const;
  bool enemy_robot_position_fresh(uint32_t now_ms, uint32_t timeout_ms = 1500U) const;
  bool radar_ally_hp_fresh(uint32_t now_ms, uint32_t timeout_ms = 1500U) const;
  bool radar_ally_ammo_fresh(uint32_t now_ms, uint32_t timeout_ms = 1500U) const;
  bool radar_ally_field_fresh(uint32_t now_ms, uint32_t timeout_ms = 1500U) const;
  bool radar_buff_status_fresh(uint32_t now_ms, uint32_t timeout_ms = 1500U) const;

  // TODO UI

private:
  const bool use_dma_;

  uint8_t buff_[PM02_BUFF_SIZE];
  // [解析] multi_buff_ 和下面的 DMA_NDTR_SIZE 目前没有被 pm02.cpp 引用，是残留代码，别被它误导。
  std::array<std::array<uint8_t, PM02_BUFF_SIZE>, 2> multi_buff_;

  static constexpr uint32_t DMA_NDTR_SIZE = PM02_BUFF_SIZE * 2;

  void update(uint8_t * frame_start, uint16_t size);
};

}  // namespace sp

#endif  // SP__PM02_HPP
