// [解析] 本工程只做一件事：M2006 拨弹盘**上电即转**，外加防堵转（反转退弹）与蜂鸣器报警。
// [解析] 没有遥控器、没有底盘：本任务独占 CAN1，config/start、发 0x1FF、收 0x205 全在这个文件里。
// [解析] 只有一个速度环、只用 P 项，所以稳态转速会略低于目标（有静差），这是刻意的。
// [解析] 防堵转：连续 kJamDetectMs 转速低于阈值 → 判卡弹 → 反转 kReverseMs 退弹 → 恢复正转，蜂鸣器响 2s。

#include "cmsis_os.h"

#include <cmath>

#include "io/buzzer/buzzer.hpp"
#include "io/can/can.hpp"
#include "motor/rm_motor/rm_motor.hpp"
#include "tools/pid/pid.hpp"

namespace
{
/*==================== 电机与帧 ====================*/
// [解析] M2006 走 C610：电流满量程 ±10000(raw) = ±10A；中间件按型号自动选了 M2006_MAX_RAW=10000。
// [解析] ID 用 5：ID1~4 的指令帧是 0x200、ID5~8 才是 0x1FF。C610 拨码要拨成 ID 5。
constexpr uint8_t kIdRammer = 5;
constexpr float kRatioM2006 = 36.0f;  // M2006 P36 减速比, 反馈与指令都换算到输出轴

// [解析] 速度环输出的量纲是"输出轴力矩 N·m"(中间件口径)：满量程 1.8 对应 10000raw=10A，
// [解析]   即 0.9 ≈ 5A、0.18 ≈ 1A。拨弹盘不需要大电流，先用 0.9 封顶。
constexpr float kMaxCmd = 0.9f;

/*==================== 控制参数 ====================*/
constexpr float kDt = 1e-3f;  // 控制周期, 单位: s, 必须与 osDelay(1) 一致

// [解析] 目标转速按"发/圈"换算: ω_out[rad/s] = 射频[发/s] × 2π / (发/圈)。
// [解析]   取 8 发/圈、射频 20 发/s ⇒ 15.7 rad/s ≈ 输出 150 rpm ≈ 转子 5400 rpm。
// [解析]   ★ 8 发/圈是估的，请按你们拨盘实测：转一圈拨出几发，就用上面的式子重算。
constexpr float kFeedOmega = 15.7f;    // 正转拨弹, 单位: 输出轴 rad/s
constexpr float kReverseOmega = 6.0f;  // 反转退弹, 故意比正转慢，避免把弹丸推得更死

// [解析] 只有 P：err 15.7 rad/s 时输出 2.36 已被限幅到 0.9，稳态靠 P 顶住负载，会有静差。
constexpr float kKp = 0.15f;  // 单位: (力矩)/(rad/s)。啸叫或抖动就往下调, 起转太肉就往上加

/*==================== 防堵转参数 ====================*/
constexpr float kJamSpeed = 2.5f;       // 低于这个转速(输出轴 rad/s, ≈16% 目标)算"没在转"
constexpr uint32_t kJamDetectMs = 300;  // 连续多久算卡弹（去抖，避免起转瞬间误判）
constexpr uint32_t kGraceMs = 250;      // 起转/换向后的宽限期，这段时间不判堵转
constexpr uint32_t kReverseMs = 300;    // 反转持续多久（退弹行程够用即可）
constexpr uint32_t kBeepMs = 2000;      // 蜂鸣器响 2s

/*==================== 报警音 ====================*/
constexpr float kBeepHz = 5000.0f;
constexpr float kBeepDuty = 0.1f;

/*==================== 对象与状态 ====================*/
sp::CAN can_rammer(&hcan1);
sp::RM_Motor rammer_motor(kIdRammer, sp::RM_Motors::M2006, kRatioM2006);
sp::Buzzer buzzer(&htim4, TIM_CHANNEL_3, 84e6);  // C 板蜂鸣器：TIM4_CH3 / PD14

// [解析] 纯 P 速度环：ki=0、kd=0，max_iout=0（不起作用）。
sp::PID pid(kDt, kKp, 0.0f, 0.0f, kMaxCmd, 0.0f);

enum class State
{
  kFeed,     // 正转拨弹
  kReverse,  // 反转退弹
};

State g_state = State::kFeed;
uint32_t g_state_since_ms = 0;  // 进入当前状态的时刻

bool g_low_active = false;    // 是否正在累计"转速过低"的时间
uint32_t g_low_since_ms = 0;  // 低转速起始时刻

bool g_beep_on = false;        // 蜂鸣器是否在响
uint32_t g_beep_since_ms = 0;  // 开始响的时刻

uint32_t g_jam_count = 0;  // 累计卡弹次数（只给调试器看）

// [解析] 状态切换统一走这里，顺便刷新计时起点；elapsed 用无符号减法，定时器回绕也安全。
void enter(State next, uint32_t now)
{
  g_state = next;
  g_state_since_ms = now;
}

// [解析] 一帧 0x1FF 只装 4 台(ID5~8)，本车只有 M2006 一台，其余槽位保持 0。
// [解析] 帧 ID 直接取 motor.tx_id（由 ID 推导：ID5~8 → 0x1FF），不要在别处再写死一遍。
void send_current()
{
  rammer_motor.write(can_rammer.tx_data);
  if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) > 0) can_rammer.send(rammer_motor.tx_id);
}

// [解析] 非阻塞地维护蜂鸣器：到点就关，不影响控制节拍（用阻塞 Delay 会把拨弹盘冻住 2s）。
void beep_service(uint32_t now)
{
  if (g_beep_on && (now - g_beep_since_ms) >= kBeepMs) {
    buzzer.stop();
    g_beep_on = false;
  }
}

void beep_start(uint32_t now)
{
  buzzer.set(kBeepHz, kBeepDuty);
  buzzer.start();
  g_beep_on = true;
  g_beep_since_ms = now;
}

// [解析] 一个控制周期：状态机 → P 速度环 → 下发。全程非阻塞，1ms 一拍。
void rammer_step()
{
  const uint32_t now = osKernelSysTick();

  // ---- 1. 反馈不在线：什么都别推。看不见转速时输出电流可能把堵转的电机烧掉 ----
  if (!rammer_motor.is_alive(now)) {
    g_low_active = false;
    pid.clear();
    rammer_motor.cmd(0.0f);
    // [解析] 把状态计时推到当下：反馈恢复后会重新获得一个完整宽限期，
    // [解析]   否则电调启动慢时会拿"上电时刻"去算 elapsed，一起转就可能误判卡弹。
    g_state_since_ms = now;
    beep_service(now);
    send_current();
    return;
  }

  // ---- 2. 状态迁移：反转时间到 → 恢复正转（刚上电时 elapsed 很小，天然处于宽限期）----
  uint32_t elapsed = now - g_state_since_ms;
  if (g_state == State::kReverse && elapsed >= kReverseMs) {
    enter(State::kFeed, now);
    elapsed = 0;
  }

  const bool reversing = (g_state == State::kReverse);
  const float target = reversing ? -kReverseOmega : kFeedOmega;

  // ---- 3. 防堵转判定：只在正转且过了宽限期时判 ----
  if (!reversing && elapsed >= kGraceMs) {
    if (std::fabs(rammer_motor.speed) < kJamSpeed) {
      if (!g_low_active) {
        g_low_active = true;
        g_low_since_ms = now;
      } else if ((now - g_low_since_ms) >= kJamDetectMs) {
        // 判定卡弹：反转退弹 + 报警
        g_jam_count++;
        enter(State::kReverse, now);
        g_low_active = false;
        beep_start(now);
      }
    } else {
      g_low_active = false;  // 转起来了就清零
    }
  } else {
    g_low_active = false;  // 反转中/宽限期内不累计
  }

  // ---- 4. 报警计时 + P 速度环 + 下发 ----
  beep_service(now);
  pid.calc(target, rammer_motor.speed);
  rammer_motor.cmd(pid.out);
  send_current();
}

}  // namespace

// [解析] 任务本体：先把 CAN1 配好并开中断，然后进入 1ms 固定节拍。函数不会返回。
extern "C" void rammer_task(void const * argument)
{
  (void)argument;

  can_rammer.config();  // 全通配过滤器（只有 0x205 有用），按 ID 分发在下面的回调里
  can_rammer.start();   // 启动 CAN1 并打开 FIFO0 接收中断

  for (;;) {
    rammer_step();
    osDelay(1);
  }
}

// [解析] CAN1 接收回调：只认 M2006 的 0x205。必须"先把 FIFO 取空再判 ID"，
// [解析]   反过来写会在总线上出现无关帧时卡死在中断里。
extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef * hcan)
{
  if (hcan != &hcan1) return;

  const uint32_t stamp_ms = osKernelSysTick();

  while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0) {
    can_rammer.recv();
    if (can_rammer.rx_id == rammer_motor.rx_id) rammer_motor.read(can_rammer.rx_data, stamp_ms);
  }
}
