// [解析] ==========================================================================
// [解析] 【文件】sp_middleware/io/vision/vision.hpp
// [解析] 【作用】定义"电控板 <-> 视觉/自瞄上位机"之间的通信协议（走 USB 虚拟串口 CDC），
// [解析]         并提供 Vision 类：收上位机算好的目标角度，回传本车姿态、弹速和超电状态。
// [解析] 【被谁用】applications/vision_task.cpp：
// [解析]           sp::Vision vis;                        // 创建对象
// [解析]           vis.send(mode, imu.q, imu.yaw, ...);   // 周期发送（该任务里是 1 kHz）
// [解析]           extern "C" void vision_callback(uint8_t * buf, uint32_t len) { vis.update(buf, len); }
// [解析]         ★该任务目前没有编译进工程（CMakeLists.txt 第 69~72 行还是注释状态）。
// [解析]           要真正启用，还得先在 CubeMX 里打开 USB_OTG_FS(Device) + CDC 中间件。
// [解析] 【关键点，先记住这四条再看代码】
// [解析]   1) 所有结构体都带 __attribute__((packed))。这是收发双方约定的"线格式"：
// [解析]      禁止编译器插入对齐填充，保证 sizeof 正好等于字节数、每个字段的偏移固定不变。
// [解析]      两端（STM32 和上位机）必须用同一份布局，否则解析出来全是乱码。
// [解析]   2) 每个结构体后面紧跟一句 static_assert(sizeof(...) <= N)：
// [解析]      编译期就把"帧太长"这种错误拦下来，运行时零开销、不占 Flash。
// [解析]   3) 帧头是两个字符：'S''P' 是与自瞄/视觉上位机的帧，'S''C' 是另一种 288 字节大包；
// [解析]      校验用 CRC16，放在结构体最后 2 个字节，且是"小端"存放（低字节在前）。
// [解析]   4) 收发两端都依赖"默认成员初始化器" head[2] = {'S','P'}：send() 里从不给 head 赋值，
// [解析]      靠它自动就位。这要求 Vision 对象是全局/静态创建的（vision_task.cpp 就是这么写的）。
// [解析] ==========================================================================

#ifndef SP__VISION_HPP
#define SP__VISION_HPP

#include <cstdint>
// [解析] 用 uint8_t / uint16_t 这些定长类型，保证字段宽度和上位机约定的一致。

namespace sp
{
// [解析] 【方向: 上位机 -> 电控板】自瞄上位机发来的控制指令，一共 30 字节。
// [解析] 构成：2(帧头) + 1(模式) + 6*4(yaw/pitch 及其角速度、角加速度) + 1(目标编号) + 2(CRC16)。
struct __attribute__((packed)) VisionToGimbal
{
  uint8_t head[2] = {'S', 'P'};
  // [解析] ↑ 帧头，固定为字符 'S'(0x53) 和 'P'(0x50)。
  // [解析]   这是"默认成员初始化器"：对象一被创建，这两个字节就自动是 'S''P'，
  // [解析]   所以后面的 send() / update() 里完全不用管它。
  // [解析]   收包方靠它判断"这一包到底是不是发给我的、是哪种包"。
  uint8_t mode;  // 0: 不控制, 1: 控制云台但不开火，2: 控制云台且开火
  // [解析] ↑ 上位机的工作模式，只有 0/1/2 三种取值：
  // [解析]   0 = 不接管（云台由电控自己控制）；1 = 接管云台但不开火；2 = 接管云台并开火。
  // [解析]   Vision::update() 会把它翻译成两个更顺手的布尔量 control 和 fire。
  float yaw;
  // [解析] ↑ 期望的云台 yaw 目标角度。单位 rad（弧度），这是本工程与上位机的约定。
  // [解析]   电控拿它和自己的 imu.yaw 相减得到误差，再送进云台 PID。
  float yaw_vel;
  // [解析] ↑ 目标的 yaw 角速度，单位 rad/s。这是"前馈"量：
  // [解析]   目标自己在转，提前把它转的速度叠加上去，跟踪会跟得更紧、滞后更小。
  float yaw_acc;
  // [解析] ↑ 目标的 yaw 角加速度，单位 rad/s²。同样是前馈量，实际用得比较少。
  float pitch;
  // [解析] ↑ 期望的云台 pitch 目标角度，单位 rad。pitch 是"抬头/低头"角，别和 yaw（左右转）搞混。
  float pitch_vel;
  // [解析] ↑ pitch 方向角速度，单位 rad/s。
  float pitch_acc;
  // [解析] ↑ pitch 方向角加速度，单位 rad/s²。
  uint8_t target;  //0:英雄,1:工程,2:3号步兵,3:4号步兵,4:5号步兵,5:哨兵,6:前哨,7:基地,8:无目标
  // [解析] ↑ 上位机当前锁定的目标编号（RoboMaster 裁判系统里的机器人 ID 约定）。
  // [解析]   ★注意 8 表示"没有锁定任何目标"，电控侧应据此判断该不该开火
  // [解析]   （就算 mode == 2，没有目标也不该真的打）。
  uint16_t crc16;
  // [解析] ↑ CRC16 校验值，占最后 2 个字节，而且是"小端"存放：
  // [解析]   低字节在偏移 len-2、高字节在偏移 len-1（见 tools/crc/crc.cpp 的 append_crc16）。
  // [解析]   收包方用 check_crc16() 重算一遍做比对，不一致就说明这一包被干扰了，整包丢弃。
};
static_assert(sizeof(VisionToGimbal) <= 64);
// [解析] 编译期检查：这一帧必须能塞进 64 字节（USB 全速 CDC 单个批量端点包的最大长度）。
// [解析] 为什么卡在 64？USBD 的 CDC 发送接口不做排队，超长/忙时的数据会被直接丢掉，
// [解析] 与其上线后莫名其妙丢包，不如让编译器当场报错。当前实际 sizeof = 30 字节，余量充足。

// [解析] 【方向: 电控板 -> 上位机】回传给自瞄上位机的本车状态，一共 57 字节。
// [解析] 上位机拿到姿态和弹速后，才能算出"该往哪打、抬多少"（弹道解算），
// [解析] 再把结果打包成上面的 VisionToGimbal 发回来 —— 这构成一个闭环。
struct __attribute__((packed)) GimbalToVision
{
  uint8_t head[2] = {'S', 'P'};
  // [解析] ↑ 帧头 'S''P'，同样靠默认初始化器自动填好。
  uint8_t mode;  // 0: 空闲, 1: 自瞄, 2: 小符, 3: 大符
  // [解析] ↑ 本车当前的工作模式：0 = 空闲（不打）；1 = 自瞄打装甲板；
  // [解析]   2 = 打小能量机关；3 = 打大能量机关。上位机据此切换识别算法。
  float q[4];    // wxyz顺序
  // [解析] ↑ 姿态四元数，4 个 float 共 16 字节，顺序是 w, x, y, z（"wxyz"）。
  // [解析]   四元数可以理解成"描述三维旋转的 4 个数"：w 是实部，xyz 是虚部，
  // [解析]   满足 w² + x² + y² + z² = 1（单位四元数）。
  // [解析]   相比欧拉角它没有"万向锁"问题，也不会有角度跳变（±180° 回绕），
  // [解析]   所以发给上位机做弹道解算更稳。★顺序千万别写反，wxyz 和 xyzw 完全对不上。
  float yaw;
  // [解析] ↑ 本车云台当前 yaw 角，单位 rad。
  float yaw_vel;
  // [解析] ↑ 本车 yaw 角速度，单位 rad/s。上位机用来做运动补偿（目标预测）。
  float pitch;
  // [解析] ↑ 本车云台当前 pitch 角，单位 rad。
  float pitch_vel;
  // [解析] ↑ 本车 pitch 角速度，单位 rad/s。
  float bullet_speed;
  // [解析] ↑ 弹丸出膛初速度，单位 m/s。★这是弹道解算的关键参数：
  // [解析]   换弹种、改摩擦轮转速、换电池电压都会让它变化，必须实测标定
  // [解析]   （vision_task.cpp 里目前写的是 24.0f）。标错了就会"瞄得准、打不中"。
  uint16_t bullet_count;  // 子弹累计发送次数
  // [解析] ↑ 累计发弹数，上位机用它做弹道修正和"还剩几发"的显示。
  // [解析] ★注意：原注释写的是"累计发送次数"，字面看像是"这个字段被发了多少次"，
  // [解析]   但结合 RoboMaster 语义更可能是"累计发射的弹丸数"。
  // [解析]   两种理解差很多，接手时要向上位机同学确认清楚。
  float supercap_power_in;
  // [解析] ↑ 超级电容的输入功率，单位 W（瓦）。来自超电模块的遥测。
  float supercap_power_out;
  // [解析] ↑ 超级电容的输出功率，单位 W。
  float supercap_voltage;
  // [解析] ↑ 超级电容电压，单位 V（伏）。上位机据此估算还能不能支撑连续射击。
  uint8_t supercap_temperature;
  // [解析] ↑ 超级电容温度，单位 ℃。用 1 个字节存（0~255 够用），所以是整数、没有小数。
  uint8_t supercap_status;
  // [解析] ↑ 超级电容状态标志字节（各厂家定义不同，一般是"就绪/充电中/故障"之类的枚举）。
  uint16_t crc16;
  // [解析] ↑ CRC16 校验，最后 2 字节，小端。算法和 VisionToGimbal 完全一致。
};

static_assert(sizeof(GimbalToVision) <= 64);
// [解析] 实际 sizeof = 57（2+1+16+4+4+4+4+4+2+4+4+4+1+1+2），小于 64，一包发得完。

// [解析] 【方向: 电控板 -> 上位机】英雄机器人专用版，比上面只多 1 个字节 game_progress，
// [解析] 一共 58 字节。为什么要单独搞一个结构体？因为英雄有"比赛阶段/血量"这类
// [解析] 别的兵种用不到的信息，而协议又是固定长度、字段顺序不能随便加的：
// [解析] ★如果直接在 GimbalToVision 里加字段，所有兵种（包括已经调好的步兵）都要跟着改，
// [解析]   所以作者选择"复制一份再加字段"，用两个 send() 重载分别发送。
// [解析]   代价是代码重复，以后改字段要记得两处都改，否则两个版本会悄悄不一致。
struct __attribute__((packed)) GimbalToVisionHero
{
  uint8_t head[2] = {'S', 'P'};
  uint8_t mode;  // 0: 空闲, 1: 自瞄, 2: 小符, 3: 大符
  float q[4];    // wxyz顺序
  float yaw;
  float yaw_vel;
  float pitch;
  float pitch_vel;
  float bullet_speed;
  uint16_t bullet_count;  // 子弹累计发送次数
  float supercap_power_in;
  float supercap_power_out;
  float supercap_voltage;
  uint8_t supercap_temperature;
  uint8_t supercap_status;
  // [解析] ↑ 以上字段含义与 GimbalToVision 完全相同，这里不再重复解释；
  // [解析]   唯一的区别就在下面这一行 —— 多出来的 game_progress。
  uint8_t game_progress;
  // [解析] ↑ 比赛进程/阶段标志（英雄专用），由裁判系统或上层逻辑给出，
  // [解析]   上位机据此判断该不该自瞄、该用什么策略。
  uint16_t crc16;
  // [解析] ↑ CRC16 校验，最后 2 字节，小端存放。注意多了一个字节后，
  // [解析]   CRC 覆盖的范围变成了 58-2 = 56 字节，两端必须同时改，否则校验永远不过。
};

static_assert(sizeof(GimbalToVisionHero) <= 64);
// [解析] 实际 sizeof = 58，依然小于 64。

// [解析] 【方向: 上位机 -> 电控板】另一种"大包"：帧头是 'S''C'，固定 293 字节。
// [解析] 2('S''C') + 1(seq) + 288(任意数据) + 2(CRC16) = 293。
// [解析] 它和 'S''P' 那套完全是两类数据：'S''P' 是一帧几十字节的控制指令，
// [解析] 这个是 288 字节的大块数据（常见用途：给下位机传图像坐标列表、自定义协议负载）。
struct __attribute__((packed)) VisionToHanging
{
  uint8_t head[2] = {'S', 'C'};
  // [解析] ↑ 帧头 'S'(0x53) + 'C'(0x43)，第二个字符不同就是为了和 'S''P' 区分开。
  uint8_t seq;
  // [解析] ↑ 包序号，每发一包加 1（0~255 循环）。收包方用它发现丢包/乱序，
  // [解析]   也顺便验证"数据是不是新鲜的"。
  uint8_t data[288];
  // [解析] ↑ 288 字节的自定义数据区。这个数字显然是上位机那边定死的固定长度，
  // [解析]   ★注意：仓库里没有文档说明 288 的来历（288 = 0x120，也等于 24*12），
  // [解析]   改动前一定要和上位机同学确认，两边长度不一致会导致 CRC 永远校验失败。
  uint16_t crc16;
  // [解析] ↑ CRC16，最后 2 字节，小端；覆盖前面 291 个字节。
};
static_assert(sizeof(VisionToHanging) <= 293);
// [解析] ★注意：这里的上限是 293 而不是 64，说明这个"大包"是故意超过 USB 单包长度的。
// [解析]   上位机发送时 USB 驱动会自动把它拆成多个包；本类的 update() 用 512 字节缓冲区
// [解析]   累积并等待凑满 293 字节才解析，所以收得下来 —— 但也因此"一次回调给不全"，见 vision.cpp。

// [解析] Vision 类：串起"收"和"发"两条路。★★这个类没有写构造函数★★
// [解析]   —— 它靠编译器自动生成的默认构造函数（无参数）来初始化。
// [解析]   后果：applications/vision_task.cpp 里原本写的 sp::Vision vis(false, false);
// [解析]   会因为"没有接收两个 bool 的构造函数"而编译报错，必须写成 sp::Vision vis;
// [解析]   这也是那个文件里被注释说明过的历史问题。
// [解析]   另外：因为没有构造函数，所有成员的初值都靠"默认成员初始化器"（= 0 / = false / = {0}）。
class Vision
{
public:
  // [解析] ↓ 下面这一段全是 public 成员，不需要 getter 就能直接读 ——
  // [解析]   这是为了调试方便：调试时把 vis 加到 watch 窗口就能看到全部状态。
  bool control = false;
  // [解析] ↑ 上位机是否要求接管云台。update() 里由 mode != 0 得到。
  bool fire = false;
  // [解析] ↑ 上位机是否要求开火。update() 里由 mode == 2 得到。
  // [解析]   ★注意：这只是"上位机的意愿"，电控侧还要自己判断有没有目标、
  // [解析]   弹丸是否允许发射（防止打自己人、防止超热量），不能拿到 true 就无脑开火。
  float yaw = 0;
  // [解析] ↑ 上位机给的目标 yaw 角，单位 rad。
  float yaw_vel = 0;
  // [解析] ↑ 目标 yaw 角速度，单位 rad/s（前馈用）。
  float yaw_acc = 0;
  // [解析] ↑ 目标 yaw 角加速度，单位 rad/s²。
  float pitch = 0;
  // [解析] ↑ 目标 pitch 角，单位 rad。
  float pitch_vel = 0;
  // [解析] ↑ 目标 pitch 角速度，单位 rad/s。
  float pitch_acc = 0;
  // [解析] ↑ 目标 pitch 角加速度，单位 rad/s²。
  uint8_t auto_aim_target = 8;  //默认无目标
  // [解析] ↑ 当前锁定的目标编号，取值含义同 VisionToGimbal::target（8 = 无目标）。
  // [解析]   初值特意设成 8：刚上电还没收到上位机数据时，逻辑上应该是"没有目标"。

  bool autoaim_alive = false;
  // [解析] ↑ 上位机"还活着"的标志：两次收到数据的时间间隔小于 100 ms 就认为在线。
  // [解析]   它由 update() 维护。★注意实现细节：update() 是先用"上一次"的时间戳算这个标志、
  // [解析]   然后再刷新时间戳，所以它反映的其实是"上一帧到再上一帧"的间隔，
  // [解析]   上电后收到的第一帧一定是 false（上一次时间戳还是 0）。

  uint8_t data[288] = {0};
  // [解析] ↑ 从 'S''C' 大包里拷出来的 288 字节负载，怎么解释由你自己和上位机约定。
  uint8_t seq = 0;
  // [解析] ↑ 最近一包大包的序号，用来发现丢包。

  uint32_t autoaim_last_read_ms_ = 0;
  // [解析] ↑ 最近一次收到有效上位机数据的时刻（osKernelSysTick() 的毫秒计数），
  // [解析]   给 autoaim_alive 做判断用。
  // [解析] ★注意：它名字带下划线后缀（本工程里下划线通常表示 private 成员），
  // [解析]   但它确实写在 public 区域里，外部可以直接改 —— 命名和访问权限不太一致。
  void update(uint8_t * buf, uint32_t len);
  // [解析] ↑ 解析从 USB 收到的一批字节。参数 buf = 数据首地址，len = 本次字节数。
  // [解析]   它通常被 USB 接收回调调用，也就是说跑在中断上下文里：
  // [解析]   ★不能在里面做耗时操作（不能 osDelay、不能长时间关中断）。
  // [解析]   功能：把数据追加到内部缓冲区 -> 找 'S''P' 或 'S''C' 帧头 -> 校验 CRC16 ->
  // [解析]   通过就把字段更新到上面那些 public 成员里。
  void send(
    uint8_t mode, float q[4], float yaw, float yaw_vel, float pitch, float pitch_vel,
    float bullet_speed, uint16_t bullet_count, float supercap_power_in, float supercap_power_out,
    float supercap_voltage, uint8_t supercap_temperature, uint8_t supercap_status);
  // [解析] ↑ 普通兵种版：把本车状态打包发给上位机（GimbalToVision，57 字节）。
  // [解析]   参数逐个含义（单位）：
  // [解析]     mode              工作模式：0 空闲 / 1 自瞄 / 2 小符 / 3 大符
  // [解析]     q[4]              姿态四元数，wxyz 顺序（注意是数组，传的是首地址）
  // [解析]     yaw, yaw_vel      本车 yaw 角(rad) 与角速度(rad/s)
  // [解析]     pitch, pitch_vel  本车 pitch 角(rad) 与角速度(rad/s)
  // [解析]     bullet_speed      弹丸初速(m/s)，弹道解算必需，必须实测标定
  // [解析]     bullet_count      累计发弹数
  // [解析]     supercap_*        超电的输入功率(W)/输出功率(W)/电压(V)/温度(℃)/状态字节
  // [解析]   函数内部会自动算 CRC16 并调用 USB 发送，不需要你自己填 head 和 crc16。
  // [解析]   返回值：无。发送是否成功它不告诉你（见 vision.cpp 里的说明）。
  void send(
    uint8_t mode, float q[4], float yaw, float yaw_vel, float pitch, float pitch_vel,
    float bullet_speed, uint16_t bullet_count, float supercap_power_in, float supercap_power_out,
    float supercap_voltage, uint8_t supercap_temperature, uint8_t supercap_status,
    uint8_t game_progress);
  // [解析] ↑ 英雄版：比上面多一个 game_progress（比赛进程标志），发出去的是
  // [解析]   GimbalToVisionHero，58 字节。其余参数含义完全一致。
  VisionToHanging rx_data_hanging_;
  // [解析] ↑ 最近一包 'S''C' 大包的原始结构体（含 head/seq/data/crc16）。
  // [解析]   ★注意它虽然定义在 public 区（因为在 private: 之前），但名字风格像私有成员，
  // [解析]   平时用上面的 data[] / seq 就够了，除非你要看原始帧头。

private:
  // [解析] ↓ 以下是内部用的缓冲区，外部看不见，避免被误改。
  VisionToGimbal rx_data_;
  // [解析] ↑ 最近一帧解析成功的上位机指令（'S''P' 包）。
  // [解析]   update() 会先把整包字节拷进来，再从它身上取各个字段 —— 这样 CRC 校验和字段提取
  // [解析]   可以分开做，代码更清楚。
  GimbalToVision tx_data_;
  // [解析] ↑ 待发送的普通版数据帧。send() 只改字段，head 靠默认初始化器保持 'S''P'。
  GimbalToVisionHero tx_data_with_game_progress_;
  // [解析] ↑ 待发送的英雄版数据帧，和上面是两个独立对象，互不影响。

  uint8_t rx_buffer_[512] = {0};
  // [解析] ↑ 接收累积缓冲区，512 字节。为什么比最大的帧(293)还大？
  // [解析]   因为 USB 回调一次可能只送来 64 字节，要攒几次才能凑满一包；
  // [解析]   而且一次回调里可能同时送来"上一包的尾巴 + 下一包的开头"。
  uint16_t rx_len_ = 0;
  // [解析] ↑ rx_buffer_ 里当前已累积了多少字节（有效数据长度），初值 0。
};

}  // namespace sp

#endif  // SP__VISION_HPP
