// [解析] ==========================================================================
// [解析] 【文件】sp_middleware/motor/cybergear_motor/cybergear_motor.cpp
// [解析] 【作用】cybergear_motor.hpp 里那些函数的具体实现：解析 4 种反馈帧，打包 5 种命令帧。
// [解析] 【看代码的重点】三个地方最容易看不懂，下面都逐行标注了：
// [解析]   1) read() 里"帧头里也藏着信息"——除了 8 字节数据，扩展帧 ID 本身也要拆开解析；
// [解析]   2) 那一大堆 (header >> n) & 掩码 是在把 29 位帧头切成若干字段；
// [解析]   3) 参数反馈里为什么要区分 index==0x7005(只取 1 字节)和其他情况(取 4 字节)。
// [解析] ==========================================================================

#include "cybergear_motor.hpp"

#include "tools/math_tools/math_tools.hpp"
// [解析] 用到定点数换算 uint_to_float / float_to_uint，以及限幅 limit_max。

namespace sp
{
// [解析] 构造函数：初始化列表把 5 个参数分别存进成员变量，函数体为空。
// [解析] master_id / motor_id 是 const 成员，只能在这里赋值。
// [解析] 注意：本构造函数没有初始化 last_read_ms_，但它在收到第一帧反馈时就会被写入，
// [解析]   而 is_alive() 又要求 has_read_ 为真，所以不会读到未初始化的值。
CyberGear_Motor::CyberGear_Motor(
  uint8_t master_id, uint8_t motor_id, float pmax, float vmax, float tmax)
: master_id(master_id), motor_id(motor_id), pmax_(pmax), vmax_(vmax), tmax_(tmax)
{
}

// [解析] 是否收到过反馈帧，用来判断电机有没有通过 CAN 连上。
bool CyberGear_Motor::is_open() const { return has_read_; }

// [解析] 通信是否正常：500ms 内收到过反馈就算活着。
// [解析] 500ms 比达妙电机的 100ms 宽松，因为 CyberGear 的反馈周期本身没那么快，
// [解析]   阈值定太小会频繁误报"掉线"。
// [解析] 用无符号相减 (now_ms - last_read_ms_) 是为了兼容 HAL_GetTick() 的溢出回绕。
bool CyberGear_Motor::is_alive(uint32_t now_ms) const
{
  return is_open() && (now_ms - last_read_ms_ < 500);
}

// [解析] read()：解析一帧反馈。它比达妙电机的 read() 多做一件事 —— 连"帧头"都要拆开看。
// [解析] 参数 header：CAN 扩展帧 ID(can.rx_id)，29 位有效，里面塞了 5 个字段。
// [解析] 参数 data  ：8 字节数据(can.rx_data)。参数 stamp_ms：当前毫秒时间戳。
// [解析] CyberGear 反馈帧头(29 位)的字段划分如下：
// [解析]   bit28~24(5 位) = 通信类型，如 0x02 电机反馈 / 0x15 故障 / 0x11 参数 / 0x00 设备ID
// [解析]   bit23~22(2 位) = 工作模式 mode
// [解析]   bit21~16(6 位) = 状态错误 status_error
// [解析]   bit15~8 (8 位) = 电机 ID
// [解析]   bit7~0  (8 位) = 主机 ID
// [解析] 5+2+6+8+8 = 29 位，正好把帧头填满 —— 所以拆帧头时移位量必须和上表严格对应。
// [解析] 语法小贴士：`header >> 8 & 0xFF` 等价于 `(header >> 8) & 0xFF`，
// [解析]   因为移位 >> 的优先级高于按位与 &。加不加括号结果一样。
void CyberGear_Motor::read(uint32_t header, uint8_t * data, uint32_t stamp_ms)
{
  has_read_ = true;
  // [解析] ↑ 只要进到这里就说明收到帧了；注意它在所有分支之前，所以任何类型的帧都算"活着"。
  last_read_ms_ = stamp_ms;
  feedback_communication_type = ((header) >> 24) & 0x1F;  //先读取帧类型，只取低五位
  // [解析] ↑ 先右移 24 位把类型挪到最低位，再用 0x1F(低 5 位 = 二进制 11111)把其余位掩掉。
  // [解析]   0x1F 而不是 0xFF，是因为这个字段只有 5 位宽。

  //正常工作时的反馈
  if (feedback_communication_type == Communication_Type_MotorRequest) {
    // [解析] ↓ 本分支和下面几个分支对"电机 ID / 主机 ID"的取法不一样，请注意：
    // [解析]   这里认为"低 8 位是主机 ID，bit15~8 是电机 ID"。
    this->feedback_master_id = header & 0xFF;    //指明这一帧发给哪台主机的(取低8位)
    this->feedback_can_id = header >> 8 & 0xFF;  //指明这一帧是哪个电机发的(取bit 8-15)
    this->status_error = (header >> 16) & 0x3F;  //只取低六位
    // [解析] ↑ 0x3F = 低 6 位，对应 bit21~16 的状态错误字段。
    this->mode = (header >> 22) & 0x03;          //只取低两位
    // [解析] ↑ 0x03 = 低 2 位，对应 bit23~22 的工作模式(0 Reset / 1 Cali / 2 Motor)。
    // [解析] ↓ 数据区的 6 个字节被平均分成 3 组，每组 2 字节，拼成 16 位定点数：
    // [解析]   data[0..1] 角度、data[2..3] 速度、data[4..5] 力矩。
    // [解析] 拼法都是"前一字节当高 8 位，后一字节当低 8 位"，这叫大端(高位在前)顺序。
    this->angle = uint_to_float(data[0] << 8 | data[1], -pmax_, pmax_, 16);
    this->speed = uint_to_float(data[2] << 8 | data[3], -vmax_, vmax_, 16);
    this->torque = uint_to_float(data[4] << 8 | data[5], -tmax_, tmax_, 16);
    this->temperature = float((data[6] << 8 | data[7]) & 0xFFFF) / 10.0f;
    // [解析] ↑ 温度是唯一不用定点数换算的量：协议直接给"摄氏度×10"的整数，
    // [解析]   所以除以 10.0f 就得到真实摄氏度(例如 425 -> 42.5℃)。
    // [解析]   写 10.0f 而不是 10，是为了让整个表达式保持浮点运算，避免整数除法把小数丢掉。
  }
  //故障反馈
  else if (feedback_communication_type == Communication_Type_ErrorFeedback) {
    // [解析] 注意：这里"低 8 位 = 电机 ID、bit15~8 = 主机 ID"，和上面 MotorRequest 分支
    // [解析]   正好相反 —— 这不是笔误，而是协议对不同反馈帧规定了不同的字段顺序：
    // [解析]     0x02(电机反馈)：低 8 位是主机 ID，bit15~8 是电机 ID；
    // [解析]     0x15/0x11      ：低 8 位是电机 ID，bit15~8 是主机 ID。
    // [解析]   所以这几个分支不能"统一改写"，照着抄最容易在这里出错。
    this->feedback_can_id = header & 0xFF;                                                // 取低8位
    this->feedback_master_id = (header >> 8) & 0xFF;                                      // 取低8位
    this->error = (data[0] << 24 | data[1] << 16 | data[2] << 8 | data[3]) & 0x0001FFFF;  //取低17位
    // [解析] ↑ 4 个字节拼成一个 32 位整数(同样是高位在前)，但只用低 17 位：
    // [解析]   0x0001FFFF = 二进制 1 后面跟 17 个 1，因为故障位域最高只到 bit16。
    // [解析] 注意：data[0] 是 uint8_t，运算时先提升为 int；若 data[0] ≥ 0x80，
    // [解析]   `data[0] << 24` 会把 1 移进符号位，属于 C++ 中"有符号数左移溢出"的未定义/实现
    // [解析]   定义行为。在 ARM 编译器上通常能得到预期结果，但严格说这么写不够安全
    // [解析]   (对比下面取 MCU_ID 的那段，作者就特意用了 static_cast<uint64_t> 来规避)。
    this->temp_warning = (data[4] << 24 | data[5] << 16 | data[6] << 8 | data[7]) & 0xFFFFFFFF;
    // [解析] ↑ 后 4 个字节拼成温度警告值，单位 0.1℃(如 800 = 80.0℃)。
    // [解析]   末尾 & 0xFFFFFFFF 其实没有实际作用(32 位与全 1 等于本身)，只是表意。
  }
  //单个参数读取的反馈
  else if (feedback_communication_type == Communication_Type_GetSingleParameter) {
    this->feedback_can_id = header & 0xFF;            // 取低8位
    this->feedback_master_id = (header >> 8) & 0xFF;  // 取低8位
    this->index = (data[1] << 8 | data[0]) & 0xFFFF;
    // [解析] ↑ 注意！索引是"低位在前"(小端)：data[0] 是低字节，data[1] 是高字节，
    // [解析]   和前面角度/速度那种"高位在前"的拼法正好相反。写反了就会读到别的参数。
    if (this->index == 0x7005) {
      // [解析] run_mode 是个很小的枚举值(0~3)，协议只用一个字节表示，所以单独处理。
      this->parameter = (int8_t)(data[4] & 0xFF);
      // [解析] ↑ 先取 1 个字节，再强制转成 int8_t —— 这一步是"按有符号数解释"，
      // [解析]     这样 0xFF 会变成 -1 而不是 255。最后隐式转成 float 存进 parameter。
    }
    else {
      this->parameter = data[4] << 24 | data[5] << 16 | data[6] << 8 | data[7];
      // [解析] ↑ 其他参数都是 4 字节大端整数，这里直接转成 float 保存。
      // [解析]   注意：超过 2^24(约 1677 万)的整数用 float 存会丢精度，好在实际参数值都很小。
    }
  }
  //获取设备ID的反馈
  else if (feedback_communication_type == Communication_Type_GetID) {
    this->feedback_can_id = (header >> 8) & 0xFF;  // 取低8位
    // [解析] ↑ 同样取 bit15~8 作为电机 ID；此时低 8 位装的是"结果状态"，见下一行。
    this->ID_gotten = (header & 0xFF) == 0xFE;     //读到0xFE说明获取ID成功
    // [解析] ↑ 0xFE 是协议规定的"成功"标志，不是随便挑的数。
    // 组装64位MCU_ID：必须先使用static_cast提升为uint64_t再移位，避免32位提升后左移>31导致未定义行为
    // [解析] 上面这行原注释就是关键，务必理解：uint8_t 参与运算时会先提升成 int(32 位)，
    // [解析]   如果直接写 data[0] << 56，就是对一个 32 位数左移 56 位，结果是未定义的。
    // [解析]   先 static_cast<uint64_t> 把它变成 64 位无符号数，再移位才安全。
    // [解析] 拼出来的顺序是"data[0] 最高、data[7] 最低"，即大端。
    this->MCU_ID = (static_cast<uint64_t>(data[0]) << 56) | (static_cast<uint64_t>(data[1]) << 48) |
                   (static_cast<uint64_t>(data[2]) << 40) | (static_cast<uint64_t>(data[3]) << 32) |
                   (static_cast<uint64_t>(data[4]) << 24) | (static_cast<uint64_t>(data[5]) << 16) |
                   (static_cast<uint64_t>(data[6]) << 8) | (static_cast<uint64_t>(data[7]));
  }
  // [解析] 如果四种类型都不匹配，函数什么也不做(null 分支)。
  // [解析]   这意味着收到没处理的反馈帧时，上面那些成员会保持上一次的值不变，
  // [解析]   上层若只看 is_open() 是判断不出"这帧没解析成功"的。
}

//CyberGear的一堆命令（真的多，不一定都用得到
// [解析] 上面这行原注释是作者的吐槽：CyberGear 的命令确实多，实际项目里常用的就
// [解析]   运控(0x01)、使能(0x03)、停止(0x04)三个，其余按需取用。
//运动控制模式，只发送力矩，不使用目标角度、角速度、Kp、Kd
// [解析] cmd(力矩)：切到运控模式并缓存目标力矩。这里有一个"教科书式"的做法值得记住：
// [解析]   既然只想控制力矩，那就把位置环、速度环的增益和相关目标值全部置 0，
// [解析]   这样电机的控制律 Kp*(p_des-p) + Kd*(v_des-v) + t_ff 就退化成纯 t_ff。
void CyberGear_Motor::cmd(float torque)
{
  communication_type = Communication_Type_MotionControl;
  // [解析] ↑ 先声明"这一帧是运动控制指令"，write() 看到它才会走对应的填数据分支。
  tar_torque = float_to_uint(torque, -tmax_, tmax_, 16);
  // [解析] ↑ 把 N·m 转成 16 位定点数。这个值同时要拼进 CAN 帧头，所以存在成员里备用。
  // [解析] 注意：这里没有做限幅！传进来的 torque 超出 ±tmax_ 时，换算结果会溢出 16 位范围
  // [解析]   (即结果 > 65535 或为负)，后面拼进帧头时会取到错误的值。用的时候自己先限幅。
  tar_position_ = float_to_uint(0, -4 * sp::SP_PI, 4 * sp::SP_PI, 16);
  // [解析] ↑ 位置目标写 0 rad，量程 ±4π。换算过程：(0-(-4π))/(8π) = 0.5，
  // [解析]   再乘 65535 得到约 32767，正好是 16 位的"中点"，代表角度 0。
  tar_speed_ = float_to_uint(0, -30.0f, 30.0f, 16);
  // [解析] ↑ 速度目标写 0 rad/s，量程 ±30，同样落到中点 32767。
  // [解析] 注意：这里的 30.0f 是写死的字面量，既没用成员 vmax_ 也没用宏 CYBERGEAR_MAX_SPEED。
  // [解析]   如果构造函数传的 vmax 不是 30，本行就与 vmax_ 不一致了；同理上一行的 ±4π
  // [解析]   也没有用 pmax_。这是原代码的写法，用的时候留意量程要按 30 / 4π 来理解。
  Kp_ = float_to_uint(0, 0.0f, 500.0f, 16);
  // [解析] ↑ Kp 量程是 [0, 500](没有负值)，所以传 0 得到的整数就是 0。
  Kd_ = float_to_uint(0, 0.0f, 5.0f, 16);
  // [解析] ↑ Kd 量程是 [0, 5]，同样得到 0 —— 位置环和速度环都被"关掉"了。
}
//获取设备ID命令
// [解析] 一行搞定：只改通信类型。注意这种"只设类型"的函数不会清空 tar_torque 等参数，
// [解析]   不过对于这些命令帧，write() 会直接把数据区填成固定值，不受旧参数影响。
void CyberGear_Motor::cmd_get_id() { communication_type = Communication_Type_GetID; }
//设置电机CAN ID命令
void CyberGear_Motor::cmd_set_can_id() { communication_type = Communication_Type_CanID; }
//使能电机命令
// [解析] 发出去后电机开始工作。第一次使能可能只完成标定(mode 变 1 或 0)，
// [解析]   要等 mode 变成 2(Motor 模式)才能真正执行运动指令。
void CyberGear_Motor::cmd_motor_enable() { communication_type = Communication_Type_MotorEnable; }
//停止电机命令
void CyberGear_Motor::cmd_motor_stop() { communication_type = Communication_Type_MotorStop; }
//设置当前位置为零点命令
void CyberGear_Motor::cmd_set_pos_zero() { communication_type = Communication_Type_SetPosZero; }
//读取单个参数命令
// [解析] 记录下要读哪个参数；参数值会在下一次收到 0x11 反馈时填进成员 parameter。
void CyberGear_Motor::cmd_get_single_parameter(uint16_t index)
{
  communication_type = Communication_Type_GetSingleParameter;
  index_ = index;
}
//设置单个参数命令
// [解析] 记录索引和目标值。真正发出去时，write() 会按"索引(小端 2 字节) + 值"的格式打包。
void CyberGear_Motor::cmd_set_single_parameter(uint16_t index, float parameter)
{
  communication_type = Communication_Type_SetSingleParameter;
  index_ = index;
  parameter_ = parameter;
  // [解析] ↑ 这里 float -> uint32_t 是隐式转换，小数会被直接丢掉(截断而不是四舍五入)。
  // [解析]   例如 parameter = 0.5 会变成 0。写 cur_kp/cur_ki 这类小数参数时要特别注意。
}

//把能写到data里的内容传到can发送数据中
// [解析] write()：整个类的"总出口"。它根据 communication_type 判断这一帧要发什么，
// [解析]   然后把 8 个字节填好。参数 data 一般直接传 can.tx_data(长度必须 ≥ 8)。
// [解析] 重要：本函数只填"数据区"，帧头(通信类型、力矩、电机 ID)由 can.send_ext() 负责拼。
// [解析]   例如运动控制帧里的力矩 tar_torque 并不在这 8 个字节里，而是在扩展帧 ID 中！
void CyberGear_Motor::write(uint8_t * data) const
{
  // [解析] 分支 1：运动控制帧(0x01)。数据区装的是"位置、速度、Kp、Kd"，
  // [解析]   力矩不在这里(它在帧头)。8 个字节两两一组，每组都是大端(高字节在前)。
  if (communication_type == Communication_Type_MotionControl) {
    // data[0] = 0;
    // data[1] = 0;
    // data[2] = 0;
    // data[3] = 0;
    // data[4] = 0;
    // data[5] = 0;
    // data[6] = 0;
    // data[7] = 0;
    //理论上这里发的都是0
    // [解析] 上面那 8 行被注释掉的代码，是"全填 0"的原始写法；下面改成从成员变量取值，
    // [解析]   于是既支持纯力矩控制，也支持位置/速度/Kp/Kd 的组合控制。
    data[0] = tar_position_ >> 8 & 0xFF;
    // [解析] ↑ 取 16 位目标位置的高 8 位。`>> 8 & 0xFF` 先移位再掩码，等价于 (x>>8)&0xFF。
    data[1] = tar_position_ & 0xFF;
    // [解析] ↑ 取低 8 位。赋给 uint8_t 自动截断，等价于 & 0xFF。
    data[2] = tar_speed_ >> 8 & 0xFF;
    data[3] = tar_speed_ & 0xFF;
    data[4] = Kp_ >> 8 & 0xFF;
    data[5] = Kp_ & 0xFF;
    data[6] = Kd_ >> 8 & 0xFF;
    data[7] = Kd_ & 0xFF;
    // [解析] 小结字节分配：data[0..1]=位置、data[2..3]=速度、data[4..5]=Kp、data[6..7]=Kd。
  }
  // [解析] 分支 2：这三种命令只需要"帧头里的通信类型"，数据区全填 0 表示"无附加参数"。
  // [解析]   把 GetID(0x00)、使能(0x03)、改 CAN ID(0x07) 合并成一个分支，因为它们的
  // [解析]   数据格式完全相同，没必要写三遍。
  else if (
    communication_type == Communication_Type_GetID ||
    communication_type == Communication_Type_MotorEnable ||
    communication_type == Communication_Type_CanID) {
    data[0] = 0x00;
    data[1] = 0x00;
    data[2] = 0x00;
    data[3] = 0x00;
    data[4] = 0x00;
    data[5] = 0x00;
    data[6] = 0x00;
    data[7] = 0x00;
  }
  // [解析] 分支 3：停止(0x04) 和 设置零位(0x06) 这两个命令要求数据区第一个字节写 0x01，
  // [解析]   相当于"确认执行"的标志位；后面 7 个字节补 0。
  // [解析]   如果这个 0x01 忘了写，电机可能收到命令却毫无反应 —— 这是新手常踩的坑。
  else if (
    communication_type == Communication_Type_MotorStop ||
    communication_type == Communication_Type_SetPosZero) {
    data[0] = 0x01;
    data[1] = 0x00;
    data[2] = 0x00;
    data[3] = 0x00;
    data[4] = 0x00;
    data[5] = 0x00;
    data[6] = 0x00;
    data[7] = 0x00;
  }
  // [解析] 分支 4：读参数(0x11)。数据区只放 2 个字节的参数索引，而且是"小端"(低字节在前)：
  // [解析]   0x7005 会被发成 data[0]=0x05、data[1]=0x70。
  // [解析]   这个顺序和 read() 里解析索引的顺序是一致的，两边必须配套。
  else if (communication_type == Communication_Type_GetSingleParameter) {
    data[0] = index_ & 0xFF;
    data[1] = (index_ >> 8) & 0xFF;
    data[2] = 0x00;
    data[3] = 0x00;
    data[4] = 0x00;
    data[5] = 0x00;
    data[6] = 0x00;
    data[7] = 0x00;
  }
  // [解析] 分支 5：写参数(0x12)。前 2 字节是索引(小端)，后 4 字节是参数值。
  else if (communication_type == Communication_Type_SetSingleParameter) {
    data[0] = index_ & 0xFF;
    data[1] = (index_ >> 8) & 0xFF;
    data[2] = 0x00;
    data[3] = 0x00;
    // [解析] ↓ 绝大多数参数是 4 字节整数，按"大端"填：data[4] 最高位、data[7] 最低位。
    if (index_ != run_mode) {
      data[4] = parameter_ >> 24 & 0xFF;
      data[5] = (parameter_ >> 16) & 0xFF;
      data[6] = (parameter_ >> 8) & 0xFF;
      data[7] = parameter_ & 0xFF;
    }
    // [解析] ↓ 唯一的例外是 run_mode(0x7005)：它只是个 0~3 的枚举，协议规定只占 1 个字节。
    // [解析]   如果这里也写 4 个字节，电机会把模式值当成"高字节在前的 32 位数"，
    // [解析]   结果就是 0，模式永远设不上 —— 这个特判绝对不能删。
    else if (index_ == run_mode) {
      data[4] = (uint8_t)(parameter_);
      // [解析] ↑ 取最低 1 个字节(这里体现了 parameter_ 用整型保存的好处)。
      data[5] = 0x00;
      data[6] = 0x00;
      data[7] = 0x00;
    }
  }
  // [解析] 若 communication_type 不在这 5 个分支里(例如 0x0C)，data 一个字节都不会被写，
  // [解析]   会原样保留上次的内容。发送前最好确认 communication_type 是有效值。
}

}  // namespace sp
