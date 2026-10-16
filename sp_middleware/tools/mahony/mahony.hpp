// [解析] ==========================================================================
// [解析] 【文件】sp_middleware/tools/mahony/mahony.hpp
// [解析] 【作用】Mahony 姿态解算（AHRS 算法）的头文件：把"加速度计"和"陀螺仪"两路数据融合，
// [解析]         算出物体的姿态（四元数 q + 欧拉角 yaw/pitch/roll）以及它们的角速度。
// [解析] 【被谁用】applications/imu_task.cpp 里创建全局对象 sp::Mahony imu(1e-3f);
// [解析]           并在 1000Hz 的循环里每 1ms 调用一次 imu.update(bmi088.acc, bmi088.gyro)。
// [解析]           算出来的姿态又被 applications/vision_task.cpp 打包发给视觉，
// [解析]           也被 sp_middleware/tools/gimbal/gimbal.cpp 用来做云台与底盘的坐标换算。
// [解析] 【关键点】最容易踩坑的四件事：
// [解析]   1) 输入单位：acc 是 m/s²（含重力 9.8），gyro 是 rad/s；两者都必须是"载体系"下的值。
// [解析]   2) 四元数顺序是 {w, x, y, z}，不是数学书常见的 {x, y, z, w}，顺序搞反结果全错。
// [解析]   3) q 的含义：把同一个向量从"载体系(车体)"换到"地面系(世界系)"，
// [解析]      即 v_world = q ⊗ v_body ⊗ q*（q* 是共轭）。
// [解析]   4) 构造函数里的 dt 必须等于真实调用周期，否则"角速度×dt"积分出来的角度会越飘越远。
// [解析] 【阅读顺序】先看本文件了解有哪些输入输出，再看 mahony.cpp 看算法怎么实现的。
// [解析] ==========================================================================

#ifndef SP__MAHONY_HPP
#define SP__MAHONY_HPP

namespace sp
{
// [解析] Mahony 类：一个对象 = 一个 IMU 的姿态解算器。
// [解析] 用法固定两步：先构造 sp::Mahony imu(1e-3f)，再每个周期 imu.update(acc, gyro)。
// [解析] 结果全部放在下面的公共成员变量里，直接读即可（这些成员都是只读的，别手动改）。
// [解析] 提醒：对象内部"有记忆"（四元数、积分项等），所以全工程只应在
// [解析]   applications/imu_task.cpp 这一个任务里调用它，不要多个任务同时调用。
class Mahony
{
public:
  // [解析] 构造函数：只把参数存起来，不算任何东西（第一次计算在 update() 里）。
  // [解析]   dt —— 调用 update() 的周期，单位 s。本项目 imu_task 是 1e-3f，即 1000Hz。
  // [解析]         必须和真实调用周期严格一致：dt 填错等于把积分时间尺度搞错，姿态会漂。
  // [解析]   kp —— 对加速度计（重力方向）的置信度，默认 0.5f。
  // [解析]         kp 越大越相信重力方向（pitch/roll 收敛快，但车体加速度/振动会被当成倾角）；
  // [解析]         kp 越小越相信陀螺仪积分（平滑，但会慢慢漂）。
  // [解析]   ki —— 积分增益，默认 0.0f 表示关闭。用来消掉陀螺仪零偏；校准过陀螺仪就填 0。
  // [解析] 注意：构造函数内部把 kp、ki 都乘了 2 存进 two_kp_ / two_ki_，
  // [解析]   这是 Mahony 公式里本来就有的那个 2 倍系数提前合并了，不是作者写错。
  Mahony(float dt, float kp = 0.5f, float ki = 0.0f);

  // 新增：用于动态修改 kp 和 ki 的接口
  //kp表示对重力的置信度,kp越高则越相信重力的方向,并将其作为pitch和roll的方向
  //kp越低则越相信角速度积分
  // [解析] set_kp()：运行中动态改 kp，改完立刻影响下一次 update()。
  // [解析]   本项目的典型用途是"打弹抗振动"：开火前 set_kp(0.01f)（几乎只信陀螺仪），
  // [解析]   打完再 set_kp(0.5f) 恢复，避免枪口振动把加速度计读数带偏导致低头。
  // [解析] set_ki()：运行中动态改 ki，一般保持 0 即可。
  void set_kp(float kp);
  void set_ki(float ki);

  // [解析] ↓↓↓ 从这里到 vpitch_geom 全是"只读输出"：算法每个周期写，业务代码只读。↓↓↓
  // [解析] q_last：本次 update() 之前的那个四元数（上一次的姿态），用来比较"这一周期转了多少"。
  // [解析] 注意：init() 阶段不会写它，所以上电后第一次完整 update() 之前它是未初始化的垃圾值。
  float q_last[4];  // 只读! 上次更新后的四元数顺序为wxyz

  // [解析] q：核心输出，单位四元数，顺序 {w, x, y, z}（第一个元素是标量 w）。
  // [解析]   含义：载体系(车体) → 地面系(世界系)的旋转（被动/换系用法）。
  // [解析]   用法：v_world = q ⊗ v_body ⊗ q*，就是把车体系下的向量换成世界系下的向量。
  // [解析]   恒等式：w² + x² + y² + z² = 1（每次做完积分都会重新归一化来保证）。
  float q[4];  // 只读! 四元数顺序为wxyz 该四元数的含义是表示某物体相对于地面的姿态
               //具体的作用是:将同一个向量从某一个系的坐标值转换到地面系
               //即 v_ground = q * v_body * q_conjugate

  // [解析] dq：文档上写的是"q_last* ⊗ q"，即本周期的姿态增量所对应的四元数。
  // [解析] 注意：全工程搜不到任何给 Mahony::dq 赋值的地方（gimbal 类里有另外一个同名 dq），
  // [解析]   也就是说本对象的 dq 一直是未初始化的垃圾值，读它没有意义，先别用。
  float dq[4];  // 只读! 四元数增量 dq = q_last* ⊗ q，表示载体系下的角速度等效四元数
  // [解析] w：陀螺仪原始角速度（未做任何融合/修正），单位 rad/s，顺序 {x, y, z}。
  // [解析]   含义：物体相对于地面的角速度，在"载体系"下的三轴分量。
  float w[3];   // 只读! 单位: rad/s 物体相对于地面的角速度 在载体系下的表示

  // [解析] yaw/pitch/roll：由四元数换算出的欧拉角，单位 rad，采用 ZYX 内旋顺序
  // [解析]   （即先绕 Z 转 yaw，再绕 Y 转 pitch，最后绕 X 转 roll）。
  // [解析]   物理含义：yaw = 车头朝向（绕竖直轴，向左为正）、pitch = 抬头/低头、
  // [解析]   roll = 左右侧倾。注意 pitch 用 asin 算出，值域只有 [-π/2, π/2]，
  // [解析]   所以车体完全倒扣时 pitch 会翻转（倒扣场景请看下面的 pitch_geom）。
  float yaw;     // 只读! 单位: rad
  float pitch;   // 只读! 单位: rad
  float roll;    // 只读! 单位: rad
  // [解析] vyaw/vpitch/vroll：欧拉角的角速度，单位 rad/s。
  // [解析]   注意它"不是"对欧拉角做差分得到的，而是把陀螺仪角速度按欧拉角运动学公式
  // [解析]   反解出来的（见 mahony.cpp 的 culculate_yaw_pitch_roll_rates），
  // [解析]   好处是不会有差分带来的噪声放大。云台速度环直接拿它当反馈就行。
  // [解析]   坑：公式里有 1/cos(pitch)，pitch 接近 ±90°（万向锁）时 vyaw/vroll 会炸。
  float vyaw;    // 只读! 单位: rad/s
  float vpitch;  // 只读! 单位: rad/s
  float vroll;   // 只读! 单位: rad/s

  // [解析] acc_euler：注释说是"欧拉角的角加速度"，单位 m/s²（这个单位其实很可疑，
  // [解析]   角加速度应该是 rad/s²，多半是复制粘贴留下的笔误）。
  // [解析] 注意：全工程搜不到给它赋值的地方，所以它一直是未初始化的垃圾值，别用。
  float acc_euler[3];  // 只读! 单位: m/s² 由微分欧拉角角速度计算得到的欧拉角的角加速度
  // [解析] pitch_geom：把 pitch 的值域从 [-π/2, π/2] 扩展到 [-π, π] 的"几何俯仰角"，
  // [解析]   做法是看车体 x 轴、z 轴在世界系下的投影落在哪个象限，从而判断是否倒扣。
  // [解析]   用途：串腿/翻车后自启时，pitch 用扩展版本才不会在倒扣瞬间跳变。
  // [解析]   在 ±π 交界处仍会有一次跳变（作者注释也承认了），这是坐标系本身的限制。
  float pitch_geom;    // 只读! 单位: rad 范围[-pi, pi]
  // [解析] pitch_geom_last：上一次的 pitch_geom，只为了做差分算 vpitch_geom。
  // [解析]   注意：它没有初值，第一次调用 pitch_geom_calc() 时会读到一个未初始化的值，
  // [解析]   导致那一拍的 vpitch_geom 出现一个巨大尖峰（下面 vpitch_geom 处会再提醒）。
  float pitch_geom_last;
  // [解析] vpitch_geom：扩展 pitch 的角速度，单位 rad/s。用 (本次-上次)/dt 差分得到，
  // [解析]   因为是差分，噪声比 vpitch 大，一般只用来判断"是否在倒扣翻转"。
  float vpitch_geom;  // 只读! 单位: rad/s

  // [解析] 数组版 update()：把 acc、gyro 两个长度 3 的数组直接传进来，内部转调下面那版。
  // [解析]   实际项目里用的就是它：imu.update(bmi088.acc, bmi088.gyro)。
  // [解析]   acc[3] = {ax, ay, az}，单位 m/s²（含重力）；gyro[3] = {wx, wy, wz}，单位 rad/s。
   void update(const float acc[3], const float gyro[3]);
  // [解析] 单变量版 update()：真正的算法实现，所有输入都是 float，避免数组退化。
  // [解析] 返回值：无（void），结果写进 q / yaw / pitch / roll / vyaw ... 等公共成员。
  // [解析] 副作用：会修改对象内部状态（四元数、积分项、滤波状态），必须在同一个任务里调用。
  // [解析] 第一次调用比较特殊：只会用加速度计确定初始姿态（init）后就 return，不积分。
  void update(
    float ax, float ay, float az, float wx, float wy,
    float wz);  //用于更新物体相对于地面的姿态 四元数和欧拉角以及其微分

  // [解析] culculate_yaw_pitch_roll_rates()：由"载体系角速度 + 欧拉角"反解欧拉角速率。
  // [解析] 参数：wx/wy/wz 是载体系下角速度(rad/s)，roll/pitch/yaw 是当前欧拉角(rad)。
  // [解析] 注意：必须带入"相对于同一个参考系"的欧拉角值，否则算出来的速率是错的
  // [解析]   （函数名里的拼写 culculate 是项目原有写法，保留不动）。
  void culculate_yaw_pitch_roll_rates(
    float wx, float wy, float wz, float roll, float pitch, float yaw);
  //用于计算物体相对于某一个系的欧拉角微分,注意要带入相对于该系的欧拉角值
private:
  // [解析] dt_：采样周期，单位 s，构造时传入，之后不可改（const）。
  const float dt_;
  // [解析] two_kp_ / two_ki_：等于 2×kp、2×ki。公式里到处是 2 倍系数，提前算好省乘法。
  float two_kp_;
  float two_ki_;

  // [解析] inited_：是否已经用加速度计做过初始对准。false 表示"上电后还没 init"，
  // [解析]   此时 update() 会先跑 init() 固定初始姿态，然后直接返回，不做积分。
  bool inited_;

  // [解析] integral_fbx_/fby_/fbz_：ki 项的积分累加器（载体系 x/y/z），用来消陀螺仪零偏。
  // [解析]   ki = 0 时每周期都会被清零（防积分饱和）。
  float integral_fbx_;
  float integral_fby_;
  float integral_fbz_;
  // [解析] base_x/base_z：载体系的 x 轴、z 轴单位向量（就是 (1,0,0) 和 (0,0,1)），
  // [解析]   作为"原料"被四元数旋转到世界系，用来判断车体是否倒扣。
  float base_x[3] = {1.0f, 0.0f, 0.0f};  //base系的x轴
  float base_z[3] = {0.0f, 0.0f, 1.0f};  //base系的z轴
  // [解析] world_x/world_z：上面两个轴被旋转到世界系后的结果，pitch_geom_calc() 内部使用。
  float world_x[3];                      //world系的x轴
  float world_z[3];                      //world系的z轴
  // [解析] quaternion_frame_transform_for_mahony()：用四元数把一个向量换系，
  // [解析]   等价于 v_out = q ⊗ v_in ⊗ q*；conjugate_q = true 时用共轭，相当于反向旋转。
  // [解析]   这是 private 辅助函数，外面用不到（gimbal 类里有另一份功能类似的实现）。
  void quaternion_frame_transform_for_mahony(
    const float q[4], const float v_in[3], float v_out[3], bool conjugate_q);

  // [解析] pitch_geom_calc()：上面说的"把 pitch 值域扩展到 ±π"的实现，每周期调一次。
  void pitch_geom_calc();
  // [解析] init()：上电首次调用时，只用加速度计（重力方向）确定初始 roll/pitch，
  // [解析]   yaw 固定为 0（因为没有磁力计，绝对朝向无从得知）。
  void init(float ax, float ay, float az);
};

}  // namespace sp

#endif  // SP__MAHONY_HPP