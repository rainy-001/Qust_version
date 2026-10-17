// [解析] Mahony 姿态解算（AHRS）：加速度计与陀螺仪融合，输出四元数 q 和欧拉角。
// [解析] applications/imu_task.cpp 建全局对象，1000Hz 每 1ms 调一次 update(acc, gyro)；
// [解析]   结果被 vision_task 打包发给视觉，也被 gimbal 用来做云台与底盘的坐标换算。
// [解析] 输入必须是载体系下的值：acc 含重力(m/s²)，gyro 为 rad/s。

#ifndef SP__MAHONY_HPP
#define SP__MAHONY_HPP

namespace sp
{
// [解析] 一个对象 = 一个 IMU 解算器，内部有记忆（四元数、积分项），只能在一个任务里调用。
// [解析] q 的顺序是 {w, x, y, z}（不是数学书的 {x,y,z,w}），含义为载体系 → 世界系的旋转。
class Mahony
{
public:
  // [解析] dt 必须等于真实调用周期；kp/ki 内部先乘 2 存成 two_kp_/two_ki_（公式自带的系数）。
  Mahony(float dt, float kp = 0.5f, float ki = 0.0f);

  // 新增：用于动态修改 kp 和 ki 的接口
  //kp表示对重力的置信度,kp越高则越相信重力的方向,并将其作为pitch和roll的方向
  //kp越低则越相信角速度积分
  void set_kp(float kp);
  void set_ki(float ki);

  float q_last[4];  // 只读! 上次更新后的四元数顺序为wxyz

  float q[4];  // 只读! 四元数顺序为wxyz 该四元数的含义是表示某物体相对于地面的姿态
               //具体的作用是:将同一个向量从某一个系的坐标值转换到地面系
               //即 v_ground = q * v_body * q_conjugate

  // [解析] 本对象的 dq 和下面的 acc_euler 全工程都没人赋值，读到的始终是未初始化值，别用。
  float dq[4];  // 只读! 四元数增量 dq = q_last* ⊗ q，表示载体系下的角速度等效四元数
  float w[3];   // 只读! 单位: rad/s 物体相对于地面的角速度 在载体系下的表示

  // [解析] 欧拉角为 ZYX 内旋；vyaw/vpitch/vroll 由角速度反解（不是差分），含 1/cos(pitch)，接近万向锁会发散。
  float yaw;     // 只读! 单位: rad
  float pitch;   // 只读! 单位: rad
  float roll;    // 只读! 单位: rad
  float vyaw;    // 只读! 单位: rad/s
  float vpitch;  // 只读! 单位: rad/s
  float vroll;   // 只读! 单位: rad/s

  float acc_euler[3];  // 只读! 单位: m/s² 由微分欧拉角角速度计算得到的欧拉角的角加速度
  // [解析] pitch_geom 把 pitch 扩到 ±π：用 world_x/world_z 的竖直分量判断是否倒扣，供翻车自启用。
  float pitch_geom;    // 只读! 单位: rad 范围[-pi, pi]
  float pitch_geom_last;
  float vpitch_geom;  // 只读! 单位: rad/s

  // [解析] 每周期调一次：第一拍只用加速度计定初始姿态就 return，之后是"陀螺积分 + 重力纠偏"。
   void update(const float acc[3], const float gyro[3]);
  void update(
    float ax, float ay, float az, float wx, float wy,
    float wz);  //用于更新物体相对于地面的姿态 四元数和欧拉角以及其微分

  void culculate_yaw_pitch_roll_rates(
    float wx, float wy, float wz, float roll, float pitch, float yaw);
  //用于计算物体相对于某一个系的欧拉角微分,注意要带入相对于该系的欧拉角值
private:
  const float dt_;
  float two_kp_;
  float two_ki_;

  bool inited_;

  float integral_fbx_;
  float integral_fby_;
  float integral_fbz_;
  float base_x[3] = {1.0f, 0.0f, 0.0f};  //base系的x轴
  float base_z[3] = {0.0f, 0.0f, 1.0f};  //base系的z轴
  float world_x[3];                      //world系的x轴
  float world_z[3];                      //world系的z轴
  void quaternion_frame_transform_for_mahony(
    const float q[4], const float v_in[3], float v_out[3], bool conjugate_q);

  void pitch_geom_calc();
  void init(float ax, float ay, float az);
};

}  // namespace sp

#endif  // SP__MAHONY_HPP