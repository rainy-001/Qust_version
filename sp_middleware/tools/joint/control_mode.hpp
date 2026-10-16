// [解析] ==========================================================================
// [解析] 【文件】sp_middleware/tools/joint/control_mode.hpp
// [解析] 【作用】只定义一个枚举：关节控制器此刻处于哪种控制模式。
// [解析] 【被谁用】tools/joint/joint_controller.hpp 里的成员 mode_（初值就是 DISABLE），
// [解析]         以及 joint_controller.cpp 里 cmd()/cmd_v()/cmd_t()/control() 的那些 if 分支。
// [解析] 【关键点】1) enum class 是"强类型枚举"，不能隐式转换成 int，
// [解析]   好处是不会和别的整数混着用（写错名字直接编译报错），
// [解析]   代价是想打印/存储时必须显式转换，例如 static_cast<int>(mode)；
// [解析]   2) 括号里的名字都带 ControlMode:: 前缀才能用，例如 ControlMode::TORQUE。
// [解析] ==========================================================================

#ifndef CONTROL_MODE_HPP
#define CONTROL_MODE_HPP

// [解析] 四种模式对应"这个关节用什么方式做闭环"，单位统一是：角度 rad、角速度 rad/s、力矩 N·m。
// [解析]   DISABLE  —— 失能：不控制，输出力矩恒为 0（上电初始状态、急停、松手用）。
// [解析]   VELOCITY —— 速度闭环：目标是一个转速，只跑内环速度 PID。
// [解析]   POSITION —— 位置闭环：目标是一个角度，外环位置 PID + 内环速度 PID（串级双环）。
// [解析]   TORQUE   —— 力矩（力控）闭环：直接把力矩指令发给电机，不做任何 PID。
// [解析] 注意：枚举的整数值是按书写顺序从 0 开始编的
// [解析]   （DISABLE=0、VELOCITY=1、POSITION=2、TORQUE=3）。
// [解析]   如果有代码把它转成整数存进日志/上位机，改动这里的顺序会让历史数据对不上号。
enum class ControlMode
{
  DISABLE,
  VELOCITY,
  POSITION,
  TORQUE  // 力控
};

#endif  // CONTROL_MODE_HPP