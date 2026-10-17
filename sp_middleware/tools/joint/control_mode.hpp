// [解析] 关节控制模式枚举：joint_controller 的 mode_ 与各 cmd_xxx()/control() 分支按它分派；enum class 不能隐式转 int。

#ifndef CONTROL_MODE_HPP
#define CONTROL_MODE_HPP

// [解析] DISABLE 不出力、VELOCITY 只跑速度环、POSITION 位置外环+速度内环串级、TORQUE 直给力矩；整数按书写顺序 0~3，别改顺序。
enum class ControlMode
{
  DISABLE,
  VELOCITY,
  POSITION,
  TORQUE  // 力控
};

#endif  // CONTROL_MODE_HPP