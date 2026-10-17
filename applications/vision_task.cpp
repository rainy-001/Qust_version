// [解析] 视觉通信任务：把姿态、弹速、超电状态发给视觉/自瞄上位机，并接收上位机算出的目标角度（走 USB CDC 虚拟串口）。
// [解析] 本文件目前在 CMakeLists.txt 里被注释掉、不参与编译；supercap 也只是没人刷新的占位对象，超电数据恒为 0。
// [解析] 启用需补 USB_OTG_FS+CDC 中间件、把 super_cap.cpp 加进编译、在 freertos.c 加 VISION 任务（见 docs/C板引脚配置说明.md 第 5 节）。

#include "cmsis_os.h"
#include "io/vision/vision.hpp"
#include "tools/mahony/mahony.hpp"
#include "motor/super_cap/super_cap.hpp"

extern sp::Mahony imu;

sp::Vision vis;

sp::SuperCap supercap;

extern "C" void vision_task()
{
  while (true) {
    // 推荐实际使用时放在imu_task

    // [解析] send() 的实参是一串位置参数：mode(0 空闲/1 自瞄/2 小符/3 大符)、四元数、yaw/vyaw/pitch/vpitch、弹速 24.0f(m/s)、已发弹数、5 个超电状态。
    vis.send(
      0, imu.q, imu.yaw, imu.vyaw, imu.pitch, imu.vpitch, 24.0f, 0, supercap.power_in,
      supercap.power_out, supercap.voltage, supercap.temperature, supercap.status);

    // 使用调试(f5)查看vis内部变量的变化

    osDelay(1);  // 1000Hz
  }
}

extern "C" void vision_callback(uint8_t * buf, uint32_t len) { vis.update(buf, len); }

// [解析] 启用时还需改 Src/usbd_cdc_if.c 的 CDC_Receive_FS：在 USER CODE BEGIN 6 里加一行 vision_callback(Buf, *Len);
