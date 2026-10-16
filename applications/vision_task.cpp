// [解析] ==========================================================================
// [解析] 【文件】applications/vision_task.cpp
// [解析] 【作用】视觉通信任务：把本车的姿态、弹速、超电状态发给视觉/自瞄上位机，
// [解析]         同时接收上位机算出来的"目标角度"，好让云台去瞄准。
// [解析] 【通信方式】USB CDC（虚拟串口）。上位机就是插在 C 板 USB 口上的那台小电脑。
// [解析] 【当前状态】★ 本文件目前在 CMakeLists.txt 里是被注释掉的（第 69-72 行），
// [解析]           也就是说它暂时不参与编译。要真正启用它，必须补齐三件事：
// [解析]             1) 在 CubeMX 里启用 USB_OTG_FS（Device Only）+ CDC 中间件，
// [解析]                并把 USB Device 库加入 CMakeLists；
// [解析]                否则 sp_middleware/io/vision/vision.cpp 里的
// [解析]                #include "usbd_cdc_if.h" 找不到文件。
// [解析]             2) 把 sp_middleware/motor/super_cap/super_cap.cpp 也加进 CMakeLists，
// [解析]                因为下面用到了超电对象。
// [解析]             3) 在 Core/Src/freertos.c 里增加一个 VISION 任务来调用 vision_task()，
// [解析]                目前 6 个任务里并没有它。
// [解析]           详细的排查过程见 docs/C板引脚配置说明.md 第 5 节。
// [解析] ==========================================================================

#include "cmsis_os.h"
#include "io/vision/vision.hpp"
#include "tools/mahony/mahony.hpp"
// [解析] 下面这行是本次新加的：因为要访问超级电容对象，必须先让编译器认识 sp::SuperCap。
#include "motor/super_cap/super_cap.hpp"

// [解析] extern 表示"这个 imu 对象定义在别处，这里只是借用一下"。
// [解析]   它的真正定义在 applications/imu_task.cpp 里：sp::Mahony imu(1e-3f);
// [解析]   用 extern 的好处是：全工程共用同一个姿态解算结果，不会出现两套互相矛盾的姿态。
extern sp::Mahony imu;

// [解析] ★ 这一行原来是 sp::Vision vis(false, false);，本次改成了 sp::Vision vis;
// [解析]   原因：当前的 io/vision/vision.hpp 里 Vision 类只有"无参数构造函数"，
// [解析]         并没有接收两个 bool 的构造函数，所以原来那种写法编不过去
// [解析]         （编译器报错：no matching function for call to 'sp::Vision::Vision(bool, bool)'）。
// [解析]         如果以后升级中间件、Vision 新增了带参构造，可以把参数再补回来。
sp::Vision vis;

// [解析] ★ 下面这行也是本次新加的。原来的代码直接使用了全局对象 supercap，
// [解析]   但整个工程里并没有定义过它，所以编译器报 'supercap' was not declared in this scope。
// [解析]   这里补一个定义，让文件能编译通过。
// [解析]   注意：这只是个"占位对象"，它没有被任何任务更新（没人调用它的 read()），
// [解析]         所以发给上位机的超电数据会一直是初始值 0。
// [解析]         等超电真正接入后，应该把它挪到超电任务里统一创建和刷新。
sp::SuperCap supercap;

extern "C" void vision_task()
{
  while (true) {
    // 推荐实际使用时放在imu_task

    // [解析] 作者的意思是：发送动作其实依赖 imu 的数据，所以更合理的做法是
    // [解析] 把下面这段 send() 合并进 imu_task，省得多开一个任务、也少一次上下文切换。
    // [解析] 现在单独放一个任务只是为了调试方便。
    vis.send(
      // [解析] 参数逐个对照 io/vision/vision.hpp 里的 send() 声明来看：
      // [解析]   0            -> mode：工作模式。0 = 空闲，1 = 自瞄，2 = 小符，3 = 大符
      // [解析]   imu.q        -> 四元数（wxyz 顺序），描述当前姿态
      // [解析]   imu.yaw      -> 当前 yaw 角（单位 rad）
      // [解析]   imu.vyaw     -> yaw 角速度
      // [解析]   imu.pitch    -> 当前 pitch 角
      // [解析]   imu.vpitch   -> pitch 角速度
      // [解析]   24.0f        -> bullet_speed：弹丸初速度（m/s），弹道解算要用，
      // [解析]                   换弹种/改摩擦轮转速后必须重新标定这个值，否则打不准
      // [解析]   0            -> bullet_count：已发弹数，这里固定填 0
      // [解析]   后面 5 个      -> 超电状态（输入功率/输出功率/电压/温度/状态标志）
      0, imu.q, imu.yaw, imu.vyaw, imu.pitch, imu.vpitch, 24.0f, 0, supercap.power_in,
      supercap.power_out, supercap.voltage, supercap.temperature, supercap.status);

    // 使用调试(f5)查看vis内部变量的变化
    // [解析] 调试技巧：按 F5 进调试，把 vis 加到 watch 窗口，
    // [解析]   重点看 vis.control / vis.fire / vis.auto_aim_target 有没有被上位机刷新，
    // [解析]   有变化就说明上位机的数据成功收到了。

    osDelay(1);  // 1000Hz
    // [解析] 每 1ms 发一帧给上位机（1000Hz）。视觉通信数据量不大，这个速率足够。
  }
}

// [解析] 上位机发来数据时的回调函数。它由 USB 驱动调用（不需要你手动调）。
extern "C" void vision_callback(uint8_t * buf, uint32_t len) { vis.update(buf, len); }
// [解析] update() 内部会校验帧头 'S''P' 和 CRC16，校验通过才解析出
// [解析]   control（是否接管控制）、fire（是否开火）、yaw/pitch（目标角度）等。

// [解析] ==========================================================================
// [解析] ↓↓↓ 以下内容是原作者粘贴进来的"USB 接入说明"，原本是 Markdown 文本
// [解析]   （含 ``` 代码块），直接放在 .cpp 里会导致编译错误：
// [解析]   error: stray '`' in program。
// [解析]   本次已把每一行都改成 // 注释，保留说明、同时让文件变成合法 C++。
// [解析]   注意：必须逐行用 // 注释，不能用 /* */ 包起来，
// [解析]   因为这些内容里本身就含有 /* USER CODE BEGIN 6 */，会提前把块注释结束掉。
// [解析] --------------------------------------------------------------------------
// [解析] 【启用视觉通信还要改的地方】打开 CubeMX 生成的 USB 中间件文件
// [解析]   Src/usbd_cdc_if.c，修改其中的 CDC_Receive_FS 函数，加上一行 vision_callback：
// [解析] --------------------------------------------------------------------------
// [解析] static int8_t CDC_Receive_FS(uint8_t * Buf, uint32_t * Len)
// [解析] {
// [解析]   /* USER CODE BEGIN 6 */
// [解析]   vision_callback(Buf, *Len);  // <- 添加这一行
// [解析]   USBD_CDC_SetRxBuffer(&hUsbDeviceFS, &Buf[0]);
// [解析]   USBD_CDC_ReceivePacket(&hUsbDeviceFS);
// [解析]   return (USBD_OK);
// [解析]   /* USER CODE END 6 */
// [解析] }
// [解析] ==========================================================================
