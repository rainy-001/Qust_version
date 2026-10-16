// [解析] ==========================================================================
// [解析] 【文件】sp_middleware/referee/ui/ui.hpp
// [解析] 【作用】UI 模块的总入口头文件。它自己不含任何代码，只负责把两个分文件
// [解析]         "打包"进来，让上层用一行 include 就拿到整个 UI 功能。
// [解析] 【被谁用】applications/uart_task.cpp（或者你自己新建的 ui 任务）：
// [解析]           #include "referee/ui/ui.hpp" 然后 sp::UI_Manager ui_manager; 即可。
// [解析]           参考同目录 readme.md 里的完整示例。
// [解析] 【关键点】1) element.hpp 是"要画什么"（直线/圆/文字…），
// [解析]           2) manager.hpp 是"怎么发出去"（打包成裁判系统 0x0301 帧并算 CRC）。
// [解析]           两者配合使用，缺一不可，所以这里一起包含。
// [解析] ==========================================================================

#ifndef SP__UI_HPP
#define SP__UI_HPP

// [解析] ↓ element.hpp 定义"要画什么"：Line / Rectangle / Circle / Ellipse / Arc /
// [解析]   Float / Integer / String 这些图形元素类。
#include "element.hpp"
// [解析] ↓ manager.hpp 定义"怎么发出去"：UI_Manager 负责把这些元素打包成
// [解析]   裁判系统 0x0301 机器人交互数据帧，并填好 CRC8 / CRC16 校验。
#include "manager.hpp"

#endif  // SP__UI_HPP