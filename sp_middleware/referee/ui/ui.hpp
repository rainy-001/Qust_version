// [解析] UI 模块总入口头文件：本身不含代码，只把下面两个分文件打包进来。
// [解析] element.hpp 是"要画什么"，manager.hpp 是"怎么发出去"，两者缺一不可。
// [解析] 上层一行 #include "referee/ui/ui.hpp" 即可拿到完整的 UI 能力。

#ifndef SP__UI_HPP
#define SP__UI_HPP

#include "element.hpp"
#include "manager.hpp"

#endif  // SP__UI_HPP