// [解析] ==========================================================================
// [解析] 【文件】sp_middleware/referee/ui/element.hpp
// [解析] 【作用】定义"要在选手端 UI 上画什么"的一组图形元素类：
// [解析]         直线、矩形、正圆、椭圆、圆弧、浮点数、整型数、字符串。
// [解析]         每个类内部都持有一个裁判系统协议结构体 referee::InteractionFigure
// [解析]         （15 字节的图形描述），构造或调用 set_xxx() 时把参数写进它的位域。
// [解析]         注意：本文件只负责"描述图形"，不负责发送，发送在 manager.hpp 里。
// [解析] 【被谁用】applications/uart_task.cpp（参考同目录 readme.md）：先定义全局对象，例如
// [解析]           sp::ui::Line line(sp::ui::Layer::LAYER_0, sp::ui::Color::RED_BLUE,
// [解析]                                  1, 500, 500, 600, 600);
// [解析]           再交给 sp::UI_Manager 的 pack() 打包发送。
// [解析] 【关键点】1) 坐标系：UI 分辨率 1920x1080，x 向右、y 向下，左上角是 (0,0)；
// [解析]              x 只有 11 bit（0~2047，实用 0~1919），y 也是 11 bit（实用 0~1079），
// [解析]              超范围不会报错，会被位域静默截断，画到莫名其妙的位置；
// [解析]           2) 颜色只有 9 种，其中 RED_BLUE 表示"己方颜色"（红方画红、蓝方画蓝）；
// [解析]           3) 图形之间靠 3 字节 figure_name 区分，由 element.cpp 自动编号，
// [解析]              想"修改"某个图形时必须复用同一个对象，不能重新 new 一个。
// [解析] ==========================================================================

#ifndef SP__UI_ELEMENT_HPP
#define SP__UI_ELEMENT_HPP

// [解析] ↓ 只有 String（字符图形）需要 std::string 来保存字符串本体。
#include <string>

// [解析] ↓ 裁判系统协议定义：InteractionFigure（图形描述结构体）等都在这里，
// [解析]   本文件所有 set_xxx() 最终都是往它的位域里写值。
#include "referee/referee_protocol/referee_protocol.hpp"

namespace sp::ui
{
// [解析] 图形操作类型：告诉选手端"这一帧是要新增、修改还是删除图形"。
// [解析] 对应协议 InteractionFigure::operate_type（占 3 bit，合法值 0~7）。
// [解析] 使用时机：第一次画图用 ADD；之后每帧只想刷新数值时，必须先调
// [解析]   set_operate_type(OperateType::MODIFY)，否则选手端会当成新图形重复添加。
enum class OperateType : uint8_t
{
  EMPTY = 0,   // [解析] 空操作：选手端收到后什么也不做
  ADD = 1,     // [解析] 新增一个图形
  MODIFY = 2,  // [解析] 修改已有图形（靠 figure_name 找到要改的那一个）
  DELETE = 3   // [解析] 删除图形
};

// [解析] 图形类型：决定那 5 个 details_a ~ details_e 位域该怎么解释。
// [解析] 举例：同样是 details_d，在直线里表示"终点 x2"，在椭圆里表示"x 半轴"。
// [解析] 对应协议 InteractionFigure::figure_type（占 3 bit），数值必须和裁判系统一致，
// [解析] 所以这里从 0 开始严格按协议枚举，不能随便调换顺序。
enum class FigureType : uint8_t
{
  LINE = 0,       // [解析] 直线：给起点和终点
  RECTANGLE = 1,  // [解析] 矩形：给左上角和右下角
  CIRCLE = 2,     // [解析] 正圆：给圆心和半径
  ELLIPSE = 3,    // [解析] 椭圆：给圆心和 x/y 半轴
  ARC = 4,        // [解析] 圆弧：给圆心、起止角度、x/y 半轴
  FLOAT = 5,      // [解析] 浮点数：显示一个带小数的数值
  INTEGER = 6,    // [解析] 整型数：显示一个整数
  STRING = 7      // [解析] 字符：显示一段文本
};

// [解析] 图层：可以把图形按"层"分组，每层独立重绘/删除。
// [解析] 选手端一共支持 10 层（0~9），对应协议 InteractionFigure::layer（占 4 bit）。
// [解析] 实用技巧：把"固定的静态图形"放一层、把"每帧变化的数据"放另一层，
// [解析] 就能用 delete_layer() 只清掉变化的那一层，不必整屏重画。
enum class Layer : uint8_t
{
  LAYER_0 = 0,  // [解析] 第 0 层
  LAYER_1 = 1,  // [解析] 第 1 层
  LAYER_2 = 2,  // [解析] 第 2 层
  LAYER_3 = 3,  // [解析] 第 3 层
  LAYER_4 = 4,  // [解析] 第 4 层
  LAYER_5 = 5,  // [解析] 第 5 层
  LAYER_6 = 6,  // [解析] 第 6 层
  LAYER_7 = 7,  // [解析] 第 7 层
  LAYER_8 = 8,  // [解析] 第 8 层
  LAYER_9 = 9   // [解析] 第 9 层
};

// [解析] 颜色：对应协议 InteractionFigure::color（占 4 bit，协议共定义 0~8 九种）。
// [解析] 为什么第一个叫 RED_BLUE 而不是 RED？因为裁判系统里 0 号颜色是"己方颜色"：
// [解析]   红方机器人画出来是红色，蓝方机器人画出来是蓝色，一套代码两队通用。
// [解析] 调试时建议优先用 RED_BLUE，这样不用根据己方阵营改代码。
enum class Color : uint8_t
{
  RED_BLUE = 0,  // [解析] 己方颜色：红方=红，蓝方=蓝
  YELLOW = 1,    // [解析] 黄色
  GREEN = 2,     // [解析] 绿色
  ORANGE = 3,    // [解析] 橙色
  PURPLE = 4,    // [解析] 紫红色
  PINK = 5,      // [解析] 粉色
  CYAN = 6,      // [解析] 青色
  BLACK = 7,     // [解析] 黑色（在深色小地图上可能看不清）
  WHITE = 8      // [解析] 白色
};

// [解析] Element 是所有图形元素的基类。它本身画不出任何东西，
// [解析] 只做两件事：1) 保存一份协议结构体 data（15 字节的图形描述）；
// [解析]           2) 提供一组 set_xxx()，把参数写进 data 的对应位域。
// [解析] 注意（容易踩坑）：构造函数里不会把 data 清零！所以元素对象最好定义成
// [解析]   全局变量或 static 变量（readme.md 的示例就是全局对象，全局变量在
// [解析]   程序启动时会自动清零），如果定义在函数栈上，那些没被 set 到的
// [解析]   details_* 位域就是随机数，画出来会乱。
struct Element
{
  // [解析] 真正要发给裁判系统的 15 字节图形数据；各派生类就是往它的位域里写值。
  referee::InteractionFigure data;
  // [解析] 参数含义（所有图形通用）：
  // [解析]   type  : 图形类型，派生类会自己填好（如 Line 传 FigureType::LINE）
  // [解析]   layer : 图层 0~9，同图层可被 delete_layer() 整体删除
  // [解析]   color : 颜色，RED_BLUE 表示"己方颜色"
  // [解析]   width : 线宽（字符图形里也表示笔画粗细），单位像素，10 bit，最大 1023
  // [解析]   x, y  : 起点 / 圆心坐标，单位像素，11 bit；
  // [解析]           UI 是 1920x1080，所以 x 实用范围 0~1919、y 实用范围 0~1079
  // [解析] 副作用：会自动给这个图形分配一个唯一编号（figure_name）作为索引，
  // [解析]   并把操作类型默认设成 ADD（即"新增"）。
  Element(FigureType type, Layer layer, Color color, uint16_t width, uint16_t x, uint16_t y);
  // [解析] 下面 7 个 set 函数各自只改一个字段，用于"改数值再重发"的动态刷新场景。
  void set_operate_type(OperateType type);
  void set_figure_type(FigureType type);
  void set_layer(Layer layer);
  void set_color(Color color);
  void set_width(uint16_t width);
  void set_x(uint16_t x);
  void set_y(uint16_t y);
};

// [解析] "可组合元素"：直线 / 矩形 / 正圆 / 椭圆 / 圆弧 / 浮点数 / 整型数 的共同基类。
// [解析] 它自己不加任何成员，唯一作用是让 UI_Manager::pack() 能用一组成员指针参数
// [解析] 统一接收这些图形（一次最多打包 7 个普通图形）。
// [解析] 为什么字符（String）不继承它？因为字符图形的长度和普通图形不同，
// [解析] 每帧只能单独发一个，见 element.hpp 末尾 String 和 manager.hpp 的说明。
struct ComposableElement : public Element
{
  // [解析] 继承基类的构造函数：写这一行等于把 Element 的构造参数表原样抄一遍，
  // [解析] 派生类就不用再写一遍构造函数了（C++11 的继承构造函数语法）。
  using Element::Element;
};

// [解析] 直线：从 (x1,y1) 画到 (x2,y2)。
// [解析] 存放位置：起点在协议第二行的 start_x / start_y，
// [解析]   终点放在第三行的 details_d / details_e（这就是为什么 set_x2 写的是 details_d）。
struct Line : public ComposableElement
{
  // [解析] 构造时先把类型固定成 LINE，再去填终点坐标；type 参数不用你自己传。
  Line(
    Layer layer, Color color, uint16_t width, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);
  void set_x2(uint16_t x2);  // [解析] 只改终点 x，配合 MODIFY 就能让线"动起来"
  void set_y2(uint16_t y2);  // [解析] 只改终点 y
};

// [解析] 矩形：由 (x1,y1) 和 (x2,y2) 两个对角点决定，内部是空的（只画边框）。
// [解析] 同样把第二个点放在 details_d / details_e 里。
struct Rectangle : public ComposableElement
{
  Rectangle(
    Layer layer, Color color, uint16_t width, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);
  void set_x2(uint16_t x2);  // [解析] 只改对角点 x
  void set_y2(uint16_t y2);  // [解析] 只改对角点 y
};

// [解析] 正圆：圆心 (x,y) + 半径 radius。
// [解析] 半径放在协议第三行的 details_c（只有 10 bit，最大 1023 像素）。
// [解析] 注意它画的是"正圆"，横竖半径一样；想要"扁圆"请用下面的 Ellipse。
struct Circle : public ComposableElement
{
  Circle(Layer layer, Color color, uint16_t width, uint16_t x, uint16_t y, uint16_t radius);
  void set_radius(uint16_t radius);  // [解析] 只改半径
};

// [解析] 椭圆：圆心 (x,y) + 两个半轴长度（不是直径！）。
// [解析] x_radius 放 details_d（11 bit），y_radius 放 details_e（11 bit）。
// [解析] 常见错误：把整个宽度当半径传进去，画出来会大一倍。
struct Ellipse : public ComposableElement
{
  Ellipse(
    Layer layer, Color color, uint16_t width, uint16_t x, uint16_t y, uint16_t x_radius,
    uint16_t y_radius);
  void set_x_radius(uint16_t x_radius);  // [解析] 只改 x 方向半轴
  void set_y_radius(uint16_t y_radius);  // [解析] 只改 y 方向半轴
};

// [解析] 圆弧：椭圆的一段。除了圆心和两个半轴，还要给起始角度和终止角度。
// [解析] 角度单位是"度"（0~360），0 度方向按裁判系统约定，画之前建议先用模拟器试一下。
// [解析] 存放位置比较绕，正好把 5 个 details 位域用满了：
// [解析]   details_a = 起始角度、details_b = 终止角度（各 9 bit）
// [解析]   details_d = x 半轴、details_e = y 半轴（各 11 bit）
// [解析]   details_c 没有用（保持 0）
struct Arc : public ComposableElement
{
  Arc(
    Layer layer, Color color, uint16_t width, uint16_t x, uint16_t y, uint16_t start_angle,
    uint16_t end_angle, uint16_t x_radius, uint16_t y_radius);
  void set_start_angle(uint16_t start_angle);  // [解析] 只改起始角度
  void set_end_angle(uint16_t end_angle);      // [解析] 只改终止角度
  void set_x_radius(uint16_t x_radius);        // [解析] 只改 x 半轴
  void set_y_radius(uint16_t y_radius);        // [解析] 只改 y 半轴
};

// [解析] 浮点数图形：在 UI 上显示一个带小数的数值（例如把当前速度显示在画面上）。
// [解析] 关键点：协议里数值是用"整数"传的，所以 element.cpp 会把 value 乘以 1000
// [解析]   再取整（见 Float::set_value），选手端收到后再除回 1000。
// [解析]   于是显示精度固定是 3 位小数，可表示范围约 ±2147483.647。
// [解析] font_size 是字号（像素高度），协议建议字号:线宽 = 10:1。
struct Float : public ComposableElement
{
  Float(
    Layer layer, Color color, uint16_t width, uint16_t x, uint16_t y, uint16_t font_size,
    float value);
  void set_font_size(uint16_t font_size);  // [解析] 只改字号
  void set_value(float value);             // [解析] 只改数值；内部会 ×1000 转成整数
};

// [解析] 整型数图形：在 UI 上显示一个整数（最常用，例如显示剩余血量、弹量）。
// [解析] 和 Float 的区别只是 set_value 不做 ×1000 缩放，直接按整数存。
// [解析] 取值范围就是 int 的范围，但位域只有 10+11+11 = 32 bit，所以超出会回绕。
struct Integer : public ComposableElement
{
  Integer(
    Layer layer, Color color, uint16_t width, uint16_t x, uint16_t y, uint16_t font_size,
    int value);
  void set_font_size(uint16_t font_size);  // [解析] 只改字号
  void set_value(int value);
  // [解析] ↑ 只改数值。注意参数是 int（4 字节），内部拆成 details_c/d/e 三段存进协议。
};

// [解析] 字符图形：在 UI 上显示一段文本，例如 "Hello, World!"。
// [解析] 为什么它不像其他图形那样继承 ComposableElement？
// [解析]   因为字符图形的数据长度 = 15（图形描述）+ 30（字符数据）字节，
// [解析]   比普通图形长，所以 UI_Manager 专门给它写了一个单独的 pack() 重载，
// [解析]   并且规定"每次只能发一个字符图形"。
struct String : public Element
{
  std::string str;  // [解析] 字符串本体；真正发送时只取前 30 字节（协议 data 字段长度）
  String(
    Layer layer, Color color, uint16_t width, uint16_t x, uint16_t y, uint16_t font_size,
    const std::string & value);
  void set_font_size(uint16_t font_size);            // [解析] 只改字号
  void set_value(const std::string & value);         // [解析] 换一段新文本
};

}  // namespace sp::ui

#endif  // SP__UI_ELEMENT_HPP