// [解析] UI 图形元素定义：直线/矩形/圆/椭圆/圆弧/浮点/整数/字符串，只负责描述、不负责发送。
// [解析] 每个类内部持有一个 referee::InteractionFigure（15 字节），set_xxx() 就是往它的位域写值；
// [解析] 真正打包发送在 manager.hpp。上层先建对象（建议全局），再交给 sp::UI_Manager::pack()。
// [解析] 坐标 1920x1080、x 向右 y 向下；位域超范围会被静默截断；改图形必须复用同一对象。

#ifndef SP__UI_ELEMENT_HPP
#define SP__UI_ELEMENT_HPP

#include <string>

#include "referee/referee_protocol/referee_protocol.hpp"

namespace sp::ui
{
// [解析] 图形操作类型（3 bit）：首次画用 ADD；之后刷新数值必须先设 MODIFY，否则选手端会当成新图形重复添加。
enum class OperateType : uint8_t
{
  EMPTY = 0,
  ADD = 1,
  MODIFY = 2,
  DELETE = 3
};

// [解析] 图形类型（3 bit）：决定 details_a~details_e 怎么解释，取值须与裁判系统一致，顺序不能调换。
enum class FigureType : uint8_t
{
  LINE = 0,
  RECTANGLE = 1,
  CIRCLE = 2,
  ELLIPSE = 3,
  ARC = 4,
  FLOAT = 5,
  INTEGER = 6,
  STRING = 7
};

// [解析] 图层 0~9（4 bit）：静态背景和每帧变化的数值分层放，就能只删一层而不必整屏重画。
enum class Layer : uint8_t
{
  LAYER_0 = 0,
  LAYER_1 = 1,
  LAYER_2 = 2,
  LAYER_3 = 3,
  LAYER_4 = 4,
  LAYER_5 = 5,
  LAYER_6 = 6,
  LAYER_7 = 7,
  LAYER_8 = 8,
  LAYER_9 = 9
};

// [解析] 颜色（4 bit，共 9 种）。0 号 RED_BLUE 是"己方颜色"：红方画红、蓝方画蓝，一套代码两队通用。
enum class Color : uint8_t
{
  RED_BLUE = 0,
  YELLOW = 1,
  GREEN = 2,
  ORANGE = 3,
  PURPLE = 4,
  PINK = 5,
  CYAN = 6,
  BLACK = 7,
  WHITE = 8
};

// [解析] 所有图形的基类：持有协议结构体 data，各 set_xxx() 往它的位域写值。
// [解析] 构造函数不清零 data，所以元素要定义成全局/static；放栈上时未赋值的位域是随机值，会画出乱图。
struct Element
{
  referee::InteractionFigure data;
  Element(FigureType type, Layer layer, Color color, uint16_t width, uint16_t x, uint16_t y);
  void set_operate_type(OperateType type);
  void set_figure_type(FigureType type);
  void set_layer(Layer layer);
  void set_color(Color color);
  void set_width(uint16_t width);
  void set_x(uint16_t x);
  void set_y(uint16_t y);
};

// [解析] 普通图形的共同基类（String 除外）：自身不加成员，只是让 pack() 用一组基类指针统一接收。
struct ComposableElement : public Element
{
  using Element::Element;
};

// [解析] 直线：起点存 start_x/start_y，终点存第二组 details_d/details_e（协议里第二组坐标是多图形共用的）。
struct Line : public ComposableElement
{
  Line(
    Layer layer, Color color, uint16_t width, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);
  void set_x2(uint16_t x2);
  void set_y2(uint16_t y2);
};

// [解析] 矩形：(x1,y1) 与 (x2,y2) 是两个对角点，只画边框；终点同样放 details_d/details_e。
struct Rectangle : public ComposableElement
{
  Rectangle(
    Layer layer, Color color, uint16_t width, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);
  void set_x2(uint16_t x2);
  void set_y2(uint16_t y2);
};

// [解析] 正圆：圆心 + 半径。半径只占 details_c 的 10 bit（最大 1023），更大的圆请改用椭圆或圆弧。
struct Circle : public ComposableElement
{
  Circle(Layer layer, Color color, uint16_t width, uint16_t x, uint16_t y, uint16_t radius);
  void set_radius(uint16_t radius);
};

// [解析] 椭圆：圆心 + x/y 两个半轴（是半径不是直径），分别占 details_d/details_e 的 11 bit。
struct Ellipse : public ComposableElement
{
  Ellipse(
    Layer layer, Color color, uint16_t width, uint16_t x, uint16_t y, uint16_t x_radius,
    uint16_t y_radius);
  void set_x_radius(uint16_t x_radius);
  void set_y_radius(uint16_t y_radius);
};

// [解析] 圆弧：椭圆上截下来的一段，角度单位为度（0~360，占 9 bit）。
// [解析] details_a=起始角、details_b=终止角、details_d/details_e=x/y 半轴，details_c 不用；含义全由 figure_type 决定。
struct Arc : public ComposableElement
{
  Arc(
    Layer layer, Color color, uint16_t width, uint16_t x, uint16_t y, uint16_t start_angle,
    uint16_t end_angle, uint16_t x_radius, uint16_t y_radius);
  void set_start_angle(uint16_t start_angle);
  void set_end_angle(uint16_t end_angle);
  void set_x_radius(uint16_t x_radius);
  void set_y_radius(uint16_t y_radius);
};

// [解析] 浮点数：协议数值字段只存整数，所以 set_value() 内部先 ×1000，显示精度固定 3 位小数。
struct Float : public ComposableElement
{
  Float(
    Layer layer, Color color, uint16_t width, uint16_t x, uint16_t y, uint16_t font_size,
    float value);
  void set_font_size(uint16_t font_size);
  void set_value(float value);
};

struct Integer : public ComposableElement
{
  Integer(
    Layer layer, Color color, uint16_t width, uint16_t x, uint16_t y, uint16_t font_size,
    int value);
  void set_font_size(uint16_t font_size);
  void set_value(int value);
};

// [解析] 字符图形：数据长度 15(图形)+30(文本) 字节，比普通图形长，所以不继承 ComposableElement。
// [解析] 后果是 UI_Manager 单独给它一个 pack() 重载，且协议规定一帧只能带一个字符图形。
struct String : public Element
{
  std::string str;
  String(
    Layer layer, Color color, uint16_t width, uint16_t x, uint16_t y, uint16_t font_size,
    const std::string & value);
  void set_font_size(uint16_t font_size);
  void set_value(const std::string & value);
};

}  // namespace sp::ui

#endif  // SP__UI_ELEMENT_HPP