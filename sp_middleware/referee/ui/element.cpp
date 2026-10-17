// [解析] element.hpp 的实现，本质都是把构造参数写进 referee::InteractionFigure 的位域。
// [解析] 图形对象由上层创建，之后交给 UI_Manager::pack() 只读打包；本文件不发送、不碰硬件。
// [解析] 值得细看只有四处：set_unique()、set_int32()、Float::set_value()、String::set_value()。

#include "element.hpp"

namespace sp::ui
{
// [解析] 给新图形分配唯一编号并写进 3 字节 figure_name：选手端靠它区分图形，MODIFY/DELETE 也得带同一个编号。
// [解析] 南航模拟器按大端序解析，所以高位字节在前；id 是 static 全程序自增，只增不减（清零会让新旧图形撞号）。
static void set_unique(referee::InteractionFigure * data)
{
  static uint32_t id = 0;
  // 南航模拟器是大端序:
  // https://github.com/bismarckkk/RM-UI-Designer/blob/eea60c463dc24055c735d837c89401e633a2a7fb/src/utils/serial/msgView.ts#L123
  data->figure_name[0] = id >> 16;
  data->figure_name[1] = id >> 8;
  data->figure_name[2] = id;
  id++;
}

// [解析] 把 32 位整数平铺进 details_c/details_d/details_e：三者宽度 10+11+11 bit，且在结构体里首尾相连。
// [解析] 于是按"低 10 位、接着 11 位、再 11 位"依次右移截断；details_b 未参与，负数靠算术右移补 1。
static void set_int32(referee::InteractionFigure * data, int32_t value)
{
  data->details_c = value;
  data->details_d = value >> 10;
  data->details_e = value >> 21;
}

// [解析] 只负责所有图形共有的字段：先领编号，操作类型默认 ADD，再写类型/图层/颜色/线宽/坐标。
Element::Element(FigureType type, Layer layer, Color color, uint16_t width, uint16_t x, uint16_t y)
{
  set_unique(&data);
  set_operate_type(OperateType::ADD);
  set_figure_type(type);
  set_layer(layer);
  set_color(color);
  set_width(width);
  set_x(x);
  set_y(y);
}

// [解析] 7 个单字段 setter：目标位域比形参窄（layer 4 bit、operate_type 3 bit …），高位会被静默丢弃。
void Element::set_operate_type(OperateType type) { data.operate_type = static_cast<uint8_t>(type); }
void Element::set_figure_type(FigureType type) { data.figure_type = static_cast<uint8_t>(type); }
void Element::set_layer(Layer layer) { data.layer = static_cast<uint8_t>(layer); }
void Element::set_color(Color color) { data.color = static_cast<uint8_t>(color); }
void Element::set_width(uint16_t width) { data.width = width; }
void Element::set_x(uint16_t x) { data.start_x = x; }
void Element::set_y(uint16_t y) { data.start_y = y; }

Line::Line(
  Layer layer, Color color, uint16_t width, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
: ComposableElement(FigureType::LINE, layer, color, width, x1, y1)
{
  set_x2(x2);
  set_y2(y2);
}

// [解析] 终点放在协议第二组坐标 details_d/details_e（矩形、椭圆、圆弧也复用这对位域）。
void Line::set_x2(uint16_t x2) { data.details_d = x2; }
void Line::set_y2(uint16_t y2) { data.details_e = y2; }

Rectangle::Rectangle(
  Layer layer, Color color, uint16_t width, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
: ComposableElement(FigureType::RECTANGLE, layer, color, width, x1, y1)
{
  set_x2(x2);
  set_y2(y2);
}

void Rectangle::set_x2(uint16_t x2) { data.details_d = x2; }
void Rectangle::set_y2(uint16_t y2) { data.details_e = y2; }

Circle::Circle(Layer layer, Color color, uint16_t width, uint16_t x, uint16_t y, uint16_t radius)
: ComposableElement(FigureType::CIRCLE, layer, color, width, x, y)
{
  set_radius(radius);
}

// [解析] 半径存 details_c，只有 10 bit（最大 1023），超出会被截断；更大的圆请改用椭圆或圆弧。
void Circle::set_radius(uint16_t radius) { data.details_c = radius; }

Ellipse::Ellipse(
  Layer layer, Color color, uint16_t width, uint16_t x, uint16_t y, uint16_t x_radius,
  uint16_t y_radius)
: ComposableElement(FigureType::ELLIPSE, layer, color, width, x, y)
{
  set_x_radius(x_radius);
  set_y_radius(y_radius);
}

// [解析] 两个半轴是水平/竖直方向的半径（不是直径），各占 11 bit。
void Ellipse::set_x_radius(uint16_t x_radius) { data.details_d = x_radius; }
void Ellipse::set_y_radius(uint16_t y_radius) { data.details_e = y_radius; }

Arc::Arc(
  Layer layer, Color color, uint16_t width, uint16_t x, uint16_t y, uint16_t start_angle,
  uint16_t end_angle, uint16_t x_radius, uint16_t y_radius)
: ComposableElement(FigureType::ARC, layer, color, width, x, y)
{
  set_start_angle(start_angle);
  set_end_angle(end_angle);
  set_x_radius(x_radius);
  set_y_radius(y_radius);
}

// [解析] 起止角度占 details_a/details_b（各 9 bit，单位度），x/y 半轴占 details_d/details_e。
void Arc::set_start_angle(uint16_t start_angle) { data.details_a = start_angle; }
void Arc::set_end_angle(uint16_t end_angle) { data.details_b = end_angle; }
void Arc::set_x_radius(uint16_t x_radius) { data.details_d = x_radius; }
void Arc::set_y_radius(uint16_t y_radius) { data.details_e = y_radius; }

Float::Float(
  Layer layer, Color color, uint16_t width, uint16_t x, uint16_t y, uint16_t font_size, float value)
: ComposableElement(FigureType::FLOAT, layer, color, width, x, y)
{
  set_font_size(font_size);
  set_value(value);
}

void Float::set_font_size(uint16_t font_size) { data.details_a = font_size; }

// [解析] 协议数值字段只存整数，所以先 ×1000 放大成整数，选手端再除回 1000，精度固定 3 位小数。
// [解析] 这里是截断取整而非四舍五入，且结果要装进 int32，value 有效范围约 ±2147483.647。
void Float::set_value(float value) { set_int32(&data, value * 1e3f); }

Integer::Integer(
  Layer layer, Color color, uint16_t width, uint16_t x, uint16_t y, uint16_t font_size, int value)
: ComposableElement(FigureType::INTEGER, layer, color, width, x, y)
{
  set_font_size(font_size);
  set_value(value);
}

void Integer::set_font_size(uint16_t font_size) { data.details_a = font_size; }

// [解析] 整数不做缩放，直接拆成三段塞进协议；int 在 STM32 上是 4 字节，与 int32_t 等宽。
void Integer::set_value(int value) { set_int32(&data, value); }

String::String(
  Layer layer, Color color, uint16_t width, uint16_t x, uint16_t y, uint16_t font_size,
  const std::string & value)
: Element(FigureType::STRING, layer, color, width, x, y)
{
  set_font_size(font_size);
  set_value(value);
}

void String::set_font_size(uint16_t font_size) { data.details_a = font_size; }

// [解析] 文本拷进成员 str，同时把长度写进 details_b（协议里字符图形用它表示文本字节数，9 bit）。
// [解析] 长度取"实际字节数"与 30 的较小值防越界；注意是字节不是字符，一个汉字 3 字节（最多 10 个）。
void String::set_value(const std::string & value)
{
  str = value;
  data.details_b = std::min(str.size(), sizeof(referee::ExtClientCustomCharacter::data));
}

}  // namespace sp::ui
