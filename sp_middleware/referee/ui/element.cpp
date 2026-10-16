// [解析] ==========================================================================
// [解析] 【文件】sp_middleware/referee/ui/element.cpp
// [解析] 【作用】element.hpp 里那些图形元素类（Line / Rectangle / Circle / ... / String）
// [解析]         的具体实现。所有代码本质上都在干同一件事：
// [解析]         把构造参数写进裁判系统协议结构体 referee::InteractionFigure 的位域里。
// [解析] 【看代码的重点】本文件只有 3 个"有点绕"的地方，下面都做了详细标注：
// [解析]   1) set_unique()：为什么要按"大端序"把图形编号拆成 3 个字节；
// [解析]   2) set_int32()：一个 32 位整数怎么塞进 10+11+11 三个位域；
// [解析]   3) Float::set_value()：浮点数为什么要 ×1000 变成整数再发。
// [解析] 【被谁用】图形对象由 applications/uart_task.cpp 创建（参考同目录 readme.md），
// [解析]         创建后交给 sp::UI_Manager::pack() 打包发送；pack() 只读 data 成员，
// [解析]         不会再调用本文件里的任何函数。
// [解析] ==========================================================================

#include "element.hpp"

namespace sp::ui
{
// [解析] set_unique：给每个新建的图形分配一个"唯一编号"，写进协议里的 3 字节
// [解析]   figure_name。这个编号就是图形的身份证：选手端靠它区分不同图形，
// [解析]   以后要 MODIFY（改）或 DELETE（删）某个图形，也必须带同一个编号。
// [解析] static 局部变量 id 的妙用：它只在第一次执行到这里时被初始化为 0，
// [解析]   之后全程序共用这一份并不断自增，于是每个新图形都会拿到不重复的编号。
// [解析]   注意：id 只增不减，delete_all() 也不会把它清零（清零反而会让
// [解析]   新图形和旧图形撞号），一直画下去理论上会回绕，实战中不用担心。
// [解析] 参数 data：要填写的图形描述结构体指针，由 Element 构造函数传入 &data。
static void set_unique(referee::InteractionFigure * data)
{
  static uint32_t id = 0;
  // 南航模拟器是大端序:
  // https://github.com/bismarckkk/RM-UI-Designer/blob/eea60c463dc24055c735d837c89401e633a2a7fb/src/utils/serial/msgView.ts#L123
  // [解析] ↑ 上面两行是原作者给的依据，原样保留：南航模拟器按"大端序"解析 figure_name。
  // [解析] ↓ 下面 3 行就是"把 24 位编号按大端序拆成 3 个字节"：
  // [解析]     id >> 16 取最高 8 位 -> 放进第 0 个字节（最前面）
  // [解析]     id >> 8  取中间 8 位 -> 放进第 1 个字节
  // [解析]     id       取最低 8 位 -> 放进第 2 个字节（最后面）
  // [解析]   因为左边是 uint8_t，赋值时高位会自动截断，所以不用再写 & 0xFF。
  // [解析]   举例：id = 0x010203 时，figure_name = { 0x01, 0x02, 0x03 }。
  // [解析] 小提示：如果换成小端存储，这 3 行的顺序就要反过来，
  // [解析]   所以"通信双方说好谁在前"就是所谓的大小端约定，协议里必须统一。
  data->figure_name[0] = id >> 16;
  data->figure_name[1] = id >> 8;
  data->figure_name[2] = id;
  id++;  // [解析] 自增，保证下一个图形拿到不同的编号
}

// [解析] set_int32：把一个 32 位整数"摊平"写进协议的 details_c / details_d / details_e。
// [解析] 这三个位域的宽度分别是 10 / 11 / 11 bit，加起来正好 32 bit，
// [解析] 而且在协议结构体里是首尾相连的（就是 InteractionFigure 的第三个 4 字节）。
// [解析] 所以本质等于"把协议第三行整行当成一个 int32 用"，只是要按位域一块块填：
// [解析]   details_c = value 的低 10 位
// [解析]   details_d = value 右移 10 位后取低 11 位（即原值的第 10~20 位）
// [解析]   details_e = value 右移 21 位后取低 11 位（即原值的第 21~31 位，含符号位）
// [解析] value 用 int32_t：既支持正数也支持负数（比如显示 -100）。
// [解析] 注意：对负数做 >> 是"算术右移"（高位补 1），这是编译器相关行为，
// [解析]   GCC/ARM 上按补码符号扩展，符合预期；换编译器时值得留意。
// [解析] 注意：details_b（第一行里的 9 bit）没有参与，会保持为 0。
// [解析]   如果官方协议把 details_b 也算作该数值的高位，那么负数还需要额外把
// [解析]   高 9 位也补成 1（符号扩展）；遇到负值显示不正常时，可以对着
// [解析]   裁判系统串口协议再核对这一点。
static void set_int32(referee::InteractionFigure * data, int32_t value)
{
  data->details_c = value;
  data->details_d = value >> 10;
  data->details_e = value >> 21;
}

// [解析] 基类构造函数：统一把"所有图形都有的字段"填好，派生类再补自己特有的参数。
// [解析] 执行顺序很关键，逐条看：
// [解析]   1) set_unique()    先领一个唯一编号（否则后面没法修改/删除这个图形）；
// [解析]   2) 操作类型默认 ADD，也就是"我这是要新增一个图形"；
// [解析]   3) 依次填类型、图层、颜色、线宽、起点/圆心坐标。
// [解析] 这里所有 set_xxx() 都只是往内存里的位域写值，不碰硬件、不发数据，
// [解析] 所以构造函数非常轻量，放在任务循环里频繁调用也没问题。
// [解析] 注意：data 的 details_a / details_b / details_c 在这里还没被赋值，
// [解析]   要由派生类的构造函数去填（比如 Circle 填 details_c）。
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

// [解析] 下面 7 个单行函数是"一次只改一个字段"的接口，全是往协议位域里写值。
// [解析] 参数虽然声明成 uint16_t（2 字节），但目标位域往往更窄
// [解析]   （比如 layer 只有 4 bit、operate_type 只有 3 bit），
// [解析]   超出的高位会被静默丢弃，所以调用前请自己保证取值范围合法。
// [解析] 它们的主要用途是"动态刷新"：改完某个字段后重新 pack() 并发送，
// [解析]   就能让画面上的数值/图形动起来，而不必重新创建对象。
// [解析] 注意：start_x / start_y 对直线、矩形来说是"起点"，对圆/椭圆/圆弧是"圆心"，
// [解析]   同一个字段在不同图形里含义不同，这也是协议里的复用设计。
void Element::set_operate_type(OperateType type) { data.operate_type = static_cast<uint8_t>(type); }
void Element::set_figure_type(FigureType type) { data.figure_type = static_cast<uint8_t>(type); }
void Element::set_layer(Layer layer) { data.layer = static_cast<uint8_t>(layer); }
void Element::set_color(Color color) { data.color = static_cast<uint8_t>(color); }
void Element::set_width(uint16_t width) { data.width = width; }
void Element::set_x(uint16_t x) { data.start_x = x; }
void Element::set_y(uint16_t y) { data.start_y = y; }

// [解析] 直线的构造函数。": ComposableElement(...)" 是初始化列表，
// [解析]   表示"先把基类构造好"（基类里再填编号、类型、图层、颜色、线宽、起点），
// [解析]   然后才执行花括号里的函数体，补上终点坐标。
// [解析] 注意第一个参数写死了 FigureType::LINE：用 Line 类画出来的必然是直线，
// [解析]   所以不需要（也不允许）调用者自己传类型。
Line::Line(
  Layer layer, Color color, uint16_t width, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
: ComposableElement(FigureType::LINE, layer, color, width, x1, y1)
{
  set_x2(x2);
  set_y2(y2);
}

// [解析] 终点 x 存进 details_d，终点 y 存进 details_e —— 这是协议规定的"直线专用"位置。
// [解析] 函数体一行搞定，set 之后重新 pack 就能让线变长变短（配合 OperateType::MODIFY）。
void Line::set_x2(uint16_t x2) { data.details_d = x2; }
void Line::set_y2(uint16_t y2) { data.details_e = y2; }

// [解析] 矩形的构造函数，套路和 Line 完全一样，只是类型换成 RECTANGLE。
// [解析] (x1,y1) 是左上角或任意一个角，(x2,y2) 是对角点；
// [解析] 至于谁是左上谁是右下，交给选手端按坐标大小自己判断，这里不用管。
Rectangle::Rectangle(
  Layer layer, Color color, uint16_t width, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
: ComposableElement(FigureType::RECTANGLE, layer, color, width, x1, y1)
{
  set_x2(x2);
  set_y2(y2);
}

// [解析] 同样放在 details_d / details_e，说明协议里"第二组坐标"是多种图形共用的位置。
void Rectangle::set_x2(uint16_t x2) { data.details_d = x2; }
void Rectangle::set_y2(uint16_t y2) { data.details_e = y2; }

// [解析] 正圆构造函数：基类填好圆心 (x,y) 和 CIRCLE 类型，这里只再补一个半径。
// [解析] 半径画出来是"实心圆的半径"，例如 radius = 50 表示直径 100 像素。
Circle::Circle(Layer layer, Color color, uint16_t width, uint16_t x, uint16_t y, uint16_t radius)
: ComposableElement(FigureType::CIRCLE, layer, color, width, x, y)
{
  set_radius(radius);
}

// [解析] 半径存进 details_c。注意它只有 10 bit，最大 1023：
// [解析]   想画更大的圆，圆会被"截断"成一个半径 1023 的圆（数值按位截断后可能更小），
// [解析]   所以超过 1000 像素的圆请改用椭圆或圆弧实现。
void Circle::set_radius(uint16_t radius) { data.details_c = radius; }

// [解析] 椭圆构造函数：基类填圆心，这里补 x/y 两个半轴长度。
// [解析] 半轴 = 半径：x_radius 是水平方向的半径，y_radius 是竖直方向的半径。
// [解析] 两个半轴相等时画出来就是正圆，所以椭圆在功能上也能完全替代圆。
Ellipse::Ellipse(
  Layer layer, Color color, uint16_t width, uint16_t x, uint16_t y, uint16_t x_radius,
  uint16_t y_radius)
: ComposableElement(FigureType::ELLIPSE, layer, color, width, x, y)
{
  set_x_radius(x_radius);
  set_y_radius(y_radius);
}

// [解析] 两个半轴分别放 details_d / details_e（各 11 bit，最大 2047）。
void Ellipse::set_x_radius(uint16_t x_radius) { data.details_d = x_radius; }
void Ellipse::set_y_radius(uint16_t y_radius) { data.details_e = y_radius; }

// [解析] 圆弧构造函数：参数最多，一共 4 个"图形专属参数"要填。
// [解析] 圆弧可以理解为"椭圆上截下来的一段"，所以既要圆心和两个半轴，
// [解析] 还要告诉选手端"从多少度画到多少度"。
// [解析] 角度单位是"度"，合法范围 0~360；角度只有 9 bit（最大 511），
// [解析]   所以填 0~360 是安全的，但填超过 511 会被截断。
// [解析] 注意：起点/终点角度没有强制大小关系，想画"跨过 0 度"的弧
// [解析]   （比如 350 度到 10 度）时行为取决于选手端实现，建议先在南航模拟器上试。
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

// [解析] 圆弧把 5 个 details 位域用得很满，对照着看就不乱了：
// [解析]   起始角度 -> details_a（9 bit）
// [解析]   终止角度 -> details_b（9 bit）
// [解析]   x 半轴   -> details_d（11 bit）
// [解析]   y 半轴   -> details_e（11 bit）
// [解析]   details_c 空着不用。
// [解析] 注意：圆弧的 details_d / details_e 和直线的"终点坐标"用的是同一对位域，
// [解析]   全靠 figure_type 字段区分该怎么解释，这也是位域复用协议的典型写法。
void Arc::set_start_angle(uint16_t start_angle) { data.details_a = start_angle; }
void Arc::set_end_angle(uint16_t end_angle) { data.details_b = end_angle; }
void Arc::set_x_radius(uint16_t x_radius) { data.details_d = x_radius; }
void Arc::set_y_radius(uint16_t y_radius) { data.details_e = y_radius; }

// [解析] 浮点数图形构造函数：基类填起点和 FLOAT 类型，这里补字号和数值。
// [解析] font_size 是显示字号（像素高度），字号只有 9 bit（最大 511）；
// [解析]   width（线宽）在浮点数图形里其实用不到，但基类构造函数要求传，
// [解析]   所以 readme 的示例里随便给了个值（比如 6）。
// [解析] 注意：协议建议"字号 : 线宽 = 10 : 1"，画字符和数字时按这个比例比较好看。
Float::Float(
  Layer layer, Color color, uint16_t width, uint16_t x, uint16_t y, uint16_t font_size, float value)
: ComposableElement(FigureType::FLOAT, layer, color, width, x, y)
{
  set_font_size(font_size);
  set_value(value);
}

// [解析] 字号存放在 details_a（9 bit）。
// [解析] 顺便说明：details_a 在圆弧里表示"起始角度"，在字符/数字里表示"字号"，
// [解析]   具体含义完全由 figure_type 决定。
void Float::set_font_size(uint16_t font_size) { data.details_a = font_size; }

// [解析] 浮点数最关键的换算就在这里：value * 1e3f 表示"乘以 1000"。
// [解析] 为什么要 ×1000？因为协议里的数值字段只支持整数，
// [解析]   于是用"先放大 1000 倍存整数，选手端再除回 1000"的办法表示小数，
// [解析]   这样既能保留 3 位小数，又避免了直接传 float 时的端序/对齐/精度问题。
// [解析] 换算举例：3.14 -> 3.14 * 1000 = 3140.0 -> 转成整数 3140，选手端显示 3.140。
// [解析] 1e3f 结尾的 f 表示这是 float 字面量，不加 f 会先按 double 算再转回 float。
// [解析] 注意：这里是"截断取整"而不是四舍五入，3.14159 会变成 3141（误差约 0.0006）。
// [解析] 注意：float 本身能到 3.4e38，但后续要存进 32 位整数，
// [解析]   所以 value 的有效范围只有约 ±2147483.647，超出就会整数溢出、显示乱掉。
void Float::set_value(float value) { set_int32(&data, value * 1e3f); }

// [解析] 整型数图形构造函数：和 Float 长得几乎一模一样，只是数值类型是 int。
// [解析] 在 UI 上显示整数时优先用它（例如剩余血量、弹量、比赛剩余时间），
// [解析]   因为它不做 ×1000 缩放，读写都更直观。
Integer::Integer(
  Layer layer, Color color, uint16_t width, uint16_t x, uint16_t y, uint16_t font_size, int value)
: ComposableElement(FigureType::INTEGER, layer, color, width, x, y)
{
  set_font_size(font_size);
  set_value(value);
}

// [解析] 字号同样放 details_a，和 Float 的规则一致。
void Integer::set_font_size(uint16_t font_size) { data.details_a = font_size; }

// [解析] 整数直接交给 set_int32 拆成 10+11+11 三块塞进协议，不做任何缩放。
// [解析] 注意：int 在 STM32（32 位 ARM）上是 4 字节，正好和 set_int32 的入参 int32_t 等宽；
// [解析]   如果把这段代码移植到 int 只有 2 字节的平台上，超范围的值会被截断。
void Integer::set_value(int value) { set_int32(&data, value); }

// [解析] 字符图形构造函数。注意这里初始化列表写的是 ": Element(...)"，
// [解析]   而不是其他图形用的 ": ComposableElement(...)" ——
// [解析]   因为 String 在 element.hpp 里就是直接继承 Element 的
// [解析]   （字符图形每帧只能单独发一个，不能和普通图形混在一帧里打包）。
// [解析] 参数 value 是 const std::string&：用引用传参避免拷贝一份字符串，
// [解析]   const 表示函数内部不会改动调用者的字符串。
String::String(
  Layer layer, Color color, uint16_t width, uint16_t x, uint16_t y, uint16_t font_size,
  const std::string & value)
: Element(FigureType::STRING, layer, color, width, x, y)
{
  set_font_size(font_size);
  set_value(value);
}

// [解析] 字符图形的字号同样存在 details_a。
void String::set_font_size(uint16_t font_size) { data.details_a = font_size; }

// [解析] 设置要显示的文本，这里做了两件事：
// [解析]   1) str = value;        把字符串本体存进成员变量 str（用的是 std::string 的拷贝赋值）；
// [解析]   2) data.details_b = …  告诉选手端"这次要显示几个字节"。
// [解析]      协议里字符图形的 details_b 就是"字符数据长度"，占 9 bit（最大 511）。
// [解析] std::min(str.size(), sizeof(...)) 是"防止越界"的关键：
// [解析]   协议给字符数据留的空间是 ExtClientCustomCharacter::data[30]（共 30 字节，
// [解析]   referee_protocol.hpp 里的 static_assert 可以确认它就是 30），
// [解析]   所以这里取"实际长度"和"30"里较小的那个，保证后面 manager.cpp 拷贝时不会溢出。
// [解析] 注意：这里算的是"字节数"不是"字符数"。一个汉字在 UTF-8 下占 3 字节，
// [解析]   所以中文字符串最多只能放 10 个汉字，超出的部分会被静默截断（不会报错，只是显示不全）。
// [解析] 注意：std::min 的声明在 <algorithm> 里，本文件并没有直接 include 它，
// [解析]   目前能编译通过是因为标准库头文件间接包含；如果哪天报"找不到 std::min"，
// [解析]   补一个 #include <algorithm> 即可（本注释只提示，不修改代码）。
void String::set_value(const std::string & value)
{
  str = value;
  data.details_b = std::min(str.size(), sizeof(referee::ExtClientCustomCharacter::data));
}

}  // namespace sp::ui
