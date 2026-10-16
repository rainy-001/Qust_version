// [解析] ==========================================================================
// [解析] 【文件】sp_middleware/io/adc/adc.cpp
// [解析] 【作用】adc.hpp 里 Adc 类的实现。核心只有三步：存参数 -> 算换算系数 -> 轮询读一次值。
// [解析] 【被谁用】被 applications/ 下的 ADC 采集任务通过 Adc 对象调用（不会有人直接调这些函数）。
// [解析] 【关键点】1) 构造函数的初始化列表顺序、2) init() 里那个 adc12_to_v_coef_ 公式、
// [解析]           3) update() 为什么必须等 HAL_ADC_PollForConversion() 成功后才读数据。
// [解析] ==========================================================================
#include "adc.hpp"

namespace sp
{
// [解析] 构造函数：只把外部传进来的参数和算好的系数存下来，不碰任何硬件。
// [解析] 冒号后面这一串叫『初始化列表』，等价于在函数体里逐个赋值，但效率更高（直接构造而不是先默认构造再赋值）。
// [解析] 注意：几个成员真正的初始化顺序由它们在 hpp 里声明的顺序决定（voltage -> adc_ -> channel_pow_ ...），
// [解析]       而不是这里写的顺序，所以不要写出互相依赖的初始化表达式。
Adc::Adc(
  ADC_HandleTypeDef * adc, uint32_t channels, float vref, float voltage_divider,
  float voltage_offset)
: voltage(0.0f),
  adc_(adc),
  // [解析] ↑ 1 << 12 = 4096，即『12 位 ADC 满量程 + 1』。
  // [解析]   为什么用 1 << channels 而不是直接写 4096：这样 channels 传 12 或 16 都能自动适配。
  // [解析]   注意：channels 传 0 会得到 1，传超过 31 是未定义行为，实际只会传 12（C 板 ADC 是 12 位）。
  channel_pow_(1 << channels),
  vref_(vref),
  voltage_divider_(voltage_divider),
  voltage_offset_(voltage_offset)
  // [解析] ↑ 电压偏移（V），标定用（标定时填实测差值）；没有偏差就传 0.0f。
  // [解析] 下面这个 { 是构造函数体的开始，函数体是空的：因为真正的硬件动作都放在 init() 里。
{
}

// [解析] init()：初始化整个对象，必须在第一次 update() 之前调用（通常只在开机时调一次）。
// [解析] 它不做『启动一次转换』那种事，只配置时钟分频并把换算系数算好。
void Adc::init()
{
  // [解析] ClockPrescaler = 同步时钟 4 分频：ADC 时钟 = PCLK2 / 4。
  // [解析] 为什么是这个值：STM32F407 的 ADC 时钟上限约 36MHz，而 PCLK2 一般是 84MHz，
  // [解析] 直接四分频得到 21MHz，既不超过上限、也不会慢到影响采样速度（分频越大越省电也越慢）。
  // [解析] 注意：这里改的是 adc_->Init 结构体成员，如果 CubeMX 生成的 adc.c 里 init 用的也是这个值，
  // [解析] 则等价于重复设置；如果不一样，则以这里为准（因为 init() 是在 CubeMX 初始化之后再调用的）。
  //C板 四分频（不超频又不功耗太高）
  adc_->Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  // [解析] ↑ 把上面设好的分频写进 ADC 寄存器。HAL_ADC_Init() 内部会重新配置整个 ADC 外设，
  // [解析] 所以它必须放在 ClockPrescaler 赋值之后、以及其他任何转换操作之前。
  HAL_ADC_Init(adc_);
  //计算ADC转换值到电压的系数
  // [解析] ↑ 把公式一次算好存起来，以后每次 update() 只做一次乘加，省掉重复的乘除法：
  // [解析]   系数 = 参考电压(V) × 分压比 ÷ 满量程(4096)，单位是『伏特 / 每个原始计数』。
  // [解析]   注意：这里用的是 this->vref_ 等成员，因为参数 vref 的作用域只在构造函数里。
  this->adc12_to_v_coef_ = this->vref_ * this->voltage_divider_ / this->channel_pow_;
}

// [解析] update()：读一次电压。返回 true 表示 voltage 已刷新，false 表示这次测量失败。
// [解析] 它内部会阻塞等待转换完成，所以调用周期不要比『一次转换时间』还短，否则任务会一直卡在这里。
bool Adc::update()
{
  // [解析] ↑ HAL_ADC_Start() 只是『启动软件转换』（把 SWSTART 位置 1），此时数据还没出来，
  // [解析]   必须再等 EOC（转换结束）标志。失败通常是外设没初始化或正忙，直接返回 false 让上层重试。
  if (HAL_ADC_Start(adc_) != HAL_OK) return false;
  // 不能启动后立即读取 DR：必须等本次转换完成，避免上电零值或旧值误报警。
  // [解析] ↑ 第二个参数 1U 是『超时时间』，单位是毫秒（HAL 源码注释写的是 millisecond），
  // [解析]   即最多等本次转换 1ms；超时就返回 HAL_TIMEOUT，本函数立刻返回 false（不会死等）。
  // [解析]   12 位一次转换大约几微秒，1ms 非常宽松，所以正常情况永远不会超时。
  // [解析]   为什么要给超时而不是 HAL_MAX_DELAY：万一 ADC 硬件异常，也不会把整个任务卡死。
  if (HAL_ADC_PollForConversion(adc_, 1U) != HAL_OK) {
    HAL_ADC_Stop(adc_);
    return false;
  }
  // [解析] ↑ raw 是 12 位原始值，范围 [0, 4095]：0 对应 0V，4095 对应『参考电压 × 分压比』那个上限。
  // [解析]   它是个中间量，用 const 表示后面不会改，防止误写。
  const uint32_t raw = HAL_ADC_GetValue(adc_);
  if (HAL_ADC_Stop(adc_) != HAL_OK) return false;
  // [解析] ↑ 核心换算：电压(V) = 原始值 × 系数 + 偏移。
  // [解析]   展开就是 (raw / 4096) × vref × 分压比 + 偏移，也就是『先把数字量归一化到 0~1，
  // [解析]   再乘参考电压得到引脚上的电压，最后乘分压比还原成真实电压』。
  // [解析]   举例：vref=3.3V、分压比 222/22=10.09，若 raw=2048（约一半量程），
  // [解析]   则 voltage ≈ 0.5 × 3.3 × 10.09 ≈ 16.65V，差不多是 4S 电池的电压量级。
  this->voltage = raw * this->adc12_to_v_coef_ + this->voltage_offset_;
  // [解析] ↑ 走到这里说明转换与关停都成功，voltage 已经是最新值，返回 true 让调用者放心读。
  return true;
}
}  // namespace sp
