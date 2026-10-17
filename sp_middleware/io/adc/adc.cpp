// [解析] ADC 电压采集的轮询式实现，核心只有三步：存参数 -> 算系数 -> 读一次值。
// [解析] applications/voltage_detect.cpp 的 adc_task() 通过 Adc 对象读电池电压（C 板是 ADC3 的 IN8/PF10）。
// [解析] 没有中断、也没有 DMA：update() 会阻塞等到本次转换结束才返回，别在高优先级任务里频繁调。
#include "adc.hpp"

namespace sp
{
// [解析] channel_pow_ 是满量程（1<<位数，12 位即 4096），update() 的换算拿它当分母。
Adc::Adc(
  ADC_HandleTypeDef * adc, uint32_t channels, float vref, float voltage_divider,
  float voltage_offset)
: voltage(0.0f),
  adc_(adc),
  channel_pow_(1 << channels),
  vref_(vref),
  voltage_divider_(voltage_divider),
  voltage_offset_(voltage_offset)
{
}

void Adc::init()
{
  //C板 四分频（不超频又不功耗太高）
  adc_->Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  HAL_ADC_Init(adc_);
  //计算ADC转换值到电压的系数
  // [解析] 系数 = vref × 分压比 ÷ 满量程，单位 V/计数；算一次存起来，之后每次只做一次乘加。
  this->adc12_to_v_coef_ = this->vref_ * this->voltage_divider_ / this->channel_pow_;
}

bool Adc::update()
{
  if (HAL_ADC_Start(adc_) != HAL_OK) return false;
  // 不能启动后立即读取 DR：必须等本次转换完成，避免上电零值或旧值误报警。
  // [解析] 第二个参数 1U 是超时毫秒数；超时返回 HAL_TIMEOUT，这里直接放弃本次测量，不会死等。
  if (HAL_ADC_PollForConversion(adc_, 1U) != HAL_OK) {
    HAL_ADC_Stop(adc_);
    return false;
  }
  const uint32_t raw = HAL_ADC_GetValue(adc_);
  if (HAL_ADC_Stop(adc_) != HAL_OK) return false;
  this->voltage = raw * this->adc12_to_v_coef_ + this->voltage_offset_;
  return true;
}
}  // namespace sp
