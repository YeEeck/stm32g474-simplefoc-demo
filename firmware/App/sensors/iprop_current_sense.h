#ifndef IPROP_CURRENT_SENSE_H
#define IPROP_CURRENT_SENSE_H

#include "main.h"
#include "../../vendors/simplefoc/src/common/base_classes/CurrentSense.h"

// DRV8316CT 内置电流检测（IPROPI）采样：
// SOA/SOB/SOC → ADC1 注入组 IN1/IN2/IN4，由 TIM1 TRGO2（PWM 中心）硬件触发，
// 注入转换完成中断中读取三路电流，再由中断内联驱动 FOC 电流环。
class IpropCurrentSense : public CurrentSense {
public:
  IpropCurrentSense(ADC_HandleTypeDef* hadc, float gain_v_per_a)
      : hadc_(hadc), gain_v_per_a_(gain_v_per_a) {}

  int init() override;
  PhaseCurrent_s getPhaseCurrents() override;

  // 电机静止时采集零电流偏移（各通道 N 次平均）
  bool calibrateOffsets(uint8_t samples = 16);

private:
  static constexpr float ADC_VOLTAGE_CONV = 3.3f / 4096.0f;

  ADC_HandleTypeDef* hadc_;
  float gain_v_per_a_; // IPROPI 增益 [V/A]，默认 1.45，需实测标定

  float sampleVoltage(int channel_index);
};

#endif
