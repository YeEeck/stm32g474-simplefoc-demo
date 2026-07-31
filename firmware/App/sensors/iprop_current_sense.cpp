#include "iprop_current_sense.h"

int IpropCurrentSense::init() {
  // 使能注入组转换 + 中断（CubeMX 已配好触发源/通道/NVIC）
  if (HAL_ADCEx_InjectedStart_IT(hadc_) != HAL_OK) return 0;
  initialized = true;
  return 1;
}

float IpropCurrentSense::sampleVoltage(int channel_index) {
  // 注入序列：rank1=IN1(SOA), rank2=IN2(SOB), rank3=IN4(SOC)
  uint32_t rank = (channel_index == 0) ? ADC_INJECTED_RANK_1 :
                  (channel_index == 1) ? ADC_INJECTED_RANK_2 : ADC_INJECTED_RANK_3;
  uint32_t raw = HAL_ADCEx_InjectedGetValue(hadc_, rank);
  return (float)raw * ADC_VOLTAGE_CONV;
}

PhaseCurrent_s IpropCurrentSense::getPhaseCurrents() {
  float va = sampleVoltage(0);
  float vb = sampleVoltage(1);
  float vc = sampleVoltage(2);
  PhaseCurrent_s currents;
  currents.a = (va - offset_ia) / gain_v_per_a_;
  currents.b = (vb - offset_ib) / gain_v_per_a_;
  currents.c = (vc - offset_ic) / gain_v_per_a_;
  return currents;
}

bool IpropCurrentSense::calibrateOffsets(uint8_t samples) {
  if (samples == 0) samples = 1;
  double sa = 0, sb = 0, sc = 0;
  for (uint8_t i = 0; i < samples; i++) {
    sa += sampleVoltage(0);
    sb += sampleVoltage(1);
    sc += sampleVoltage(2);
  }
  offset_ia = (float)(sa / samples);
  offset_ib = (float)(sb / samples);
  offset_ic = (float)(sc / samples);
  return true;
}
