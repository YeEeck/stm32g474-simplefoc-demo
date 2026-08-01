#include "iprop_current_sense.h"

// channel_index: 0/1/2 = SOA/SOB/SOC → 注入 rank 1/2/3（IN1/IN2/IN4）
IpropCurrentSense::IpropCurrentSense(ADC_HandleTypeDef* hadc, float gain_v_per_a)
    : hadc_(hadc) {
  // 通道映射存入基类字段，使基类 driverAlign 的相序自校正（交换 pinA/B/C、offset、gain）生效
  pinA = 0;
  pinB = 1;
  pinC = 2;
  gain_a = gain_b = gain_c = gain_v_per_a; // IPROPI 增益 [V/A]，来自电机配置
}

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
  float va = sampleVoltage(pinA);
  float vb = sampleVoltage(pinB);
  float vc = sampleVoltage(pinC);
  PhaseCurrent_s currents;
  currents.a = (va - offset_ia) / gain_a;
  currents.b = (vb - offset_ib) / gain_b;
  currents.c = (vc - offset_ic) / gain_c;
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
