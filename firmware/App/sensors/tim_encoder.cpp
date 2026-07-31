#include "tim_encoder.h"
#include "main.h"

void TimEncoder::init() {
  __HAL_TIM_SET_COUNTER(htim_, 0);
  HAL_TIM_Encoder_Start(htim_, TIM_CHANNEL_ALL);
  Sensor::init();
}

float TimEncoder::getSensorAngle() {
  uint32_t cnt = __HAL_TIM_GET_COUNTER(htim_);
  return _2PI * (float)(cnt % CPR) / (float)CPR;
}

void TimEncoder::onIndexPulse() {
  // Z 到达：将计数器校准到最近的整圈零点（0 … CPR-1）
  uint32_t cnt = __HAL_TIM_GET_COUNTER(htim_);
  int32_t nearest = (int32_t)(_round((float)cnt / (float)CPR) * (float)CPR);
  int32_t corrected = nearest % (int32_t)CPR;
  if (corrected < 0) corrected += CPR;
  __HAL_TIM_SET_COUNTER(htim_, (uint32_t)corrected);
}
