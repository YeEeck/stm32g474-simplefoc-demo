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
