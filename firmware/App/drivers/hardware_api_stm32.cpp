#include "main.h"
#include "tim.h"
#include "../../vendors/simplefoc/src/drivers/hardware_api.h"

// ---- SimpleFOC hardware_api 的 STM32 HAL 实现（3PWM 模式）----
// TIM1 三通道 PWM 已在 CubeMX 配置（20kHz 中心对齐），SimpleFOC 只写占空比。

typedef struct Stm32DriverParams {
  TIM_HandleTypeDef* tim;
  uint32_t arr;
} Stm32DriverParams;

void* _configure3PWM(long pwm_frequency, const int pinA, const int pinB, const int pinC) {
  (void)pwm_frequency;
  (void)pinA; (void)pinB; (void)pinC;
  static Stm32DriverParams params = {&htim1, 0};
  params.arr = __HAL_TIM_GET_AUTORELOAD(&htim1);
  // 中心对齐模式下更新事件在上溢+下溢都产生；RCR=1 使每 PWM 周期仅触发一次
  // （SimpleFOC 官方同样配置），触发点在上溢 = 脉宽中心（电流纹波最小处采样）
  htim1.Instance->RCR = 1;
  // 启动 PWM（中心对齐，MOE 使能），占空比 0
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  return &params;
}

void _writeDutyCycle3PWM(float dc_a, float dc_b, float dc_c, void* params) {
  Stm32DriverParams* p = (Stm32DriverParams*)params;
  if (!p) return;
  uint32_t ccr_a = (uint32_t)(dc_a * (float)p->arr);
  uint32_t ccr_b = (uint32_t)(dc_b * (float)p->arr);
  uint32_t ccr_c = (uint32_t)(dc_c * (float)p->arr);
  __HAL_TIM_SET_COMPARE(p->tim, TIM_CHANNEL_1, ccr_a);
  __HAL_TIM_SET_COMPARE(p->tim, TIM_CHANNEL_2, ccr_b);
  __HAL_TIM_SET_COMPARE(p->tim, TIM_CHANNEL_3, ccr_c);
}

// 其他驱动模式未使用：提供空实现占位（SimpleFOC 链接需要符号）
void* _configure1PWM(long, const int) { return (void*)-1; }
void* _configure2PWM(long, const int, const int) { return (void*)-1; }
void* _configure4PWM(long, const int, const int, const int, const int) { return (void*)-1; }
void* _configure6PWM(long, float, const int, const int, const int, const int, const int, const int) { return (void*)-1; }
void _writeDutyCycle1PWM(float, void*) {}
void _writeDutyCycle2PWM(float, float, void*) {}
void _writeDutyCycle4PWM(float, float, float, float, void*) {}
void _writeDutyCycle6PWM(float, float, float, PhaseState*, void*) {}
