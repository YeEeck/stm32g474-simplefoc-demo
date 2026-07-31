// SIL 测试：HAL 函数 stub 与全局外设实例。
// 电机模型通过 sim_set_* 接口把模拟结果写入"寄存器"，让真实固件代码原样运行。

#include "stm32g4xx_hal.h"

#include <string.h>

// ---- 外设实例（SIL 全局，供固件 extern 引用）----
TIM_TypeDef tim1_regs;
TIM_TypeDef tim3_regs;
ADC_TypeDef adc1_regs;
DWT_TypeDef dwt_inst;
CoreDebug_TypeDef core_debug_inst;
uint32_t SystemCoreClock = 170000000;

TIM_HandleTypeDef htim1 = {&tim1_regs};
TIM_HandleTypeDef htim3 = {&tim3_regs};
ADC_HandleTypeDef hadc1 = {&adc1_regs};
UART_HandleTypeDef huart2 = {2};

// ---- SIL 观测/注入接口 ----
static volatile uint32_t sim_time_us_ = 0;   // 由仿真主循环推进
static GPIO_PinState nsleep_state_ = GPIO_PIN_RESET;
static uint8_t uart_out_[2048];
static size_t uart_out_len_ = 0;

// HAL_Delay 期间推进仿真：SIL 注册电机模型步进函数
void (*g_sil_delay_hook)(uint32_t ms) = nullptr;

extern "C" {

uint32_t sim_get_time_us() { return sim_time_us_; }
void sim_advance_time_us(uint32_t us) {
  sim_time_us_ += us;
  dwt_inst.CYCCNT = sim_time_us_ * SystemCoreClock / 1000000ul;
}
GPIO_PinState sim_get_nsleep() { return nsleep_state_; }
void sim_set_phase_currents(float ia, float ib, float ic, float offset_v, float gain_v_per_a) {
  adc1_regs.JDR1 = (uint32_t)((offset_v + ia * gain_v_per_a) / 3.3f * 4096.0f);
  adc1_regs.JDR2 = (uint32_t)((offset_v + ib * gain_v_per_a) / 3.3f * 4096.0f);
  adc1_regs.JDR3 = (uint32_t)((offset_v + ic * gain_v_per_a) / 3.3f * 4096.0f);
}
uint32_t sim_get_encoder_cnt() { return tim3_regs.CNT; }
float sim_get_phase_voltage(int ch) { // ch: 1..3，读 TIM1 CCR（驱动输出）
  return ch == 1 ? (float)tim1_regs.CCR1 : ch == 2 ? (float)tim1_regs.CCR2 : (float)tim1_regs.CCR3;
}
uint32_t sim_get_pwm_arr() { return tim1_regs.ARR; }
void sim_clear_uart_out() { uart_out_len_ = 0; }
size_t sim_get_uart_out_len() { return uart_out_len_; }
const uint8_t* sim_get_uart_out() { return uart_out_; }
void sim_inject_uart_bytes(const char* s) {
  extern void HAL_UART_RxCpltCallback(void* huart);
  extern uint8_t rx_byte_;
  while (*s) {
    rx_byte_ = (uint8_t)*s++;
    HAL_UART_RxCpltCallback(&huart2);
  }
}
void sim_set_delay_hook(void (*hook)(uint32_t ms)) { g_sil_delay_hook = hook; }

// ---- HAL 函数 stub ----
HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef*, uint32_t) { return HAL_OK; }
HAL_StatusTypeDef HAL_TIM_Encoder_Start(TIM_HandleTypeDef*, uint32_t) { return HAL_OK; }
HAL_StatusTypeDef HAL_ADCEx_InjectedStart_IT(ADC_HandleTypeDef*) { return HAL_OK; }
uint32_t HAL_ADCEx_InjectedGetValue(const ADC_HandleTypeDef* hadc, uint32_t rank) {
  switch (rank) {
    case ADC_INJECTED_RANK_1: return hadc->Instance->JDR1;
    case ADC_INJECTED_RANK_2: return hadc->Instance->JDR2;
    case ADC_INJECTED_RANK_3: return hadc->Instance->JDR3;
    default: return 0;
  }
}
void HAL_GPIO_WritePin(GPIO_TypeDef*, uint16_t, GPIO_PinState PinState) {
  nsleep_state_ = PinState;
}
void HAL_NVIC_SetPriority(uint32_t, uint32_t, uint32_t) {}
void HAL_NVIC_EnableIRQ(uint32_t) {}
void HAL_Delay(uint32_t ms) {
  sim_advance_time_us(ms * 1000ul);
  if (g_sil_delay_hook) g_sil_delay_hook(ms);
}
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef*, uint8_t* pData, uint16_t Size, uint32_t) {
  if (uart_out_len_ + Size <= sizeof(uart_out_)) {
    memcpy(uart_out_ + uart_out_len_, pData, Size);
    uart_out_len_ += Size;
  }
  return HAL_OK;
}
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef*, uint8_t*, uint16_t) { return HAL_OK; }

} // extern "C"
