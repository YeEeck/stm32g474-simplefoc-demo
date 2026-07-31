#include "Arduino.h"
#include "main.h"

// ---- 时间：micros 用 DWT 周期计数器（170MHz），delay 用 SysTick ----
static void dwt_init() {
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

unsigned long micros() {
  static bool initialized = false;
  if (!initialized) { dwt_init(); initialized = true; }
  return DWT->CYCCNT / (SystemCoreClock / 1000000ul); // 周期计数 → us
}

void delay(unsigned long ms) { HAL_Delay(ms); }

void delayMicroseconds(unsigned int us) {
  unsigned long start = micros();
  while ((micros() - start) < us) {}
}

// ---- 引脚 API（本项目未使用，提供空实现避免链接错误）----
void pinMode(uint8_t, uint8_t) {}
void digitalWrite(uint8_t, uint8_t) {}
int digitalRead(uint8_t) { return 0; }
