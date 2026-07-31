// SIL 版 platform 实现：仿真时间由主循环（sim_advance_time_us）推进，
// delayMicroseconds 无需真实等待（避免死等）。

#include "Arduino.h"
#include "stm32g4xx_hal.h"

extern "C" {
uint32_t sim_get_time_us();
}

unsigned long micros() { return sim_get_time_us(); }

void delay(unsigned long ms) { HAL_Delay(ms); }

void delayMicroseconds(unsigned int) {
  // 仿真时间由 SIL 主循环推进，这里不等待
}

void pinMode(uint8_t, uint8_t) {}
void digitalWrite(uint8_t, uint8_t) {}
int digitalRead(uint8_t) { return 0; }

StubSerialStream Serial;
