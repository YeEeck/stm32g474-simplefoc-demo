#include "Arduino.h"
#include "main.h"

// ---- Print 实现：通过派生类的 write() 输出 ----
size_t Print::print(const char* s) {
  size_t n = 0;
  if (s) { while (*s) { n += write((uint8_t)*s++); } }
  return n;
}
size_t Print::print(char c) { return write((uint8_t)c); }
size_t Print::print(unsigned char c) { return write((uint8_t)c); }

static size_t printUInt(Print& p, unsigned long n) {
  char buf[12];
  buf[11] = '\0';
  char* out = buf + 11;
  do { *--out = '0' + (n % 10); n /= 10; } while (n);
  return p.print(out);
}

size_t Print::print(int n) {
  if (n < 0) { write((uint8_t)'-'); return 1 + printUInt(*this, (unsigned long)(-n)); }
  return printUInt(*this, (unsigned long)n);
}
size_t Print::print(unsigned int n) { return printUInt(*this, n); }
size_t Print::print(long n) { return print((int)n); }
size_t Print::print(unsigned long n) { return printUInt(*this, n); }

size_t Print::print(float f, int decimals) { return print((double)f, decimals); }
size_t Print::print(double d, int decimals) {
  if (d < 0) { write((uint8_t)'-'); d = -d; }
  size_t n = printUInt(*this, (unsigned long)d);
  if (decimals > 0) {
    write((uint8_t)'.');
    double frac = d - (double)(unsigned long)d;
    for (int i = 0; i < decimals; i++) {
      frac *= 10.0;
      write((uint8_t)('0' + (int)frac));
      frac -= (int)frac;
    }
    n += decimals + 1;
  }
  return n;
}

size_t Print::println() { return write((uint8_t)'\n'); }
size_t Print::println(const char* s) { return print(s) + println(); }
size_t Print::println(char c) { return print(c) + println(); }
size_t Print::println(int n) { return print(n) + println(); }
size_t Print::println(unsigned int n) { return print(n) + println(); }
size_t Print::println(long n) { return print(n) + println(); }
size_t Print::println(unsigned long n) { return print(n) + println(); }
size_t Print::println(float f, int decimals) { return print(f, decimals) + println(); }
size_t Print::println(double d, int decimals) { return print(d, decimals) + println(); }

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
