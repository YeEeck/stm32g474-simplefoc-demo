#ifndef ARDUINO_COMPAT_H
#define ARDUINO_COMPAT_H
// 最小 Arduino 兼容层：仅提供 SimpleFOC 核心源码需要的符号。
// 位于本目录使 `#include "Arduino.h"` 解析到此处（-I 顺序优先）。

#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <math.h>

// ---- 基础类型 ----
typedef uint8_t byte;
typedef uint8_t boolean;
// Arduino 的 flash 字符串助手：不透明类型，F() 宏负责转换，保证与 const char* 重载可区分
struct __FlashStringHelper;
#define F(s) ((const __FlashStringHelper*)(s))
#define PROGMEM
typedef const char* PGM_P;

// ---- 引脚常量（本项目不使用 Arduino 引脚 API 的运行时功能）----
#define HIGH 0x1
#define LOW 0x0
#define INPUT 0x0
#define OUTPUT 0x1
#define INPUT_PULLUP 0x2

void pinMode(uint8_t pin, uint8_t mode);
void digitalWrite(uint8_t pin, uint8_t val);
int digitalRead(uint8_t pin);

// ---- 时间 ----
unsigned long micros();
void delay(unsigned long ms);
void delayMicroseconds(unsigned int us);

// ---- Print / Stream（Commander 与 FOCMotor monitor 的输出通道）----
class Print {
public:
  virtual size_t write(uint8_t b) = 0;
  size_t print(const char* s);
  size_t print(const __FlashStringHelper* s) { return print((const char*)s); }
  size_t print(char c);
  size_t print(unsigned char c);
  size_t print(int n);
  size_t print(unsigned int n);
  size_t print(long n);
  size_t print(unsigned long n);
  size_t print(float f, int decimals = 2);
  size_t print(double d, int decimals = 2);
  size_t println();
  size_t println(const char* s);
  size_t println(const __FlashStringHelper* s) { return println((const char*)s); }
  size_t println(char c);
  size_t println(int n);
  size_t println(unsigned int n);
  size_t println(long n);
  size_t println(unsigned long n);
  size_t println(float f, int decimals = 2);
  size_t println(double d, int decimals = 2);
};

class Stream : public Print {
public:
  virtual int available() = 0;
  virtual int read() = 0;
  using Print::print;
  using Print::println;
};

// ---- 杂项 ----
#include <ctype.h>
#define isDigit(c) isdigit((unsigned char)(c))
#define isAlpha(c) isalpha((unsigned char)(c))
#define isAlphaNumeric(c) isalnum((unsigned char)(c))
#ifndef min
#define min(a, b) (((a) < (b)) ? (a) : (b))
#endif
#ifndef max
#define max(a, b) (((a) > (b)) ? (a) : (b))
#endif
#define constrain(amt, low, high) _constrain(amt, low, high)
#define map(x, in_min, in_max, out_min, out_max) \
  (((x) - (in_min)) * ((out_max) - (out_min)) / ((in_max) - (in_min)) + (out_min))

#endif
