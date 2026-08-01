#ifndef HARDWARE_UART_STREAM_H
#define HARDWARE_UART_STREAM_H

#include "Arduino.h"
#include "main.h"

extern volatile uint8_t rx_byte_; // USART2 单字节接收缓冲（由 onRxByte 转发到环形缓冲）

// USART2 → Stream：接收用中断 + 环形缓冲，发送轮询。
class HardwareUartStream : public Stream {
public:
  bool init();

  // Stream
  int available() override;
  int read() override;
  size_t write(uint8_t b) override;

  // 由 HAL_UART_RxCpltCallback 调用（中断上下文）
  void onRxByte(uint8_t b);

  static HardwareUartStream& instance() { static HardwareUartStream s; return s; }

private:
  static constexpr size_t RX_BUF_SIZE = 256;
  volatile uint8_t rx_buf_[RX_BUF_SIZE];
  volatile size_t rx_head_ = 0; // 写入位置（ISR）
  volatile size_t rx_tail_ = 0; // 读取位置（主循环）
};

#endif
