#include "hardware_uart_stream.h"

extern UART_HandleTypeDef huart2;
uint8_t rx_byte_;

bool HardwareUartStream::init() {
  // 启用 USART2 接收中断并启动单字节接收（CubeMX 未开 USART2 NVIC，这里补上）
  HAL_NVIC_SetPriority(USART2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(USART2_IRQn);
  return HAL_UART_Receive_IT(&huart2, &rx_byte_, 1) == HAL_OK;
}

void HardwareUartStream::onRxByte(uint8_t b) {
  size_t next = (rx_head_ + 1) % RX_BUF_SIZE;
  if (next != rx_tail_) { // 丢弃满缓冲时的覆盖
    rx_buf_[rx_head_] = b;
    rx_head_ = next;
  }
  HAL_UART_Receive_IT(&huart2, &rx_byte_, 1); // 继续接收
}

int HardwareUartStream::available() {
  return (int)((rx_head_ - rx_tail_ + RX_BUF_SIZE) % RX_BUF_SIZE);
}

int HardwareUartStream::read() {
  if (rx_head_ == rx_tail_) return -1;
  uint8_t b = rx_buf_[rx_tail_];
  rx_tail_ = (rx_tail_ + 1) % RX_BUF_SIZE;
  return (int)b;
}

size_t HardwareUartStream::write(uint8_t b) {
  return HAL_UART_Transmit(&huart2, &b, 1, 100) == HAL_OK ? 1 : 0;
}
