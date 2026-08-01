# 04 rx_byte_ 非 volatile（跨中断/回调共享）

Status: resolved

## 背景

架构评审（2026-08-01）发现。`rx_byte_`（hardware_uart_stream.cpp:5）由 HAL 中断写入、`HAL_UART_RxCpltCallback`（app.cpp:48-52）读取，跨 TU 共享非 `volatile` 全局；开优化后可能被寄存器提升，SIL 的 `sim_inject_uart_bytes` 也直接写它。

## 修复方案

- `extern volatile uint8_t rx_byte_;` / 定义处加 `volatile`（hardware_uart_stream.h:7、cpp:4、app.cpp:50、hal_stubs.cpp:76 的 extern 声明同步）。

## SIL 覆盖

- 现有 SIL 串口注入测试即覆盖（编译期强制类型一致）。

## 验收

- 固件交叉编译通过；SIL 全 PASS。

- 2026-08-01 修复并合入：

## Comments

- 2026-08-01 已修复（5021984）：rx_byte_ 全链路补 volatile（含 SIL stub extern），HAL 调用处显式 (uint8_t*) 转型。
