# 07 上板验证文档的启动日志与监控格式与实现不符

Status: resolved

## 背景

架构评审（2026-08-01）发现。onboard-validation.md 阶段 1/4 预期 `MOT:Init`/`MOT:Enable driver.` 等启动日志，但：
1. 固件 Makefile 与 EIDE 工程均定义 `SIMPLEFOC_DISABLE_DEBUG` → SimpleFOCDebug 整体编译掉，任何 debug 输出都不可能出现；
2. 本 vendored SimpleFOC v3 的实际字符串无 `MOT:` 前缀（真实输出：`Align current sense.` / `Success: 1` / `Align sensor.` / `sensor dir: CW` / `Zero elec. angle: x` / `Ready.` / `Init FOC fail`）；
3. 文档称 monitor 的 `Iq` 单位为 mA，实际 SimpleFOC monitor 按 SI 打印（A，4 位小数）。

## 修复方案

- 固件启用 SimpleFOCDebug：两处构建移除 `SIMPLEFOC_DISABLE_DEBUG`，`app_init()` 在 `motor.init()` 前调用 `SimpleFOCDebug::enable(&HardwareUartStream::instance())`（排障时错误路径可打印 "Init FOC fail"）。
- onboard-validation.md：启动日志预期改为真实字符串；monitor 格式改为 `target \t Vq \t Iq(A) \t velocity \t angle`；排查表措辞同步。

## SIL 覆盖

- test 1（启动流程）追加断言：UART 出现 `Align sensor.` 与 `Ready.`（SIL 侧本就启用 debug，此断言固化启动日志契约）。

## 验收

- SIL 全 PASS；固件交叉编译通过且 .elf 含 "Ready." 字符串；上板启动日志与文档一致。

- 2026-08-01 修复并合入：

## Comments

- 2026-08-01 已修复（5021984 + 5895926）：固件两构建移除 SIMPLEFOC_DISABLE_DEBUG 并补编 SimpleFOCDebug.cpp，app_init 启用；文档改为真实启动日志（MOT: 前缀 + Success 语义）与 monitor 单位 A。
