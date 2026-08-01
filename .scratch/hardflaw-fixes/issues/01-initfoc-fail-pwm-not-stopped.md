# 01 initFOC 失败路径未停止 PWM，电机持续锁定电流

Status: ready-for-agent

## 背景

架构评审（2026-08-01）发现。`driver.init()`（app.cpp:65）已启动 TIM1 PWM；电压对齐失败后 `while(1){}`（app.cpp:100-103）但从不停止 PWM / 失能电机：最后一次对齐电压（≤3V）常驻电机 → ~0.55A 锁定电流、发热，nSLEEP 保持拉高。design.md:142 要求"对齐失败→禁止使能 PWM"。

## 修复方案

- app.cpp 失败路径（`motor.init()` 与 `motor.initFOC()` 失败分支）：调用 `HAL_TIM_PWM_Stop`（三通道）+ `drv8316ct::sleep()`（nSLEEP 拉低，驱动全关）后再 `while(1)`。
- `drv8316ct` 补 `sleep()`（与 `wakeup()` 对称）。
- 硬件侧验证：无 SimpleFOC 钩子可停 TIM，需直接 HAL 调用。

## SIL 覆盖

- hal_stubs.cpp 记录 PWM 启动/停止状态（`sim_get_pwm_started()`），`HAL_TIM_PWM_Stop` stub 置 false。
- 扩展 test 7（对齐失败路径）：断言 `sim_get_pwm_started()==false` 且 `sim_get_nsleep()==GPIO_PIN_RESET`。

## 验收

- SIL 19+ 项全 PASS，新增断言在修复前失败、修复后通过。
- 固件交叉编译通过。
