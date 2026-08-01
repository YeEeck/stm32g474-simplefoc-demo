# 03 监控输出已配置但永不发送（monitor() 从未调用）

Status: ready-for-agent

## 背景

架构评审（2026-08-01）发现。app.cpp:111-113 设置 `motor.useMonitoring(...)` + `monitor_variables`（提交 3146874 宣称"开启电流监控输出"），但 vendored SimpleFOC 全树无 `motor.monitor()` 调用点——`FOCMotor::monitor()` 是公开方法，须由主循环调用。onboard-validation.md 阶段 2/3/5 的验收全部依赖 monitor 输出，目前硬件上永远不会出现。

## 修复方案

- `app_loop()` 中 `commander.run(); motor.move();` 之后追加 `motor.monitor();`（monitor_downsample 默认 100 → 1ms 循环下每 100ms 一行）。

## SIL 覆盖

- 扩展 test 2（T0.5 收敛）：run_sim 后断言 UART 中出现 monitor 行（目标值 `0.5000` 打头 + 制表符分隔）。
- 修复前断言失败（无 monitor 输出），修复后通过。

## 验收

- SIL 全 PASS；固件交叉编译通过；上板串口每 ~100ms 一行 `target \t Vq \t Iq \t velocity \t angle`。
