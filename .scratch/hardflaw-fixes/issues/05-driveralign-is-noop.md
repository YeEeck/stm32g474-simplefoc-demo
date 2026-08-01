# 05 电流采样相序自校正（driverAlign）为空操作

Status: resolved

## 背景

架构评审（2026-08-01）发现。`initFOC` 调用 `CurrentSense::driverAlign`（FOCMotor.cpp:833）期望通过交换 `pinA/pinB/pinC`、`offset_ia/b/c`、`gain_a/b/c` 纠正 SOx 相序/极性；但 `IpropCurrentSense` 硬编码 channel→注入 rank 映射、单一增益，基类字段从未被读取——SOx 接错时 initFOC 仍报"成功"，相位错配静默保留。SIL 接线 1:1 无法捕获。

## 修复方案

- `IpropCurrentSense` 委托基类字段：构造时 `pinA=0; pinB=1; pinC=2; gain_a=gain_b=gain_c=gain_v_per_a_`；`getPhaseCurrents()` 按 `pinA/B/C` 取通道、除以 `gain_a/b/c`。基类 `alignBLDCDriver` 随即生效（自愈相序/极性，返回 2/3/4）。

## SIL 覆盖

- 新增测试：stub 支持相序交换（`sim_swap_phases()` 交换 JDR 写入映射），交换接线下跑完整 `app_init` + T0.5 → 断言闭环仍收敛（自校正生效）、`Init FOC` 不失败。
- 修复前该测试失败（对齐后相序错 → Iq 不收敛），修复后通过。

## 验收

- SIL 全 PASS；固件交叉编译通过；上板 SOx 接错时 initFOC 输出 "CS: Switch A-B/A-C" 自愈。

- 2026-08-01 修复并合入：

## Comments

- 2026-08-01 已修复（5021984 + b68793a）：IpropCurrentSense 委托基类 pinA/B/C 与 gain_a/b/c，基类 alignBLDCDriver 生效；另修复上游 _swap(c_a.b, c_a.b) 笔误（b68793a）；SIL 新增 test_phase_swap_self_correct（交换接线 → 自愈 → T0.5 收敛）。
