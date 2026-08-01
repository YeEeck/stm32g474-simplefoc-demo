# 02 Z 索引处理破坏已校准电角零位（且逻辑恒等于 0）

Status: resolved

## 背景

架构评审（2026-08-01）发现，Standards 与 Spec 两轴独立命中。`tim_encoder.cpp:15-21` `onIndexPulse()` 把 TIM3 计数改写为"最近 CPR 倍数"，但计数范围 0..4095 下 `_round(cnt/CPR)*CPR % CPR` 恒等于 0——即每个 Z 脉冲把计数回写 0。`zero_electric_angle` 以校准时刻的计数为基准（FOCMotor.cpp:917），Z 后的基准平移 Δ∈[−2048,+2048] 计数 → 电角误差 = 11×Δ/4096×2π 可达 ±180°，首个 Z 即污染 d/q 轴，扭矩极性错误直至重新上电。

此外代码已明确选择电压对齐即可（tim_encoder.h:16 "不强制 Z 搜索，needsSearch 默认 0"），Z 校正与 `getSensorAngle` 的 `cnt % CPR` 角度读取重复实现同一概念。

## 修复方案

- 删除 `TimEncoder::onIndexPulse()`；删除 app.cpp 中 `HAL_GPIO_EXTI_Callback` 的 Z 分支与 `EXTI4` 中断使能（PB4 Z 输入保留接线，不启用中断）。
- 同步领域词：CONTEXT.md "ABZ 编码器"词条中"Z 用于上电角度对齐"改为"Z 索引已接出但当前对齐采用电压对齐，Z 不参与"；design.md §5.4 同步。

## SIL 覆盖

- 新增测试：仿真中直接调用 `HAL_GPIO_EXTI_Callback(Z_INDEX_Pin)`（模拟 Z 脉冲），断言电机角度连续（无跳变）、扭矩闭环不受影响。
- 修复前该测试失败（角度跳变 → 电流尖峰），修复后通过。

## 验收

- SIL 全 PASS；固件交叉编译通过。
- 上板行为：转动中过 Z 不再引起角度/扭矩扰动。

- 2026-08-01 修复并合入：

## Comments

- 2026-08-01 已修复（5021984）：删除 onIndexPulse() 与 app.cpp 的 EXTI4 接线/回调；hal_stubs 提供弱默认 HAL_GPIO_EXTI_Callback；SIL 新增 test_z_index_noop（注入 Z 脉冲断言计数连续）。CONTEXT.md/design.md 同步（Z 不参与对齐）。
