# 设计文档：STM32G474 扭矩控制 Demo（SimpleFOC + DRV8316CT）

## 1. 概述

### 1.1 目标

基于 NUCLEO-G474RE 与 DRV8316CT 驱动板，驱动 24N22P（11 极对）Delta 绕组 BLDC 电机，实现**基础电流环（扭矩控制）**。扭矩指令经串口 CLI（SimpleFOC Commander，兼容 SimpleFOC Studio）下发。优先使用现成库（SimpleFOC），FOC 运行频率 20kHz。

### 1.2 非目标

- 速度环、位置环、无感 FOC
- 弱磁、再生制动、能量管理
- DRV8316CT 故障诊断（nFAULT 未接出，无 SPI 状态读取）
- 上位机 GUI（除 SimpleFOC Studio 外）

### 1.3 关键约束

- 驱动板不可改（无外置采样电阻、无 SPI，配置全由板上引脚固定）
- 供电固定 24V（12–36V 范围）
- 扭矩指令单位：安培（电流），扭矩 = 电流 × 0.23 N·m/A

## 2. 硬件平台

### 2.1 组成

| 部件 | 型号 | 说明 |
|---|---|---|
| 主控 | 第三方最小系统板 | STM32G474VET6，170MHz，板上 8MHz 有源晶振，SWD 接出（DAP-Link 烧录） |
| 驱动板 | DRV8316CTRGFR | 引脚配置版（无 SPI），集成 FET 与电流检测，40V/8A 峰值 |
| 编码器 | MT6701 | 磁编码器，ABZ 模式 1024 线，OTP 默认配置，Z 索引已接出 |
| 电机 | 4015 24N22P | 11 极对，Delta 绕组，24V 额定 |

### 2.2 电机参数

| 参数 | 值 | 用途 |
|---|---|---|
| 极对数 | 11 | 电角 = 机械角 × 11 |
| 相电阻 | 5.4 Ω | SimpleFOC `phase_resistance`（开环/估算用） |
| 相电感 | 1.3 mH | 电流环带宽估算（L×BW 定 Kp 初值） |
| 反电动势常数 | 0.16 V/(rad/s) | 与 KV=54 自洽（换算约 60 RPM/V，含线-线/相约定差异） |
| 扭矩常数 | 0.23 N·m/A | 目标电流 → 扭矩换算 |
| 额定/堵转电流 | 1.4 A / 4.9 A | 调参安全上限 |
| 额定电压 | 24 V | `voltage_power_supply` |
| 额定转速 | 1000 RPM @24V | 电频率 ≈ 183 Hz |

### 2.3 引脚映射

| 功能 | 引脚 | 外设配置 |
|---|---|---|
| INH_A/B/C | PA8 / PA9 / PA10 | TIM1 CH1/CH2/CH3，PWM 20kHz 中心对齐 |
| SOA/SOB/SOC | PA0 / PA1 / PA3 | ADC1 IN1/IN2/IN4，TIM1 触发注入采样 |
| MT6701 A/B | PA6 / PA7 | TIM3 CH1/CH2 编码器模式（4x 解码） |
| MT6701 Z | PB4 | GPIO EXTI 中断输入（下降沿，内部上拉） |
| nSLEEP | PC7 | GPIO 推挽输出（拉高唤醒 DRV8316CT） |
| UART TX/RX | PD5 / PD6 | USART2，115200 8N1，外接 USB-TTL（DAP 虚拟串口可用） |

未使用：SPI（DRV8316CT 无 SPI）、nFAULT（未接出）、INLx（驱动板已接地）。

接线约束：编码器 A/B 建议使用屏蔽/双绞线，远离 PWM 线；SOx 走线远离开关节点。

## 3. 软件架构

### 3.1 分层

```
┌─────────────────────────────────────────────┐
│ 应用层：main + 启动流程 + Commander 接线      │
├─────────────────────────────────────────────┤
│ SimpleFOC v3：FOCMotor / BLDCDriver3PWM /    │
│              Encoder / CurrentSense 接口     │
├─────────────────────────────────────────────┤
│ 自定义适配层                                  │
│  ├── IpropCurrentSense（实现 CurrentSense）  │
│  └── drv8316ct 初始化（nSLEEP 时序）          │
├─────────────────────────────────────────────┤
│ HAL（CubeMX 生成）：TIM1/ADC1/TIM3/EXTI/     │
│                     USART2/GPIO/时钟          │
└─────────────────────────────────────────────┘
```

### 3.2 模块职责

| 模块 | 职责 |
|---|---|
| `main.c` | HAL 初始化 → DRV8316CT 唤醒 → SimpleFOC 配置 → `initFOC`（电压对齐）→ 主循环（Commander 轮询） |
| `IpropCurrentSense` | 实现 SimpleFOC `CurrentSense` 接口（`init/read/currentFromSensor`）；读取 ADC1 注入组三路电流；换算为相电流 |
| `drv8316ct` | nSLEEP 上电时序；无 SPI 配置，仅引脚驱动 |
| Commander | `T` 指令设置目标电流；`C` 读取电流/角度等变量（Studio 兼容） |

### 3.3 SimpleFOC 组件配置

| 组件 | 配置 |
|---|---|
| `BLDCDriver3PWM` | TIM1 三通道，PWM 频率 20kHz，`voltage_power_supply = 24` |
| `Encoder` | TIM3 编码器模式，PPR=1024（4x 解码 4096 计数），Z 引脚 PB4 中断索引对齐 |
| `FOCMotor` | `pole_pairs=11`，`phase_resistance=5.4`，`torque_controller=current`，`motion_control=torque`，`voltage_sensor_align` 默认 |

### 3.4 电流采样设计（IpropCurrentSense）

- **采样通道**：ADC1 注入组，IN1（SOA）/ IN2（SOB）/ IN4（SOC），12bit
- **触发**：TIM1 TRGO2 → ADC1 注入组，与 PWM 同步
- **采样时刻**：中心对齐模式中点（计数器上溢，占空比中央），采样时刻电流纹波最小
- **增益**：DRV8316CT 为引脚配置版，CSA 增益由 **GAIN 引脚**电阻决定（四电平）：

  | GAIN 引脚配置 | GCSA | 电流量程（VREF=3.3V，±1.65V 输出） |
  |---|---|---|
  | 接 AGND | 0.15 V/A | ±11.0 A |
  | Hi-Z（悬空） | 0.3 V/A | ±5.5 A |
  | 47kΩ ±5% 接 AVDD | 0.6 V/A | ±2.75 A |
  | 接 AVDD | 1.2 V/A | ±1.375 A |

  SOx 输出 = VREF/2 + I×GCSA；VREF 引脚由板上供电（2.8V~AVDD，外接 0.1µF）。
  **已确认：本驱动板 GAIN 引脚接地 → GCSA = 0.15 V/A**（量程 ±11A，电流分辨率约 5.4 mA/LSB，额定 1.4A 时约 260 LSB，满足扭矩控制精度）。
  固件 `IpropCurrentSense` 按 0.15 V/A 配置；上板后按 §7 步骤 3 实测标定复核。
- **信号特性**：IPROPI 输出为内部滤波后的电流信号，带宽有限。缓解措施：
  - 采样时刻对齐 PWM 中心，避免开关噪声
  - 电流环带宽预期 ≤ 500 Hz–1 kHz 起步（PID 初值见 §4），观察噪声再提升
- 采样校验：电机静止时三路输出应接近 0；转动时三路和为 0（i_a+i_b+i_c=0）检查接线/极性

## 4. 电流环参数初值

以电感为基准的带宽设计（Kp ≈ L·ω_bw，Ki ≈ R·ω_bw，SimpleFOC 单位：Kp [V/A]、Ki [V/A/s]）：

| 参数 | 初值 | 依据 |
|---|---|---|
| 电流环 Kp | 1.5 | 1.3 mH × 约 1.2 krad/s |
| 电流环 Ki | 5000 | 5.4 Ω × 约 0.9 krad/s |
| 带宽预期 | ~1 kHz | 受 IPROPI 滤波带宽限制，需现场验证 |

调参流程：先纯 Kp 小步试探（无超调、无高频振荡），再加 Ki 消除稳态误差；电流阶跃 0.2A 起步，观察 Iq 跟踪。限幅：电流环输出限幅为 ±12V（半母线），安全电流上限 ≤ 2 A 起步。

## 5. 启动与校准流程

1. 板上电：VM 24V → nSLEEP 保持低（芯片内部下拉，输出高阻，安全）
2. 等待 VM 稳定（≥ 10 ms）→ nSLEEP 拉高 → DRV8316CT 唤醒（t_wake 按 datasheet，约 100 µs–1 ms）
3. `motor.init()`：编码器初始化、PWM 输出使能（占空比 0）
4. `motor.initFOC()`：**电压对齐校准**——施加固定电压矢量锁转子，标定编码器方向与电角零位；依赖 Z 索引时每次上电以 Z 为机械零位基准
5. 进入主循环：Commander 轮询，默认目标电流 0，等待 `T` 指令

异常路径：对齐失败（编码器读数无变化）→ 打印错误，禁止使能 PWM。

## 6. 交互协议

Commander 命令（USART2，115200，PD5/PD6 → 外接 USB-TTL 或 DAP-Link 虚拟串口）：

| 命令 | 功能 | 示例 |
|---|---|---|
| `T<电流>` | 设置目标电流（A，torque 模式下 target 即电流） | `T0.5` |
| `M<子命令>` | 电机参数/状态：PID（`MQ`/`MD`）、限幅（`MLC`/`MLU`）、调制方式等 | `MLC2` |
| `C` | 运动控制类型（本项目固定 torque） | `C0` |
| `?` | 扫描已注册命令标签（SimpleFOC Studio 连接握手） | `?` |

SimpleFOC Studio 可直接连接串口，通过上述标准命令实时查看/设置电流环参数与目标电流。

## 7. 验证与调参计划（按序执行）

1. **上电基础**：串口打印正常；nSLEEP 拉高后驱动板无故障（无电流流过时 VM 电流 < 10 mA）
2. **编码器验证**：手动转动转子，角度连续变化、方向正确；Z 每圈触发一次
3. **IPROPI 标定**：静止零电流读 ADC 偏移（软件归零）；施加已知方向直流电压测换算增益，与 datasheet 名义值核对
4. **对齐验证**：initFOC 成功，电压对齐后角度误差 < 5° 电角
5. **电流环验证**：`T0.2` 阶跃，Studio 观察 Iq 跟踪与纹波；逐步提高 Kp/Ki 至无振荡
6. **限流验证**：堵转（手捏/夹具）下发 2A 指令，电流被环限制，无硬件跳闸

## 8. 风险与缓解

| 风险 | 影响 | 缓解 |
|---|---|---|
| IPROPI 带宽不足 | 电流环振荡/相位裕度差 | 采样时刻对齐；低带宽 PID 起步；验证步骤 5 逐步提速 |
| CSA 增益未知（板上固定） | 电流读数偏差 | 步骤 3 标定；文档记录最终增益 |
| 1024 线 / 11 极对不整除 | 电角量化约 1° | 对扭矩控制可接受（扭矩波动 < 2%） |
| 无 nFAULT/SPI 状态 | 故障不可见 | 依赖 OCP 硬件保护；异常时 PWM 停止、串口提示 |
| 对齐失败 | 上电即飞车 | initFOC 失败禁止使能（§5 异常路径） |

## 9. 参考资料

- TI DRV8316C datasheet（引脚配置版 DRV8316CT 的 nSLEEP 时序、CSA 增益、OCP）
- SimpleFOC v3 文档：STM32 移植指南、CurrentSense 接口、Encoder/Commander
- MT6701 手册：ABZ 模式 1024 线、Z 脉冲宽度
