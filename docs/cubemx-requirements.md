# CubeMX 配置需求清单（第三方最小系统板 STM32G474VET6）

按此清单在 STM32CubeMX 中配置并生成 **Makefile 工程**（Toolchain: Makefile）。生成后把工程目录纳入本仓库（含 `.ioc` 文件，后续可回归）。

## 1. 时钟树

- HSE = 8 MHz（板上 8 MHz 有源晶振）
- SYSCLK = 170 MHz（G474 最高主频，经 PLL 倍频）
- ADC 时钟由 CubeMX 自动分配（ADC 高速时钟经 PLL，采样时钟 ≤ 60 MHz）

## 2. 引脚与外设

### 2.1 TIM1 — 3 路 PWM（PA8/PA9/PA10）

| 项 | 值 |
|---|---|
| 通道 | CH1=PA8, CH2=PA9, CH3=PA10，PWM Generation CH1/2/3 |
| 计数模式 | 中心对齐（Center-Aligned Mode 1） |
| 预分频 PSC | 0 |
| 自动重载 ARR | 4249（170 MHz 计数 → 20 kHz PWM：2×ARR=8500 计数/周期） |
| 初始脉冲 | 随意（SimpleFOC 运行时设置占空比） |
| 互补通道 | 不需要（3PWM，INLx 板上接地） |
| TRGO2 | 触发输出选择 Update 事件（或上溢/下溢，用于触发 ADC1 注入采样） |

### 2.2 ADC1 — 三路电流采样（PA0/PA1/PA4）

| 项 | 值 |
|---|---|
| 通道 | IN1（PA0）、IN2（PA1）、IN4（PA4） |
| 注入组（Injected Group） | 三个通道全部加入注入序列 |
| 触发源 | TIM1 TRGO2（外部触发） |
| 分辨率 | 12 bit |
| 采样时间 | 先短（如 2.5–8 cycles），调参时按需调整 |
| 注入转换完成中断 | 开启（SimpleFOC 电流环同步读值） |

### 2.3 TIM3 — 编码器接口（PA6/PA7）

| 项 | 值 |
|---|---|
| 模式 | Encoder Mode TI1 and TI2（正交解码，4x） |
| 通道 | CH1=PA6（A 相）、CH2=PA7（B 相） |
| 计数范围 | 自动回绕（0 … 4095，匹配 1024×4） |
| 输入滤波器 | 开启（默认值起步，抗振铃） |

### 2.4 GPIO

| 引脚 | 模式 | 说明 |
|---|---|---|
| PC7 | GPIO 输出，推挽，初始电平 Low | nSLEEP（上电保持低 → 代码拉高唤醒） |
| PB4 | GPIO 输入，EXTI 中断（下降沿或双沿，Z 极性以实测为准） | MT6701 Z 索引 |

### 2.5 USART2 — 调试/指令串口（PA2/PA3）

- 异步 115200，8N1，无流控
- 物理串口外接：USB-TTL 转接模块或 DAP-Link 虚拟串口（TX→RX 交叉接线）

### 2.6 明确不配置

- **SPI 全部不配置**（DRV8316CT 为引脚配置版，无 SPI 接口）
- nFAULT / SPOUT：未接出
- 其余外设默认关闭

## 3. 生成选项

- Toolchain/IDE：**Makefile**
- 生成所有初始化代码（HAL 库），不启用 FreeRTOS
- 工程目录建议：`firmware/`（生成后提交 `.ioc` 与核心初始化代码；HAL 库源码可 .gitignore）

## 4. 生成后核对

- `main.c` 中 `MX_*` 初始化函数存在且顺序正确
- TIM1 ARR 确认 4249、中心对齐
- ADC1 注入组触发源为 TIM1_TRGO2
- 编译一次确认工具链可用（`make` + `arm-none-eabi-gcc`）
