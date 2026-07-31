# 0002 SimpleFOC v3 + CubeMX HAL + Makefile 技术栈

采用 SimpleFOC v3 + STM32CubeMX 生成 HAL 初始化代码 + Makefile + arm-none-eabi-gcc + J-Link 的构建路线。这是 SimpleFOC 官方对 STM32 的推荐路径：CubeMX 负责外设初始化，SimpleFOC 在 HAL 之上以寄存器方式使用外设。

放弃的方案：PlatformIO + STM32duino（Arduino 抽象层与 SimpleFOC 底层寄存器访问混杂，且自定义 IPROPI 采样需要精细控制 ADC 触发，Arduino 层碍事）；纯寄存器手写初始化（工作量最大，且 SimpleFOC 的 STM32 支持本就假设 HAL 已就绪）。

后果：CubeMX 工程（.ioc + 生成代码）是构建链的必需输入，由用户在本地生成后纳入仓库；构建机无需 CubeIDE，仅需 arm-none-eabi-gcc 与 Make。
