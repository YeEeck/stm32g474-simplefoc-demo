#ifndef MOTOR_CONFIG_H
#define MOTOR_CONFIG_H

// 电机配置：电机物理参数与板级常量的唯一事实来源。
// 固件（app.cpp / 适配层）与 SIL 电机模型（test/sil）共同编译此头文件，
// 保证两端对同一电机持有同一组参数（上板实测标定也只改这里）。
namespace motor_config {

// 电机 4015 24N22P（Delta 绕组）
constexpr int pole_pairs = 11;
constexpr float phase_resistance_ohm = 5.4f;

// 供电
constexpr float vbus_v = 24.0f;

// IPROPI：DRV8316CT 内置电流检测增益（GAIN 引脚接地 → 0.15 V/A，量程 ±11A @VREF=3.3V）
constexpr float ipropi_gain_v_per_a = 0.15f;

// MT6701 ABZ 编码器：1024 线 × TIM3 正交解码 4x
constexpr unsigned encoder_cpr = 4096u;

} // namespace motor_config

#endif
