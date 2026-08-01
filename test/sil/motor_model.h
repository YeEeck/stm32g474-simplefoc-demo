#ifndef SIL_MOTOR_MODEL_H
#define SIL_MOTOR_MODEL_H

#include <stdint.h>
#include <cmath>
#include "../../firmware/App/motor_config.h"

// 电机模拟：三相 RL 相模型 + 正弦反电动势 + 机械方程。
// 输入：三相电压（来自 PWM 占空比 × 母线电压）
// 输出：相电流（写入 IPROPI 模拟寄存器）、机械角（写入编码器计数）
class MotorModel {
public:
  // 电机参数（4015 24N22P，与本项目一致；与固件共享的物理真值来自电机配置）
  static constexpr float R_PHASE = motor_config::phase_resistance_ohm; // 相电阻 Ω
  static constexpr float L_PHASE = 1.3e-3f; // 相电感 H
  static constexpr int POLE_PAIRS = motor_config::pole_pairs;
  static constexpr float KE = 0.16f;        // 线-线反电动势常数 V/(rad/s)（近似用于相 EMF）
  static constexpr float KT = 0.23f;        // 扭矩常数 N·m/A
  static constexpr float J = 5e-4f;         // 转子+负载惯量 kg·m²（SIL 场景参数）
  static constexpr float B = 0.05f;         // 粘滞摩擦负载 N·m·s/rad（对齐 3V 下稳态 ~4 rad/s）
  static constexpr float CPR4 = (float)motor_config::encoder_cpr; // 编码器 4x 计数/圈

  // 母线电压、IPROPI 参考与增益（与固件配置一致；VREF=3.3V，GCSA=0.15 V/A，GAIN 接地）
  static constexpr float VBUS = motor_config::vbus_v;
  static constexpr float IPROPI_OFFSET_V = 1.65f; // = VREF/2
  static constexpr float IPROPI_GAIN = motor_config::ipropi_gain_v_per_a; // V/A（量程 ±11A）

  void reset();
  // 步进 dt 秒：读 PWM 占空比（经 sim 接口）→ 更新电流/机械/编码器/IPROPI
  void step(float dt, uint32_t pwm_arr, float ccr[3]);

  // 观测
  float current_a() const { return ia_; }
  float current_b() const { return ib_; }
  float current_c() const { return ic_; }
  float angle_mech() const { return theta_; } // rad
  float velocity() const { return omega_; }   // rad/s
  float electrical_angle() const { return theta_ * POLE_PAIRS; }
  // d/q 电流（与 SimpleFOC 同约定：q = ibeta·cos − ialpha·sin）
  float current_q() const {
    float theta_e = theta_ * POLE_PAIRS;
    float i_alpha = ia_;
    float i_beta = (ia_ + 2.0f * ib_) / 1.7320508f;
    return i_beta * cosf(theta_e) - i_alpha * sinf(theta_e);
  }

private:
  float ia_ = 0, ib_ = 0, ic_ = 0;
  float theta_ = 0; // 机械角 rad
  float omega_ = 0; // rad/s
  uint32_t cnt_ = 0;
};

#endif
