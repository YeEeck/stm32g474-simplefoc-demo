#include "motor_model.h"
#include "stm32g4xx_hal.h"

#include <math.h>

extern "C" {
void sim_set_phase_currents(float ia, float ib, float ic, float offset_v, float gain_v_per_a);
void sim_set_encoder_cnt(uint32_t cnt);
}

void MotorModel::reset() {
  ia_ = ib_ = ic_ = 0;
  // 初始转子角取非对齐值（真实上电时转子在任意位置；若与电压矢量同轴会零扭矩死锁）
  theta_ = 1.0f;
  omega_ = 0;
  cnt_ = (uint32_t)(theta_ / (2.0f * 3.14159265f) * CPR4) % (uint32_t)CPR4;
  sim_set_encoder_cnt(cnt_);
}

// 三相相电压（星形等效，中线不引出时用电压参考）
void MotorModel::step(float dt, uint32_t pwm_arr, float ccr[3]) {
  // 占空比 → 相对 GND 的输出电压
  float vabc[3];
  for (int i = 0; i < 3; i++) {
    float duty = (pwm_arr > 0) ? ccr[i] / (float)pwm_arr : 0.0f;
    vabc[i] = duty * VBUS;
  }
  // 星形无中线：共模电压不产生相电流（ia+ib+ic=0），需按中点参考计算相电压
  float vmid = (vabc[0] + vabc[1] + vabc[2]) / 3.0f;
  float va = vabc[0] - vmid;
  float vb = vabc[1] - vmid;
  float vc = vabc[2] - vmid;

  float theta_e = theta_ * POLE_PAIRS;
  // 线-线反电动势常数按相近似：E_ph = KE/sqrt(3) * omega * sin(theta_e - offset)
  float ke_ph = KE / 1.7320508f;
  float ea = ke_ph * omega_ * sinf(theta_e);
  float eb = ke_ph * omega_ * sinf(theta_e - 2.0943951f);
  float ec = ke_ph * omega_ * sinf(theta_e + 2.0943951f);

  // 欧拉积分（RL 相模型）
  float di_a = (va - R_PHASE * ia_ - ea) / L_PHASE;
  float di_b = (vb - R_PHASE * ib_ - eb) / L_PHASE;
  float di_c = (vc - R_PHASE * ic_ - ec) / L_PHASE;
  ia_ += di_a * dt;
  ib_ += di_b * dt;
  ic_ += di_c * dt;

  // 扭矩：d/q 电流（Park 变换）→ Te = 1.5 * KT * Iq
  // Clarke
  float i_alpha = ia_;
  float i_beta = (ia_ + 2.0f * ib_) / 1.7320508f;
  // Park
  float iq = i_beta * cosf(theta_e) - i_alpha * sinf(theta_e);
  float te = 1.5f * KT * iq;

  // 机械
  omega_ += (te - B * omega_) / J * dt;
  theta_ += omega_ * dt;
  if (theta_ > 2.0f * 3.14159265f) theta_ -= 2.0f * 3.14159265f;
  if (theta_ < 0.0f) theta_ += 2.0f * 3.14159265f;

  // 编码器（4x 计数，回绕）
  cnt_ = (uint32_t)(theta_ / (2.0f * 3.14159265f) * CPR4) % (uint32_t)CPR4;
  sim_set_encoder_cnt(cnt_);

  // IPROPI 输出（零电流电平 + 增益）
  sim_set_phase_currents(ia_, ib_, ic_, IPROPI_OFFSET_V, IPROPI_GAIN);
}
