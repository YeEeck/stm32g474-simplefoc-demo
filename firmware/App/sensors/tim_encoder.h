#ifndef TIM_ENCODER_H
#define TIM_ENCODER_H

#include "main.h"
#include "../../vendors/simplefoc/src/common/base_classes/Sensor.h"
#include "../../vendors/simplefoc/src/common/foc_utils.h"

// MT6701 ABZ 编码器：TIM3 正交解码（4x，1024 线 → 4096 计数/圈）
// Z 索引（PB4 EXTI 下降沿）将计数校准到最近的整圈。
class TimEncoder : public Sensor {
public:
  TimEncoder(TIM_HandleTypeDef* htim) : htim_(htim) {}

  void init() override;
  // 电压对齐校准即可完成初始化，不强制 Z 搜索（基类 needsSearch 默认返回 0）

  // 由 HAL_GPIO_EXTI_Callback 调用（中断上下文）：Z 脉冲到达
  void onIndexPulse();

private:
  float getSensorAngle() override;

  TIM_HandleTypeDef* htim_;
  static constexpr uint32_t CPR = 4096; // 4x 解码计数
};

#endif
