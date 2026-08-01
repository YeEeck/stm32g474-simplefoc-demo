#ifndef TIM_ENCODER_H
#define TIM_ENCODER_H

#include "main.h"
#include "../../vendors/simplefoc/src/common/base_classes/Sensor.h"
#include "../../vendors/simplefoc/src/common/foc_utils.h"
#include "../motor_config.h"

// MT6701 ABZ 编码器：TIM3 正交解码（4x，1024 线 → 4096 计数/圈）
// Z 索引（PB4）已接出但当前对齐采用电压对齐（needsSearch=0），Z 不参与；若未来启用需重审零位基准。
class TimEncoder : public Sensor {
public:
  TimEncoder(TIM_HandleTypeDef* htim) : htim_(htim) {}

  void init() override;
  // 电压对齐校准即可完成初始化，不强制 Z 搜索（基类 needsSearch 默认返回 0）

private:
  float getSensorAngle() override;

  TIM_HandleTypeDef* htim_;
  static constexpr uint32_t CPR = motor_config::encoder_cpr; // 4x 解码计数
};

#endif
