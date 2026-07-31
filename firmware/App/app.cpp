#include "app.h"
#include "main.h"
#include "adc.h"
#include "tim.h"

#include <stdlib.h>
// 按需引入 SimpleFOC 头（SimpleFOC.h 总入口会拉入 Arduino 库依赖如 SPI.h）
#include "BLDCMotor.h"
#include "drivers/BLDCDriver3PWM.h"
#include "communication/Commander.h"
#include "common/base_classes/CurrentSense.h"

#include "platform/hardware_uart_stream.h"
#include "drivers/drv8316ct.h"
#include "sensors/tim_encoder.h"
#include "sensors/iprop_current_sense.h"

extern UART_HandleTypeDef huart2;

// ---- 全局对象（中断回调与主循环共享）----
// pin 参数在 HAL 适配层中不使用，占位即可
BLDCDriver3PWM driver = BLDCDriver3PWM(0, 1, 2);
TimEncoder encoder(&htim3);
IpropCurrentSense current_sense(&hadc1, 1.45f); // gain 默认 1.45 V/A，调参时标定
BLDCMotor motor = BLDCMotor(11, 5.4f, 54.0f);
Commander commander = Commander(HardwareUartStream::instance(), '\n', false);

// SimpleFOC Studio 兼容：M/Q/D/V/L/C 等命令
static void cmd_motor(char* cmd) { commander.motor(&motor, cmd); }
static void cmd_target(char* cmd) { commander.target(&motor, cmd); }
static void cmd_motion(char* cmd) { commander.motion(&motor, cmd); }

// ---- HAL 弱回调覆盖（中断上下文）----
extern "C" void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef* hadc) {
  if (hadc == &hadc1) {
    motor.loopFOC(); // 20kHz 电流环：由 PWM 中心触发的采样中断驱动
  }
}

extern "C" void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  if (GPIO_Pin == Z_INDEX_Pin) {
    encoder.onIndexPulse();
  }
}

extern "C" void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart) {
  if (huart == &huart2) {
    HardwareUartStream::instance().onRxByte(rx_byte_);
  }
}

// ---- 启动流程 ----
void app_init(void) {
  // 1. DRV8316CT 唤醒（nSLEEP 拉高；VM 已由供电稳定）
  drv8316ct::wakeup();

  // 2. 各组件显式初始化（SimpleFOC 不自动调用；PWM 启动、注入转换使能）
  driver.init();
  encoder.init();
  current_sense.init();
  HAL_NVIC_SetPriority(EXTI4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  // 3. 电机配置：扭矩控制（电流环）
  driver.voltage_power_supply = 24.0f;
  driver.pwm_frequency = 20000;

  motor.linkDriver(&driver);
  motor.linkSensor(&encoder);
  motor.linkCurrentSense(&current_sense);

  motor.controller = MotionControlType::torque;
  motor.torque_controller = TorqueControlType::foc_current;
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;

  // 电流环 PID 初值（带宽 ~1kHz），调参见 docs/design.md §4
  motor.PID_current_q.P = 1.5f;
  motor.PID_current_q.I = 5000.0f;
  motor.PID_current_d.P = 1.5f;
  motor.PID_current_d.I = 5000.0f;

  motor.voltage_limit = 12.0f;
  motor.current_limit = 2.0f; // 安全起步限流，调参后按需提高

  // 4. 初始化硬件（PWM 启动）并校准零电流偏移
  if (!motor.init()) {
    HardwareUartStream::instance().println("motor init failed");
    while (1) {}
  }
  current_sense.calibrateOffsets(32);

  // 5. 电压对齐校准（电机会短暂转动/锁定）
  if (!motor.initFOC()) {
    HardwareUartStream::instance().println("initFOC failed");
    while (1) {}
  }

  // 6. 使能 + 命令接口
  motor.enable();
  motor.target = 0;

  motor.useMonitoring(HardwareUartStream::instance());
  commander.add('M', cmd_motor, "motor");
  commander.add('T', cmd_target, "target");
  commander.add('C', cmd_motion, "motion control");

  HardwareUartStream::instance().println("simplefoc torque demo ready: T<current>A");
}

void app_loop(void) {
  commander.run();
}
