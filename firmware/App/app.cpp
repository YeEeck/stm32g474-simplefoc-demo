#include "app.h"
#include "main.h"
#include "adc.h"
#include "tim.h"

// 按需引入 SimpleFOC 头（SimpleFOC.h 总入口会拉入 Arduino 库依赖如 SPI.h）
#include "BLDCMotor.h"
#include "drivers/BLDCDriver3PWM.h"
#include "communication/Commander.h"
#include "communication/SimpleFOCDebug.h"

#include "platform/hardware_uart_stream.h"
#include "drivers/drv8316ct.h"
#include "sensors/tim_encoder.h"
#include "sensors/iprop_current_sense.h"
#include "motor_config.h"

extern UART_HandleTypeDef huart2;

// ---- 全局对象（中断回调与主循环共享）----
// pin 参数在 HAL 适配层中不使用，占位即可
BLDCDriver3PWM driver = BLDCDriver3PWM(0, 1, 2);
TimEncoder encoder(&htim3);
// CSA 增益来自电机配置（GAIN 引脚接地 → 0.15 V/A，量程 ±11A @VREF=3.3V）
IpropCurrentSense current_sense(&hadc1, motor_config::ipropi_gain_v_per_a);
BLDCMotor motor = BLDCMotor(motor_config::pole_pairs, motor_config::phase_resistance_ohm, 54.0f);
Commander commander = Commander(HardwareUartStream::instance(), '\n', false);

// 零电流偏移校准完成前，禁止中断驱动电流环（避免以假读数驱动电机）
volatile bool foc_gate_open = false;

// 致命错误停机：停止 PWM 输出 + 驱动板 sleep（避免对齐电压常驻电机造成锁定电流/发热）
static void fatal_shutdown() {
  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
  drv8316ct::sleep();
}

// SimpleFOC Studio 兼容：M/Q/D/V/L/C 等命令
static void cmd_motor(char* cmd) { commander.motor(&motor, cmd); }
static void cmd_target(char* cmd) { commander.target(&motor, cmd); }
static void cmd_motion(char* cmd) { commander.motion(&motor, cmd); }

// ---- HAL 弱回调覆盖（中断上下文）----
extern "C" void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef* hadc) {
  if (hadc == &hadc1 && foc_gate_open) {
    motor.loopFOC(); // 20kHz 电流环：由 PWM 中心触发的采样中断驱动
  }
}

extern "C" void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart) {
  if (huart == &huart2) {
    HardwareUartStream::instance().onRxByte(rx_byte_);
  }
}

// ---- 启动流程 ----
void app_init(void) {
  // 1. DRV8316CT 唤醒（nSLEEP 拉高；VM 上电稳定预留 10ms）
  HAL_Delay(10);
  drv8316ct::wakeup();

  // 2. 串口（NVIC + 中断接收）；SimpleFOC 调试输出指向同一串口（排障用）
  HardwareUartStream::instance().init();
  SimpleFOCDebug::enable(&HardwareUartStream::instance());

  // 3. 各组件显式初始化（SimpleFOC 不自动调用）
  driver.init();        // PWM 启动（TRGO2 触发注入转换）
  current_sense.init(); // 注入转换开始（校准前 foc_gate 关闭，不会驱动电机）
  current_sense.calibrateOffsets(32); // 电机静止、零电流 → 偏移基准
  encoder.init();

  // 3. 电机配置：扭矩控制（电流环）
  driver.voltage_power_supply = motor_config::vbus_v;
  driver.pwm_frequency = 20000;

  motor.linkDriver(&driver);
  motor.linkSensor(&encoder);
  motor.linkCurrentSense(&current_sense);
  current_sense.linkDriver(&driver); // initFOC 的电流采样对齐需要知道驱动类型

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
    fatal_shutdown();
    while (1) {}
  }

  // 5. 电压对齐校准（电机会短暂转动/锁定）
  if (!motor.initFOC()) {
    HardwareUartStream::instance().println("initFOC failed");
    fatal_shutdown();
    while (1) {}
  }

  // 6. 使能 + 命令接口；校准已完成，开放中断驱动的电流环
  motor.enable();
  motor.target = 0;
  foc_gate_open = true;

  motor.useMonitoring(HardwareUartStream::instance());
  // 监控输出：target、Vq、Iq(mA)、速度、角度（便于上板验证）
  motor.monitor_variables = _MON_TARGET | _MON_VOLT_Q | _MON_CURR_Q | _MON_VEL | _MON_ANGLE;
  commander.add('M', cmd_motor, "motor");
  commander.add('T', cmd_target, "target");
  commander.add('C', cmd_motion, "motion control");

  HardwareUartStream::instance().println("simplefoc torque demo ready: T<current>A");
}

void app_loop(void) {
  commander.run();
  motor.move(); // torque 模式下将 target 同步到电流环给定（current_sp）
  motor.monitor(); // 每 monitor_downsample 次主循环输出一行监控（target/Vq/Iq/vel/angle）
}
