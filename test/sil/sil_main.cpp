// SIL 预测试：在 host 上跑真实固件代码（SimpleFOC + App），接入电机模拟器。
// 仿真主循环：20kHz 步进（50us）：电机模型 → 模拟"注入转换完成中断" → 真实电流环。
// 主循环逻辑（commander/move）按 1ms 周期调用。

#include <cstdio>
#include <cstring>
#include <cmath>

#include "stm32g4xx_hal.h"
#include "adc.h"
#include "motor_model.h"
#include "BLDCMotor.h"
#include "../../firmware/App/app.h"

extern BLDCMotor motor; // app.cpp 全局对象

// ---- SIL 接口（hal_stubs.cpp）----
extern "C" {
void sim_set_delay_hook(void (*hook)(uint32_t ms));
uint32_t sim_get_time_us();
void sim_advance_time_us(uint32_t us);
void sim_clear_uart_out();
size_t sim_get_uart_out_len();
const uint8_t* sim_get_uart_out();
void sim_inject_uart_bytes(const char* s);
uint32_t sim_get_pwm_arr();
float sim_get_phase_voltage(int ch);
uint32_t sim_get_encoder_cnt();
GPIO_PinState sim_get_nsleep();
void sim_set_phase_currents(float ia, float ib, float ic, float offset_v, float gain_v_per_a);
}

// ---- 固件回调（app.cpp 定义）----
extern "C" void HAL_ADCEx_InjectedConvCpltCallback(void* hadc);

#include "communication/SimpleFOCDebug.h"
#include "hardware_uart_stream.h"

static MotorModel model_motor;

static constexpr float DT = 50e-6f;        // 20kHz
static constexpr uint32_t LOOP_INTERVAL_US = 1000; // 主循环 1ms

static bool g_fail = false;
static int g_pass = 0;

static void check(bool cond, const char* name) {
  if (cond) { g_pass++; printf("  [PASS] %s\n", name); }
  else { g_fail = true; printf("  [FAIL] %s\n", name); }
}

static bool uart_contains(const char* needle) {
  size_t len = sim_get_uart_out_len();
  const uint8_t* buf = sim_get_uart_out();
  size_t nl = strlen(needle);
  for (size_t i = 0; i + nl <= len; i++) {
    if (memcmp(buf + i, needle, nl) == 0) return true;
  }
  return false;
}

// HAL_Delay 期间推进电机模型（对齐/延时阶段电机仍受电压驱动）
static int delay_hook_logged = 0;
static void delay_hook(uint32_t ms) {
  float ccr[3];
  uint32_t arr = sim_get_pwm_arr();
  for (int i = 0; i < 3; i++) ccr[i] = sim_get_phase_voltage(i + 1);
  uint32_t n = ms * 1000u / (uint32_t)(DT * 1e6f);
  for (uint32_t i = 0; i < n; i++) model_motor.step(DT, arr, ccr);
  if (delay_hook_logged < 5 && (ccr[0] > 100 || ccr[1] > 100)) {
    printf("  [hook] ccr=(%u,%u,%u) omega=%.2f ia=%.3f cnt=%u\n",
           (unsigned)ccr[0], (unsigned)ccr[1], (unsigned)ccr[2],
           model_motor.velocity(), model_motor.current_a(), sim_get_encoder_cnt());
    delay_hook_logged++;
  }
}

// 运行仿真 ms 毫秒（20kHz 步进 + 1ms 主循环；每步推进仿真时钟）
static void run_sim(uint32_t ms) {
  uint32_t steps = ms * 1000u / (uint32_t)(DT * 1e6f);
  uint32_t loop_counter = 0;
  for (uint32_t i = 0; i < steps; i++) {
    sim_advance_time_us((uint32_t)(DT * 1e6f)); // 推进仿真时钟（PID/速度计算依赖）
    float ccr[3];
    uint32_t arr = sim_get_pwm_arr();
    for (int ch = 0; ch < 3; ch++) ccr[ch] = sim_get_phase_voltage(ch + 1);
    model_motor.step(DT, arr, ccr);
    HAL_ADCEx_InjectedConvCpltCallback(&hadc1); // 模拟注入转换完成中断 → 真实电流环
    if (++loop_counter >= LOOP_INTERVAL_US / (uint32_t)(DT * 1e6f)) {
      loop_counter = 0;
      app_loop(); // 真实主循环：commander + move
    }
  }
}

// ---- 测试 1：启动流程 ----
static bool test_startup() {
  printf("[TEST] startup flow\n");
  sim_clear_uart_out();
  app_init();
  check(sim_get_nsleep() == GPIO_PIN_SET, "nSLEEP pulled high (driver woken)");
  check(uart_contains("simplefoc torque demo ready"), "ready banner on uart");
  return true;
}

// ---- 测试 2：T 指令 → 电流环收敛 + 电机转动 ----
static bool test_torque_control() {
  printf("[TEST] torque control via 'T' command\n");
  model_motor.reset();
  sim_clear_uart_out();
  sim_inject_uart_bytes("T0.5\n");
  run_sim(3000); // 3 秒仿真

  float iq_err = model_motor.current_a(); // 定性检查：相电流应明显非零
  // 稳态 omega = Te/B = 1.5*Kt*iq/B（本模型参数下 iq=0.5 → 3.45 rad/s）
  check(model_motor.velocity() > 2.0f, "motor rotates (omega>2 rad/s)");
  check(iq_err != 0.0f && model_motor.velocity() > 0, "phase current nonzero while spinning");
  extern ADC_TypeDef adc1_regs;
  extern volatile bool foc_gate_open;
  printf("        omega=%.2f rad/s, model iq=%.3f A, fw sees iq=%.3f A (target 0.5)\n",
         model_motor.velocity(), model_motor.current_q(), motor.current.q);
  printf("        JDR=(%u,%u,%u) loopfoc_time=%u us, status=%d, gate=%d\n",
         adc1_regs.JDR1, adc1_regs.JDR2, adc1_regs.JDR3,
         motor.loopfoc_time_us, (int)motor.motor_status, (int)foc_gate_open);
  // 电流环闭环：固件反馈与模型真值一致，且接近目标 0.5A
  check(fabsf(model_motor.current_q() - 0.5f) < 0.1f, "q current converges to target (0.5A)");
  check(fabsf(motor.current.q - model_motor.current_q()) < 0.05f, "firmware feedback matches model");
  return true;
}

// ---- 测试 3：停止指令与协议 ----
static bool test_stop_and_protocol() {
  printf("[TEST] stop command and scan protocol\n");
  sim_clear_uart_out();
  sim_inject_uart_bytes("T0\n");
  run_sim(1500);
  // 电流环目标归零后相电流应回到零附近（静止）
  check(fabsf(model_motor.current_a()) < 0.3f, "phase current decays after T0");
  printf("        ia=%.3f A\n", model_motor.current_a());

  sim_clear_uart_out();
  sim_inject_uart_bytes("?\n");
  run_sim(100);
  check(uart_contains("motor"), "scan lists 'motor'");
  check(uart_contains("target"), "scan lists 'target'");
  check(uart_contains("motion"), "scan lists 'motion'");
  return true;
}

int main() {
  setbuf(stdout, nullptr);
  printf("=== SIL pre-test (STM32G474 simplefoc torque demo) ===\n");

  // 模拟 CubeMX 已配置的外设初始值
  extern TIM_TypeDef tim1_regs;
  extern TIM_TypeDef tim3_regs;
  tim1_regs.ARR = 4249; // 20kHz 中心对齐
  tim3_regs.ARR = 4095; // 编码器 4x
  tim1_regs.RCR = 0;
  // IPROPI 静止基准（零电流）
  sim_set_phase_currents(0.0f, 0.0f, 0.0f, MotorModel::IPROPI_OFFSET_V, MotorModel::IPROPI_GAIN);
  model_motor.reset(); // 初始转子角非对齐（模拟真实上电状态）

  // 打开 SimpleFOC 内部调试输出（对齐失败等错误会打印）
  SimpleFOCDebug::enable(&HardwareUartStream::instance());

  sim_set_delay_hook(delay_hook);

  test_startup();
  test_torque_control();
  test_stop_and_protocol();

  printf("=== result: %s (%d checks passed) ===\n", g_fail ? "FAIL" : "PASS", g_pass);
  return g_fail ? 1 : 0;
}
