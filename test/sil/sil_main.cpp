// SIL 预测试：在 host 上跑真实固件代码（SimpleFOC + App），接入电机模拟器。
// 仿真主循环：20kHz 步进（50us）：电机模型 → 模拟"注入转换完成中断" → 真实电流环。
// 主循环逻辑（commander/move）按 1ms 周期调用。

#include <cstdio>
#include <cstring>
#include <cmath>
#include <new>

#include "stm32g4xx_hal.h"
#include "adc.h"
#include "motor_model.h"
#include "BLDCMotor.h"
#include "../../firmware/App/app.h"
#include "../../firmware/App/motor_config.h"

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
void sim_freeze_encoder(bool freeze);
bool sim_get_pwm_started();
void sim_swap_phases(bool swap);
GPIO_PinState sim_get_nsleep();
void sim_set_phase_currents(float ia, float ib, float ic, float offset_v, float gain_v_per_a);
}

// ---- 固件回调（app.cpp 定义；Z 索引用例经 hal_stubs 弱默认回调注入）----
extern "C" void HAL_ADCEx_InjectedConvCpltCallback(void* hadc);
extern "C" void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);

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
  check(uart_contains("Align sensor."), "debug: Align sensor. printed");
  check(uart_contains("Ready."), "debug: Ready. printed");
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
  check(uart_contains("0.5000\t"), "monitor line emitted (target 0.5 + tab-separated)");
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

// 带采样回调的仿真：每 sample_ms 毫秒回调一次（观测动态过程）
template <typename Sampler>
static void run_sim_sampled(uint32_t ms, uint32_t sample_ms, Sampler&& sampler) {
  uint32_t steps = ms * 1000u / (uint32_t)(DT * 1e6f);
  uint32_t sample_interval = sample_ms * 1000u / (uint32_t)(DT * 1e6f);
  uint32_t loop_counter = 0;
  uint32_t sample_counter = 0;
  for (uint32_t i = 0; i < steps; i++) {
    sim_advance_time_us((uint32_t)(DT * 1e6f));
    float ccr[3];
    uint32_t arr = sim_get_pwm_arr();
    for (int ch = 0; ch < 3; ch++) ccr[ch] = sim_get_phase_voltage(ch + 1);
    model_motor.step(DT, arr, ccr);
    HAL_ADCEx_InjectedConvCpltCallback(&hadc1);
    if (++loop_counter >= LOOP_INTERVAL_US / (uint32_t)(DT * 1e6f)) {
      loop_counter = 0;
      app_loop();
    }
    if (++sample_counter >= sample_interval) {
      sample_counter = 0;
      sampler(i * (uint32_t)(DT * 1e6f) / 1000u, model_motor);
    }
  }
}

// ---- 测试 4：限流（current_limit=2A 生效；0.15 V/A 增益下量程 ±11A）----
static bool test_current_limit() {
  printf("[TEST] current limit (T3 > limit 2A)\n");
  model_motor.reset();
  sim_inject_uart_bytes("T3\n");
  run_sim(3000);
  float iq = model_motor.current_q();
  printf("        iq=%.3f A (target 3, limit 2)\n", iq);
  // 电流被限制（限流 2A 生效，远小于 3A 目标）
  check(iq < 2.2f && iq > 0.3f, "current is limited well below 3A target");
  check(fabsf(model_motor.current_q()) > 0.0f, "motor still producing torque");
  sim_inject_uart_bytes("T0\n");
  run_sim(500);
  return true;
}

// ---- 测试 5：动态响应（T0.5 阶跃的收敛时间与超调）----
static bool test_dynamic_response() {
  printf("[TEST] dynamic response (T0.5 step)\n");
  model_motor.reset();
  sim_inject_uart_bytes("T0.5\n");
  float settle_t = -1.0f, peak_iq = 0.0f, final_iq = 0.0f;
  int entered = 0;
  run_sim_sampled(200, 1, [&](uint32_t t_ms, const MotorModel& m) {
    float iq = m.current_q();
    if (iq > 0.2f && !entered) entered = 1;
    if (entered && iq > 0.45f && settle_t < 0) settle_t = (float)t_ms;
    if (fabsf(iq) > peak_iq) peak_iq = fabsf(iq);
    final_iq = iq;
  });
  printf("        settle@~%.0fms (|iq|>0.45), peak iq=%.3f, final iq=%.3f\n",
         settle_t, peak_iq, final_iq);
  check(settle_t > 0 && settle_t < 60.0f, "q current settles within 60ms");
  float overshoot = (peak_iq - 0.5f) / 0.5f;
  check(overshoot < 0.5f, "overshoot < 50%");
  sim_inject_uart_bytes("T0\n");
  run_sim(500);
  return true;
}

// ---- 测试 6：正反转（T-0.5 → 反向转动）----
static bool test_reverse_direction() {
  printf("[TEST] reverse direction (T-0.5)\n");
  model_motor.reset();
  sim_inject_uart_bytes("T-0.5\n");
  run_sim(3000);
  printf("        omega=%.2f rad/s, iq=%.3f A\n", model_motor.velocity(), model_motor.current_q());
  check(model_motor.velocity() < -2.0f, "motor rotates in reverse (omega<-2)");
  sim_inject_uart_bytes("T0\n");
  run_sim(500);
  return true;
}

// ---- 测试 7：编码器冻结 → 电压对齐失败，错误路径正确 ----
#include <pthread.h>
#include <unistd.h>

// 重建固件全局对象（模拟重新上电）
#include "drivers/BLDCDriver3PWM.h"
#include "drivers/drv8316ct.h"
#include "sensors/tim_encoder.h"
#include "sensors/iprop_current_sense.h"
#include "communication/Commander.h"
extern BLDCDriver3PWM driver;
extern TimEncoder encoder;
extern IpropCurrentSense current_sense;
extern Commander commander;
extern volatile bool foc_gate_open;
extern TIM_HandleTypeDef htim3;
static void firmware_reset() {
  new (&driver) BLDCDriver3PWM(0, 1, 2);
  new (&encoder) TimEncoder(&htim3);
  new (&current_sense) IpropCurrentSense(&hadc1, motor_config::ipropi_gain_v_per_a);
  new (&motor) BLDCMotor(11, 5.4f, 54.0f);
  new (&commander) Commander(HardwareUartStream::instance(), '\n', false);
  foc_gate_open = false;
}

static void* align_fail_thread(void*) {
  app_init(); // 预期：打印 "initFOC failed" 后进入 while(1) 死循环（固件错误路径）
  return nullptr;
}
static bool test_align_failure_path() {
  printf("[TEST] alignment failure path (encoder frozen)\n");
  firmware_reset(); // 重新上电语义：清除前序测试对全局状态的污染
  model_motor.reset();
  sim_freeze_encoder(true); // 传感器读数冻结 → 方向检测 moved≈0 → 对齐失败
  sim_clear_uart_out();
  pthread_t th;
  pthread_create(&th, nullptr, align_fail_thread, nullptr);
  // 等待错误输出（最多 5 秒真实时间）
  bool seen = false;
  for (int i = 0; i < 500; i++) {
    usleep(10000);
    if (uart_contains("initFOC failed")) { seen = true; break; }
  }
  check(seen, "error path prints 'initFOC failed' and does not enable motor");
  if (!seen) {
    size_t len = sim_get_uart_out_len();
    const uint8_t* buf = sim_get_uart_out();
    printf("        uart[%zu]: %s\n", len, (const char*)buf);
  }
  check(sim_get_nsleep() == GPIO_PIN_RESET, "driver put to sleep (nSLEEP low) after failure");
  check(!sim_get_pwm_started(), "PWM stopped after initFOC failure (no locked current)");
  sim_freeze_encoder(false);
  return true;
}

// ---- 测试 7b：Z 索引脉冲不扰动编码器角度（回归：Z 处理曾被删除）----
static bool test_z_index_noop() {
  printf("[TEST] Z index pulse does not disturb encoder angle\n");
  model_motor.reset();
  sim_clear_uart_out();
  sim_inject_uart_bytes("T0.5\n");
  run_sim(1500);
  uint32_t cnt_before = sim_get_encoder_cnt();
  HAL_GPIO_EXTI_Callback(GPIO_PIN_4); // 模拟 Z（PB4 EXTI）到达；固件无处理 → 弱默认 no-op
  run_sim(2);
  uint32_t cnt_after = sim_get_encoder_cnt();
  int32_t delta = (int32_t)(cnt_after - cnt_before);
  if (delta < 0) delta = -delta;
  printf("        cnt %u -> %u (delta %d)\n", cnt_before, cnt_after, delta);
  check(delta < 100, "counter continuous across Z pulse (no angle jump)");
  sim_inject_uart_bytes("T0\n");
  run_sim(500);
  return true;
}

// ---- 测试 7c：SOA/SOB 接线接反时 driverAlign 自校正，闭环仍收敛 ----
static bool test_phase_swap_self_correct() {
  printf("[TEST] swapped SOA/SOB wiring self-corrected by driverAlign\n");
  firmware_reset();
  model_motor.reset();
  sim_swap_phases(true); // SOA ↔ SOB 接反：JDR1 收到 B 相电流，JDR2 收到 A 相电流
  sim_clear_uart_out();
  app_init();            // driverAlign 应检测并交换 pinA/pinB 映射，不失败
  check(uart_contains("simplefoc torque demo ready"), "initFOC succeeds despite swapped phases");
  check(uart_contains("CS: Switch A-B"), "driverAlign reports phase swap");
  sim_clear_uart_out();
  sim_inject_uart_bytes("T0.5\n");
  run_sim(3000);
  check(fabsf(model_motor.current_q() - 0.5f) < 0.1f, "q current converges (mapping self-corrected)");
  sim_inject_uart_bytes("T0\n");
  run_sim(500);

  // 恢复接线并重新上电：driverAlign 把映射换回（pinA=0/pinB=1），后续测试获得干净状态
  sim_swap_phases(false);
  firmware_reset();
  model_motor.reset();
  sim_clear_uart_out();
  app_init();
  check(uart_contains("simplefoc torque demo ready"), "re-init with normal wiring restores mapping");
  sim_inject_uart_bytes("T0\n");
  run_sim(100);
  return true;
}

// ---- 测试 8：长时间稳定性（60s 仿真，无 NaN/无漂移）----
static bool test_long_run_stability() {
  printf("[TEST] long-run stability (60s sim)\n");
  model_motor.reset();
  sim_inject_uart_bytes("T0.5\n");
  float omega_end = 0;
  bool nan_detected = false;
  run_sim_sampled(60000, 1000, [&](uint32_t, const MotorModel& m) {
    if (!isfinite(m.velocity()) || !isfinite(m.current_a())) nan_detected = true;
  });
  omega_end = model_motor.velocity();
  printf("        omega=%.2f rad/s after 60s\n", omega_end);
  check(!nan_detected, "no NaN in state over 60s");
  check(fabsf(omega_end - 3.45f) < 0.3f, "steady-state omega stable (~3.45)");
  sim_inject_uart_bytes("T0\n");
  run_sim(500);
  return true;
}

int main() {
  setbuf(stdout, nullptr);
  printf("=== SIL pre-test (STM32G474 simplefoc torque demo) ===\n");

  // 模拟 CubeMX 已配置的外设初始值
  extern TIM_TypeDef tim1_regs;
  extern TIM_TypeDef tim3_regs;
  tim1_regs.ARR = 4249; // 20kHz 中心对齐
  tim3_regs.ARR = motor_config::encoder_cpr - 1; // 编码器 4x（0…CPR-1 回绕）
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
  test_current_limit();
  test_dynamic_response();
  test_reverse_direction();
  test_phase_swap_self_correct();
  test_z_index_noop();
  test_long_run_stability();
  test_align_failure_path(); // 放最后：app_init 卡在 while(1)（固件错误路径），由线程承载

  printf("=== result: %s (%d checks passed) ===\n", g_fail ? "FAIL" : "PASS", g_pass);
  return g_fail ? 1 : 0;
}
