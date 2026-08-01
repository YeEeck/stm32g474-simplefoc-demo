#ifndef DRV8316CT_H
#define DRV8316CT_H

#include "main.h"
#include "gpio.h"

// DRV8316CT（引脚配置版）驱动接口。
// 配置全部由板上引脚固定（3PWM、CSA 增益），固件仅控制 nSLEEP 时序。

namespace drv8316ct {

// 拉高 nSLEEP 唤醒驱动板。调用前需保证 VM 已上电稳定（≥10ms）。
// 唤醒等待按 datasheet t_WAKE 预留 1ms 余量。
bool wakeup();

// 拉低 nSLEEP 进入 sleep（输出全关，无锁定电流）。用于致命错误路径停机。
void sleep();

} // namespace drv8316ct

#endif
