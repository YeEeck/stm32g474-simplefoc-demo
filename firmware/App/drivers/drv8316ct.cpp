#include "drv8316ct.h"

namespace drv8316ct {

bool wakeup() {
  HAL_GPIO_WritePin(NSLEEP_GPIO_Port, NSLEEP_Pin, GPIO_PIN_SET);
  HAL_Delay(1); // t_WAKE 余量
  return true;
}

void sleep() {
  HAL_GPIO_WritePin(NSLEEP_GPIO_Port, NSLEEP_Pin, GPIO_PIN_RESET);
}

} // namespace drv8316ct
