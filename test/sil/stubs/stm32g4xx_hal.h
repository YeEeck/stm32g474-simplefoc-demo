#ifndef STM32G4XX_HAL_STUB_H
#define STM32G4XX_HAL_STUB_H
// SIL 测试用的 HAL stub：仅覆盖固件代码用到的类型/宏/函数。
// 真实 HAL 头（stm32g4xx_hal.h）在 ARM 交叉编译下才可用。

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// ---- 基础类型 ----
typedef enum { GPIO_PIN_RESET = 0, GPIO_PIN_SET = 1 } GPIO_PinState;
typedef uint16_t GPIO_TypeDef;

#define GPIO_PIN_0   ((uint16_t)0x0001)
#define GPIO_PIN_4   ((uint16_t)0x0010)
#define GPIO_PIN_7   ((uint16_t)0x0080)
#define GPIOB        ((GPIO_TypeDef)0)
#define GPIOC        ((GPIO_TypeDef)0)

typedef enum { HAL_OK = 0, HAL_ERROR = 1, HAL_BUSY = 2, HAL_TIMEOUT = 3 } HAL_StatusTypeDef;
typedef enum { DISABLE = 0, ENABLE = 1 } FunctionalState;

// ---- TIM / ADC 寄存器类型（仅 SIL 需要的字段，地址无意义）----
typedef struct {
  volatile uint32_t CR1;
  volatile uint32_t CR2;
  volatile uint32_t SMCR;
  volatile uint32_t DIER;
  volatile uint32_t SR;
  volatile uint32_t EGR;
  volatile uint32_t CCMR1;
  volatile uint32_t CCMR2;
  volatile uint32_t CCER;
  volatile uint32_t CNT;
  volatile uint32_t PSC;
  volatile uint32_t ARR;
  volatile uint32_t RCR;
  volatile uint32_t CCR1;
  volatile uint32_t CCR2;
  volatile uint32_t CCR3;
  volatile uint32_t CCR4;
} TIM_TypeDef;

typedef struct {
  volatile uint32_t ISR;
  volatile uint32_t IER;
  volatile uint32_t CR;
  volatile uint32_t CFGR;
  volatile uint32_t CFGR2;
  volatile uint32_t SMPR1;
  volatile uint32_t SMPR2;
  volatile uint32_t TR1;
  volatile uint32_t TR2;
  volatile uint32_t TR3;
  volatile uint32_t JDR1;
  volatile uint32_t JDR2;
  volatile uint32_t JDR3;
  volatile uint32_t JDR4;
} ADC_TypeDef;

typedef struct { TIM_TypeDef* Instance; } TIM_HandleTypeDef;
typedef struct { ADC_TypeDef* Instance; } ADC_HandleTypeDef;
typedef struct { uint32_t Instance; } UART_HandleTypeDef;

typedef struct { volatile uint32_t CTRL; volatile uint32_t CYCCNT; } DWT_TypeDef;
typedef struct { volatile uint32_t DEMCR; } CoreDebug_TypeDef;
extern DWT_TypeDef dwt_inst;
extern CoreDebug_TypeDef core_debug_inst;
#define DWT         (&dwt_inst)
#define CoreDebug   (&core_debug_inst)
#define CoreDebug_DEMCR_TRCENA_Msk (1UL << 24)
#define DWT_CTRL_CYCCNTENA_Msk     (1UL << 0)

// IRQ 编号（stub，仅占位）
#define EXTI4_IRQn   10
#define USART2_IRQn  38
#define ADC1_2_IRQn  18

extern uint32_t SystemCoreClock;

// ---- HAL 宏 ----
#define __HAL_TIM_GET_COUNTER(h)    ((h)->Instance->CNT)
#define __HAL_TIM_SET_COUNTER(h, c) ((h)->Instance->CNT = (c))
#define __HAL_TIM_GET_AUTORELOAD(h) ((h)->Instance->ARR)
#define __HAL_TIM_SET_COMPARE(h, ch, c) \
  (((ch) == TIM_CHANNEL_1) ? ((h)->Instance->CCR1 = (c)) : \
   ((ch) == TIM_CHANNEL_2) ? ((h)->Instance->CCR2 = (c)) : \
   ((h)->Instance->CCR3 = (c)))

#define HAL_MAX_DELAY 0xFFFFFFFFU
#define HAL_GetTick() 0UL

#define ADC_INJECTED_RANK_1 1UL
#define ADC_INJECTED_RANK_2 2UL
#define ADC_INJECTED_RANK_3 3UL

#define TIM_CHANNEL_1 0x0000U
#define TIM_CHANNEL_2 0x0004U
#define TIM_CHANNEL_3 0x0008U
#define TIM_CHANNEL_ALL 0xFFFFU

// ---- HAL 函数（stub 实现在 hal_stubs.cpp）----
HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef* htim, uint32_t Channel);
HAL_StatusTypeDef HAL_TIM_Encoder_Start(TIM_HandleTypeDef* htim, uint32_t Channel);
HAL_StatusTypeDef HAL_ADCEx_InjectedStart_IT(ADC_HandleTypeDef* hadc);
uint32_t HAL_ADCEx_InjectedGetValue(const ADC_HandleTypeDef* hadc, uint32_t InjectedRank);
void HAL_GPIO_WritePin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState);
void HAL_NVIC_SetPriority(uint32_t IRQn, uint32_t PreemptPriority, uint32_t SubPriority);
void HAL_NVIC_EnableIRQ(uint32_t IRQn);
void HAL_Delay(uint32_t Delay);
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef* huart, uint8_t* pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef* huart, uint8_t* pData, uint16_t Size);

#ifdef __cplusplus
}
#endif

#endif
