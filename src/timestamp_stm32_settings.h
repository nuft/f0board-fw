#ifndef TIMESTAMP_STM32_SETTINGS_H
#define TIMESTAMP_STM32_SETTINGS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <ch.h>

// settings
#define TIMESTAMP_TIMER TIM2
#define TIMER_REG       STM32_TIM2
#define TIMER_IRQ_NAME  STM32_TIM2_HANDLER
#define RCC_EN()        rccEnableTIM2(FALSE)
#define RCC_RESET()     rccResetTIM2()
#define NVIC_NB         STM32_TIM2_NUMBER

#define COUNTER_MAX     0xffff

// CK_CNT = CK_INT / (PSC[15:0] + 1)
#define PRESCALER       (STM32_PCLK1/1000000 - 1)
#define INTERRUPT_PRIO  5

#ifdef __cplusplus
}
#endif

#endif /* TIMESTAMP_STM32_SETTINGS_H */
