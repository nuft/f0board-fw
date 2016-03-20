#ifndef EXIT_H
#define EXIT_H

#ifdef __cplusplus
extern "C" {
#endif

#include <hal.h>

#define EXTI_EVENT_MPU6050_INT (1<<0)
#define EXTI_EVENT_NRF_IRQ (1<<1)

extern event_source_t exti_events;

void exti_setup(void);

#ifdef __cplusplus
}
#endif

#endif /* EXIT_H */
