#ifndef MAIN_H
#define MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include <ch.h>

#define EXTI_EVENT_MPU6050_INT (1<<0)
#define EXTI_EVENT_NRF_IRQ (1<<1)

extern event_source_t exti_events;

#ifdef __cplusplus
}
#endif

#endif /* MAIN_H */