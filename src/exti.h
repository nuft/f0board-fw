#ifndef EXIT_H
#define EXIT_H

#ifdef __cplusplus
extern "C" {
#endif

#include <hal.h>

#define EXTI_EVENT_NRF_IRQ (1<<0)

extern event_source_t exti_events;

#ifdef __cplusplus
}
#endif

#endif /* EXIT_H */
