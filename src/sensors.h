#ifndef SENSORS_H
#define SENSORS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

extern int32_t mpu_gyro[3];
extern int32_t mpu_acc[3];
extern uint32_t mpu_temp;

void mpu_start(void);

#ifdef __cplusplus
}
#endif

#endif /* SENSORS_H */
