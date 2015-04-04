#include <stdint.h>
#include <ch.h>
#include <hal.h>
#include "main.h"
#include "mpu60X0.h"
#include "sensors.h"

#define MPU_INT_MASK 0x01

int32_t mpu_gyro[3] = {0};
int32_t mpu_acc[3] = {0};
uint32_t mpu_temp = 0;

static THD_WORKING_AREA(mpu_thread_wa, 256);
static THD_FUNCTION(mpu_thread, arg) {

    chRegSetThreadName("MPU");
    I2CDriver *driver = (I2CDriver *)arg;
    static mpu60X0_t mpu6050;

    static event_listener_t mpu_int_listener;
    chEvtRegisterMaskWithFlags(&exti_events, &mpu_int_listener,
                               (eventmask_t)MPU_INT_MASK,
                               (eventflags_t)EXTI_EVENT_MPU6050_INT);

    chThdSleepMilliseconds(100);

    i2cAcquireBus(driver);
    mpu60X0_init_using_i2c(&mpu6050, driver, 0);

    if (!mpu60X0_ping(&mpu6050)) {
        i2cReleaseBus(driver);
        return 0;
    }

    mpu60X0_setup(&mpu6050, MPU60X0_SAMPLE_RATE_DIV(0) | MPU60X0_ACC_FULL_RANGE_2G
        | MPU60X0_GYRO_FULL_RANGE_500DPS | MPU60X0_LOW_PASS_FILTER_6);
    i2cReleaseBus(driver);

    while (1) {
        int32_t gyro[3], acc[3];
        uint32_t temp;
        chEvtWaitAnyTimeout(MPU_INT_MASK, OSAL_MS2ST(100));
        i2cAcquireBus(driver);
        mpu60X0_read_int(&mpu6050, gyro, acc, &temp);
        i2cReleaseBus(driver);

        chSysLock();
        mpu_gyro[0] = gyro[0];
        mpu_gyro[1] = gyro[1];
        mpu_gyro[2] = gyro[2];
        mpu_acc[0] = acc[0];
        mpu_acc[1] = acc[1];
        mpu_acc[2] = acc[2];
        mpu_temp = temp;
        chSysUnlock();

        chThdSleepMilliseconds(10);
    }
}

void mpu_start(void)
{
    /*
     * I2C setup
     */
    I2CDriver *driver = &I2CD1;
    static const I2CConfig i2c_cfg = {
        .cr1 = 0,
        .cr2 = 0,
        // took from reference manual rev7 p.640 example settings
        // PRESC 5, SCLDEL 0x3, SDADEL 0x3, SCLH 0x3, SCLL 0x3
        .timingr = (5<<28) | (0x3<<20) | (0x3<<16) | (0x3<<8) | (0x9<<0)
    };

    i2cStart(driver, &i2c_cfg);
    chThdCreateStatic(mpu_thread_wa, sizeof(mpu_thread_wa), NORMALPRIO + 1, mpu_thread, driver);
}
