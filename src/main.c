#include <ch.h>
#include <hal.h>
#include <chprintf.h>
#include "mpu60X0.h"
#include "shell_cmd.h"

BaseSequentialStream *stdout;

#define EXTI_EVENT_MPU6050_INT (1<<0)

event_source_t exti_events;

static void gpio_exti_callback(EXTDriver *extp, expchannel_t channel) {
    (void)extp;
    if (channel == GPIOB_MPU_INT) {
        chSysLockFromISR();
        chEvtBroadcastFlagsI(&exti_events, EXTI_EVENT_MPU6050_INT);
        chSysUnlockFromISR();
    }
}

static const EXTConfig extcfg = {{
    // GPIOB_MPU_INT, PB0
    {EXT_CH_MODE_RISING_EDGE | EXT_CH_MODE_AUTOSTART | EXT_MODE_GPIOB, gpio_exti_callback}, // 10
    {EXT_CH_MODE_DISABLED, NULL}, // 0
    {EXT_CH_MODE_DISABLED, NULL}, // 2
    {EXT_CH_MODE_DISABLED, NULL}, // 3
    {EXT_CH_MODE_DISABLED, NULL}, // 4
    {EXT_CH_MODE_DISABLED, NULL}, // 5
    {EXT_CH_MODE_DISABLED, NULL}, // 6
    {EXT_CH_MODE_DISABLED, NULL}, // 7
    {EXT_CH_MODE_DISABLED, NULL}, // 8
    {EXT_CH_MODE_DISABLED, NULL}, // 9
    {EXT_CH_MODE_DISABLED, NULL}, // 10
    {EXT_CH_MODE_DISABLED, NULL}, // 11
    {EXT_CH_MODE_DISABLED, NULL}, // 12
    {EXT_CH_MODE_DISABLED, NULL}, // 13
    {EXT_CH_MODE_DISABLED, NULL}, // 14
    {EXT_CH_MODE_DISABLED, NULL}, // 15
    {EXT_CH_MODE_DISABLED, NULL}, // 16
    {EXT_CH_MODE_DISABLED, NULL}, // 17
    {EXT_CH_MODE_DISABLED, NULL}, // 18
    {EXT_CH_MODE_DISABLED, NULL}, // 19
    {EXT_CH_MODE_DISABLED, NULL}, // 20
    {EXT_CH_MODE_DISABLED, NULL}, // 21
    {EXT_CH_MODE_DISABLED, NULL}  // 22
}};

void exti_setup(void)
{
    chEvtObjectInit(&exti_events);
    extStart(&EXTD1, &extcfg);
}

#define MPU_INT_MASK 0x01

int32_t mpu_gyro[3] = {0};
int32_t mpu_acc[3] = {0};
uint32_t mpu_temp = 0;

static THD_WORKING_AREA(mpu_thread_wa, 256);
static THD_FUNCTION(mpu_thread, arg) {

    palSetPad(GPIOB, GPIOB_LED);

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

    palClearPad(GPIOB, GPIOB_LED);

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
    chThdCreateStatic(mpu_thread_wa, sizeof(mpu_thread_wa), NORMALPRIO, mpu_thread, driver);
}

static THD_WORKING_AREA(led_thread_wa, 128);
static THD_FUNCTION(led_thread, arg) {
    (void)arg;
    chRegSetThreadName("LED");
    while (1) {
        palSetPad(GPIOB, GPIOB_LED);
        chThdSleepMilliseconds(80);
        palClearPad(GPIOB, GPIOB_LED);
        chThdSleepMilliseconds(80);
        palSetPad(GPIOB, GPIOB_LED);
        chThdSleepMilliseconds(80);
        palClearPad(GPIOB, GPIOB_LED);
        chThdSleepMilliseconds(760);
    }
    return 0;
}

void cmd_mpu(BaseSequentialStream *chp, int argc, char *argv[])
{
    (void) argc;
    (void) argv;
    int i;
    for (i = 0; i < 500; i++) {
        chprintf(chp, "gyro: %5d %5d %5d acc: %5d %5d %5d t: %u\n",
            mpu_gyro[0], mpu_gyro[1], mpu_gyro[2],
            mpu_acc[0], mpu_acc[1], mpu_acc[2],
            mpu_temp);
        chThdSleepMilliseconds(10);
    }
}

void cmd_reset(BaseSequentialStream *chp, int argc, char *argv[])
{
    (void) argc;
    (void) argv;
    NVIC_SystemReset();
    return;
}

static const ShellCommand commands[] = {
    {"mpu", cmd_mpu},
    {"reset", cmd_reset},
    {NULL, NULL}
};

int main(void) {
    halInit();
    chSysInit();

    sdStart(&SD1, NULL);
    stdout = (BaseSequentialStream*)&SD1;
    chprintf(stdout, "\n> start\n");

    chThdCreateStatic(led_thread_wa, sizeof(led_thread_wa), NORMALPRIO, led_thread, NULL);

    exti_setup();
    mpu_start();

    while (1) {
        chThdSleepMilliseconds(100);
        shell_spawn(stdout, commands);
    }
}
