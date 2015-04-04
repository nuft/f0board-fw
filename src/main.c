#include <ch.h>
#include <hal.h>
#include <chprintf.h>
#include "mpu60X0.h"
#include "shell_cmd.h"
#include "sensors.h"
#include "main.h"

BaseSequentialStream *stdout;

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
    {EXT_CH_MODE_RISING_EDGE | EXT_CH_MODE_AUTOSTART | EXT_MODE_GPIOB, gpio_exti_callback},
    {EXT_CH_MODE_DISABLED, NULL}, // 1
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
    (void) chp;
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
