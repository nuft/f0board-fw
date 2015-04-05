#include <ch.h>
#include <hal.h>
#include <chprintf.h>
#include "shell_cmd.h"
#include "usbcfg.h"

BaseSequentialStream *stdout;

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

void panic_hook(const char *reason) {
    (void)reason;
    while (1) {
        int i;
        for (i = 0; i < 500000; i++) {
            __asm__ volatile ("nop":::);
        }
        palTogglePad(GPIOB, GPIOB_LED);
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
    {"reset", cmd_reset},
    {NULL, NULL}
};

SerialUSBDriver SDU1;

static BaseSequentialStream *usb_cdc_init(void)
{
    // USB CDC
    sduObjectInit(&SDU1);
    sduStart(&SDU1, &serusbcfg);
    // usbDisconnectBus(serusbcfg.usbp);
    chThdSleepMilliseconds(1000);
    usbStart(serusbcfg.usbp, &usbcfg);
    // usbConnectBus(serusbcfg.usbp);
    while (SDU1.config->usbp->state != USB_ACTIVE) {
        chThdSleepMilliseconds(10);
    }
    return (BaseSequentialStream *) &SDU1;
}

int main(void) {
    halInit();
    chSysInit();

    sdStart(&SD1, NULL);
    stdout = (BaseSequentialStream*)&SD1;
    chprintf(stdout, "\n> start\n");

    chThdCreateStatic(led_thread_wa, sizeof(led_thread_wa), NORMALPRIO, led_thread, NULL);

    BaseSequentialStream *stream;
    stream = usb_cdc_init();

    while (1) {
        // shell_spawn(stdout, commands);
        chprintf(stream, "hello world\n");
        chThdSleepMilliseconds(1000);
    }
}
