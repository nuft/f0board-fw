#include <ch.h>
#include <hal.h>
#include <chprintf.h>
#include <shell.h>
#include "shell_cmd.h"
#include "usbcfg.h"
#include "main.h"

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

void usb_cdc_init(void)
{
    // USB CDC
    sduObjectInit(&SDU1);
    sduStart(&SDU1, &serusbcfg);

    usbDisconnectBus(serusbcfg.usbp);
    chThdSleepMilliseconds(1500);
    usbStart(serusbcfg.usbp, &usbcfg);
    usbConnectBus(serusbcfg.usbp);
}

int main(void) {
    halInit();
    chSysInit();

    chThdCreateStatic(led_thread_wa, sizeof(led_thread_wa), NORMALPRIO, led_thread, NULL);

    usb_cdc_init();

    shellInit();
    static thread_t *shelltp = NULL;
    static ShellConfig shell_cfg;
    shell_cfg.sc_channel = (BaseSequentialStream*)&SDU1;
    shell_cfg.sc_commands = shell_commands;
    while (true) {
        if (!shelltp) {
            if (SDU1.config->usbp->state == USB_ACTIVE) {
                shelltp = shellCreate(&shell_cfg, THD_WORKING_AREA_SIZE(2048), NORMALPRIO);
            }
        } else if (chThdTerminatedX(shelltp)) {
            chThdRelease(shelltp);
            shelltp = NULL;
        }
        chThdSleepMilliseconds(500);
    }
}
