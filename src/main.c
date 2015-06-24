#include <ch.h>
#include <hal.h>
#include <chprintf.h>
#include <string.h>
#include "main.h"

BaseSequentialStream *stdout = NULL;

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

static const CANConfig can1_config = {
    .mcr = (1 << 6)  /* Automatic bus-off management enabled. */
         | (1 << 2), /* Message are prioritized by order of arrival. */
    /* APB Clock is 48 Mhz
       Data rate: 48MHz / 3 / (1tq + 9tq + 6tq) = 1MHz => 1Mbit */
    .btr = ((3 - 1) << 0)  /* Baudrate prescaler (10 bits) */
         | ((9 - 1) << 16) /* Time segment 1 (4 bits) */
         | ((6 - 1) << 20) /* Time segment 2 (3 bits) */
         | (0 << 24) /* Resync jump width (2 bits) */
#if 1
         | (1 << 30) /* Loopback mode enabled */
#endif
};

void can_test(void) {
    chprintf(stdout, "can_test()\n");
    // CAN1 gpio init
    // iomode_t mode = PAL_STM32_MODE_ALTERNATE | PAL_STM32_OTYPE_PUSHPULL
    //     | PAL_STM32_OSPEED_HIGHEST | PAL_STM32_PUDR_FLOATING
    //     | PAL_STM32_ALTERNATE(4);
    // palSetPadMode(GPIOB, GPIOB_CAN_RX, mode); // RX
    // palSetPadMode(GPIOB, GPIOB_CAN_TX, mode); // TX
    canStart(&CAND1, &can1_config);
    chprintf(stdout, "canStart()\n");
    // todo: filter
    // canSTM32SetFilters(uint32_t can2sb, uint32_t num, const CANFilter *cfp);
    while (1) {
        chThdSleepMilliseconds(1000);
        CANTxFrame txf;
        txf.SID = 42;
        txf.IDE = 0;
        txf.RTR = 0;
        txf.DLC = 6;
        memcpy(&txf.data8[0], "CVRA!", 6);
        canTransmit(&CAND1, CAN_ANY_MAILBOX, &txf, MS2ST(100));

        CANRxFrame rxf;
        msg_t m = canReceive(&CAND1, CAN_ANY_MAILBOX, &rxf, TIME_IMMEDIATE);
        if (m != MSG_OK) {
            continue;
        }
        chprintf(stdout, "%x: %s\n", rxf.SID, &rxf.data8[0]);
    }
}

int main(void) {
    halInit();
    chSysInit();

    sdStart(&SD1, NULL);
    stdout = (BaseSequentialStream *)&SD1;
    chprintf(stdout, "\nstart\n");

    chThdCreateStatic(led_thread_wa, sizeof(led_thread_wa), NORMALPRIO, led_thread, NULL);

    can_test();
    while(1) {
        chThdSleepMilliseconds(100);
    }
}
