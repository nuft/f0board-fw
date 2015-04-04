#include <ch.h>
#include <hal.h>
#include <chprintf.h>
#include "mpu60X0.h"
#include "shell_cmd.h"
#include "radio.h"
#include "main.h"
#include <string.h>

BaseSequentialStream *stdout;

event_source_t exti_events;

static void gpio_exti_callback(EXTDriver *extp, expchannel_t channel) {
    (void)extp;
    if (channel == GPIOB_MPU_INT) {
        chSysLockFromISR();
        chEvtBroadcastFlagsI(&exti_events, EXTI_EVENT_MPU6050_INT);
        chSysUnlockFromISR();
    } else if (channel == GPIOB_NRF_INT) {
        chSysLockFromISR();
        chEvtBroadcastFlagsI(&exti_events, EXTI_EVENT_NRF_IRQ);
        chSysUnlockFromISR();
    }
}

static const EXTConfig extcfg = {{
    // GPIOB_MPU_INT, PB0
    {EXT_CH_MODE_RISING_EDGE | EXT_CH_MODE_AUTOSTART | EXT_MODE_GPIOB, gpio_exti_callback},
    // GPIOB_NRF_INT, PB1
    {EXT_CH_MODE_FALLING_EDGE | EXT_CH_MODE_AUTOSTART | EXT_MODE_GPIOB, gpio_exti_callback},
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

void rx(void)
{
    radio_rx_start();

    struct radio_packet *pkt;
    while (1) {
        if (chMBFetch(&radio_rx_mailbox, (msg_t *)&pkt, MS2ST(1000)) != MSG_OK) {
            continue;
        }
        uint32_t st = pkt->data[0]
                    | (pkt->data[1] << 8)
                    | (pkt->data[2] << 16)
                    | (pkt->data[3] << 24);
        chprintf(stdout, "st: %u\n", st);
        chPoolFree(&radio_rx_pool, pkt);
    }
}

void tx(void)
{
    radio_tx_start();

    struct radio_packet *pkt;
    while (1) {
        pkt = chPoolAlloc(&radio_tx_pool);
        if (pkt == NULL) {
            chThdSleepMilliseconds(10);
            continue;
        }
        uint32_t st = chVTGetSystemTime();
        memcpy(pkt, &st, sizeof(st));
        if (chMBPost(&radio_tx_mailbox, (msg_t)pkt, TIME_IMMEDIATE) != MSG_OK) {
            chPoolFree(&radio_rx_pool, pkt);
        }
    }
}

int main(void) {
    halInit();
    chSysInit();

    sdStart(&SD1, NULL);
    stdout = (BaseSequentialStream*)&SD1;
    chprintf(stdout, "\n> start\n");

    chThdCreateStatic(led_thread_wa, sizeof(led_thread_wa), NORMALPRIO, led_thread, NULL);

    exti_setup();

    chprintf(stdout, "radio start\n");

    // rx();
    tx();

    while (1);
}
