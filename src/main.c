#include <ch.h>
#include <hal.h>
#include <chprintf.h>
#include "usbcfg.h"
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


#define CAN_FRAME_STD_ID_MASK    ((1<<11) - 1)
#define CAN_FRAME_EXT_ID_MASK    ((1<<29) - 1)
#define CAN_FRAME_EXT_FLAG       (1<<29)
#define CAN_FRAME_RTR_FLAG       (1<<30)

/** id bit layout:
 * [0-28]:  29 bit extended id
 * ([0-10]:  11 bit basic id)
 * [29]:    extended frame flag
 * [30]:    remote transmission request flag
 * [31]:    unused, set to 0
 */

struct can_frame {
    uint32_t id;
    uint8_t dlc;
    union {
        uint8_t u8[8];
        uint16_t u16[4];
        uint32_t u32[2];
    } data;
};

#define CAN_RX_QUEUE_SIZE   8
#define CAN_TX_QUEUE_SIZE   8

memory_pool_t can_rx_pool;
mailbox_t can_rx_queue;
msg_t rx_mbox_buf[CAN_RX_QUEUE_SIZE];
struct can_frame rx_pool_buf[CAN_RX_QUEUE_SIZE];
// memory_pool_t can_tx_pool;
// mailbox_t can_tx_queue;
// msg_t tx_mbox_buf[CAN_TX_QUEUE_SIZE];
// struct can_frame tx_pool_buf[CAN_TX_QUEUE_SIZE];


// static THD_WORKING_AREA(can_tx_thread_wa, 256);
// static THD_FUNCTION(can_tx_thread, arg) {
//     (void)arg;
//     chRegSetThreadName("CAN tx");
//     while (1) {
//         struct can_frame *framep;
//         msg_t m = chMBFetch(&can_tx_queue, (msg_t *)&framep, TIME_INFINITE);
//         if (m != MSG_OK) {
//             continue;
//         }
//         CANTxFrame txf;
//         uint32_t id = framep->id;
//         txf.RTR = 0;
//         if (id & CAN_FRAME_EXT_FLAG) {
//             txf.EID = id & CAN_FRAME_EXT_ID_MASK;
//             txf.IDE = 1;
//         } else {
//             txf.SID = id & CAN_FRAME_STD_ID_MASK;
//             txf.IDE = 0;
//         }

//         if (id & CAN_FRAME_RTR_FLAG) {
//             txf.RTR = 1;
//         }

//         txf.DLC = framep->dlc;
//         txf.data32[0] = framep->data.u32[0];
//         txf.data32[1] = framep->data.u32[1];

//         chPoolFree(&can_tx_pool, framep);
//         canTransmit(&CAND1, CAN_ANY_MAILBOX, &txf, MS2ST(100));
//     }
//     return 0;
// }

static THD_WORKING_AREA(can_rx_thread_wa, 256);
static THD_FUNCTION(can_rx_thread, arg) {
    (void)arg;
    chRegSetThreadName("CAN rx");
    while (1) {
        uint32_t id;
        CANRxFrame rxf;
        msg_t m = canReceive(&CAND1, CAN_ANY_MAILBOX, &rxf, MS2ST(1000));
        if (m != MSG_OK) {
            continue;
        }
        if (rxf.IDE) {
            id = rxf.EID | CAN_FRAME_EXT_FLAG;
        } else {
            id = rxf.SID;
        }
        if (rxf.RTR) {
            id |= CAN_FRAME_RTR_FLAG;
        }
        // if (!can_bridge_id_passes_filter(id)) {
        //     continue;
        // }
        struct can_frame *f = (struct can_frame *)chPoolAlloc(&can_rx_pool);
        if (f == NULL) {
            continue;
        }
        f->id = id;
        f->dlc = rxf.DLC;
        f->data.u32[0] = rxf.data32[0];
        f->data.u32[1] = rxf.data32[1];
        if (chMBPost(&can_rx_queue, (msg_t)f, TIME_IMMEDIATE) != MSG_OK) {
            // couldn't post message: drop data & free the memory
            chPoolFree(&can_rx_pool, f);
        }
    }
    return 0;
}

void can_rx_buffer_flush(void)
{
    void *p;
    while (chMBFetch(&can_rx_queue, (msg_t *)&p, TIME_IMMEDIATE) == MSG_OK) {
        chPoolFree(&can_rx_pool, p);
    }
}

// void can_tx_buffer_flush(void)
// {
//     void *p;
//     while (chMBFetch(&can_tx_queue, (msg_t *)&p, TIME_IMMEDIATE) == MSG_OK) {
//         chPoolFree(&can_tx_pool, p);
//     }
// }


#define MAX_LINE_LEN (sizeof("T1111222281122334455667788EA5F\r")+1)

const char *hex4_table = "0123456789ABCDEF";
bool use_timestamps = false;

static char _hex4(const uint8_t b)
{
    return hex4_table[b & 0x0f];
}

void print_can_frame(BaseSequentialStream *out, struct can_frame *f)
{
    static unsigned char buf[MAX_LINE_LEN];
    unsigned char *p = &buf[0];
    int i;
    uint32_t id = f->id;
    bool extended, remote_frame;

    if (id & CAN_FRAME_RTR_FLAG) {
        remote_frame = true;
    } else {
        remote_frame = false;
    }

    if (id & CAN_FRAME_EXT_FLAG) {
        extended = true;
        id = id & CAN_FRAME_EXT_ID_MASK;
    } else {
        extended = false;
        id = id & CAN_FRAME_STD_ID_MASK;
    }

    // type
    if (remote_frame) {
        if (extended) {
            *p++ = 'R';
        } else {
            *p++ = 'r';
        }
    } else {
        if (extended) {
            *p++ = 'T';
        } else {
            *p++ = 't';
        }
    }

    // ID
    if (extended) {
        for (i = 3; i > 0; i--) {
            *p++ = _hex4(id>>(8*i + 4));
            *p++ = _hex4(id>>(8*i));
        }
    } else {
        *p++ = _hex4(id>>8);
        *p++ = _hex4(id>>4);
        *p++ = _hex4(id);
    }

    // DLC
    *p++ = _hex4(f->dlc);

    // data
    if (!remote_frame) {
        for (i = 0; i < f->dlc; i++) {
            *p++ = _hex4(f->data.u8[i]>>4);
            *p++ = _hex4(f->data.u8[i]);
        }
    }

    // timestamp
    if (use_timestamps) {
        uint16_t timestamp = 42;
        *p++ = _hex4(timestamp>>12);
        *p++ = _hex4(timestamp>>8);
        *p++ = _hex4(timestamp>>4);
        *p++ = _hex4(timestamp);
    }

    *p++ = '\r';

    size_t len = (size_t)p - (size_t)&buf[0];
    chSequentialStreamWrite(out, buf, len);
}

// reference: http://www.fischl.de/usbtin/
void execute_line(BaseSequentialStream *out, const char *line)
{
    switch (*line) {
    case 'T':
        // extended frame
        break;
    case 't':
        // standard frame
        break;
    case 'R':
        // extended remote frame
        break;
    case 'r':
        // standard remote frame
        break;
    case 'S':
        // set baud rate, {10k, 20k, 50k, 100k, 125k, 250k, 500k, 800k, 1M}
        break;
    case 'V':
        // hardware version
        break;
    case 'v':
        // firmware version
        break;
    case 'N':
        // serial number, read as 0xffff
        break;
    case 'O':
        // open CAN channel
        break;
    case 'l':
        // loop back mode
        break;
    case 'L':
        // silent mode (listen only)
        break;
    case 'C':
        // close CAN cahnnel
        break;
    case 'F':
        // read status byte
        break;
    case 'Z':
        // timestamp on/off, Zx[CR]
        if (line[1] == '1') {
            use_timestamps = true;
        } else {
            use_timestamps = false;
        }
        break;
    case 'm':
        // acceptance mask, mxxxxxxxx[CR]
        break;
    case 'M':
        // acceptance code, Mxxxxxxxx[CR]
        break;
    };
}

uint32_t hex_read(const char *s, int len)
{
    uint32_t x = 0;
    while (*s && len-- > 0) {
        if (*s >= '0' && *s <= '9') {
            x = (x << 4) | (*s - '0');
        } else if (*s >= 'a' && *s <= 'f') {
            x = (x << 4) | (*s - 'a' + 0x0a);
        } else if (*s >= 'A' && *s <= 'F') {
            x = (x << 4) | (*s - 'A' + 0x0A);
        } else {
            break;
        }
        s++;
    }
    return x;
}

static void candump(BaseSequentialStream *chp)
{
    while (1) {
        struct can_frame *framep;
        msg_t m = chMBFetch(&can_rx_queue, (msg_t *)&framep, MS2ST(100));
        if (m != MSG_OK) {
            continue;
        }
        print_can_frame(chp, framep);
        chSequentialStreamPut(&SDU1, '\n');
        chPoolFree(&can_rx_pool, framep);
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
#if 0
         | (1 << 30) /* Loopback mode enabled */
#endif
};

void can_init(void)
{
    // rx queue
    chMBObjectInit(&can_rx_queue, rx_mbox_buf, CAN_RX_QUEUE_SIZE);
    chPoolObjectInit(&can_rx_pool, sizeof(struct can_frame), NULL);
    chPoolLoadArray(&can_rx_pool, rx_pool_buf, sizeof(rx_pool_buf)/sizeof(struct can_frame));

    // // tx queue
    // chMBObjectInit(&can_tx_queue, tx_mbox_buf, CAN_TX_QUEUE_SIZE);
    // chPoolObjectInit(&can_tx_pool, sizeof(struct can_frame), NULL);
    // chPoolLoadArray(&can_tx_pool, tx_pool_buf, sizeof(tx_pool_buf)/sizeof(struct can_frame));

    canStart(&CAND1, &can1_config);
    // todo: filter
    // canSTM32SetFilters(uint32_t can2sb, uint32_t num, const CANFilter *cfp);
}

int main(void) {
    halInit();
    chSysInit();

    chThdCreateStatic(led_thread_wa, sizeof(led_thread_wa), NORMALPRIO, led_thread, NULL);

    //*
    usb_cdc_init();

    while (SDU1.config->usbp->state != USB_ACTIVE) {
        chThdSleepMilliseconds(10);
    }

    can_init();
    // chThdCreateStatic(can_tx_thread_wa, sizeof(can_tx_thread_wa), NORMALPRIO, can_tx_thread, NULL);
    chThdCreateStatic(can_rx_thread_wa, sizeof(can_rx_thread_wa), NORMALPRIO+1, can_rx_thread, NULL);

    while (1) {
       candump((BaseSequentialStream*)&SDU1);
    }
    /*/
    canStart(&CAND1, &can1_config);

    uint8_t count = 0;
    while (1) {
        chThdSleepMilliseconds(1);
        CANTxFrame txf;
        txf.SID = count;
        txf.IDE = 0;
        if (count > 127) {
            txf.EID = count | (1<<28);
            txf.IDE = 1;
        }
        txf.RTR = 0;
        txf.DLC = 8;
        memcpy(&txf.data8[0], "CVRA!  ", 8);
        txf.data8[6] = '0' + count++ % 10;
        canTransmit(&CAND1, CAN_ANY_MAILBOX, &txf, MS2ST(100));
    }
    //*/

    // shellInit();
    // static thread_t *shelltp = NULL;
    // static ShellConfig shell_cfg;
    // shell_cfg.sc_channel = (BaseSequentialStream*)&SDU1;
    // shell_cfg.sc_commands = shell_commands;
    // while (true) {
    //     if (!shelltp) {
    //         if (SDU1.config->usbp->state == USB_ACTIVE) {
    //             shelltp = shellCreate(&shell_cfg, THD_WORKING_AREA_SIZE(2048), NORMALPRIO);
    //         }
    //     } else if (chThdTerminatedX(shelltp)) {
    //         chThdRelease(shelltp);
    //         shelltp = NULL;
    //     }
    //     chThdSleepMilliseconds(500);
    // }
}
