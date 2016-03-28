#include <ch.h>
#include <hal.h>
#include <chprintf.h>
#include "usbcfg.h"
#include <string.h>
#include <libcanard/src/canard.h>
#include <timestamp/timestamp.h>
#include <timestamp/timestamp_stm32.h>
#include "exti.h"
#include "mpu60X0.h"
#include "mpu60X0_registers.h"
#include "main.h"

BaseSequentialStream *stdout = NULL;

static inline int printf(const char *fmt, ...)
{
  va_list ap;
  int formatted_bytes;

  va_start(ap, fmt);
  formatted_bytes = chvprintf(stdout, fmt, ap);
  va_end(ap);

  return formatted_bytes;
}

void HardFault_Handler(void)
{
    chSysHalt("HardFault");
}

void panic_hook(const char *reason)
{
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
#if 0
         | (1 << 30) /* Loopback mode enabled */
#endif
};

void can_init(void)
{
    canStart(&CAND1, &can1_config);
    // todo: filter
    // canSTM32SetFilters(uint32_t can2sb, uint32_t num, const CANFilter *cfp);
}


#define CLEANUP_STALE_TRANSFERS 2000000

#define TIME_TO_SEND_NODE_STATUS 1000000
#define TIME_TO_SEND_MULTI 6000000
#define TIME_TO_SEND_REQUEST 5000000

static uint8_t uavcan_node_id;

int can_send(uint32_t extended_can_id, const uint8_t* frame_data, uint8_t frame_data_len)
{
    if (frame_data_len > 8 || frame_data == NULL) {
        return -1;
    }
    CANTxFrame txf;
    txf.EID = extended_can_id;
    txf.IDE = 1;
    txf.RTR = 0;
    txf.DLC = frame_data_len;
    memcpy(txf.data8, frame_data, frame_data_len);
    canTransmit(&CAND1, CAN_ANY_MAILBOX, &txf, MS2ST(100));
    return 0;
}

uint64_t get_monotonic_usec(void)
{
    return timestamp_get();
}

// / Arbitrary priority values
static const uint8_t PRIORITY_HIGHEST = 0;
static const uint8_t PRIORITY_HIGH    = 8;
static const uint8_t PRIORITY_MEDIUM  = 16;
static const uint8_t PRIORITY_LOW     = 24;
static const uint8_t PRIORITY_LOWEST  = 31;

// / Defined for the standard data type uavcan.protocol.NodeStatus
enum node_health
{
    HEALTH_OK       = 0,
    HEALTH_WARNING  = 1,
    HEALTH_ERROR    = 2,
    HEALTH_CRITICAL = 3
};

// / Defined for the standard data type uavcan.protocol.NodeStatus
enum node_mode
{
    MODE_OPERATIONAL     = 0,
    MODE_INITIALIZATION  = 1,
    MODE_MAINTENANCE     = 2,
    MODE_SOFTWARE_UPDATE = 3,
    MODE_OFFLINE         = 7
};

// / Standard data type: uavcan.protocol.NodeStatus
int publish_node_status(CanardInstance* ins, enum node_health health, enum node_mode mode,
                        uint16_t vendor_specific_status_code)
{
    static uint64_t startup_timestamp_usec = 0;
    if (startup_timestamp_usec == 0)
    {
        startup_timestamp_usec = get_monotonic_usec();
    }

    uint8_t payload[7];

    // Uptime in seconds
    const uint32_t uptime_sec = (get_monotonic_usec() - startup_timestamp_usec) / 1000000ULL;
    payload[0] = (uptime_sec >> 0)  & 0xFF;
    payload[1] = (uptime_sec >> 8)  & 0xFF;
    payload[2] = (uptime_sec >> 16) & 0xFF;
    payload[3] = (uptime_sec >> 24) & 0xFF;

    // Health and mode
    payload[4] = ((uint8_t)health << 6) | ((uint8_t)mode << 3);

    // Vendor-specific status code
    payload[5] = (vendor_specific_status_code >> 0) & 0xFF;
    payload[6] = (vendor_specific_status_code >> 8) & 0xFF;

    static const uint16_t data_type_id = 341;
    static uint8_t transfer_id;
    uint64_t data_type_signature = 0x8899AABBCCDDEEFF;
    return canardBroadcast(ins, data_type_signature,
                           data_type_id, &transfer_id, PRIORITY_LOW, payload, sizeof(payload));
}

/*
 * Float16 support
 */
uint16_t make_float16(float value)
{
    union fp32
    {
        uint32_t u;
        float f;
    };

    const union fp32 f32infty = { 255U << 23 };
    const union fp32 f16infty = { 31U << 23 };
    const union fp32 magic = { 15U << 23 };
    const uint32_t sign_mask = 0x80000000U;
    const uint32_t round_mask = ~0xFFFU;

    union fp32 in;

    uint16_t out = 0;

    in.f = value;

    uint32_t sign = in.u & sign_mask;
    in.u ^= sign;

    if (in.u >= f32infty.u)
    {
        out = (in.u > f32infty.u) ? 0x7FFFU : 0x7C00U;
    }
    else
    {
        in.u &= round_mask;
        in.f *= magic.f;
        in.u -= round_mask;
        if (in.u > f16infty.u)
        {
            in.u = f16infty.u;
        }
        out = (uint16_t)(in.u >> 13);
    }

    out |= (uint16_t)(sign >> 16);

    return out;
}

// / Standard data type: uavcan.equipment.multi
int publish_multi(CanardInstance* ins)
{
    static int len = 80;
    uint8_t payload[len];
    uint8_t i;
    for (i = 0; i<len; i++)
    {
        payload[i] = i + 1;
    }
    static const uint16_t data_type_id = 420;
    static uint8_t transfer_id;
    uint64_t data_type_signature = 0x8899AABBCCDDEEFF;
    return canardBroadcast(ins, data_type_signature,
                           data_type_id, &transfer_id, PRIORITY_HIGH, payload, sizeof(payload));
}

int publish_request(CanardInstance* ins)
{
    static int len = 43;
    uint8_t payload[len];
    uint8_t i;
    for (i = 0; i<len; i++)
    {
        payload[i] = i + 3;
    }
    uint8_t dest_id = 33;
    static const uint16_t data_type_id = 15;
    static uint8_t transfer_id;
    uint64_t data_type_signature = 0x8899AABBCCDDEEFF;
    return canardRequestOrRespond(ins, dest_id, data_type_signature,
                                  data_type_id, &transfer_id, PRIORITY_LOW, CanardRequest, payload, sizeof(payload));
}

void printframe(const CanardCANFrame* frame)
{
    printf("%X ", frame->id);
    printf("[%u] ", frame->data_len);
    int i;
    for (i = 0; i < frame->data_len; i++)
    {
        printf(" %X ", frame->data[i]);
    }
    printf("\n");
}

void on_reception(CanardInstance* ins, CanardRxTransfer* transfer)
{
    // printf("\n");
    printf("transfer type: ");
    switch (transfer->transfer_type)
    {
    case CanardTransferTypeResponse:
        printf("reponse\n");
        break;
    case CanardTransferTypeRequest:
        printf("request\n");
        break;
    case CanardTransferTypeBroadcast:
        printf("broadcast\n");
        break;
    default:
        break;
    }
    uint8_t payload[transfer->payload_len];
    memset(payload, 0, sizeof payload);
    if (transfer->payload_len > 7)
    {
        CanardBufferBlock* block = transfer->payload_middle;
        uint16_t i;
        uint8_t index = 0;
        if (CANARD_RX_PAYLOAD_HEAD_SIZE > 0)
        {
            for (i = 0; i < CANARD_RX_PAYLOAD_HEAD_SIZE; i++, index++)
            {
                payload[i] = transfer->payload_head[i];
            }
        }

        for (i = 0; index < (CANARD_RX_PAYLOAD_HEAD_SIZE + transfer->middle_len); i++, index++)
        {
            payload[index] = block->data[i];
            if (i==CANARD_BUFFER_BLOCK_DATA_SIZE - 1)
            {
                i = -1;
                block = block->next;
            }
        }

        uint16_t tail_len = transfer->payload_len - (CANARD_RX_PAYLOAD_HEAD_SIZE + transfer->middle_len);
        for (i = 0; i<(tail_len); i++, index++)
        {
            payload[index] = transfer->payload_tail[i];
        }
    }
    else
    {
        uint8_t i;
        for (i = 0; i<transfer->payload_len; i++)
        {
            payload[i] = transfer->payload_head[i];
        }
    }

    printf("payload:%016llu\n", canardReadRxTransferPayload(transfer, 0, 64));

    canardReleaseRxTransferPayload(ins, transfer);
    // do stuff with the data then call canardReleaseRxTransferPayload() if there are blocks (multi-frame transfers)

    size_t i;
    for (i = 0; i<sizeof(payload); i++)
    {
        printf("%02X ", payload[i]);
    }
    printf("\n");
}

bool should_accept(const CanardInstance* ins, uint64_t* out_data_type_signature,
                   uint16_t data_type_id, CanardTransferType transfer_type, uint8_t source_node_id)
{
    (void) ins;
    (void) data_type_id;
    (void) transfer_type;
    (void) source_node_id;
    *out_data_type_signature = 0x8899AABBCCDDEEFF;
    return true;
}

THD_WORKING_AREA(receive_thread, 512);
THD_FUNCTION(receive_thread_main, arg)
{
    CanardInstance *canard_instance = (CanardInstance *)arg;
    CanardCANFrame canard_frame;
    while (1) {
        CANRxFrame rxf;
        msg_t m = canReceive(&CAND1, CAN_ANY_MAILBOX, &rxf, MS2ST(1000));
        if (m != MSG_OK || !rxf.IDE || rxf.RTR) {
            continue;
        }
        canard_frame.id = rxf.EID;
        canard_frame.data_len = rxf.DLC;
        memcpy(canard_frame.data, &rxf.data8[0], rxf.DLC);
        canardHandleRxFrame(canard_instance, &canard_frame, get_monotonic_usec());
    }
}

bool imu_sample_update = false;
float imu_sample[6];
THD_WORKING_AREA(send_thread, 512);
THD_FUNCTION(send_thread_main, arg)
{
    CanardInstance *canard_instance = (CanardInstance *)arg;
    uint64_t last_node_status = 0;
    uint64_t last_clean = 0;
    while (1) {
        if ((get_monotonic_usec() - last_node_status) > TIME_TO_SEND_NODE_STATUS) {
            enum node_health health = HEALTH_OK;
            const uint16_t vendor_specific_status_code = rand(); // Can be used to report vendor-specific status info
            publish_node_status(canard_instance, health, MODE_OPERATIONAL, vendor_specific_status_code);
            last_node_status = get_monotonic_usec();
        }
        if (imu_sample_update) {
            static uint8_t payload[6*sizeof(float)];
            chSysLock();
            memcpy(payload, imu_sample, sizeof(imu_sample));
            imu_sample_update = false;
            chSysUnlock();
            static const uint16_t data_type_id = 20200;
            static uint8_t transfer_id;
            uint64_t data_type_signature = 0x15C4382BF7ED7695ULL;
            canardBroadcast(canard_instance, data_type_signature, data_type_id, &transfer_id, PRIORITY_HIGH, payload, sizeof(payload));
        }
        if ((get_monotonic_usec() - last_clean) > CLEANUP_STALE_TRANSFERS)
        {
            canardCleanupStaleTransfers(canard_instance, get_monotonic_usec());
            last_clean = get_monotonic_usec();
        }
        const CanardCANFrame* transmit_frame = canardPeekTxQueue(canard_instance);
        if (transmit_frame != NULL) {
            can_send(transmit_frame->id, transmit_frame->data, transmit_frame->data_len);
            canardPopTxQueue(canard_instance);
        } else {
            chThdSleepMilliseconds(1);
        }
    }
}

void fault_printf(const char *fmt, ...)
{
    (void) fmt;
}

#define EXTI_INTERRUPT_EVENT    1

static THD_WORKING_AREA(imu_thread, 256);
THD_FUNCTION(imu_thread_main, arg)
{
    (void) arg;
    palClearPad(GPIOA, GPIOA_MPU_PWR);
    chThdSleepMilliseconds(100); // power cycle
    palSetPad(GPIOA, GPIOA_MPU_PWR);
    chThdSleepMilliseconds(500);

    I2CDriver *i2c_driver = &I2CD1;
    static const I2CConfig i2c_cfg = {
        .cr1 = 0,
        .cr2 = 0,
        // took from reference manual rev7 p.640 example settings
        // PRESC 5, SCLDEL 0x3, SDADEL 0x3, SCLH 0x3, SCLL 0x3
        .timingr = (5<<28) | (0x3<<20) | (0x3<<16) | (0x3<<8) | (0x9<<0)
    };
    i2cStart(i2c_driver, &i2c_cfg);

    static mpu60X0_t mpu;
    mpu60X0_init_using_i2c(&mpu, i2c_driver, 0);

    i2cAcquireBus(i2c_driver);
    bool ok = false;
    if (mpu60X0_ping(&mpu)) {
        int config = MPU60X0_LOW_PASS_FILTER_6 | MPU60X0_SAMPLE_RATE_DIV(0);
        config |= MPU60X0_ACC_FULL_RANGE_2G;
        config |= MPU60X0_GYRO_FULL_RANGE_250DPS;
        mpu60X0_setup(&mpu, config);
        // check that the sensor still pings
        if (mpu60X0_ping(&mpu)) {
            ok = true;
        }
    }
    i2cReleaseBus(i2c_driver);

    if (!ok) {
        chSysHalt("mpu60x0 init failed");
    }
    printf("IMU initialized\n");

    static event_listener_t mpu_int_listener;
    chEvtRegisterMaskWithFlags(&exti_events, &mpu_int_listener,
                               (eventmask_t)EXTI_INTERRUPT_EVENT,
                               (eventflags_t)EXTI_EVENT_MPU6050_INT);

    while (1) {
        chEvtWaitAnyTimeout(EXTI_INTERRUPT_EVENT, OSAL_MS2ST(100));
        eventflags_t event_flag = chEvtGetAndClearFlags(&mpu_int_listener);
        if (event_flag & EXTI_EVENT_MPU6050_INT) {
            palSetPad(GPIOB, GPIOB_LED);

            static float gyro[3], acc[3], temp;
            i2cAcquireBus(i2c_driver);
            mpu60X0_read(&mpu, gyro, acc, &temp);
            i2cReleaseBus(i2c_driver);

            chSysLock();
                imu_sample[0] = gyro[0];
                imu_sample[1] = gyro[1];
                imu_sample[2] = gyro[2];
                imu_sample[3] = acc[0];
                imu_sample[4] = acc[1];
                imu_sample[5] = acc[2];
                imu_sample_update = true;
            chSysUnlock();
            // printf("IMU %d\n", (int)(1000*gyro[0]));

            palClearPad(GPIOB, GPIOB_LED);
        } else {
            chSysHalt("IMU timeout\n");
        }

        chThdSleepMilliseconds(10);
    }
}

int main(void) {
    halInit();
    chSysInit();
    timestamp_stm32_init();
    exti_setup();

    palSetPad(GPIOB, GPIOB_LED);

    // // USB CDC
    // sduObjectInit(&SDU1);
    // sduStart(&SDU1, &serusbcfg);

    // usbDisconnectBus(serusbcfg.usbp);
    // chThdSleepMilliseconds(1500);
    // usbStart(serusbcfg.usbp, &usbcfg);
    // usbConnectBus(serusbcfg.usbp);

    // while (SDU1.config->usbp->state != USB_ACTIVE) {
    //     chThdSleepMilliseconds(10);
    // }

    sdStart(&SD1, NULL);
    stdout = (BaseSequentialStream *) &SD1;

    chThdCreateStatic(imu_thread, sizeof(imu_thread), NORMALPRIO, imu_thread_main, NULL);

    can_init();
    static CanardInstance canard_instance;
    static CanardPoolAllocatorBlock buffer[16];           // pool blocks
    uavcan_node_id = 80;
    canardInit(&canard_instance, buffer, sizeof(buffer), on_reception, should_accept);
    canardSetLocalNodeID(&canard_instance, uavcan_node_id);
    printf("Initialized.\n");

    send_thread_main(&canard_instance);
    // chThdCreateStatic(send_thread, sizeof(send_thread), NORMALPRIO+1, send_thread_main, &canard_instance);
    // chThdCreateStatic(recevie_thread, sizeof(recevie_thread), NORMALPRIO, recevie_thread_main, &canard_instance);

    while (1) {
        chThdSleepMilliseconds(100);
    }
}
