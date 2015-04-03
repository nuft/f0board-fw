#include <ch.h>
#include <hal.h>
#include <stdint.h>
#include "nrf24l01p.h"
#include "nrf24l01p_registers.h"
#include "main.h"
#include "radio.h"

#define NRF_INTERRUPT_EVENT 1

static void nrf_ce_active(void)
{
    palSetPad(GPIOA, GPIOA_NRF_CE);
}

static void nrf_ce_inactive(void)
{
    palClearPad(GPIOA, GPIOA_NRF_CE);
}

void spi_init(void)
{
    /*
     * SPI1 configuration structure.
     * SPI1 is on APB1 @ 48MHz / 8 = 6MHz
     * CPHA=0, CPOL=0, 8bits frames, MSb transmitted first.
     */
    static SPIConfig spi_cfg = {
        .end_cb = NULL,
        .ssport = GPIOA,
        .sspad = GPIOA_SPI1_CS,
        .cr1 = SPI_CR1_BR_1
    };
    spiStart(&SPID1, &spi_cfg);
}

/* TX thread */

#define RADIO_TX_BUF_SIZE 2
memory_pool_t radio_tx_pool;
struct radio_packet radio_tx_pool_buf[RADIO_TX_BUF_SIZE];
mailbox_t radio_tx_mailbox;
msg_t radio_tx_mbox_buf[RADIO_TX_BUF_SIZE];

void radio_setup_tx(nrf24l01p_t *dev, uint8_t channel, const uint8_t *address)
{
    nrf_ce_inactive();
    // 2 byte CRC, enable TX_DS, RX_DR and MAX_RT IRQ
    uint8_t config = EN_CRC;
    nrf24l01p_write_register(dev, CONFIG, config);
    // frequency = 2400 + <channel> [MHz], maximum: 2525MHz
    nrf24l01p_set_channel(dev, channel);
    // 0dBm power, datarate 2M/1M/250K
    nrf24l01p_write_register(dev, RF_SETUP, RF_PWR(3) | RF_DR_250K);
    // disable retransmission, 1500us delay
    nrf24l01p_write_register(dev, SETUP_RETR, ARD(0) | ARC(0));
    // disable retransmission, 1500us delay
    nrf24l01p_write_register(dev, SETUP_RETR, 0);
    // disable dynamic packet length (DPL)
    nrf24l01p_write_register(dev, FEATURE, 0);
    // disable Enhanced ShockBurst Auto Acknowledgment
    nrf24l01p_write_register(dev, EN_AA, 0);
    // 3 byte address length
    nrf24l01p_write_register(dev, SETUP_AW, AW_3);
    // TX address
    nrf24l01p_set_addr(dev, TX_ADDR, address, 3);
    // clear data fifo
    nrf24l01p_flush_tx(dev);
    // clear IRQ flags
    nrf24l01p_write_register(dev, STATUS, RX_DR | TX_DS | MAX_RT);
    nrf24l01p_write_register(dev, CONFIG, config | PWR_UP);
}

static THD_WORKING_AREA(radio_tx_thread_wa, 256);
static THD_FUNCTION(radio_tx_thread, arg)
{
    nrf24l01p_t *nrf = (nrf24l01p_t *)arg;

    const uint8_t address[] = {0x2A, 0x2A, 0x2A};
    const uint8_t channel = 0;
    radio_setup_tx(nrf, channel, address);

    event_listener_t radio_event_listener;
    chEvtRegisterMaskWithFlags(&exti_events, &radio_event_listener,
        NRF_INTERRUPT_EVENT, EXTI_EVENT_NRF_IRQ);

    struct radio_packet *packet = NULL;
    while (1) {
        // clear interrupts
        nrf24l01p_write_register(nrf, STATUS, RX_DR | TX_DS | MAX_RT);

        msg_t m = chMBFetch(&radio_tx_mailbox, (msg_t *)&packet, TIME_IMMEDIATE);
        if (m == MSG_OK) {
            nrf24l01p_flush_tx(nrf);
            nrf24l01p_write_tx_payload(nrf, packet->data, 32);
            chPoolFree(&radio_tx_pool, packet);
        } else {
            nrf24l01p_reuse_tx_payload(nrf);
        }

        nrf_ce_active();
        chEvtWaitAnyTimeout(NRF_INTERRUPT_EVENT, MS2ST(1000));
        nrf_ce_inactive();
    }
    return 0;
}

void radio_tx_start(void)
{
    spi_init();

    chMBObjectInit(&radio_tx_mailbox, radio_tx_mbox_buf, RADIO_TX_BUF_SIZE);
    chPoolObjectInit(&radio_tx_pool, sizeof(struct radio_packet), NULL);
    chPoolLoadArray(&radio_tx_pool, radio_tx_pool_buf, RADIO_TX_BUF_SIZE);

    static nrf24l01p_t nrf24l01p;
    nrf24l01p_init(&nrf24l01p, &SPID1);

    chThdCreateStatic(radio_tx_thread_wa, sizeof(radio_tx_thread_wa), NORMALPRIO, radio_tx_thread, &nrf24l01p);
}

/* RX thread */

#define RADIO_RX_BUF_SIZE 4
memory_pool_t radio_rx_pool;
struct radio_packet radio_rx_pool_buf[RADIO_RX_BUF_SIZE];
mailbox_t radio_rx_mailbox;
msg_t radio_rx_mbox_buf[RADIO_RX_BUF_SIZE];

void radio_setup_rx(nrf24l01p_t *dev, uint8_t channel, const uint8_t *address)
{
    nrf_ce_inactive();
    // 2 byte CRC, enable RX_DR, mask MAX_RT and TX_DS IRQ
    nrf24l01p_write_register(dev, CONFIG, PRIM_RX | PWR_UP | EN_CRC | MASK_TX_DS | MASK_MAX_RT);
    // frequency = 2400 + <channel> [MHz], maximum: 2525MHz
    nrf24l01p_set_channel(dev, channel);
    // 0dBm power, datarate 2M/1M/250K
    nrf24l01p_write_register(dev, RF_SETUP, RF_PWR(3) | RF_DR_250K);
    // disable dynamic packet length (DPL)
    nrf24l01p_write_register(dev, FEATURE, 0);
    // 3 byte address length
    nrf24l01p_write_register(dev, SETUP_AW, AW_3);
    // RX address
    nrf24l01p_write_register(dev, EN_RXADDR, ERX_P0);
    nrf24l01p_set_addr(dev, RX_ADDR_P0, address, 3);
    nrf24l01p_write_register(dev, RX_PW_P0, 32);
    // disable Enhanced ShockBurst Auto Acknowledgment
    nrf24l01p_write_register(dev, EN_AA, 0);
    // clear data fifo
    nrf24l01p_flush_rx(dev);
    // clear IRQ flags
    nrf24l01p_write_register(dev, STATUS, RX_DR | TX_DS | MAX_RT);
}

static THD_WORKING_AREA(radio_rx_thread_wa, 256);
static THD_FUNCTION(radio_rx_thread, arg)
{
    nrf24l01p_t *nrf = (nrf24l01p_t *)arg;

    const uint8_t address[] = {0x2A, 0x2A, 0x2A};
    const uint8_t channel = 0;
    radio_setup_rx(nrf, channel, address);

    event_listener_t radio_event_listener;
    chEvtRegisterMaskWithFlags(&exti_events, &radio_event_listener,
        NRF_INTERRUPT_EVENT, EXTI_EVENT_NRF_IRQ);

    nrf_ce_active();

    struct radio_packet * packet;
    while (1) {
        chEvtWaitAnyTimeout(NRF_INTERRUPT_EVENT, MS2ST(100));

        uint8_t status = nrf24l01p_status(nrf);
        if (status & RX_DR) {
            // clear status flags
            nrf24l01p_write_register(nrf, STATUS, RX_DR | TX_DS | MAX_RT);

            uint8_t len = nrf24l01p_read_rx_payload_len(nrf);
            if (len != 32) { // invalid length
                nrf24l01p_flush_rx(nrf);
                continue;
            }

            packet = (struct radio_packet *) chPoolAlloc(&radio_rx_pool);
            if (packet == NULL) {
                // remove oldest packet from mailbox
                msg_t m = chMBFetch(&radio_rx_mailbox, (msg_t *)&packet, TIME_IMMEDIATE);
                if (m != MSG_OK) {
                    nrf24l01p_flush_rx(nrf);
                    continue;
                }
            }
            nrf24l01p_read_rx_payload(nrf, packet->data, len);
            msg_t m = chMBPost(&radio_rx_mailbox, (msg_t)packet, TIME_IMMEDIATE);
            if (m != MSG_OK) {
                chPoolFree(&radio_rx_pool, packet);
            }
        } else {
            // nrf24l01p_write_register(nrf, STATUS, RX_DR | TX_DS | MAX_RT);
            nrf24l01p_flush_rx(nrf);
        }
    }
    return 0;
}

void radio_rx_start(void)
{
    spi_init();

    chMBObjectInit(&radio_rx_mailbox, radio_rx_mbox_buf, RADIO_RX_BUF_SIZE);
    chPoolObjectInit(&radio_rx_pool, sizeof(struct radio_packet), NULL);
    chPoolLoadArray(&radio_rx_pool, radio_rx_pool_buf, RADIO_RX_BUF_SIZE);

    static nrf24l01p_t nrf24l01p;
    nrf24l01p_init(&nrf24l01p, &SPID1);

    chThdCreateStatic(radio_rx_thread_wa, sizeof(radio_rx_thread_wa), NORMALPRIO, radio_rx_thread, &nrf24l01p);
}

