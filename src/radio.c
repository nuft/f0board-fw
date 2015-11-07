#include <ch.h>
#include <hal.h>
#include <stdint.h>
#include <string.h>
#include "nrf24l01p.h"
#include "nrf24l01p_registers.h"
#include "radio.h"
#include "exti.h"
#include <chprintf.h>
#include <serial-datagram/serial_datagram.h>
#include <datagram_packer.h>

#define NRF_INTERRUPT_EVENT 1

const uint8_t address[] = {0x2A, 0x2A, 0x2A};
const uint8_t channel = 42;
const uint8_t datarate = RF_DR_250K;

event_listener_t radio_event_listener;

static nrf24l01p_t nrf24;
static THD_WORKING_AREA(radio_thread_wa, 512);
uint8_t datagram_buffer[100];

SPIDriver *spi_init(void);
void nrf_ce_active(void);
void nrf_ce_inactive(void);

extern BaseSequentialStream *stdout;

static void nrf_setup_ptx(nrf24l01p_t *dev)
{
    SPIDriver *spi = spi_init();
    nrf24l01p_init(dev, spi);

    nrf_ce_inactive();
    // 2 byte CRC, enable TX_DS, RX_DR and MAX_RT IRQ
    uint8_t config = EN_CRC;
    nrf24l01p_write_register(dev, CONFIG, config);
    // frequency = 2400 + <channel> [MHz], maximum: 2525MHz
    nrf24l01p_set_channel(dev, channel);
    // 0dBm power, datarate 2M/1M/250K
    nrf24l01p_write_register(dev, RF_SETUP, RF_PWR(3) | datarate);
    // Disable retransmission, 1500us delay
    nrf24l01p_write_register(dev, SETUP_RETR, ARD(5) | ARC(1));
    // enable dynamic packet length (DPL)
    nrf24l01p_write_register(dev, FEATURE, EN_DPL | EN_ACK_PAY);
    // enable Enhanced ShockBurst Auto Acknowledgment
    nrf24l01p_write_register(dev, EN_AA, ENAA_P0);
    // 3 byte address length
    nrf24l01p_write_register(dev, SETUP_AW, AW_3);
    // TX address
    nrf24l01p_set_addr(dev, TX_ADDR, address, 3);
    // RX address
    nrf24l01p_set_addr(dev, RX_ADDR_P0, address, 3);
    nrf24l01p_write_register(dev, DYNPD, DPL_P0);
    // clear data fifo
    nrf24l01p_flush_tx(dev);
    nrf24l01p_flush_rx(dev);
    // clear IRQ flags
    nrf24l01p_write_register(dev, STATUS, RX_DR | TX_DS | MAX_RT);
    nrf24l01p_write_register(dev, CONFIG, config | PWR_UP);

    chEvtRegisterMaskWithFlags(&exti_events, &radio_event_listener,
        NRF_INTERRUPT_EVENT, EXTI_EVENT_NRF_IRQ);
}


void radio_send_datagram(const void *datagram, size_t size, void *arg)
{
    nrf24l01p_t *nrf = (nrf24l01p_t *)arg;
    datagram_sender_t sender;
    dp_sender_init(&sender, datagram, size);
    bool done;
    do {
        size_t len;
        static uint8_t packet[32];
        done = datagram_packer_next_packet(&sender, packet, 32, &len);
        // clear interrupts
        nrf24l01p_write_register(nrf, STATUS, RX_DR | TX_DS | MAX_RT);
        nrf24l01p_write_tx_payload(nrf, packet, len);
        nrf_ce_active();
        eventmask_t ret = chEvtWaitAnyTimeout(NRF_INTERRUPT_EVENT, MS2ST(100));
        nrf_ce_inactive();
        if (ret == 0) {
            nrf24l01p_flush_tx(nrf);
            continue;
        }
        uint8_t status = nrf24l01p_status(nrf);
        if (status & RX_DR) {
            len = nrf24l01p_read_rx_payload_len(nrf);
            if (len == 0 || len > 32) { // invalid length
                nrf24l01p_flush_rx(nrf);
                continue;
            }
            nrf24l01p_read_rx_payload(nrf, packet, len);
            // discard data
        } else if (status & MAX_RT) {
            nrf24l01p_flush_tx(nrf);
            nrf24l01p_flush_rx(nrf);
        }
    } while (!done);
}

static THD_FUNCTION(radio_thread_tx, arg)
{
    nrf24l01p_t *nrf = &nrf24;
    nrf_setup_ptx(nrf);
    BaseChannel *in = (BaseChannel *)arg;
    serial_datagram_rcv_handler_t sd;
    serial_datagram_rcv_handler_init(&sd, datagram_buffer, sizeof(datagram_buffer),
        radio_send_datagram, nrf);
    static uint8_t buf[50];

    // masgpack string
    // uint8_t dg[] = {0xab,'h','e','l','l','o',' ','w','o','r','l','d'};
    while (1) {
        // radio_send_datagram(dg, sizeof(dg), nrf);
        // chThdSleepMilliseconds(100);
        palTogglePad(GPIOB, GPIOB_LED);
        size_t len = chnReadTimeout(in, buf, sizeof(buf), MS2ST(10));
        if (len > 0) {
            serial_datagram_receive(&sd, buf, len);
        }
    }
}

static void nrf_setup_prx(nrf24l01p_t *dev)
{
    SPIDriver *spi = spi_init();
    nrf24l01p_init(dev, spi);

    nrf_ce_inactive();
    // 2 byte CRC, enable RX_DR, mask MAX_RT and TX_DS IRQ
    uint8_t config = PRIM_RX | PWR_UP | EN_CRC | MASK_TX_DS | MASK_MAX_RT;
    nrf24l01p_write_register(dev, CONFIG, config);
    // frequency = 2400 + <channel> [MHz], maximum: 2525MHz
    nrf24l01p_set_channel(dev, channel);
    // 0dBm power, datarate 2M/1M/250K
    nrf24l01p_write_register(dev, RF_SETUP, RF_PWR(3) | datarate);
    // enable dynamic packet length (DPL)
    nrf24l01p_write_register(dev, FEATURE, EN_DPL | EN_ACK_PAY);
    // 3 byte address length
    nrf24l01p_write_register(dev, SETUP_AW, AW_3);
    // RX address
    nrf24l01p_write_register(dev, EN_RXADDR, ERX_P0);
    nrf24l01p_set_addr(dev, RX_ADDR_P0, address, 3);
    // enable Enhanced ShockBurst Auto Acknowledgment
    nrf24l01p_write_register(dev, EN_AA, ENAA_P0);
    nrf24l01p_write_register(dev, DYNPD, DPL_P0);
    // clear data fifo
    nrf24l01p_flush_tx(dev);
    nrf24l01p_flush_rx(dev);
    // default ack payload
    uint8_t ack[] = {0};
    nrf24l01p_write_ack_payload(dev, 0, &ack[0], sizeof(ack));
    // clear IRQ flags
    nrf24l01p_write_register(dev, STATUS, RX_DR | TX_DS | MAX_RT);

    chEvtRegisterMaskWithFlags(&exti_events, &radio_event_listener,
        NRF_INTERRUPT_EVENT, EXTI_EVENT_NRF_IRQ);
}

void _sd_send_fn(void *arg, const void *p, size_t len)
{
    if (len == 0) {
        return;
    }
    chSequentialStreamWrite((BaseSequentialStream*)arg, (const uint8_t*)p, len);
}

void datagram_cb(void *dg, size_t len, void *arg)
{
    serial_datagram_send(dg, len, _sd_send_fn, arg);
}

static THD_FUNCTION(radio_thread_rx, arg)
{
    nrf24l01p_t *nrf = &nrf24;
    datagram_buffer_t receiver;
    datagram_buffer_init(&receiver, datagram_buffer, sizeof(datagram_buffer), datagram_cb, arg);
    nrf_setup_prx(nrf);
    nrf_ce_active();
    while (1) {
        static uint8_t packet[32];
        nrf24l01p_write_ack_payload(nrf, 0, packet, 1);
        nrf24l01p_write_register(nrf, STATUS, RX_DR | TX_DS | MAX_RT);
        eventmask_t ret = chEvtWaitAnyTimeout(NRF_INTERRUPT_EVENT, MS2ST(100));
        if (ret == 0) {
            nrf24l01p_flush_rx(nrf);
            continue;
        }
        uint8_t status = nrf24l01p_status(nrf);
        if (status & RX_DR) {
            uint8_t len = nrf24l01p_read_rx_payload_len(nrf);
            if (len == 0 || len > 32) {
                nrf24l01p_flush_rx(nrf);
                continue;
            }
            nrf24l01p_read_rx_payload(nrf, packet, len);
            packet_receive(&receiver, packet, len);
        } else {
            nrf24l01p_flush_rx(nrf);
        }
    }
}

void radio_start_rx(void *arg)
{
    chThdCreateStatic(radio_thread_wa, sizeof(radio_thread_wa), NORMALPRIO, radio_thread_rx, arg);
}

void radio_start_tx(void *arg)
{
    chThdCreateStatic(radio_thread_wa, sizeof(radio_thread_wa), NORMALPRIO, radio_thread_tx, arg);
}
