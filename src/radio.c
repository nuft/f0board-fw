#include <ch.h>
#include <hal.h>
#include <stdint.h>
#include "nrf24l01p.h"
#include "radio.h"

const uint8_t address[] = {0x2A, 0x2A, 0x2A};
const uint8_t channel = 0;

#define NRF_INTERRUPT_EVENT 1

static void spi_init(void)
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

static void nrf_ce_active(void)
{
    palSetPad(GPIOA, GPIOA_NRF_CE);
}

static void nrf_ce_inactive(void)
{
    palClearPad(GPIOA, GPIOA_NRF_CE);
}

void nrf_setup_tx(nrf24l01p_t *dev)
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

static THD_WORKING_AREA(radio_thread_wa, 256);
static THD_FUNCTION(radio_thread, arg)
{
    (void)arg;
    static nrf24l01p_t nrf24l01p;
    nrf24l01p_t *nrf = &nrf24l01p;

    spi_init();
    nrf24l01p_init(nrf, &SPID1);
    nrf_setup_tx(nrf);

    chThdSleepMilliseconds(1);

    event_listener_t radio_event_listener;
    chEvtRegisterMaskWithFlags(&exti_events, &radio_event_listener,
        NRF_INTERRUPT_EVENT, EXTI_EVENT_NRF_IRQ);

    static uint32_t tx_count = 0;
    while (1) {
        // clear interrupts
        nrf24l01p_write_register(nrf, STATUS, RX_DR | TX_DS | MAX_RT);

        static uint32_t tx_buf[8];
        nrf24l01p_write_tx_payload(nrf, (uint8_t *) tx_buf, 32);

        nrf_ce_active();
        eventmask_t ret = chEvtWaitAnyTimeout(NRF_INTERRUPT_EVENT, MS2ST(1000));
        nrf_ce_inactive();

        if (ret == 0) {
            nrf24l01p_flush_tx(nrf);
            palTogglePad(GPIOE, GPIOE_LED3_RED);
            continue;
        }

        palTogglePad(GPIOE, GPIOE_LED7_GREEN);
    }
}
