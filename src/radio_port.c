#include <hal.h>

void nrf_ce_active(void)
{
    palSetPad(GPIOB, GPIOB_NRF_CE);
}

void nrf_ce_inactive(void)
{
    palClearPad(GPIOB, GPIOB_NRF_CE);
}

SPIDriver *spi_init(void)
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
    return &SPID1;
}