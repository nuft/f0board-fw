#ifndef NRF24L01P_H
#define NRF24L01P_H

#include <stdint.h>
#include <hal.h>

/*
 *  nRF24L01+ interface:
 *  IRQ: interrupt (out)
 *  CE: chip enable (in)
 *  SPI: CSN, SCK, MOSI, MISO
 *  SPI mode: MSBit first, LSByte first, CSN active low,
 *    max 10Mbps, cpol=0 (idle=low), cpha=0 (first edge)
 */

typedef struct {
    SPIDriver *spi;
} nrf24l01p_t;

void nrf24l01p_init(nrf24l01p_t *dev, SPIDriver *spi);
uint8_t nrf24l01p_command(nrf24l01p_t *dev, uint8_t cmd);
void nrf24l01p_command_write(nrf24l01p_t *dev, uint8_t cmd, const uint8_t *buf, uint8_t len);
void nrf24l01p_command_read(nrf24l01p_t *dev, uint8_t cmd, uint8_t *buf, uint8_t len);
uint8_t nrf24l01p_read_register(nrf24l01p_t *dev, uint8_t addr);
void nrf24l01p_write_register(nrf24l01p_t *dev, uint8_t addr, uint8_t val);
void nrf24l01p_read_multibyte_register(nrf24l01p_t *dev, uint8_t addr, uint8_t *buf, uint8_t len);
void nrf24l01p_write_multibyte_register(nrf24l01p_t *dev, uint8_t addr, const uint8_t *buf, uint8_t len);
uint8_t nrf24l01p_status(nrf24l01p_t *dev);
void nrf24l01p_flush_tx(nrf24l01p_t *dev);
void nrf24l01p_flush_rx(nrf24l01p_t *dev);
uint8_t nrf24l01p_read_rx_payload_len(nrf24l01p_t *dev);
void nrf24l01p_read_rx_payload(nrf24l01p_t *dev, uint8_t *buf, uint8_t len);
void nrf24l01p_write_tx_payload(nrf24l01p_t *dev, const uint8_t *buf, uint8_t len);
void nrf24l01p_write_tx_payload_no_ack(nrf24l01p_t *dev, const uint8_t *buf, uint8_t len);
void nrf24l01p_write_ack_payload(nrf24l01p_t *dev, uint8_t pipe, const uint8_t *buf, uint8_t len);
void nrf24l01p_set_addr(nrf24l01p_t *dev, uint8_t reg, const uint8_t *addr, uint8_t len);
void nrf24l01p_set_channel(nrf24l01p_t *dev, uint8_t ch);

#endif // NRF24L01P_H
