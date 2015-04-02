#include <hal.h>
#include <stdint.h>
#include "nrf24l01p.h"
#include "nrf24l01p_registers.h"

// Commands
#define R_REGISTER          0x00 // 000A AAAA
#define W_REGISTER          0x20 // 001A AAAA
#define R_RX_PAYLOAD        0x61
#define W_TX_PAYLOAD        0xA0
#define FLUSH_TX            0xE1
#define FLUSH_RX            0xE2
#define REUSE_TX_PL         0xE3
#define R_RX_PL_WID         0x60
#define W_ACK_PAYLOAD       0xA8 // 1010 1PPP
#define W_TX_PAYLOAD_NOACK  0xB0
#define NOP                 0xFF

void nrf24l01p_init(nrf24l01p_t *dev, SPIDriver *spi)
{
    dev->spi = spi;
}

uint8_t nrf24l01p_command(nrf24l01p_t *dev, uint8_t cmd)
{
    spiSelect(dev->spi);
    spiExchange(dev->spi, 1, &cmd, &cmd);
    spiUnselect(dev->spi);
    return cmd;
}

void nrf24l01p_command_write(nrf24l01p_t *dev, uint8_t cmd, const uint8_t *buf, uint8_t len)
{
    spiSelect(dev->spi);
    spiSend(dev->spi, 1, &cmd);
    spiSend(dev->spi, len, buf);
    spiUnselect(dev->spi);
}

void nrf24l01p_command_read(nrf24l01p_t *dev, uint8_t cmd, uint8_t *buf, uint8_t len)
{
    spiSelect(dev->spi);
    spiSend(dev->spi, 1, &cmd);
    spiReceive(dev->spi, len, buf);
    spiUnselect(dev->spi);
}

// commands

uint8_t nrf24l01p_read_register(nrf24l01p_t *dev, uint8_t addr)
{
    // uint8_t buf[2] = {R_REGISTER | (addr & 0x1F), 0};
    // spiSelect(dev->spi);
    // spiExchange(dev->spi, 2, buf, buf);
    // spiUnselect(dev->spi);
    // return buf[1];

    uint8_t val;
    nrf24l01p_command_read(dev, R_REGISTER | (addr & 0x1F), &val, 1);
    return val;
}

void nrf24l01p_write_register(nrf24l01p_t *dev, uint8_t addr, uint8_t val)
{
    // uint8_t buf[2] = {W_REGISTER | (addr & 0x1F), val};
    // spiSelect(dev->spi);
    // spiSend(dev->spi, 2, buf);
    // spiUnselect(dev->spi);

    nrf24l01p_command_write(dev, W_REGISTER | (addr & 0x1F), &val, 1);
}

void nrf24l01p_read_multibyte_register(nrf24l01p_t *dev, uint8_t addr, uint8_t *buf, uint8_t len)
{
    uint8_t cmd = R_REGISTER | (addr & 0x1F);
    nrf24l01p_command_read(dev, cmd, buf, len);
}

void nrf24l01p_write_multibyte_register(nrf24l01p_t *dev, uint8_t addr, const uint8_t *buf, uint8_t len)
{
    uint8_t cmd = W_REGISTER | (addr & 0x1F);
    nrf24l01p_command_write(dev, cmd, buf, len);
}

uint8_t nrf24l01p_status(nrf24l01p_t *dev)
{
    return nrf24l01p_command(dev, NOP);
}

void nrf24l01p_flush_tx(nrf24l01p_t *dev)
{
    nrf24l01p_command(dev, FLUSH_TX);
}

void nrf24l01p_flush_rx(nrf24l01p_t *dev)
{
    nrf24l01p_command(dev, FLUSH_RX);
}

uint8_t nrf24l01p_read_rx_payload_len(nrf24l01p_t *dev)
{
    uint8_t len = 0;
    nrf24l01p_command_read(dev, R_RX_PL_WID, &len, 1);
    return len;
}

void nrf24l01p_read_rx_payload(nrf24l01p_t *dev, uint8_t *buf, uint8_t len)
{
    nrf24l01p_command_read(dev, R_RX_PAYLOAD, buf, len);
}

void nrf24l01p_write_tx_payload(nrf24l01p_t *dev, const uint8_t *buf, uint8_t len)
{
    nrf24l01p_command_write(dev, W_TX_PAYLOAD, buf, len);
}

void nrf24l01p_write_tx_payload_no_ack(nrf24l01p_t *dev, const uint8_t *buf, uint8_t len)
{
    nrf24l01p_command_write(dev, W_TX_PAYLOAD_NOACK, buf, len);
}

void nrf24l01p_write_ack_payload(nrf24l01p_t *dev, uint8_t pipe, const uint8_t *buf, uint8_t len)
{
    nrf24l01p_command_write(dev, W_ACK_PAYLOAD | (pipe & 0x07), buf, len);
}

void nrf24l01p_set_addr(nrf24l01p_t *dev, uint8_t reg, const uint8_t *addr, uint8_t len)
{
    nrf24l01p_write_multibyte_register(dev, reg, addr, len);
}

void nrf24l01p_set_channel(nrf24l01p_t *dev, uint8_t ch)
{
    nrf24l01p_write_register(dev, RF_CH, (ch & 0b01111111));
}

// ////////////////////////////////////////////////////
// //
// //  High level functions
// //

// void nrf24l01p_power_up(void)
// {
//     uint8_t config = nrf24l01p_read_register(CONFIG);
//     nrf24l01p_write_register(CONFIG, config | PWR_UP);
// }

// void nrf24l01p_power_down(void)
// {
//     uint8_t config = nrf24l01p_read_register(CONFIG);
//     nrf24l01p_write_register(CONFIG, config & ~PWR_UP);
// }

// void nrf24l01p_set_tx_pwr(uint8_t pwr)
// {
//     if (pwr > 3)
//         pwr = 3;
//     uint8_t setup = nrf24l01p_read_register(RF_SETUP);
//     setup &= ~RF_PWR(3);
//     setup |= RF_PWR(pwr);
//     nrf24l01p_write_register(RF_SETUP, setup);
// }

// bool nrf24l01p_get_rx_pwr(void)
// {
//     return nrf24l01p_read_register(RPD);
// }

// void nrf24l01p_rx_mode(void)
// {
//     uint8_t config = nrf24l01p_read_register(CONFIG);
//     nrf24l01p_write_register(CONFIG, config | PRIM_RX);
// }

// void nrf24l01p_tx_mode(void)
// {
//     uint8_t config = nrf24l01p_read_register(CONFIG);
//     nrf24l01p_write_register(CONFIG, config & ~PRIM_RX);
// }
