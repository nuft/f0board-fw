#ifndef RADIO_H
#define RADIO_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <ch.h>

struct radio_packet {
    uint8_t data[32];
};

extern memory_pool_t radio_tx_pool;
extern mailbox_t radio_tx_mailbox;

extern memory_pool_t radio_rx_pool;
extern mailbox_t radio_rx_mailbox;

void radio_tx_start(void);
void radio_rx_start(void);

#ifdef __cplusplus
}
#endif

#endif /* RADIO_H */
