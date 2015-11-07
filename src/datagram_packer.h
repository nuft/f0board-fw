#ifndef DATAGRAM_PACKER_H
#define DATAGRAM_PACKER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#define LAST_PACKET_MASK        0x80
#define SEQUENCE_NUMBER_MASK    0x7F

// receive

#define NO_ERRROR               0
#define DATAGRAM_TOO_LONG       1
#define DATAGRAM_INCOMPLETE     2
#define CRC_MISMATCH            3
#define PROTOCOL_ERROR          4

typedef void (*nrf_datagram_cb_t)(void *datagram, size_t length, void *arg);

typedef struct {
    void *buffer;
    size_t buffer_size;
    size_t write_index;
    uint8_t last_sequence_number;
    uint16_t crc;
    nrf_datagram_cb_t callback;
    void *callback_arg;
} datagram_buffer_t;

void datagram_buffer_init(datagram_buffer_t *rec, void *buffer, size_t size,
        nrf_datagram_cb_t cb_fn, void *cb_arg);
int packet_receive(datagram_buffer_t *rec, void *packet, size_t len);

// send

typedef struct {
    const void *datagram;
    size_t length;
    size_t read_index;
    uint16_t crc;
    uint8_t sequence_number;
} datagram_sender_t;

void dp_sender_init(datagram_sender_t *snd, const void *datagram, size_t length);
bool datagram_packer_next_packet(datagram_sender_t *snd, void *buf, size_t length, size_t *written);

#ifdef __cplusplus
}
#endif

#endif /* DATAGRAM_PACKER_H */
