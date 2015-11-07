#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <stdbool.h>

#include <crc/crc16.h>
#include "datagram_packer.h"

void datagram_buffer_init(datagram_buffer_t *rec, void *buffer, size_t size,
        nrf_datagram_cb_t cb_fn, void *cb_arg)
{
    rec->buffer = buffer;
    rec->buffer_size = size;
    rec->callback = cb_fn;
    rec->callback_arg = cb_arg;
    rec->write_index = 0;
    rec->crc = 0;
}

int packet_receive(datagram_buffer_t *rec, void *packet, size_t len)
{
    int error_code = NO_ERRROR;
    uint8_t *data = (uint8_t *)packet;
    uint8_t seq = data[0];
    bool last = (seq & LAST_PACKET_MASK) != 0;
    seq &= ~LAST_PACKET_MASK;

    if (rec->write_index == 0) {
        rec->crc = 0; // new datagram, reset crc
    } else {
        // check sequence number for missing packets
        if (seq != ((rec->last_sequence_number + 1) & SEQUENCE_NUMBER_MASK)) {
            rec->write_index = 0;
            return DATAGRAM_INCOMPLETE;
        }
    }
    rec->last_sequence_number = seq;
    data++;
    len--;

    if (rec->buffer_size - rec->write_index < len) {
        rec->write_index = 0;
        return DATAGRAM_TOO_LONG;
    }
    rec->crc = crc16(rec->crc, data, len);

    if (last) {
        len -= sizeof(rec->crc);
        memcpy((char *)rec->buffer + rec->write_index, data, len);
        rec->write_index += len;
        if (rec->crc == 0) { // CRC is 0 if correct
            rec->callback(rec->buffer, rec->write_index, rec->callback_arg);
        } else {
            error_code = CRC_MISMATCH;
        }
        rec->write_index = 0;
    } else {
        memcpy((char *)rec->buffer + rec->write_index, data, len);
        rec->write_index += len;
    }
    return error_code;
}

#define MIN_PACKET_LENGTH   3 // 1 header + 2 crc16

void dp_sender_init(datagram_sender_t *snd, const void *datagram, size_t length)
{
    snd->datagram = datagram;
    snd->length = length;
    snd->read_index = 0;
    snd->crc = crc16(0, datagram, length);
    snd->sequence_number = 0;
}

bool datagram_packer_next_packet(datagram_sender_t *snd, void *buf, size_t length, size_t *written)
{
    uint8_t *packet = (uint8_t *)buf;
    // assert(length > MIN_PACKET_LENGTH);
    packet[0] = snd->sequence_number;
    snd->sequence_number = (snd->sequence_number + 1) & SEQUENCE_NUMBER_MASK;
    size_t len = length - 1;
    if (len > snd->length - snd->read_index) {
        len = snd->length - snd->read_index;
    }
    memcpy(&packet[1], (char *)snd->datagram + snd->read_index, len);
    snd->read_index += len;
    if (snd->read_index == snd->length && len + 1 + sizeof(snd->crc) <= length) {
        // write CRC at end
        packet[len+1] = snd->crc;
        packet[len+2] = snd->crc>>8;
        *written = len + 1 + sizeof(snd->crc);
        packet[0] |= LAST_PACKET_MASK;
        return true;
    }
    *written = len + 1;
    return false;
}
