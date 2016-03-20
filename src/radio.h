#ifndef RADIO_H
#define RADIO_H

#ifdef __cplusplus
extern "C" {
#endif

void radio_start_rx(void *arg);
void radio_start_tx(void *arg);

#ifdef __cplusplus
}
#endif

#endif /* RADIO_H */
