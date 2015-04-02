#ifndef RADIO_H
#define RADIO_H

#ifdef __cplusplus
extern "C" {
#endif

#include <ch.h>

void cmd_radio_tx(BaseSequentialStream *chp, int argc, char *argv[]);
void cmd_radio_rx(BaseSequentialStream *chp, int argc, char *argv[]);

#ifdef __cplusplus
}
#endif

#endif /* RADIO_H */
