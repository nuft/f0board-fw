#ifndef SHELL_CMD_H
#define SHELL_CMD_H

#ifdef __cplusplus
extern "C" {
#endif

#include <ch.h>
#include <shell.h>

void shell_spawn(BaseSequentialStream *stream, const ShellCommand *commands);

#ifdef __cplusplus
}
#endif

#endif /* SHELL_CMD_H */