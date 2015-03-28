#include <ch.h>
#include <chprintf.h>
#include <shell.h>
#include "shell_cmd.h"

void shell_spawn(BaseSequentialStream *stream, const ShellCommand *commands)
{
    ShellConfig shell_cfg1 = {
        stream,
        commands
    };
    shellInit();
    thread_t *shelltp = shellCreate(&shell_cfg1, THD_WORKING_AREA_SIZE(512), NORMALPRIO);
    while (!chThdTerminatedX(shelltp)) {
        chThdSleepMilliseconds(10);
    }
    chThdRelease(shelltp);
}
