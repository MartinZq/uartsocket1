/* globals.h
 *
 * Sam Chen <xuejian1354@163.com>
 *
 */
#ifndef __GLOBALS_H__
#define __GLOBALS_H__

#include <mconfig.h>
#include <debug/dconfig.h>
#include <debug/dlog.h>
#include <signal.h>
#include <sys/time.h>
#include <sys/stat.h>
#include "time.h"

#ifdef __cplusplus
extern "C" {
#endif

int get_end();

int start_params(int argc, char **argv);
#ifdef DAEMON_PROCESS_CREATE
int daemon_init();
int get_daemon_cmdline();
#endif
int mach_init();

#ifdef READ_CONF_FILE
int conf_read_from_file();
#endif

void process_signal_register();

char *get_current_time();
char *get_system_time();

#ifdef __cplusplus
}
#endif

#endif	//__GLOBALS_H__
