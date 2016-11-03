/*
 * dconfig.h
 *
 * Sam Chen <xuejian1354@163.com>
 *
 */
#ifndef __DCONFIG_H__
#define __DCONFIG_H__

#include <mconfig.h>

#ifdef __cplusplus
extern "C" {
#endif

#define THREAD_MAX_NUM	4

//default program name
#ifndef TARGET_NAME
#define TARGET_NAME "uartsocket(default)"
#endif

#define CONF_FILE	"/etc/uartsocket.conf"
#define STATUS_FILE	"/tmp/uartsocket_status.dat"

#ifdef __cplusplus
}
#endif

#endif //__DCONFIG_H__

