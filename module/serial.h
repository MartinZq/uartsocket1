/*
 * serial.h
 *
 * Sam Chen <xuejian1354@163.com>
 *
 */
#ifndef __SERIAL_H__
#define __SERIAL_H__

#include <services/globals.h>
#include <session/protocol.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct SerialDev
{
	char dev[64];
	int speed;
	int stopbits;
	int serial_fd;
	int num;
	trsess_t *session;
	struct SerialDev *next;
}serial_dev_t;

serial_dev_t *get_serial_dev();
int add_serial_dev(serial_dev_t *t_serial_dev);
serial_dev_t *query_serial_dev(char *dev);
int del_serial_dev(char *dev);
void serial_dev_free();

int serial_init(trsess_t *session);

#ifdef __cplusplus
}
#endif

#endif //__SERIAL_H__
