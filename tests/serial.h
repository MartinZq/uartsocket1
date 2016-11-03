/*
 * serial.h
 *
 *  Created on: Mar 2, 2016
 *      Author: yuyue <yuyue2200@hotmail.com>
 */

#ifndef __SERIAL_H__
#define __SERIAL_H__

enum {
	SERIAL_READ_INDEX = 0,
#define SERIAL_READ_INDEX SERIAL_READ_INDEX
	SERIAL_WRITE_INDEX = 1,
#define SERIAL_WRITE_INDEX SERIAL_WRITE_INDEX
	SERIAL_MAX_INDEX = 2,
#define SERIAL_MAX_INDEX SERIAL_MAX_INDEX
};

/**
 * This structure describes serial handle.
 *
 * serial_handle_t is typically allocated once and then reused multiple times
 * to open the different serial port.
 * serial_handle_t MUST be allocated using serial_new() and MUST be freed with
 * serial_free().
 * @WARNING sizeof(serial_handle_t) is not a part of the public ABI.
 */
typedef struct serial_handle_t serial_handle_t;
typedef ssize_t (*serial_rdwr_t)(serial_handle_t *, void *, size_t, int);

/**
 * @WARNING Callback is a functions of thread environment.
 *          MUST be returned normally
 */
typedef void *(*serial_callback_t)(serial_handle_t *, serial_rdwr_t []);

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Open the serial port device file(.e.g /dev/ttyxxx)
 *
 * @s		serial port handle.
 * @dev	    serial port device filename.
 * @speed	baud rate.
 * @opt     serial port configure options.
 *			opts is the characters group of bits, odd/even parity and stop bits.
 *			bits:      5, 6, 7, 8
 *			odd/even:  O, E, N
 *			stop bits: 1, 0
 *			example: "8n1"
 *
 * @return  0 on success, < 0 on failure and errno is set appropriately.
 */
int serial_open(serial_handle_t **s, const char *dev, unsigned int speed, const char *opts, serial_callback_t callback);

/**
 * Close the serial port /dev/ttyxxx of serial handle and free memory block.
 *
 * @s        serial port handle
 */
void serial_free(serial_handle_t **s);

/**
 * Find serial port connection status.
 *
 * @h        serial port handle
 *
 * @return   0, connection is closed, > 0 connect is online.
 */
int serial_alive(serial_handle_t *h);

#ifdef __cplusplus
}
#endif
#endif /* __SERIAL_H__ */
