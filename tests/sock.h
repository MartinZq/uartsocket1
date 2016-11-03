/*
 * sock.h
 *
 *  Created on: Feb 29, 2016
 *      Author: yuyue <yuyue2200@hotmail.com>
 */

#ifndef __SOCK_H__
#define __SOCK_H__
enum {
	MAX_CONN_TIMEOUT = 75 * 1000,
#define MAX_CONN_TIMEOUT MAX_CONN_TIMEOUT
};

/**
 * This structure describes socket handle.
 *
 * sock_handle_t is typically allocated once and then reused multiple times
 * to connect different peer host.
 * sock_handle_t MUST be allocated and connected using sock_connect() and MUST be freed with
 * sock_free().
 * @warning sizeof(sock_handle_t) is not a part of the public ABI.
 */
typedef struct sock_handle_t sock_handle_t;
typedef ssize_t (*sock_rdwr_t)(sock_handle_t *, void *, size_t, int);

/**
 * @WARNING Callback is a functions of thread environment and that MUST be returned normally.
 *          DON'T call exit(), Exit(), _exit(), pthread_exit(), pthread_cancel().
 *          when it returned, the master's program will free memory block and exit too.
 *
 * @sock_rdwr_t is functions type for read/write, like the read/write.
 */
enum {
	SOCK_READ_INDEX = 0,
#define SOCK_READ_INDEX SOCK_READ_INDEX
	SOCK_WRITE_INDEX = 1,
#define SOCK_WRITE_INDEX SOCK_WRITE_INDEX
	SOCK_MAX_INDEX = 2,
#define SOCK_MAX_INDEX SOCK_MAX_INDEX
};
typedef void *(*sock_callback_t)(sock_handle_t *, sock_rdwr_t []);

#ifndef EEOF
# define EEOF ((signed)(('E' << 16) | ('O' << 8) | ('F' << 0)))
#endif

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Connect to the host:port and waiting for @timeout milliseconds of times and call callback
 *
 * @h		socket handle.
 * @host	host name or IP address.
 * @port	service or port.
 * @timeout awaiting for connect to the host(MAX 75s, parameter unit: milliseconds).
 * @callback callback function have two parameters, the first is structure point,
 *           the last is read/write functions array.
 *
 * @return  0 on success, < 0 on failure and errno is set appropriately.
 */
int sock_connect(sock_handle_t **h, const char *host, \
				const char *serv, int timeout, \
				sock_callback_t callback);

/**
 * Close the socket connections of socket handle and free memory block.
 *
 * @h        socket handle
 */
void sock_free(sock_handle_t **h);

/**
 * Find socket handle status.
 *
 * @h        socket handle
 *
 * @return   0, connection is opened, < 0 connection is closed.
 */
int sock_alive(sock_handle_t *h);

#ifdef __cplusplus
}
#endif
#endif /* NET_H_ */
