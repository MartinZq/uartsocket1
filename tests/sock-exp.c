/*
 * sock-exp.c
 *
 *  Created on: Feb 29, 2016
 *      Author: yuyue <yuyue2200@hotmail.com>
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <fcntl.h>
#include "sock.h"

enum {
	BUFFER_MAXSIZE = 1024,
#define BUFFER_MAXSIZE BUFFER_MAXSIZE
};

void *callback(sock_handle_t *h, sock_rdwr_t *rdwr)
{
	sock_rdwr_t sock_read = rdwr[SOCK_READ_INDEX];
	sock_rdwr_t sock_write = rdwr[SOCK_WRITE_INDEX];
	char buf[BUFFER_MAXSIZE] = {0}, *pos;
	ssize_t len;

	while (1) {
		pos = buf;
		len = 0;

		while (read(STDIN_FILENO, pos, sizeof(char)) > 0 && pos < &buf[BUFFER_MAXSIZE] && *pos != '\n')
			pos++;
		*pos = '\0';
		if (pos == buf)
			continue;

		/** write */
		if ((len = sock_write(h, buf, pos - buf, 1000)) <= 0) {
			fprintf(stderr, "[%p] socket write(%ld): %s\n",callback, len, strerror(errno));
			return ((void *)0);
		}

		/** read */
		memset(buf, 0, BUFFER_MAXSIZE);
		pos = buf;
		while ((len = sock_read(h, buf, BUFFER_MAXSIZE, 1000)) >= 0) {
			if (pos + len >= &buf[BUFFER_MAXSIZE -1]) {
				buf[BUFFER_MAXSIZE - 1] = '\0';
				break;
			}
			pos += len;
		}
		if (len < 0 && errno != EEOF) {
			fprintf(stderr, "[%p] socket read(%ld): %s\n", callback, len, strerror(errno));
			return ((void *)0);
		} else {
			fprintf(stderr, "%s\n", buf);
		}
	}

	return (void *)0;
}

void show_help(const char *s)
{
	if (s)
		fprintf(stderr, "%s: %s.\n", s, strerror(errno));
	else
		fprintf(stderr, \
				"socktest [host:serv]\n" \
				"example: socktest 127.0.0.1:80\n");
	exit(EXIT_FAILURE);
}

int main(int argc, char *argv[])
{
	sock_handle_t *hd = NULL;
	const char *host = NULL, *serv = NULL, *colon = NULL;

	if (argc < 2)
		show_help(NULL);
	if (!(colon = strrchr(argv[1], ':')) | \
		!(host = strndup(argv[1], colon - argv[1])) | \
		!(serv = strdup(colon + 1))) {
		errno = EINVAL;
		show_help("Parameter");
	}

	if (sock_connect(&hd, host, serv, 0, callback))
		show_help("sock_connect()");
	while (!sock_alive(hd))
		sleep(2);
	sock_free(&hd);

	return 0;
}
