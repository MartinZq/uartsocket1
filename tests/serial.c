/*
 * serial.c
 *
 *  Created on: Mar 2, 2016
 *      Author: yuyue <yuyue2200@hotmail.com>
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <sys/epoll.h>
#include <termios.h>
#include <errno.h>
#include <pthread.h>
#include "serial.h"

struct serial_handle_t {
	int fd;
	pthread_t tid;
	speed_t speed;
	const char *dev;
	const char *opts;
	serial_callback_t callback;
	serial_rdwr_t rdwr[SERIAL_MAX_INDEX];
};

typedef struct epoll_handle_t {
	int epollfd;
	struct epoll_event ev;
	pthread_mutex_t lock;
	int nb_fds;
} epoll_handle_t;
static epoll_handle_t global_epoll = {-1, {0}, {0}, 0};

static int fd_set_nonblock(int fd, int enable)
{
	if (enable)
		return fcntl(fd, F_SETFL, fcntl(fd, F_GETFL) | O_NONBLOCK);
	else
		return fcntl(fd, F_SETFL, fcntl(fd, F_GETFL) & (~O_NONBLOCK));
}

static serial_handle_t *serial_new(void)
{
	serial_handle_t *serial = NULL;

	serial = malloc(sizeof(*serial));
	if (serial) {
		memset(serial, 0, sizeof(*serial));
		serial->fd    = -1;
	}

	return serial;
}


/**
 *	opts is the characters group of bits, odd/even parity and stop bits
 *  bits:      5, 6, 7, 8
 *  odd/even:  O, E, N
 *	stop bits: 1, 0
 *	example: "8n1"
 */
static int serial_set_params(serial_handle_t *s)
{
	struct termios old, new;
	speed_t speed;

	if (tcgetattr(s->fd, &old))
		return -1;
	cfmakeraw(&old);

	/** set speed */
	printf("input speed: %d\n", s->speed);
	switch (s->speed) {
	case 0: speed = B0; break;
	case 300: speed = B300; break;
	case 600: speed = B600; break;
	case 1200: speed = B1200; break;
	case 1800: speed = B1800; break;
	case 2400: speed = B2400; break;
	case 4800: speed = B4800; break;
	case 9600: speed = B9600; break;
	case 19200: speed = B19200; break;
	case 38400: speed = B38400; break;
	case 57600: speed = B57600; break;
	case 115200: speed = B115200; break;
	case 230400: speed = B230400; break;
	case 460800: speed = B460800; break;
	case 500000: speed = B500000; break;
	case 576000: speed = B576000; break;
	case 921600: speed = B921600; break;
	case 1000000: speed = B1000000; break;
	case 1152000: speed = B1152000; break;
	case 1500000: speed = B1500000; break;
	case 2000000: speed = B2000000; break;
	case 2500000: speed = B2500000; break;
	case 3000000: speed = B3000000; break;
	case 3500000: speed = B3500000; break;
	case 4000000: speed = B4000000; break;
	default: errno = EINVAL; return -1;
	}
	if (cfsetispeed(&old, speed) || cfsetospeed(&old, speed))
		return -1;

	/** set bits */
	switch (s->opts[0]) {
	case '5':
		old.c_cflag = (old.c_cflag & ~CSIZE) | CS5;
		break;
	case '6':
		old.c_cflag = (old.c_cflag & ~CSIZE) | CS6;
		break;
	case '7':
		old.c_cflag = (new.c_cflag & ~CSIZE) | CS7;
		break;
	case '8':
		old.c_cflag = (old.c_cflag & ~CSIZE) | CS8;
		break;
	default: errno = EINVAL; return -1;
	}

	/** set odd/even parity */
	switch (s->opts[1]) {
	case 'O':
	case 'o':
		old.c_cflag |= (PARENB | PARODD);
		break;
	case 'E':
	case 'e':
		old.c_cflag |= PARENB;
		old.c_cflag &= ~PARODD;
		break;
	case 'N':
	case 'n':
		old.c_cflag &= ~(PARENB | PARODD);
		break;
	default: errno = EINVAL; return -1;
	}

	/** set stop bits */
	switch (s->opts[2]) {
	case '1':
		old.c_cflag &= ~CSTOPB;
		break;
	case '2':
		old.c_cflag |= CSTOPB;
		break;
	default: errno = EINVAL; return -1;
	}

	/** FIXME: */
	old.c_cflag |= (CLOCAL | CREAD);

	if (!tcsetattr(s->fd, TCSANOW, &old) && !tcgetattr(s->fd, &new)) {
		if (cfgetispeed(&new) == speed && \
			cfgetospeed(&new) == speed && \
			new.c_cflag == old.c_cflag) {
			tcflush(s->fd, TCIOFLUSH);
			return 0;
		}
	}

	return -1;
}

static ssize_t serial_asyn_read(serial_handle_t *s, void *buf, size_t count, int timeout)
{
	int nfds;
	struct epoll_event *rev;

	rev = malloc(sizeof(*rev) * global_epoll.nb_fds);
	if (!rev)
		return -1;
	pthread_mutex_lock(&global_epoll.lock);
	nfds = epoll_wait(global_epoll.epollfd, rev, global_epoll.nb_fds, timeout);
	pthread_mutex_unlock(&global_epoll.lock);
	if (nfds > 0) {
		int n;
		for (n = 0; n < nfds; n++) {
			if (((serial_handle_t *)rev[n].data.u64)->fd == s->fd) {
				struct epoll_event save = rev[n];
				ssize_t len;

				free(rev);
				if ((save.events & EPOLLERR) || (save.events & EPOLLRDHUP))
					return -1;
				if (!(save.events & EPOLLIN))
					return 0;
				len = read(s->fd, buf, count);
				if (len < 0)
					if (errno == EAGAIN)
						len = 0;
				return len;
			}
		}
	}
	free(rev);

	return nfds < 0 ? -1 : 0;
}

static ssize_t serial_asyn_write(serial_handle_t *s, void *buf, size_t count, int timeout)
{
	int nfds;
	struct epoll_event *rev;

	rev = malloc(sizeof(*rev) * global_epoll.nb_fds);
	if (!rev)
		return -1;
	pthread_mutex_lock(&global_epoll.lock);
	nfds = epoll_wait(global_epoll.epollfd, rev, global_epoll.nb_fds, timeout);
	pthread_mutex_unlock(&global_epoll.lock);
	if (nfds > 0) {
		int n;
		for (n = 0; n < nfds; n++) {
			if (((serial_handle_t *)rev[n].data.u64)->fd == s->fd) {
				struct epoll_event save = rev[n];
				ssize_t len;

				free(rev);
				if ((save.events & EPOLLERR) || (save.events & EPOLLRDHUP))
					return -1;
				if (!(save.events & EPOLLOUT))
					return 0;
				len = write(s->fd, buf, count);
				if (len < 0)
					if (errno == EAGAIN)
						len = 0;
				return len;
			}
		}
	}
	free(rev);

	return nfds < 0 ? -1 : 0;
}

static int serial_close(serial_handle_t *s)
{
	if (s->fd >= 0)
		close(s->fd);
	free((void *)s->dev);
	free((void *)s->opts);
	memset(s, 0, sizeof(*s));
	s->fd = -1;

	return 0;
}

void serial_free(serial_handle_t **s)
{
	serial_handle_t *serial = *s;
	if (serial) {
		if (serial->fd >= 0)
			serial_close(serial);
		free(serial);
		*s = NULL;
	}
}

static int serial_alive2(serial_handle_t *s, int timeout)
{
	int nfds;
	struct epoll_event *rev;

	if (s->fd < 0)
		return 0;
	rev = malloc(sizeof(*rev) * global_epoll.nb_fds);
	if (!rev)
		return -1;
	pthread_mutex_lock(&global_epoll.lock);
	nfds = epoll_wait(global_epoll.epollfd, rev, global_epoll.nb_fds, timeout);
	pthread_mutex_unlock(&global_epoll.lock);
	if (nfds > 0) {
		int n;
		for (n = 0; n < nfds; n++) {
			if (((serial_handle_t *)rev[n].data.u64)->fd == s->fd && \
				(rev[n].events & EPOLLOUT)) {
				free(rev);
				return 0;
			}
		}
	}
	free(rev);
	return -1;
}

int serial_alive(serial_handle_t *s)
{
	if (!s)
		return -1;
	return s->fd < 0 ? -1 : 0;
}

static void *serial_callback(void *args)
{
	serial_handle_t *s = (serial_handle_t *)args;
	void *ret;

	ret = s->callback(s, s->rdwr);
	epoll_ctl(global_epoll.epollfd, EPOLL_CTL_DEL, s->fd, NULL);
	global_epoll.nb_fds--;
	serial_close(s);

	return ret;
}

int serial_open(serial_handle_t **s, const char *dev, unsigned int speed, const char *opts, serial_callback_t callback)
{
	serial_handle_t *sd = NULL;
	if (!dev || !opts || opts[3] || !callback) {
		errno = EINVAL;
		return -1;
	}
	sd = *s ? *s : serial_new();
	if (!sd)
		return -1;
	if ((sd->fd = open(dev, O_RDWR | O_NOCTTY | O_NDELAY)) < 0)
		return -1;
	fd_set_nonblock(sd->fd, 1);
	sd->speed                    = speed;
	sd->dev                      = strdup(dev);
	sd->opts                     = strdup(opts);
	sd->callback                 = callback;
	sd->rdwr[SERIAL_READ_INDEX]  = serial_asyn_read;
	sd->rdwr[SERIAL_WRITE_INDEX] = serial_asyn_write;
	if (serial_set_params(sd) < 0)
		goto failure;
	if (global_epoll.epollfd < 0) {
		/** Don't mind parameter values of epoll_create() */
		global_epoll.epollfd = epoll_create(1);
		pthread_mutex_init(&global_epoll.lock, NULL);
	}
	global_epoll.ev.events   = EPOLLIN | EPOLLOUT | EPOLLRDHUP;
	global_epoll.ev.data.u64 = (uint64_t)sd;
	pthread_mutex_lock(&global_epoll.lock);
	if (epoll_ctl(global_epoll.epollfd, EPOLL_CTL_ADD, sd->fd, &global_epoll.ev)) {
		pthread_mutex_unlock(&global_epoll.lock);
		goto failure;
	}
	pthread_mutex_unlock(&global_epoll.lock);
	global_epoll.nb_fds++;
	if (!serial_alive2(sd, 400)) {
		pthread_attr_t attr;
		int err;
		if (!(err = pthread_attr_init(&attr))) {
			if (!(err = pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED))) {
				err = pthread_create(&sd->tid, &attr, &serial_callback, (void *)sd);
				if (!*s)
					*s = sd;
			}
			pthread_attr_destroy(&attr);
			return 0;
		}
		errno = err;
	}
failure:
	(!*s) ? serial_free(&sd) : serial_close(sd);

	return -1;
}
