/*
 * netlist.h
 *
 * Sam Chen <xuejian1354@163.com>
 *
 */
#ifndef __NETLIST_H__
#define __NETLIST_H__

#include <services/globals.h>
#include <arpa/inet.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct TCPConn
{
	int fd;
	struct sockaddr_in client_addr;
	void *parent;
	struct TCPConn *next;
}tcp_conn_t;

int addto_tcpconn_list(tcp_conn_t **plist, tcp_conn_t *list);
tcp_conn_t *queryfrom_tcpconn_list(tcp_conn_t *plist, int fd);
tcp_conn_t *queryfrom_tcpconn_list_with_ipaddr(tcp_conn_t *plist, char *ipaddr);
int delfrom_tcpconn_list(tcp_conn_t **plist, int fd);

#ifdef __cplusplus
}
#endif

#endif  // __NETLIST_H__