/*
 * netlist.c
 *
 * Sam Chen <xuejian1354@163.com>
 *
 */

#include "netlist.h"

#ifdef __cplusplus
extern "C" {
#endif

int addto_tcpconn_list(tcp_conn_t **plist, tcp_conn_t *list)
{
	tcp_conn_t *pre_list =  NULL;
	tcp_conn_t *t_list = *plist;

	if(list == NULL)
	{
		return -1;
	}
	else
	{
		list->next = NULL;
	}

	while(t_list != NULL)
	{
		if(t_list->fd != list->fd)
		{
			pre_list = t_list;
			t_list = t_list->next;
		}
		else
		{
			if(pre_list != NULL)
			{
				pre_list->next = t_list->next;
				t_list->next = *plist;
				*plist = t_list;
			}
			
			return 1;
		}
	}

	list->next = *plist;
	*plist = list;

	return 0;
}

tcp_conn_t *queryfrom_tcpconn_list(tcp_conn_t *plist, int fd)
{
	tcp_conn_t *t_list = plist;

	while(t_list != NULL)
	{
		if(t_list->fd != fd)
		{
			t_list = t_list->next;
		}
		else
		{
			return t_list;
		}
	}

	return NULL;
}

tcp_conn_t *queryfrom_tcpconn_list_with_ipaddr(tcp_conn_t *plist, char *ipaddr)
{
	tcp_conn_t *t_list = plist;

	while(t_list != NULL)
	{
		char t_ipaddr[24] = {0};
		sprintf(t_ipaddr, "%s:%u", 
			inet_ntoa(t_list->client_addr.sin_addr), 
			ntohs(t_list->client_addr.sin_port));

		if(memcmp(t_ipaddr, ipaddr, strlen(t_ipaddr)))
		{
			t_list = t_list->next;
		}
		else
		{
			return t_list;
		}
	}

	return NULL;
}

int delfrom_tcpconn_list(tcp_conn_t **plist, int fd)
{
	tcp_conn_t *pre_list =  NULL;
	tcp_conn_t *t_list = *plist;


	while(t_list != NULL)
	{
		if(t_list->fd != fd)
		{
			pre_list = t_list;
			t_list = t_list->next;
		}
		else
		{
			if(pre_list != NULL)
			{
				pre_list->next = t_list->next;
			}
			else
			{
				*plist = t_list->next;
			}

			free(t_list);

			return 0;
		}
	}

	return -1;
}

#ifdef __cplusplus
}
#endif
