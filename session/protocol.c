/*
 * protocol.c
 *
 * Sam Chen <xuejian1354@163.com>
 *
 */

#include "protocol.h"
#include <sys/socket.h>
#include <arpa/inet.h>
#include <module/netlist.h>

static trsess_t *g_sess = NULL;

utocol_t get_utocol_fromstr(char* s)
{
	
	if(!strcmp(s, "tcp"))
	{
		return UT_TCP;
	}
	else
	{
		return UT_UDP;
	}
}

char *get_utocol_tostr(utocol_t tocol)
{
	switch(tocol)
	{
	case UT_TCP:
		return "tcp";

	default:
		return "udp";
	}
}

char get_utocol_tochr(utocol_t tocol)
{
	switch(tocol)
	{
	case UT_TCP:
		return '1';

	default:
		return '2';
	}
}

umode_t get_umode_fromstr(char *s)
{
	if(!strcmp(s, "slave"))
		return UM_SLAVE;
	else
		return UM_MASTER;
}

char *get_umode_tostr(umode_t mode)
{
	switch(mode)
	{
	case UM_SLAVE:
		return "slave";

	default:
		return "master";
	}
}

trsess_t *get_global_session()
{
	return g_sess;
}

int add_global_session(trsess_t *session)
{
	return add_trans_session(&g_sess, session);
}

trsess_t *query_global_session(char *name)
{
	return query_trans_session(g_sess, name);
}

int del_global_session(char *name)
{
	return del_trans_session(&g_sess, name);
}

int add_trans_session(trsess_t **g_session, trsess_t *session)
{
	trsess_t *pre_sess = NULL;
	trsess_t *t_sess = *g_session;


	if(session == NULL)
	{
		return -1;
	}
	else
	{
		session->next = NULL;
	}

	while(t_sess != NULL)
	{
		if(strcmp(t_sess->name, session->name))
		{
			pre_sess = t_sess;
			t_sess = t_sess->next;
		}
		else
		{
			return 1;
		}
	}

	session->next = *g_session;
	*g_session = session;

	return 0;
}

trsess_t *query_trans_session(trsess_t *g_session, char *name)
{
	trsess_t *t_sess = g_session;

	while(t_sess != NULL)
	{
		if(strcmp(t_sess->name, name))
		{
			t_sess = t_sess->next;
		}
		else
		{
			return t_sess;
		}
	}

	return NULL;
}

int del_trans_session(trsess_t **g_session, char *name)
{
	trsess_t *pre_sess = NULL;
	trsess_t *t_sess = *g_session;

	while(t_sess != NULL)
	{
		if(strcmp(t_sess->name, name))
		{
			pre_sess = t_sess;
			t_sess = t_sess->next;
		}
		else
		{
			if(pre_sess != NULL)
			{
				pre_sess->next = t_sess->next;
			}
			else
			{
				*g_session = t_sess->next;
			}

			free(t_sess);
			return 0;
		}
	}

	return -1;
}

void session_free(trsess_t *g_session)
{
	trsess_t *m_session = g_session;
	while(m_session != NULL)
	{
		//trsess_print(m_session);
		trsess_t *t_session = m_session;
		m_session = m_session->next;

		if(t_session->tocol == UT_TCP && t_session->mode == UM_MASTER)
		{
			tcp_conn_t *t_conn = (tcp_conn_t *)t_session->arg;
			while(t_conn != NULL)
			{
				tcp_conn_t *pre_conn = t_conn;
				t_conn = t_conn->next;
				free(pre_conn);
			}
		}

		free(t_session);
	}
}

int transcomm_thread_create(trsess_t *session)
{
	if(session == NULL)
	{
		return -1;
	}

	int ret = serial_init(session);

	switch(ret)
	{
	case 0:
		AO_PRINTF("[%s]\n    status: OK\n\n", session->name);
		break;

	case 1:
		AO_PRINTF("[%s]\n    status: disabled\n\n", session->name);
		break;

	case -2:
		AO_PRINTF("[%s]\n    status: open \"%s\" error\n\n", session->name, session->dev);
		break;

	case -3:
		AO_PRINTF("[%s]\n    status: bind server %d error\n\n", session->name, session->port);
		break;

	case -4:
	{
		struct in_addr maddr;
		maddr.s_addr = session->ip;
		AO_PRINTF("[%s]\n    status: client connect %s:%d error\n\n", session->name, inet_ntoa(maddr), session->port);
	}
		break;

	default:
		break;
	}
}

void trsess_print(trsess_t *session)
{
	if(session == NULL)
	{
		AI_PRINTF("(NULL)\n");
		return;
	}

	AI_PRINTF("session [%s]\n\tdev:\t%s\n\ttocol:\t%s\n\tmode:\t%s\n\tip:\t%04X\n\tport:\t%d\n\tisactive:\t%d\n\ttimeout:\t%d\n\tenabled:\t%d\n\n",
				session->name,
				session->dev,
				get_utocol_tostr(session->tocol),
				get_umode_tostr(session->mode),
				session->ip,
				session->port,
				session->isactive,
				session->timeout,
				session->enabled);
}
