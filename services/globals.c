/* globals.c
 *
 * Sam Chen <xuejian1354@163.com>
 *
 */
#include "globals.h"
#include <pthread.h>
#include <errno.h>
#include <dirent.h>
#include <signal.h>
#include <arpa/inet.h>
#include <session/protocol.h>
#include <module/serial.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
	LINE_NAME,
	LINE_MODE,
	LINE_DEV,
	LINE_SPEED,
	LINE_STOPBITS,
	LINE_IP,
	LINE_PORT,
	LINE_LPORT,
	LINE_ENABLED,
	LINE_NONE
}line_data_type_t;

typedef struct
{
	line_data_type_t type;
	char data[64];
}line_data_t;

static char current_time[64];
static char conf_file[128] = {0};
static int end_flag = 1;

static void end_handler(int sig);

#ifdef READ_CONF_FILE
line_data_t *get_linehandle(char *buf, int len);
#endif

int get_end()
{
	return end_flag;
}

int start_params(int argc, char **argv)
{
	int ch;
	opterr = 0; 
	sprintf(conf_file, "%s", CONF_FILE);

	const char *optstrs = "c:h";

    while((ch = getopt(argc, argv, optstrs)) != -1)
    {
		switch(ch)
		{
		case 'h':
			AI_PRINTF("Usage: %s [-c <Configuration files>]\n", argv[0]);
			AI_PRINTF("Default: \n\t-c %s\n", CONF_FILE);
			return 1;

		case '?':
			AI_PRINTF("Unrecognize arguments.\n");
			AI_PRINTF("\'%s -h\' get more help infomations.\n", argv[0]);
			return 1;

		case 'c':
			memset(conf_file, 0, sizeof(conf_file));
			sprintf(conf_file, "%s", optarg);
			break;
		}
	}

	return 0;
}

#ifdef READ_CONF_FILE

line_data_type_t get_line_data_type_fromstr(char *str)
{
	if(!strcmp(str, "mode"))
	{
		return LINE_MODE;
	}
	else if(!strcmp(str, "dev"))
	{
		return LINE_DEV;
	}
	else if(!strcmp(str, "speed"))
	{
		return LINE_SPEED;
	}
    else if(!strcmp(str, "stopbits"))
	{
		return LINE_STOPBITS;
	}
	else if(!strcmp(str, "ip"))
	{
		return LINE_IP;
	}
	else if(!strcmp(str, "port"))
	{
		return LINE_PORT;
	}
	else if(!strcmp(str, "lport"))
	{
		return LINE_LPORT;
	}
	else if(!strcmp(str, "enabled"))
	{
		return LINE_ENABLED;
	}

	return LINE_NONE;
}


int conf_read_from_file()
{
	FILE *fp = NULL;
	char buf[1024] = {0};

	if((fp = fopen(conf_file, "r")) != NULL)
	{
		trsess_t *t_session = NULL;

		while(fgets(buf, sizeof(buf), fp) != NULL)
		{
			line_data_t *ldata = get_linehandle(buf, strlen(buf));
			if(ldata != NULL)
			{
				switch(ldata->type)
				{
				case LINE_NAME:
					t_session = calloc(1, sizeof(trsess_t));
					strcpy(t_session->name, ldata->data);
					strcpy(t_session->dev, "/dev/ttyS1");
					t_session->speed = 115200;
					t_session->tocol = UT_TCP;
					t_session->mode= UM_MASTER;
					t_session->ip = inet_addr("0.0.0.0");
					t_session->port = 8888;
					t_session->lport = 10000;
					t_session->isactive = 1;
					t_session->timeout = 0;
					t_session->parent = NULL;
					t_session->refd = -1;
					t_session->enabled = 0;
					t_session->arg = NULL;

					if(add_global_session(t_session) != 0)
					{
						free(t_session);
						t_session = query_global_session(ldata->data);
					}
					break;

				case LINE_MODE:
					if(t_session != NULL)
					{
						t_session->mode = get_umode_fromstr(ldata->data);
					}
					break;

				case LINE_DEV:
					if(t_session != NULL)
					{
						strcpy(t_session->dev, ldata->data);
					}
					break;

				case LINE_SPEED:
					if(t_session != NULL)
					{
						t_session->speed = atoi(ldata->data);
					}
					break;
				case LINE_STOPBITS:
						if(t_session != NULL)
						{
							t_session->stopbits= atoi(ldata->data);
						}
						break;

				case LINE_IP:
					if(t_session != NULL)
					{
						t_session->ip = inet_addr(ldata->data);
					}
					break;

				case LINE_PORT:
					if(t_session != NULL)
					{	
						t_session->port = atoi(ldata->data);
						AI_PRINTF("t_session->port %d ;\n",t_session->port);
					}
					break;
				case LINE_LPORT:
						if(t_session != NULL)
						{
							t_session->lport = atoi(ldata->data);
							AI_PRINTF("t_session->lport %d ;\n",t_session->lport);							
						}
						break;

				case LINE_ENABLED:
					if(t_session != NULL)
					{
						t_session->enabled = atoi(ldata->data);
					}
					break;

				case LINE_NONE:
					break;
				}

				free(ldata);
			}

			memset(buf, 0, sizeof(buf));
		}

		fclose(fp);
	}
	else
	{
		AI_PRINTF("%s()%d :  Read \"%s\" error, please set configuration file\n", 
			__FUNCTION__, __LINE__, 
			conf_file);
		return -1;
	}

	return 0;
}

line_data_t *get_linehandle(char *buf, int len)
{
	int i, s;
	int is_ignore = 0;

	int cmd_ph, cmd_pt, val_ph, val_pt;
	char p_isset = 0x07;

	if(buf == NULL || len < 3)
	{
		return NULL;
	}

	if(buf[0] == '[')
	{
		int x, isFind = 0;

		for(x=0; x<len; x++)
		{
			if(buf[x] == ']')
			{
				isFind = 1;
				break;
			}
		}

		if(isFind && x>1)
		{
			line_data_t *line_data = (line_data_t *)calloc(1, sizeof(line_data_t));
			line_data->type = LINE_NAME;
			memcpy(line_data->data, buf+1, x-1);
			return line_data;
		}
	}

	for(i=0; i<len; i++)
	{	
		switch(*(buf+i))
		{
		case ' ':
			break;

		case '#':
			if(!is_ignore)
			{
				return NULL;
			}

		default:
			is_ignore = 1;
			if(p_isset & 0x01)
			{
				cmd_ph = i;
				p_isset &= ~0x01;
			}
			else if(p_isset & 0x02)
			{
				if(*(buf+i) == '=')
				{
					int j;
					for(j=i-1; j>=cmd_ph; j--)
					{
						if(*(buf+j) != ' ')
						{
							cmd_pt = j+1;
							p_isset &= ~0x02;
							break;
						}
					}
				}
			}
			else if(p_isset & 0x04)
			{
				val_ph = i;
				p_isset &= ~0x04;
			}

			break;
		}
	}

	for(s=len-2; s>=val_ph; s--)
	{
		if(*(buf+s) != ' ')
		{
			val_pt = s+1;
			break;
		}
	}

	if((cmd_ph < cmd_pt)
		&& (cmd_pt < val_ph)
		&& (val_ph < val_pt))
	{
		char cmd[64] = {0};
		char val[64] = {0};

		memcpy(cmd, buf+cmd_ph, cmd_pt-cmd_ph);
		memcpy(val, buf+val_ph, val_pt-val_ph);

		line_data_t *line_data = (line_data_t *)calloc(1, sizeof(line_data_t));
		line_data->type = get_line_data_type_fromstr(cmd);
		strcpy(line_data->data, val);
		return line_data;
	}

	return NULL;
}
#endif

void process_signal_register()
{
	 signal(SIGINT, end_handler);//来自键盘的中断信号 ctrl+C
	 signal(SIGTSTP, end_handler);//
}

void end_handler(int sig)
{
	switch(sig)
	{
	case SIGINT:
		AI_PRINTF(" SIGINT\t");
		break;

	case SIGTSTP:
		AI_PRINTF(" SIGTSTP\t");
		break;
	}

	AI_PRINTF("\n");

	session_free(get_global_session());
	serial_dev_free();

	end_flag = 0;
}

#ifdef DAEMON_PROCESS_CREATE
static int daemon_cmdline_flag = 0;

int daemon_init()
{
    int pid;
	
    if(pid = fork())
	{
		return 1;
    }
	else if(pid < 0)
	{
		AI_PRINTF("%s()%d : excute failed\n", __FUNCTION__, __LINE__);
		return -1;
	}

    setsid();
	
	if(pid = fork())
	{
		return 1;
	}
    else if(pid < 0)
	{
		AI_PRINTF("%s()%d : excute failed\n", __FUNCTION__, __LINE__);
		return -1;
    }

	int i;
	int fdtablesize = getdtablesize();
    for(i = 0; i < fdtablesize; i++)
    {
        close(i);
    }
	
    int ret = chdir("/tmp");
    umask(0);

	daemon_cmdline_flag = 1;
    return 0;
}

int get_daemon_cmdline()
{
	return daemon_cmdline_flag;
}
#endif

int mach_init()
{
	system("rm -f /tmp/uartsocket_status.dat");

	trsess_t *g_session = get_global_session();
	trsess_t *t_session = g_session;

	while(t_session != NULL)
	{
		//trsess_print(t_session);
		transcomm_thread_create(t_session);
		t_session = t_session->next;
	}

	return 0;
}

char *get_current_time()
{
	time_t t;
	time(&t);
	bzero(current_time, sizeof(current_time));
	struct tm *tp= localtime(&t);
	strftime(current_time, 100, "%Y-%m-%d %H:%M:%S", tp); 

	return current_time;
}

char *get_system_time()
{
	struct timeval time;
	gettimeofday(&time, NULL);
	bzero(current_time, sizeof(current_time));
	sprintf(current_time, "%u", (uint32)time.tv_sec);

	return current_time;
}

#ifdef __cplusplus
}
#endif

