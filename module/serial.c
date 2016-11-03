/*
 * serial.c
 *
 * Sam Chen <xuejian1354@163.com>
 *
 */
#include "serial.h"
#include <termios.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <module/netlist.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>

#define SYSCFG0_OFFSET  0x00000010
#define GPIOMODE_OFFSET 0x00000060/* mode bits */
#define GPIOMODE_GPIO_MODE (0x00000001 << 9) 
#define SYSCFG0_JTAG_MODE  (0x00000001 << 7)
//GPIO24
#define GPI24_DATA_OFFSET 0x00000648
#define GPI24_DIR_OFFSET  0x0000064C
//GPIO25
#define GPI25_DATA_OFFSET 0x00000648
#define GPI25_DIR_OFFSET  0x0000064C
//GPIO26
#define GPI26_DATA_OFFSET 0x00000648
#define GPI26_DIR_OFFSET  0x0000064C
//GPIO27
#define GPI27_DATA_OFFSET 0x00000648
#define GPI27_DIR_OFFSET  0x0000064C


static int led_init(int enable)
{
	int mfd = -1;	
	void *base = (void *)-1;	
	volatile unsigned int *gpiomode/*, *syscfg0*/;	
	//volatile unsigned int *gpio72_dir, *gpio72_data;	
	//volatile unsigned int *gpio09_dir, *gpio09_data;
	//	volatile unsigned int *gpio43_dir, *gpio43_data;
	//	volatile unsigned int *gpio44_dir, *gpio44_data;	

	volatile unsigned int *gpio24_dir, *gpio24_data;	
	volatile unsigned int *gpio25_dir, *gpio25_data;	
	volatile unsigned int *gpio26_dir, *gpio26_data;	
	volatile unsigned int *gpio27_dir, *gpio27_data;			
	mfd = open("/dev/mem", O_RDWR);	
	if (mfd < 0) 
		{		
		printf("error: %s\n", strerror(errno));		
		return -1;	
		}	
	base = mmap(NULL, 0x1000, PROT_READ | PROT_WRITE, MAP_SHARED, mfd, 0x10000000);	
	if (base == (void *)-1) 
		{		
		printf("err: %s\n", strerror(errno));		
		close(mfd);		
		return -1;	
		}	
	gpiomode = base + GPIOMODE_OFFSET;
	gpio24_dir  =  base + GPI24_DIR_OFFSET;
	gpio24_data =  base + GPI24_DATA_OFFSET;	
	gpio25_dir  =  base + GPI25_DIR_OFFSET;	
	gpio25_data =  base + GPI25_DATA_OFFSET;	
	gpio26_dir  =  base + GPI26_DIR_OFFSET;	
	gpio26_data =  base + GPI26_DATA_OFFSET;	
	gpio27_dir  =  base + GPI27_DIR_OFFSET;	
	gpio27_data =  base + GPI27_DATA_OFFSET;		
	//39-24	
	*gpio24_dir |= (0x01 << 0);	
	if (enable)		
		*gpio24_data &= ~(0x01 << 0);	
	else		
		*gpio24_data |= (0x01 << 0);			
	*gpio25_dir |= (0x01 << 1);	
	if (enable)		
		*gpio25_data &= ~(0x01 << 1);	
	else		
		*gpio25_data |= (0x01 << 1);			
	*gpio26_dir |= (0x01 << 2);		
	if (enable)		
		*gpio26_data &= ~(0x01 << 2);	
	else		
		*gpio26_data |= (0x01 << 2);			
	*gpio27_dir |= (0x01 << 3);		
	if (enable)		
		*gpio27_data &= ~(0x01 << 3);	
	else		
		*gpio27_data |= (0x01 << 3);					

		*gpiomode |= GPIOMODE_GPIO_MODE;
	munmap(base, 0x1000);	
	close(mfd);	
	return 0;
		}


#ifdef __cplusplus
extern "C" {
#endif

serial_dev_t *g_serial_dev = NULL;

static int serial_open(char *dev);
static int set_serial_params(int fd, uint32 speed, uint8 databit, uint8 stopbit, uint8 parity);

static void *uart_read_handler(void *p);
static void *tcpserver_accept_handler(void *p);
static void *tcpserver_read_handler(void *p);
static void *tcpclient_read_handler(void *p);
static void *udpserver_read_handler(void *p);
static void *udpclient_read_handler(void *p);

serial_dev_t *get_serial_dev()
{
	return g_serial_dev;
}

int add_serial_dev(serial_dev_t *t_serial_dev)
{
	serial_dev_t *pre_dev = NULL;
	serial_dev_t *t_dev = g_serial_dev;

	if(t_serial_dev == NULL)
	{
		return -1;
	}
	else
	{
		t_serial_dev->next = NULL;
	}

	while(t_dev != NULL)
	{
		if(strcmp(t_dev->dev, t_serial_dev->dev))
		{
			pre_dev = t_dev;
			t_dev = t_dev->next;
		}
		else
		{
			return 1;
		}
	}

	t_serial_dev->next = g_serial_dev;
	g_serial_dev = t_serial_dev;

	return 0;
}

serial_dev_t *query_serial_dev(char *dev)
{
	serial_dev_t *t_sesial_dev = g_serial_dev;

	while(t_sesial_dev != NULL)
	{
		if(strcmp(t_sesial_dev->dev, dev))
		{
			t_sesial_dev = t_sesial_dev->next;
		}
		else
		{
			return t_sesial_dev;
		}
	}

	return NULL;
}

int del_serial_dev(char *dev)
{
	serial_dev_t *pre_serial_dev = NULL;
	serial_dev_t *t_serial_dev = g_serial_dev;

	while(t_serial_dev != NULL)
	{
		if(strcmp(t_serial_dev->dev, dev))
		{
			pre_serial_dev = t_serial_dev;
			t_serial_dev = t_serial_dev->next;
		}
		else
		{
			if(pre_serial_dev != NULL)
			{
				pre_serial_dev->next = t_serial_dev->next;
			}
			else
			{
				g_serial_dev = t_serial_dev->next;
			}

			free(t_serial_dev);
			return 0;
		}
	}

	return -1;
}

void serial_dev_free()
{
	serial_dev_t *m_serial_dev = g_serial_dev;
	while(m_serial_dev != NULL)
	{
		serial_dev_t *t_serial_dev = m_serial_dev;
		m_serial_dev = m_serial_dev->next;
		session_free(t_serial_dev->session);
		free(t_serial_dev);
	}

	g_serial_dev = NULL;
}
//led_init(0) 发送高电平
//led_init(1) 接收低电平
int serial_init(trsess_t *session)
{
	led_init(1);
	AI_PRINTF("session->lport %d ;\n",session->lport);

	if(session == NULL)
	{
		return -1;
	}

	if(!session->enabled)
	{
		return 1;
	}

	serial_dev_t *m_serial_dev = query_serial_dev(session->dev);
	if(m_serial_dev == NULL)
	{
		serial_dev_t *t_serial_dev = calloc(1, sizeof(serial_dev_t));
		strcpy(t_serial_dev->dev, session->dev);
		t_serial_dev->speed = session->speed;
		t_serial_dev->num = 0;
		t_serial_dev->session = NULL;
		t_serial_dev->stopbits = session->stopbits;
		if(add_serial_dev(t_serial_dev) != 0)
		{
			free(t_serial_dev);
			t_serial_dev = query_serial_dev(session->dev);
		}

		m_serial_dev = t_serial_dev;

		if ((t_serial_dev->serial_fd=serial_open(session->dev)) < 0
			|| set_serial_params(t_serial_dev->serial_fd, t_serial_dev->speed, 8, t_serial_dev->stopbits, 0) < 0)
		{
			del_serial_dev(session->dev);
			return -2;
		}
		//led_init(0);
		int ret = write(t_serial_dev->serial_fd, "(^_^)(^_^)", 10);	//just enable serial port, no pratical meaning
		//led_init(0);

		pthread_t uartRead;
		pthread_create(&uartRead, NULL, uart_read_handler, (void *)t_serial_dev);
	}

	trsess_t *t_session = calloc(1, sizeof(trsess_t));
	memcpy(t_session, session, sizeof(trsess_t));
	t_session->parent = m_serial_dev;

	int ret = add_trans_session(&m_serial_dev->session, t_session);
	if(ret != 0)
	{
		free(t_session);
		t_session = query_trans_session(m_serial_dev->session, session->name);
	}
	else
	{
		m_serial_dev->num++;

		if(t_session->tocol == UT_TCP)
		{
			if(t_session->mode == UM_MASTER)
			{
				struct sockaddr_in reserver_addr;
				reserver_addr.sin_family = PF_INET;
				reserver_addr.sin_port = htons(t_session->port);
				reserver_addr.sin_addr.s_addr = htonl(INADDR_ANY);

				if ((t_session->refd = socket(PF_INET, SOCK_STREAM, 0)) < 0
					|| bind(t_session->refd, (struct sockaddr *)&reserver_addr, sizeof(struct sockaddr)) < 0)
				{
					perror("reser socket fail");
					return -3;
				}

				listen(t_session->refd, 5);

				pthread_t reserAccept;
				pthread_create(&reserAccept, NULL, tcpserver_accept_handler, (void *)t_session);
				
			}
			else if(t_session->mode == UM_SLAVE)
			{
				struct sockaddr_in reclient_addr;
				reclient_addr.sin_family = PF_INET;

				/*
				struct sockaddr_in reclient_addr,mine;

				bzero(&mine,sizeof(mine));
				mine.sin_family = AF_INET;
				mine.sin_port = htons(3334);
				inet_pton(AF_INET,"192.168.1.26",&mine.sin_addr);
			    int b = bind(t_session->refd,(struct sockaddr*)&mine,sizeof(mine));
			    if(b==-1)perror("");					
*/
				
				reclient_addr.sin_port = htons(t_session->port);
				reclient_addr.sin_addr.s_addr = t_session->ip;

		/*		if ((t_session->refd = socket(PF_INET, SOCK_STREAM, 0)) < 0
					|| connect(t_session->refd, (struct sockaddr *)&reclient_addr, sizeof(reclient_addr)) < 0)
				{
					perror("recli socket fail");
					return -4;
				}
		*/		
				pthread_t recliAccept;
				pthread_create(&recliAccept, NULL, tcpclient_read_handler, (void *)t_session);
			}
		}
/*		else if(t_session->tocol == UT_UDP)
		{
			if(t_session->mode == UM_MASTER)
			{}
			else if(t_session->mode == UM_SLAVE)
			{}
		}
*/		
	}

	return 0;
}

void *uart_read_handler(void *p)
{
	unsigned char rbuf[2048];
	int rlen;

	serial_dev_t *m_serial_dev = (serial_dev_t *)p;
	
    while(1)
    {
        memset(rbuf, 0, sizeof(rbuf));
    	rlen = read(m_serial_dev->serial_fd, rbuf, sizeof(rbuf));
		trsess_t *m_session = m_serial_dev->session;
		while(m_session != NULL)
		{
			if(m_session->tocol == UT_TCP && m_session->mode == UM_MASTER)
			{
				tcp_conn_t *t_conn = (tcp_conn_t *)m_session->arg;
				//led_init(0);
				while(t_conn != NULL)
				{
					int ret = write(t_conn->fd, rbuf, rlen);
					t_conn = t_conn->next;
				}
				//led_init(1);

			}
			else if(m_session->tocol == UT_TCP && m_session->mode == UM_SLAVE)
			{
				//led_init(0);
				int ret = write(m_session->refd, rbuf, rlen);
				//led_init(1);
			}

			m_session = m_session->next;
		}
    }
}

void *tcpserver_accept_handler(void *p)
{
	int rw;
	struct sockaddr_in client_addr;
	socklen_t len = sizeof(client_addr);

	trsess_t *m_session = (trsess_t *)p;

	while(1)
	{
		rw = accept(m_session->refd, (struct sockaddr *)&client_addr, &len);

		tcp_conn_t *t_conn = calloc(1, sizeof(tcp_conn_t));
		t_conn->fd = rw;
		t_conn->client_addr = client_addr;
		t_conn->parent = p;
		t_conn->next = NULL;

		if(addto_tcpconn_list((tcp_conn_t **)&m_session->arg, t_conn) != 0)
		{
			free(t_conn);
			t_conn = queryfrom_tcpconn_list((tcp_conn_t *)m_session->arg, rw);
		}
		
		pthread_t reserRead;
		pthread_create(&reserRead, NULL, tcpserver_read_handler, (void *)t_conn);
	}
}

void *tcpserver_read_handler(void *p)
{
	int nbytes;
	char buf[2048];

	tcp_conn_t *m_conn = (tcp_conn_t *)p;
	int isStart = 1;

	while(isStart)
	{
		memset(buf, 0, sizeof(buf));
	   	if ((nbytes = recv(m_conn->fd, buf, sizeof(buf), 0)) <= 0)
	   	{
	      	close(m_conn->fd);
			delfrom_tcpconn_list((tcp_conn_t **)&((trsess_t *)m_conn->parent)->arg, m_conn->fd);
			isStart = 0;
		}
		else
		{
			int ret = write(((serial_dev_t *)((trsess_t *)m_conn->parent)->parent)->serial_fd, buf, nbytes);
		}
	}
}

void *tcpclient_read_handler(void *p)
{
	trsess_t *m_session = (trsess_t *)p;
	int on,ret;
	int num = 0;
	static int sin_port = 10001;
	struct sockaddr_in mine;
	struct sockaddr_in reclient_addr;
	int nbytes,relink  = 1;
	char buf[2048];

	int isStart = 1;



	reclient_addr.sin_family = PF_INET;
	reclient_addr.sin_port = htons(m_session->port);
	reclient_addr.sin_addr.s_addr = m_session->ip;
	if(m_session == NULL)
	{
		return NULL;
	}



	//while((m_session->refd = socket(PF_INET, SOCK_STREAM, 0)) < 0); 					


/*	on = 1;
    ret = setsockopt( m_session->refd, SOL_SOCKET, SO_REUSEADDR, &on, sizeof(on) );
	bzero(&mine,sizeof(mine));
	mine.sin_family = PF_INET;
	mine.sin_port = htons(10001);//m_session->lport
	mine.sin_addr.s_addr = htonl(INADDR_ANY);
	//inet_pton(PF_INET,"192.168.1.26",&mine.sin_addr);
	//inet_pton(PF_INET,"192.168.1.26",&reclient_addr.sin_addr);
	int b = bind(m_session->refd,(struct sockaddr*)&mine,sizeof(mine));
	if(b==-1)
	{
		AI_PRINTF("bind error ;\n");
		perror("");

	 }	
	 

     inet_pton(PF_INET,"192.168.1.26",&reclient_addr.sin_addr);
*/
	 
//	while((m_session->refd = socket(PF_INET, SOCK_STREAM, 0)) < 0
//		|| connect(m_session->refd, (struct sockaddr *)&reclient_addr, sizeof(reclient_addr)) < 0);




	while(isStart)
	{
		if(relink == 1)
		{
		    while((m_session->refd = socket(PF_INET, SOCK_STREAM, 0)) < 0)
				{
					//usleep(100);
				}; 

			    on = 1;
				ret = setsockopt( m_session->refd, SOL_SOCKET, SO_REUSEADDR, &on, sizeof(on) );
				bzero(&mine,sizeof(mine));
				mine.sin_family = PF_INET;
				mine.sin_port = htons(m_session->lport);//
				mine.sin_addr.s_addr = htonl(INADDR_ANY);
				//inet_pton(PF_INET,"192.168.1.26",&mine.sin_addr);
				//inet_pton(PF_INET,"192.168.1.26",&reclient_addr.sin_addr);
				AI_PRINTF("lport %d ;\n",m_session->lport);
				int b = bind(m_session->refd,(struct sockaddr*)&mine,sizeof(mine));
				if(b==-1)
				{
					AI_PRINTF("bind error ;\n");
					perror("");
			
				}	
				 
			
				// inet_pton(PF_INET,"192.168.1.26",&reclient_addr.sin_addr);


				AI_PRINTF("relink = 1;\n");

			//


				//  num = connect(m_session->refd, (struct sockaddr *)&reclient_addr, sizeof(reclient_addr)) ;
			    //  AI_PRINTF("connect num %d;\n",num);
			   while(connect(m_session->refd, (struct sockaddr *)&reclient_addr, sizeof(reclient_addr)) < 0)
				{				  
				  
					//usleep(100);
			    }
				
			relink = 0;
			AI_PRINTF("relink = 0;\n");
		}
		else
		{
			memset(buf, 0, sizeof(buf));
			//if ((nbytes = read(m_session->refd, buf, sizeof(buf))) <= 0)
			if ((nbytes = recv(m_session->refd, buf, sizeof(buf),0)) <= 0)
			{
				if(nbytes == EINTR || nbytes == EWOULDBLOCK ||errno == EAGAIN)
				{
					//close(m_session->refd);
					isStart = 0;
					AI_PRINTF("relink = 0;\n");
					
				}
				else
				{
				  close(m_session->refd);
				  AI_PRINTF("relink = 1;\n");
			      relink = 1;
				}
			
			}
			else
			{
				int ret = write(((serial_dev_t *)(m_session->parent))->serial_fd, buf, nbytes);
			}

		}



	}
}

void *udpserver_read_handler(void *p)
{}

void *udpclient_read_handler(void *p)
{}

int serial_open(char *dev)
{
	int fd;
	if(NULL == dev)
	{
		perror("Can't open serial port");
		return -1;
	}
	
    fd = open(dev, O_RDWR | O_NOCTTY);

	if(fd < 0)
	{
		perror("Can't open serial port");
	}
	
    return fd;
}

int set_serial_params(int fd, uint32 speed, uint8 databit, uint8 stopbit, uint8 parity)
{
    int iSpeed = 0;
    struct termios options;
    tcgetattr(fd, &options);

    cfmakeraw(&options);
    //Set Baudrate
    switch(speed) {
        case 38400:
            iSpeed = B38400;
            break;
        case 19200:
            iSpeed = B19200;
            break;
        case 115200:
            iSpeed = B115200;
            break;
		case 57600:
			iSpeed = B57600;
			break;
        case 9600:
            iSpeed = B9600;
            break;
        case 4800:
            iSpeed = B4800;
            break;
        case 2400:
            iSpeed = B2400;
            break;
        case 1200:
            iSpeed = B1200;
            break;
        case 300:
            iSpeed = B300;
            break;
        default:
            perror("Unsupport Baudrate");
            return -1;
    }
    cfsetispeed(&options,iSpeed);
    cfsetospeed(&options,iSpeed);

    //  Set DataBits
    options.c_cflag &= ~CSIZE;
    switch(databit) {
        case 5:
          options.c_cflag |= CS5;
          break;
        case 6:
            options.c_cflag |= CS6;
            break;
        case 7:
            options.c_cflag |= CS7;
            break;
        case 8:
            options.c_cflag |= CS8;
            break;
        default:
            perror("Unsupported data size");
            return -1;
    }
    //  Set Parity
    switch(parity) {
        case 0:
            options.c_cflag &= ~PARENB;
            options.c_iflag &= ~INPCK;
            break;
        case 1:
            options.c_cflag |= (PARODD | PARENB);
            options.c_iflag |= INPCK;
            break;
        case 2:
            options.c_cflag |= PARENB;
            options.c_cflag &= ~PARODD;
            options.c_iflag |= INPCK;
            break;
        default:
            perror("Unsupported Parity");
            return -1;
    }
	AI_PRINTF("stopbit %d\n",stopbit);
    // Set Stop Bits
    switch(stopbit) {
        case 0:
            options.c_cflag &= ~CSTOPB;
            break;
        case 1:
            options.c_cflag |= CSTOPB;
            break;
        default:
            perror("Unsupported Stop Bits");
            return -1;
    }


    options.c_lflag &= ~(ICANON | ECHO | ECHONL | ECHOE | ISIG | IEXTEN);
    options.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);

    if(tcsetattr(fd, TCSANOW, &options) != 0) {
        perror("Tcsetattr Fail");
        return -1;
    }

    tcflush(fd, TCIFLUSH);
    return 0;
}

#ifdef __cplusplus
}
#endif
