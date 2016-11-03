/*
 * main.c
 *
 * Sam Chen <xuejian1354@163.com>
 *
 */
#include <services/globals.h>
#include <module/serial.h>

#ifdef __cplusplus
extern "C" {
#endif

int main(int argc, char **argv)
{
	if(start_params(argc, argv) != 0)
	{
		return 1;
	}

#ifdef READ_CONF_FILE
	if(conf_read_from_file() < 0)
	{
		return -1;
	}
#endif

	process_signal_register();

	AI_PRINTF("%s Start!\n", TARGET_NAME);

#ifdef DAEMON_PROCESS_CREATE
	signal(SIGCHLD, SIG_IGN);
    if(daemon_init() != 0)
    {
		return 0;
	}
#endif

	if(mach_init() < 0)
	{
		return -1;
	}

	if(get_serial_dev() != NULL)
	{
		while(get_end())
		{
			sleep(1);
		}
	}
	else
	{
		AI_PRINTF("No serial device enable\n");
	}

	AI_PRINTF("%s End!\n", TARGET_NAME);
	return 0;
}

#ifdef __cplusplus
}
#endif
