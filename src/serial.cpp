#include <termios.h>
#include <stdio.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <ctype.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>

#include "serial.h"

static int	crport_fd = -1;
static bool serial_init_done = false;

static struct termios	orgopt, curopt;

void serial_init(const char* port, int baudrate) {
	char buf[1024];

	if( crport_fd != -1 )
		return;

	sprintf(buf, "%s", port);
	crport_fd = open(buf, O_RDWR | O_NOCTTY | O_NONBLOCK);
	if (crport_fd == -1)
		return;

	fcntl(crport_fd, F_SETFL, FNDELAY);

	tcgetattr(crport_fd, &orgopt);
	tcgetattr(crport_fd, &curopt);

	speed_t CR_BAUDRATE;
	switch(baudrate){
		case 9600:
			CR_BAUDRATE = B9600;
			break;
		case 19200:
			CR_BAUDRATE = B19200;
			break;
		case 38400:
			CR_BAUDRATE = B38400;
			break;
		case 57600:
			CR_BAUDRATE = B57600;
			break;
		case 115200:
			CR_BAUDRATE = B115200;	
			break;
		case 230400:
			CR_BAUDRATE = B230400;
			break;
		default:
			break;
			
	}
	cfsetispeed(&curopt, CR_BAUDRATE);
	cfsetospeed(&curopt, CR_BAUDRATE);	

	printf("serial speed %d\n",baudrate);
	/* Mostly 8N1 */
	curopt.c_cflag &= ~PARENB;
	curopt.c_cflag &= ~CSTOPB;
	curopt.c_cflag &= ~CSIZE;
	curopt.c_cflag |= CS8;
	curopt.c_cflag |= CREAD;
	curopt.c_cflag |= CLOCAL;	//disable modem statuc check

	cfmakeraw(&curopt);		//make raw mode

	if (tcsetattr(crport_fd, TCSANOW, &curopt) == 0){
		serial_init_done = true;
		printf("serial init done...\n");
	}
	//read(crport_fd, buf, 1024);
}

bool is_serial_ready()
{
	return serial_init_done;
}

int serial_flush()
{
	return fsync(crport_fd);
}

int serial_close()
{
	int reval;
	char buf[128];

	if (crport_fd == -1)
		return -1;

	read(crport_fd, buf, 128);
	tcsetattr(crport_fd, TCSANOW, &orgopt);
	reval = close(crport_fd);
	if (reval == 0) {
		crport_fd = -1;
		serial_init_done = false;
	}
	return reval;
}

int serial_write(uint8_t len, uint8_t *buf) {
	int	retval;

	retval = write(crport_fd, buf, len);
	//fflush(NULL);

	return retval;
}

int serial_read(int len,uint8_t *buf){
	int		r_ret = 0, s_ret;
	fd_set	read_serial_fds;
	uint8_t	t_buf[len];

	struct timeval	timeout;

	timeout.tv_sec = 1;
	timeout.tv_usec = 0;// ms

	while (1){
		FD_ZERO(&read_serial_fds);
		FD_SET(crport_fd,&read_serial_fds);

		s_ret = select(crport_fd + 1, &read_serial_fds, NULL, NULL, &timeout);
		if (s_ret < 0) {
			printf("select error\n");
			break;
		} else if (s_ret > 0) {
			if (FD_ISSET(crport_fd, &read_serial_fds)) {
				usleep(6000);
				r_ret = read(crport_fd, t_buf, len);
				memcpy(buf, t_buf, r_ret);
				/*
				int o_len = 0;
				while (r_ret != len){
					o_len = r_ret;
					len = len - r_ret;
					r_ret = read(crport_fd, t_buf, len);
					memcpy(buf + o_len, t_buf, r_ret);
				}
				*/
				break;
			}
		} else {
			break;
		}
	}
	return s_ret;
}


