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

#include "log.h"

#include "serial.h"

#ifndef CR_PORT
#define CR_PORT			"/dev/ttyS0"
#endif

#define CR_BAUDRATE		B115200

#define	TAG			"Ser. (%d):\t"

static int	fd = -1;
static int	msg_id = 0;

static struct termios	orgopt, curopt;

extern double rowCount, columnCount;

void serial_init() {
	char buf[1024];

	if( fd != -1 )
		return;

	sprintf(buf, "%s", CR_PORT);
	fd = open(buf, O_RDWR | O_NOCTTY | O_NDELAY);
	if (fd == -1)
		return;

	fcntl(fd, F_SETFL, FNDELAY);

	tcgetattr(fd, &orgopt);
	tcgetattr(fd, &curopt);

	cfsetispeed(&curopt, CR_BAUDRATE);
	cfsetospeed(&curopt, CR_BAUDRATE);

	/* Mostly 8N1 */
	curopt.c_cflag &= ~PARENB;
	curopt.c_cflag &= ~CSTOPB;
	curopt.c_cflag &= ~CSIZE;
	curopt.c_cflag |= CS8;
	curopt.c_cflag |= CREAD;
	curopt.c_cflag |= CLOCAL;	//disable modem statuc check

	cfmakeraw(&curopt);		//make raw mode

	tcsetattr(fd, TCSANOW, &curopt);

	log_msg(LOG_TRACE, "Serial port (%s): initialized, fd(%d)!\n", buf, fd);
	read(fd, buf, 1024);
}

void serial_close()
{
	char buf[128];

	if (fd == -1)
		return;

	read(fd, buf, 128);
	tcsetattr(fd, TCSANOW, &orgopt);
	close(fd);
	fd = -1;
}

int serial_write(uint8_t len, uint8_t *buf) {
	int	retval;

	log_msg(LOG_VERBOSE, TAG "Output %d byte(s)\n", __LINE__, len);
	retval = write(fd, buf, len);
	fflush(NULL);

	return retval;
}

#if 0
SerialCommandType serial_read() {
	int	i, val, nread, x, y, value, xtmp, ytmp;
	char	c, buf[512];
	fd_set	rfds;
	SerialCommandType	cmd = SERIAL_NONE;

	if (fd == -1)
		goto done;

	nread = 0;

#if 1
	if ((val = ioctl(fd, 0x541B, &nread)) < 0)
		perror("ioctl error");
#endif
	if (nread > 0) {
		//fprintf(stdout, "readsize %d\n", nread);
		goto no_select;
	} else {
		//printf("Buffer has no data.\n");
		//goto done;
	}

	FD_ZERO(&rfds);
	FD_SET(fd, &rfds);
	val = select(fd + 1, &rfds, NULL, NULL, NULL);

	if  (val == -1) {
		log_msg(LOG_VERBOSE, TAG "Select Error.\n", __LINE__);
		goto done;
	} else if (val == 0) {
		log_msg(LOG_VERBOSE, TAG "No data within one seconds\n", __LINE__);
		goto done;
	}

no_select:
	usleep(1* 50* 1000);
	memset(buf, 0, sizeof(char) * strlen(buf));
	nread = read(fd, buf, 4);

	if (strncmp(buf, "$UPD", 4) == 0) {
		log_msg(LOG_VERBOSE, TAG "Updating Map\n", __LINE__);
		i = 0;
		memset(buf, 0, sizeof(char) * strlen(buf));
		xtmp = ytmp = -1;
		while (1) {
again:
			val = read(fd, &c, 1);
			if (val <= 0) {
				//printf("buf %s\n", buf);
				usleep(1* 50* 1000);
				goto again;
			}

			if (c == '#') {
				log_msg(LOG_VERBOSE, TAG "Set current position (%d,%d)\n", __LINE__, xtmp, ytmp);
				break;
			}

			if (c == ';') {
				buf[i] = c;
				if (sscanf(buf, "%d,%d,%x;", &x, &y, &value) == 3) {
					if (value == 2 || value == 1 || value == 0) {
						log_msg(LOG_VERBOSE, TAG "Set obstcal (%d,%d %d)\n", __LINE__, x, y, value);
					} else if (value == 10) {
						log_msg(LOG_VERBOSE, TAG "Set current position (%d,%d)\n", __LINE__, x, y);
						xtmp = x;
						ytmp = y;
					} else if (value == 11) {
						log_msg(LOG_VERBOSE, TAG "Set current position (%d,%d)\n", __LINE__, x, y);
						xtmp = x;
						ytmp = y;
					} else if (value == 12) {
						log_msg(LOG_VERBOSE, TAG "Set current position (%d,%d)\n", __LINE__, x, y);
						xtmp = x;
						ytmp = y;
					} else if (value == 13) {
						log_msg(LOG_VERBOSE, TAG "Set current position (%d,%d)\n", __LINE__, x, y);
						xtmp = x;
						ytmp = y;
					}
				}
				i = 0;
				memset(buf, 0, sizeof(char) * strlen(buf));
			} else {
				buf[i++] = c;
			}
		}
		cmd = SERIAL_UPDATE;
	} else if (strncmp(buf, "$RS#", 4) == 0) {
		log_msg(LOG_VERBOSE, TAG "Resetting Map\n", __LINE__);
		cmd = SERIAL_RESET;
		msg_id = 0;
	}

done:
	log_msg(LOG_VERBOSE, TAG "Return cmd %d\n", __LINE__, cmd);
	return cmd;
}

#endif


