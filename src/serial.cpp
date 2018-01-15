#include "ros/ros.h"
#include "boost/thread.hpp"
#include "serial.h"
#include <sys/ioctl.h>
#include <fcntl.h>

boost::mutex g_send_stream_mutex;
boost::mutex g_receive_stream_mutex;

Serial serial;

Serial::Serial()
{
	crport_fd_ = 0;
	serial_init_done_ = false;
	made_table_ = 0;
	sleep_status_ = false;
}

bool Serial::init(const char* port,int baudrate)
{
	char buf[1024];

	bardrate_ = baudrate;
	sprintf(buf, "%s", port);
	crport_fd_ = open(buf, O_RDWR | O_NOCTTY | O_NONBLOCK);
	if (crport_fd_ == -1)
		return false;

	fcntl(crport_fd_, F_SETFL, FNDELAY);

	tcgetattr(crport_fd_, &orgopt_);
	tcgetattr(crport_fd_, &curopt_);
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
			CR_BAUDRATE = B115200;
			break;
	}
	cfsetispeed(&curopt_, CR_BAUDRATE);
	cfsetospeed(&curopt_, CR_BAUDRATE);
	/* Mostly 8N1 */
	curopt_.c_cflag &= ~PARENB;
	curopt_.c_cflag &= ~CSTOPB;
	curopt_.c_cflag &= ~CSIZE;
	curopt_.c_cflag |= CS8;
	curopt_.c_cflag |= CREAD;
	curopt_.c_cflag |= CLOCAL;	//disable modem statuc check

	cfmakeraw(&curopt_);		//make raw mode_

	if (tcsetattr(crport_fd_, TCSANOW, &curopt_) != 0){
		return false;
	}
	serial_init_done_ = true;
	ROS_INFO("\033[32mserial.cpp\033[0m init done...\n");
	return true;
}

bool Serial::is_ready()
{
	return serial_init_done_;
}

int Serial::flush()
{
	return tcflush(crport_fd_,TCIFLUSH);
}

int Serial::close()
{
	char buf[128];
	int retval;
	if (crport_fd_ == -1)
		return -1;

	::read(crport_fd_, buf, 128);
	tcsetattr(crport_fd_, TCSANOW, &orgopt_);
	retval = ::close(crport_fd_);
	if(retval==0){
		crport_fd_ = -1;
		serial_init_done_ = false;
	}
	return retval;
}

void Serial::sleep(void)
{
	sleep_status_ = true;
}

void Serial::wakeUp(void)
{
	sleep_status_ = false;
}

bool Serial::isSleep()
{
	return sleep_status_;
}

int Serial::write(uint8_t len, uint8_t *buf)
{
	int	retval;
	retval = ::write(crport_fd_, buf, len);
	return retval;
}

int Serial::read(int len,uint8_t *buf)
{
	int r_ret=0,s_ret=0;
	uint8_t *t_buf;
	t_buf = (uint8_t*)calloc(len,sizeof(uint8_t));
	//memset(t_buf,0,len);
	fd_set read_fd_set;
	struct timeval timeout;
	timeout.tv_sec = 4;
	timeout.tv_usec = 0;// ms
	size_t length = 0;
	if(is_ready()){
		if(ioctl(crport_fd_,FIONREAD,&length)==-1)
		{
			ROS_WARN("%s,%d,ioctl return -1",__FUNCTION__,__LINE__);
			free(t_buf);
			return -1;
		}
		if( length >= (size_t)len){
			r_ret = ::read(crport_fd_,t_buf,len);
			if(r_ret == len)
				for(int i =0;i<len;i++){
					buf[i] = t_buf[i];
				}
				//memcpy(buf,t_buf,len);
			free(t_buf);
			return r_ret;
		}
	}
	while (is_ready()){
		FD_ZERO(&read_fd_set);
		FD_SET(crport_fd_,&read_fd_set);

		s_ret = select(FD_SETSIZE,&read_fd_set,NULL,NULL,&timeout);
		if (s_ret <0){
			ROS_ERROR("%s %d: -------select error------------", __FUNCTION__, __LINE__);
			free(t_buf);
			FD_CLR(crport_fd_,&read_fd_set);
			return -1;
		}
		else if(s_ret ==0){
			ROS_INFO("%s %d: select function \033[33mtimeout!\033[0m", __FUNCTION__, __LINE__);
			free(t_buf);
			FD_CLR(crport_fd_,&read_fd_set);
			return 0;
		}
		else if(s_ret >0){
			if(FD_ISSET(crport_fd_,&read_fd_set)){
				if(ioctl(crport_fd_,FIONREAD,&length)==-1)
				{
					ROS_WARN("%s,%d,ioctl return -1",__FUNCTION__,__LINE__);
					free(t_buf);
					FD_CLR(crport_fd_,&read_fd_set);
					return -1;
				}
				if(length>= (size_t)len){
					r_ret = ::read(crport_fd_,t_buf,len);
					if(r_ret == len)
						for(int i =0;i<len;i++){
							buf[i] = t_buf[i];
						}
						//memcpy(buf,t_buf,len);
					free(t_buf);
					FD_CLR(crport_fd_,&read_fd_set);
					return r_ret;
				}
				else{
					int time_remain = timeout.tv_sec*1000000 + timeout.tv_usec;
					int time_expect = (len - length)*1000000*8/bardrate_;
					if(time_remain > time_expect)
						usleep(time_expect);
				}
			}
		}
	}
	free(t_buf);
	return s_ret;
}

void Serial::setSendData(uint8_t seq, uint8_t val)
{
	boost::mutex::scoped_lock(g_send_stream_mutex);
	if (seq >= CTL_WHEEL_LEFT_HIGH && seq <= CTL_GYRO) {
		send_stream[seq] = val;
	}
}

uint8_t Serial::getSendData(uint8_t seq)
{
	uint8_t tmp_data;
	g_send_stream_mutex.lock();
	tmp_data = send_stream[seq];
	g_send_stream_mutex.unlock();
	return tmp_data;
}

void Serial::setReceiveData(uint8_t (&buf)[RECEI_LEN])
{
//	for (auto j = 0; j < RECEI_LEN; j++) {
//		receive_stream[j + 2] = buf[j];
//	}
}

/*
int Serial::get_sign(uint8_t *key, uint8_t *sign, uint8_t key_length, int sequence_number)
{
	int num_send_packets = key_length / KEY_DOWNLINK_LENGTH;
	uint8_t ptr[RECEI_LEN], buf[SEND_LEN];

	//Set random seed.
	//srand(time(NULL));
	//Send random key to robot.
	for (int i = 0; i < num_send_packets; i++) {
		//Populate dummy.
		for (int j = 0; j < DUMMY_DOWNLINK_LENGTH; j++)
			setSendData(j + DUMMY_DOWNLINK_OFFSET, (uint8_t) (rand() % 256));

		//Populate Sequence number.
		for (int j = 0; j < SEQUENCE_DOWNLINK_LENGTH; j++)
			setSendData(j + DUMMY_DOWNLINK_OFFSET, (uint8_t) ((sequence_number >> 8 * j) % 256));

		//Populate key.
#if VERIFY_DEBUG
		printf("appending key: ");
#endif

		for (int k = 0; k < KEY_DOWNLINK_LENGTH; k++) {
			setSendData(k + KEY_DOWNLINK_OFFSET, key[i * KEY_DOWNLINK_LENGTH + k]);

#if VERIFY_DEBUG
			printf("%02x ", g_send_stream[k + KEY_DOWNLINK_OFFSET]);
			if (k == KEY_DOWNLINK_LENGTH - 1)
				printf("\n");
#endif

		}

		//Fill command field
		switch (i) {
			case 0:
				setSendData(SEND_LEN - 4, CMD_KEY1);
				break;
			case 1:
				setSendData(SEND_LEN - 4, CMD_KEY2);
				break;
			case 2:
				setSendData(SEND_LEN - 4, CMD_KEY3);
				break;
			default:

#if VERIFY_DEBUG
				printf("control_get_sign : Error! key_length too large.");
#endif

				return -1;
				//break;
		}

		for (int i = 0; i < 40; i++) {  //200ms (round trip takes at leat 15ms)
			int counter = 0, ret;

			g_send_stream_mutex.lock();
			memcpy(buf, send_stream, sizeof(uint8_t) * SEND_LEN);
			g_send_stream_mutex.unlock();
			buf[CTL_CRC] = calc_buf_crc8(buf, SEND_LEN - 3);
			write(SEND_LEN, buf);

#if VERIFY_DEBUG
			printf("sending data to robot: i: %d\n", i);
			for (int j = 0; j < SEND_LEN; j++) {
				printf("%02x ", buf[j]);
			}
			printf("\n");
#endif

			while (counter < 200) {

				ret = read(1, ptr);
				if (ptr[0] != 0xAA)
					continue;

				ret = read(1, ptr);
				if (ptr[0] != 0x55)
					continue;

				ret = read(RECEI_LEN - 2, ptr);
				if (RECEI_LEN - 2 != ret) {

#if VERIFY_DEBUG
					printf("%s %d: receive count error: %d\n", __FUNCTION__, __LINE__, ret);
#endif

					usleep(100);
					counter++;
				}
				else {
					break;
				}
			}
			if (counter < 200) {

#if VERIFY_DEBUG
				printf("%s %d: counter: %d\tdata count: %d\treceive cmd: 0x%02x\n", __FUNCTION__, __LINE__, counter, ret, ptr[CMD_UPLINK_OFFSET]);

				printf("receive from robot: %d\n");
				for (int j = 0; j < RECEI_LEN - 2; j++) {
					printf("%02x ", ptr[j]);
				}
				printf("\n");
#endif

				if (ptr[CMD_UPLINK_OFFSET - 2] == CMD_NCK)   //robot received bronen packet
					continue;

				if (ptr[CMD_UPLINK_OFFSET - 2] == CMD_ACK) {  //set finished
					//printf("Downlink command ACKed!!\n");
					setSendData(CTL_KEY_VALIDATION, 0x00);
					break;
				}
			}
			else {

#if VERIFY_DEBUG
				printf("%s %d: max read count reached: %d\n", counter);
#endif

			}
			usleep(500);
		}
	}

	//Block and wait for signature.
	for (int i = 0; i < 400; i++) {                              //200ms (round trip takes at leat 15ms)
		int counter = 0, ret;
		while (counter < 400) {
			ret = read(1, ptr);
			if (ptr[0] != 0xAA)
				continue;

			ret = read(1, ptr);
			if (ptr[0] != 0x55)
				continue;

			ret = read(RECEI_LEN - 2, ptr);

#if VERIFY_DEBUG
			printf("%s %d: %d %d %d\n", __FUNCTION__, __LINE__, ret, RECEI_LEN - 2, counter);
#endif

			if (RECEI_LEN - 2 != ret) {
				usleep(100);
				counter++;
			}
			else {
				break;
			}
		}

#if VERIFY_DEBUG
		for (int j = 0; j < RECEI_LEN - 2; j++) {
			printf("%02x ", ptr[j]);
		}
		printf("\n");
#endif

		if (counter < 400 && ptr[CMD_UPLINK_OFFSET - 3] == CMD_ID) {
			//set finished

#if VERIFY_DEBUG
			printf("Signature received!!\n");
#endif

			for (int j = 0; j < KEY_UPLINK_LENGTH; j++)
				sign[j] = ptr[KEY_UPLINK_OFFSET - 2 + j];

			//Send acknowledge back to MCU.
			setSendData(CTL_KEY_VALIDATION, CMD_ACK);
			for (int k = 0; k < 20; k++) {
				g_send_stream_mutex.lock();
				memcpy(buf, send_stream, sizeof(uint8_t) * SEND_LEN);
				g_send_stream_mutex.unlock();
				buf[CTL_CRC] = calc_buf_crc8(buf, SEND_LEN - 3);
				write(SEND_LEN, buf);

				usleep(500);
			}
			setSendData(CTL_KEY_VALIDATION, 0x00);

#if VERIFY_DEBUG
			printf("%s %d: exit\n", __FUNCTION__, __LINE__);
#endif

			return KEY_UPLINK_LENGTH;
		}
		usleep(500);
	}
	setSendData(CTL_KEY_VALIDATION, 0x00);

#if VERIFY_DEBUG
	printf("%s %d: exit\n", __FUNCTION__, __LINE__);
#endif

	return -1;
}
*/

void Serial::setCleanMode(uint8_t val)
{
	setSendData(CTL_CLEAN_MODE, val & 0xff);
}

uint8_t Serial::getCleanMode()
{
	return getSendData(CTL_CLEAN_MODE);
}

void Serial::init_crc8(void)
{
	int		i, j;
	uint8_t crc;

	if (!made_table_) {
		for (i = 0; i < 256; i++) {
			crc = i;
			for (j = 0; j < 8; j++)
				crc = (crc << 1) ^ ((crc & 0x80) ? DI : 0);
			crc8_table_[i] = crc & 0xFF;
		}
		made_table_ = 1;
	}
}

/*
 * For a byte array whose accumulated crc value is stored in *crc, computes
 * resultant crc obtained by appending m to the byte array
 */
void Serial::crc8(uint8_t *crc, const uint8_t m)
{
	if (!made_table_) {
		init_crc8();
	}

	*crc = crc8_table_[(*crc) ^ m];
	*crc &= 0xFF;
}

uint8_t Serial::calc_buf_crc8(const uint8_t *inBuf, uint32_t inBufSz)
{
	uint8_t		crc_base = 0;
	uint32_t	i;

	for (i = 0; i < inBufSz; i++) {
		crc8(&crc_base, inBuf[i]);
	}
	return crc_base;
}

