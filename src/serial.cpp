#include "ros/ros.h"
#include "boost/thread.hpp"
#include "serial.h"
#include <sys/ioctl.h>
#include <fcntl.h>
#include <brush.h>
#include <wheel.hpp>
#include <beeper.h>
#include <key_led.h>
#include <robot.hpp>
#include <water_tank.hpp>
#include <vacuum.h>
#include <wifi_led.hpp>
#include "wifi/wifi.h"

boost::mutex send_stream_mutex;

Serial serial;

Serial::Serial()
{
	crport_fd_ = 0;
	serial_port_ready_ = false;
	made_table_ = 0;
}

Serial::~Serial()
{
	bool closed = false;
	if (isReady())
	{
		for (auto i = 0; i < 10; i++)
		{
			if (close() == 0);
			{
				ROS_INFO("%s %d: Close serial port %s.", __FUNCTION__, __LINE__, port_.c_str());
				closed = true;
				break;
			}
		}

		if (!closed)
			ROS_ERROR("%s %d: Close serial port %s failed.", __FUNCTION__, __LINE__, port_.c_str());
	}
}

bool Serial::init(const std::string port,int baudrate)
{
	char buf[1024];

	bardrate_ = baudrate;
	sprintf(buf, "%s", port.c_str());
	port_ = port;
	crport_fd_ = open(buf, O_RDWR | O_NOCTTY | O_NONBLOCK);
	if (crport_fd_ == -1)
	{
		ROS_ERROR("%s %d: Open device %s, on baudrate %d failed!.", __FUNCTION__, __LINE__, buf, baudrate);
		return false;
	}

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
	curopt_.c_cflag |= CLOCAL;	//disable modem status check

	cfmakeraw(&curopt_);		//make raw mode_

	if (tcsetattr(crport_fd_, TCSANOW, &curopt_) != 0){
		return false;
	}
	serial_port_ready_ = true;
	ROS_INFO("\033[32mserial port %s\033[0m init done...", buf);
	return true;
}

bool Serial::isReady()
{
	return serial_port_ready_;
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
	if (retval == 0)
	{
		crport_fd_ = -1;
		serial_port_ready_ = false;
	}
	return retval;
}

int Serial::write(uint8_t *buf, uint8_t len)
{
	int ret;
	ret = static_cast<int>(::write(crport_fd_, buf, len));
	return ret;
}

int Serial::read(uint8_t *buf, int len)
{
	int r_ret=0,s_ret=0;
	uint8_t *t_buf;
	t_buf = (uint8_t*)calloc(len,sizeof(uint8_t));
	//memset(t_buf,0,size_of_path);
	fd_set read_fd_set;
	struct timeval timeout;
	timeout.tv_sec = 1;
	timeout.tv_usec = 0;// ms
	size_t length = 0;
	if(isReady()){
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
				//memcpy(buf,t_buf,size_of_path);
			free(t_buf);
			return r_ret;
		}
	}
	while (isReady()){
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
						//memcpy(buf,t_buf,size_of_path);
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

void Serial::resetSendStream(void)
{
	for (int i = 0; i < SEND_LEN; i++) {
		if (i != CTL_LED_GREEN)
			setSendData(i, 0x00);
		else
			setSendData(i, 0x64);
	}
	setSendData(0, 0xaa);
	setSendData(1, 0x55);
	setSendData(CTL_TRAILER_1, 0xcc);
	setSendData(CTL_TRAILER_2, 0x33);

	setWorkMode(IDLE_MODE);
	uint8_t buf[SEND_LEN];
	{
		boost::mutex::scoped_lock lock(send_stream_mutex);
		memcpy(buf, send_stream, sizeof(uint8_t) * SEND_LEN);
	}
	uint8_t crc;
	crc = calBufCrc8(buf, CTL_CRC);
	setSendData(CTL_CRC, crc);
}

void Serial::setSendData(uint8_t seq, uint8_t val)
{
	boost::mutex::scoped_lock lock(send_stream_mutex);
	if (seq >= CTL_WHEEL_LEFT_HIGH && seq <= CTL_CRC) {
		send_stream[seq] = val;
	}
}

uint8_t Serial::getSendData(uint8_t seq)
{
	uint8_t tmp_data;
	send_stream_mutex.lock();
	tmp_data = send_stream[seq];
	send_stream_mutex.unlock();
	return tmp_data;
}

void Serial::setWorkMode(uint8_t val)
{
	setSendData(CTL_WORK_MODE, val);
}

/*
int Serial::get_sign(uint8_t *key, uint8_t *sign, uint8_t key_length, int sequence_number)
{
	int num_send_packets = key_length / KEY_DOWNLINK_LENGTH;
	uint8_t ptr[REC_LEN], buf[SEND_LEN];

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

			send_stream_mutex.lock();
			memcpy(buf, send_stream, sizeof(uint8_t) * SEND_LEN);
			send_stream_mutex.unlock();
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

				ret = read(REC_LEN - 2, ptr);
				if (REC_LEN - 2 != ret) {

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
				for (int j = 0; j < REC_LEN - 2; j++) {
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

			ret = read(REC_LEN - 2, ptr);

#if VERIFY_DEBUG
			printf("%s %d: %d %d %d\n", __FUNCTION__, __LINE__, ret, REC_LEN - 2, counter);
#endif

			if (REC_LEN - 2 != ret) {
				usleep(100);
				counter++;
			}
			else {
				break;
			}
		}

#if VERIFY_DEBUG
		for (int j = 0; j < REC_LEN - 2; j++) {
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
				send_stream_mutex.lock();
				memcpy(buf, send_stream, sizeof(uint8_t) * SEND_LEN);
				send_stream_mutex.unlock();
				buf[CTL_CRC] = calBufCrc8(buf, SEND_LEN - 3);
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

void Serial::initCrc8(void)
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
		initCrc8();
	}

	*crc = crc8_table_[(*crc) ^ m];
	*crc &= 0xFF;
}

uint8_t Serial::calBufCrc8(const uint8_t *inBuf, uint32_t inBufSz)
{
	uint8_t		crc_base = 0;
	uint32_t	i;

	for (i = 0; i < inBufSz; i++) {
		crc8(&crc_base, inBuf[i]);
	}
	return crc_base;
}

void Serial::receive_routine_cb()
{
	ROS_INFO("robotbase,\033[32m%s\033[0m,%d is up.",__FUNCTION__,__LINE__);
	int i, j, ret, wh_len, wht_len, whtc_len;

	uint8_t r_crc, c_crc;
	uint8_t h1 = 0xaa, h2 = 0x55, header[2], t1 = 0xcc, t2 = 0x33;
	uint8_t header1 = 0x00;
	uint8_t header2 = 0x00;
	uint8_t tempData[REC_LEN], receiData[REC_LEN];
	tempData[0] = h1;
	tempData[1] = h2;

	wh_len = REC_LEN - 2; //length without header bytes
	wht_len = wh_len - 2; //length without header and tail bytes
	whtc_len = wht_len - 1; //length without header and tail and crc bytes

	while (ros::ok() && (!recei_thread_kill)) {
		if (!recei_thread_enable)
		{
			usleep(10000);
			continue;
		}

		ret = serial.read(&header1, 1);
		if (ret <= 0 ){
			ROS_WARN("%s, %d, serial read return %d ,  request %d byte",__FUNCTION__,__LINE__,ret,1);
			continue;
		}
		if(header1 != h1)
			continue;
		ret= serial.read(&header2, 1);
		if (ret <= 0 ){
			ROS_WARN("%s,%d,serial read return %d , request %d byte",__FUNCTION__,__LINE__,ret,1);
			continue;
		}
		if(header2 != h2){
			continue;
		}
		ret = serial.read(receiData, wh_len);
		if(ret > 0 && ret < wh_len){
			ROS_WARN("%s,%d,serial read %d bytes, request %d bytes",__FUNCTION__,__LINE__,ret,wh_len);
		}
		if(ret <= 0){
			ROS_WARN("%s,%d,serial read return %d",__FUNCTION__,__LINE__,ret);
			continue;
		}

		for (i = 0; i < whtc_len; i++){
			tempData[i + 2] = receiData[i];
		}

		c_crc = serial.calBufCrc8(tempData, wh_len - 1);//calculate crc8 with header bytes
		r_crc = receiData[whtc_len];

		if (r_crc == c_crc){
			if (receiData[wh_len - 1] == t2 && receiData[wh_len - 2] == t1) {
				ROS_ERROR_COND(pthread_mutex_lock(&recev_lock)!=0, "serial pthread receive lock fail");
				for (j = 0; j < wht_len; j++) {
					serial.receive_stream[j + 2] = receiData[j];
				}
				ROS_ERROR_COND(pthread_cond_signal(&recev_cond)<0, "in serial read, pthread signal fail !");
				ROS_ERROR_COND(pthread_mutex_unlock(&recev_lock)!=0, "serial pthread receive unlock fail");
			}
			else {
				debugReceivedStream(tempData);
				ROS_WARN(" in serial read ,data tail error\n");
			}
		}
		else {
			debugReceivedStream(tempData);
			ROS_ERROR("%s,%d,in serial read ,data crc error\n",__FUNCTION__,__LINE__);
		}
	}
	pthread_cond_signal(&recev_cond);
	robotbase_thread_kill = true;
	ROS_ERROR("%s,%d,exit!",__FUNCTION__,__LINE__);
}

void Serial::send_routine_cb()
{
	ROS_INFO("robotbase,\033[32m%s\033[0m,%d is up.",__FUNCTION__,__LINE__);
	ros::Rate r(_RATE);
	resetSendStream();
	//tmp test
	uint16_t wifi_send_count = 0;

	while(ros::ok() && !send_thread_kill){
		if (!send_thread_enable)
		{
			r.sleep();
			continue;
		}

		//tmp test
		wifi_send_count++;
		if(wifi_send_count >= 250)
		{
			wifi_send_count = 0;
			s_wifi.testSend();
		}

		r.sleep();
		/*-------------------Process for beeper.play and key_led -----------------------*/
		key_led.processLed();
		wifi_led.processLed();
		if (getSendData(CTL_WORK_MODE) != DESK_TEST_WRITE_BASELINE_MODE)
		{
			beeper.processBeep();

			/*---pid for wheels---*/
			wheel.pidAdjustSpeed();
			brush.updatePWM();
			water_tank.updatePWM();
		}

		sendData();
		robot::instance()->publishCtrlStream();
	}
	core_thread_kill = true;
	ROS_ERROR("%s,%d exit",__FUNCTION__,__LINE__);
	//pthread_exit(NULL);
}

void Serial::sendData()
{
	uint8_t buf[SEND_LEN];
	send_stream_mutex.lock();
	memcpy(buf, serial.send_stream, sizeof(uint8_t) * SEND_LEN);
	send_stream_mutex.unlock();
	buf[CTL_CRC] = serial.calBufCrc8(buf, CTL_CRC);
//	printf("buf[CTL_CRC] = %x\n", buf[CTL_CRC]);
	serial.write(buf, SEND_LEN);
//	debugSendStream(buf);
	serial.setSendData(CTL_CRC, buf[CTL_CRC]);
}

void Serial::debugReceivedStream(const uint8_t *buf)
{
	ROS_INFO("%s %d: Received stream:", __FUNCTION__, __LINE__);
	for (int i = 0; i < REC_LEN; i++)
		printf("%02d ", i);
	printf("\n");

	for (int i = 0; i < REC_LEN; i++)
		printf("%02x ", buf[i]);
	printf("\n");
}

void Serial::debugSendStream(const uint8_t *buf)
{
	ROS_INFO("%s %d: Send stream:", __FUNCTION__, __LINE__);
	for (int i = 0; i < SEND_LEN; i++)
		printf("%02d ", i);
	printf("\n");

	for (int i = 0; i < SEND_LEN; i++)
		printf("%02x ", buf[i]);
	printf("\n");
}

