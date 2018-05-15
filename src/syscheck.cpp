#include <elf.h>
#include <fcntl.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <sys/types.h>

#include <random>

#include "config.h"
#include "syscheck.hpp"
#include "des.h"

#define	TARGET_FILE "/opt/ros/indigo/lib/pp/pp"
#define VER

using namespace std;

const unsigned char primary_key[24] = {0x36, 0xa1, 0xfc, 0xa6, 0xed, 0x29, 0x77, 0x42,
										0xa5, 0xc3, 0x44, 0x4f, 0x50, 0xb, 0x57, 0x6e,
										0x19, 0xe6, 0x4e, 0xc0, 0xf4, 0x2e, 0xba, 0xff};

using namespace SERIAL;

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
			if (close() == 0)
			{
				printf("%s %d: Close serial port %s.", __FUNCTION__, __LINE__, port_.c_str());
				closed = true;
				break;
			}
		}

		if (!closed)
			printf("%s %d: Close serial port %s failed.", __FUNCTION__, __LINE__, port_.c_str());
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
		printf("%s %d: Open device %s, on baudrate %d failed!.", __FUNCTION__, __LINE__, buf, baudrate);
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

	cfmakeraw(&curopt_);		//make raw is_max_clean_state_

	if (tcsetattr(crport_fd_, TCSANOW, &curopt_) != 0){
		return false;
	}
	serial_port_ready_ = true;
	printf("\033[32mserial port %s\033[0m init done...", buf);
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
			printf("%s,%d,ioctl return -1",__FUNCTION__,__LINE__);
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
			printf("%s %d: -------select error------------", __FUNCTION__, __LINE__);
			free(t_buf);
			FD_CLR(crport_fd_,&read_fd_set);
			return -1;
		}
		else if(s_ret ==0){
			printf("%s %d: select function \033[33mtimeout!\033[0m", __FUNCTION__, __LINE__);
			free(t_buf);
			FD_CLR(crport_fd_,&read_fd_set);
			return 0;
		}
		else if(s_ret >0){
			if(FD_ISSET(crport_fd_,&read_fd_set)){
				if(ioctl(crport_fd_,FIONREAD,&length)==-1)
				{
					printf("%s,%d,ioctl return -1",__FUNCTION__,__LINE__);
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

void Serial::setSendData(uint8_t seq, uint8_t val)
{
	if (seq >= CTL_WHEEL_LEFT_HIGH && seq <= CTL_CRC) {
		send_stream[seq] = val;
	}
}

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

void Serial::sendData()
{
	uint8_t buf[SEND_LEN];
	memcpy(buf, serial.send_stream, sizeof(uint8_t) * SEND_LEN);
	buf[CTL_CRC] = serial.calBufCrc8(buf, CTL_CRC);
//	printf("buf[CTL_CRC] = %x\n", buf[CTL_CRC]);
	serial.write(buf, SEND_LEN);
	serial.setSendData(CTL_CRC, buf[CTL_CRC]);
}

#if VERIFY_CPU_ID
static unsigned char cpuid[33];
static unsigned char cpuid_cal[32];

static char proc_file[14];

int process_proc(void)
{
	int		retval = 0;
	FILE	*fp = NULL;
	char	*line = NULL;
	size_t	len = 0;
	ssize_t	read;

	proc_file[0] = proc_file[5] = '/';
	proc_file[1] = proc_file[7] = 112;
	proc_file[2] = 0x72;
	proc_file[3] = proc_file[12] = 'o';
	proc_file[4] = proc_file[6] = 99;
	proc_file[8] = 0x75;
	proc_file[9] = 'i';
	proc_file[10] = 110;
	proc_file[11] = 0x66;

	fp = fopen(proc_file, "r");

	if (fp == NULL) {
		return retval;
	}

	while ((read = getline(&line, &len, fp)) != -1) {
		if (line[0] == 67 && line[1] == 104 && line[2] == 105 && line[3] == 112 && line[4] == 105 && line[5] == 100) {
			memcpy(cpuid, line + 10, 32);

#if VERIFY_DEBUG
			fprintf(stderr, "%s\n", cpuid);
#endif

			retval = 1;
			break;
		}
	}

	free(line);
	fclose(fp);

#if VERIFY_DEBUG
	for (int i = 0; i < 32; i++) {
		printf("%d: %c\n", i, cpuid[i]);
	}
#endif

	mbedtls_des3_context ctx1;
	mbedtls_des3_init(&ctx1);
	mbedtls_des3_set3key_enc(&ctx1, primary_key);
	for (int i = 0; i < 32; i += 8)
		mbedtls_des3_crypt_ecb(&ctx1, cpuid + i, cpuid_cal + i);

#if VERIFY_DEBUG
	for (int i = 0; i < 32; i++) {
		printf("%d: 0x%02x\n", i, cpuid_cal[i]);
	}
#endif

	return retval;
}

int process_bin(void)
{
	int		i, fd, retval;
	char	*addr, *ptr, needle[32];
	off_t	offset, pa_offset;
	size_t	length;

	struct stat	st;

	retval = 0;
	fd = open(TARGET_FILE, O_RDWR);

	needle[0] = needle[2] = needle[8] = needle[10] = needle[16] = needle[18] = needle[24] = needle[26] ='i';
	needle[1] = needle[9] = needle[17] = needle[25] = 'l';
	needle[3] = needle[11] = needle[19] = needle[27] = 'f';
	needle[4] = needle[12] = needle[20] = needle[28] = 'e';
	needle[5] = needle[13] = needle[21] = needle[29] = '9';
	needle[6] = needle[7] = needle[14] = needle[15] = needle[22] = needle[23] = needle[30] = needle[31] = '0';

	if (fd < 0) {

#if VERIFY_DEBUG
		fprintf(stderr, "Failed to open file pp, %d\n", errno);
#endif

		goto out;
	}

	if (fstat(fd, &st) == -1) {

#if VERIFY_DEBUG
		fprintf(stderr, "Failed to get file stat: pp\n");
#endif

		goto out;
	}

	offset = 0;
	pa_offset = offset & ~(sysconf(_SC_PAGE_SIZE) - 1);
	/* offset for mmap() must be page aligned */

	length = st.st_size;

#if VERIFY_DEBUG
	printf("%s %d: length: %d\tpa_offset: %d\n", __FUNCTION__, __LINE__, length, pa_offset);
#endif

	addr = (char *) mmap(NULL, length + offset - pa_offset, PROT_READ | PROT_WRITE, MAP_SHARED, fd, pa_offset);
	if (addr == MAP_FAILED) {

#if VERIFY_DEBUG
		fprintf(stderr, "Mmap failed.\n");
#endif

		goto out;
	}

	ptr = addr;
	for (i = 0; i < length; i++) {

		if (strncmp(ptr, needle, 8) == 0) {

#if VERIFY_DEBUG
			fprintf(stderr, "Special string found\n");
#endif

			break;
		}
		ptr++;
	}

	if (i == length) {

#if VERIFY_DEBUG
		fprintf(stderr, "Special string not found\n");
#endif

		goto out;
	}

#if VERIFY_DEBUG
	fprintf(stderr, "Special string found, %c%c%c%c%c\n", ptr[0], ptr[1], ptr[2], ptr[3], ptr[4]);
#endif

	memcpy(ptr, cpuid_cal, 32);

	retval = 1;

out:
	munmap(addr, fd);
	close(fd);
	return retval;
}
#endif

#if VERIFY_KEY

#define K_SIGN_OFFSET			0x00 // 0x0C
#define SIGNATURE_SIZE			16
#define ENCRYPTION_KEY_SIZE		24

#define DUMMY_DOWNLINK_OFFSET		2
#define KEY_DOWNLINK_OFFSET			9
#define SEQUENCE_DOWNLINK_OFFSET	7

#define DUMMY_DOWNLINK_LENGTH		5
#define SEQUENCE_DOWNLINK_LENGTH	2
#define KEY_DOWNLINK_LENGTH			8

#define KEY_UPLINK_OFFSET			16
#define KEY_UPLINK_LENGTH			16

#define CMD_UPLINK_OFFSET			REC_KEY_VALIDATION

#define CMD_KEY1					0x40
#define CMD_KEY2					0x41
#define CMD_KEY3					0x42
#define CMD_ID						0x43

#define CMD_ACK						0x23
#define CMD_NCK						0x25

enum {
	K_SIGN_CORRECT	= 1,
	K_SIGN_WR		= 0,
	K_ERR_INPUT		= -1,
	K_ERR_FILE		= -2,
	K_ERR_HEADER	= -3,
	K_ERR_UPDATE	= -4,
	K_ERR_SIGN		= -5
};

const unsigned char origin_key[16] = {0xF8, 0x8E, 0xC9, 0xEA, 0xC8, 0x3D, 0xA5, 0x67, 0x6B, 0xC1, 0xAE, 0x9A, 0x12, 0xF0, 0xB4, 0x7F};

int control_get_sign(uint8_t* key, uint8_t* sign, uint8_t key_length, int sequence_number)
{
	int		num_send_packets = key_length / KEY_DOWNLINK_LENGTH;
	uint8_t	ptr[REC_LEN], buf[SEND_LEN];

	//Set random seed.
	//srand(time(NULL));
	//Send random key to robot.
	for (int i = 0; i < num_send_packets; i++) {
		//Populate dummy.
		for (int j = 0; j < DUMMY_DOWNLINK_LENGTH; j++)
			serial.setSendData(j + DUMMY_DOWNLINK_OFFSET, (uint8_t) (rand() % 256));

		//Populate Sequence number.
		for (int j = 0; j < SEQUENCE_DOWNLINK_LENGTH; j++)
			serial.setSendData(j + SEQUENCE_DOWNLINK_OFFSET, (uint8_t) ((sequence_number >> 8 * j) % 256));

		//Populate key.
#if VERIFY_DEBUG
		printf("appending key: ");
#endif

		for (int k = 0; k < KEY_DOWNLINK_LENGTH; k++) {
			serial.setSendData(k + KEY_DOWNLINK_OFFSET, key[i * KEY_DOWNLINK_LENGTH + k]);

#if VERIFY_DEBUG
			if (k == KEY_DOWNLINK_LENGTH - 1)
				printf("\n");
#endif

		}

		//Fill command field
		switch (i) {
			case 0:
				serial.setSendData(SEND_LEN - 4, CMD_KEY1);
				break;
			case 1:
				serial.setSendData(SEND_LEN - 4, CMD_KEY2);
				break;
			case 2:
				serial.setSendData(SEND_LEN - 4, CMD_KEY3);
				break;
			default:

#if VERIFY_DEBUG
				printf("control_get_sign : Error! key_length too large.");
#endif

				return -1;
				//break;
		}

		for (int i = 0; i < 40; i++) {	//200ms (round trip takes at leat 15ms)
			int counter = 0, ret;

			serial.sendData();

#if VERIFY_DEBUG
			printf("sending data to robot: i: %d\n", i);
			for (int j = 0; j < SEND_LEN; j++) {
				printf("%02x ", buf[j]);
			}
			printf("\n");
#endif

			while (counter < 200) {

				ret = serial.read(ptr, 1);
				if (ptr[0] != 0xAA)
					continue;

				ret = serial.read(ptr, 1);
				if (ptr[0] != 0x55)
					continue;

				ret = serial.read(ptr, REC_LEN - 2);
				if (REC_LEN - 2!= ret) {

#if VERIFY_DEBUG
					printf("%s %d: receive count error: %d\n", __FUNCTION__, __LINE__, ret);
#endif

					usleep(100);
					counter++;
				} else {
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

				if(ptr[CMD_UPLINK_OFFSET - 2] ==  CMD_NCK)   //robot received bronen packet
					continue;

				if(ptr[CMD_UPLINK_OFFSET - 2] == CMD_ACK) {	//set finished
					//printf("Downlink command ACKed!!\n");
					serial.setSendData(CTL_KEY_VALIDATION, 0);                              //clear command byte
					break;
				}
			} else {

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
			ret = serial.read(ptr, 1);
			if (ptr[0] != 0xAA)
				continue;

			ret = serial.read(ptr, 1);
			if (ptr[0] != 0x55)
				continue;

			ret = serial.read(ptr, REC_LEN - 2);

#if VERIFY_DEBUG
			printf("%s %d: %d %d %d\n", __FUNCTION__, __LINE__, ret, REC_LEN - 2, counter);
#endif

			if (REC_LEN - 2!= ret) {
				usleep(100);
				counter++;
			} else {
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
			serial.setSendData(CTL_KEY_VALIDATION, CMD_ACK);
			for (int k = 0; k < 20; k++) {
				serial.sendData();

				usleep(500);
			}
			serial.setSendData(CTL_KEY_VALIDATION, 0);

#if VERIFY_DEBUG
			printf("%s %d: exit\n", __FUNCTION__, __LINE__);
#endif

			return KEY_UPLINK_LENGTH;
		}
		usleep(500);
	}
	serial.setSendData(CTL_KEY_VALIDATION, 0);

#if VERIFY_DEBUG
	printf("%s %d: exit\n", __FUNCTION__, __LINE__);
#endif

	return -1;
}

unsigned char set_sign(unsigned char *a_esno, unsigned char *a_rand)
{
	int			fd;
	Elf32_Ehdr	elfhdr;
	Elf32_Shdr	sechdr;

	unsigned char	*ptr, header[6], signature[SIGNATURE_SIZE], signature_cal[SIGNATURE_SIZE], signature_mcu[SIGNATURE_SIZE];

	if ((a_esno == NULL) || (a_rand == NULL))
		return K_ERR_INPUT;

	//Decrypt the encrypted serial number
	//...
	//unsigned char rand_key[16];

	//Get signature from ilife section
	fd = open(TARGET_FILE, O_RDWR);
	if (fd <= 0)
		return K_ERR_FILE;

	read(fd, (unsigned char *) &elfhdr, sizeof(elfhdr));
	ptr = (unsigned char *) &elfhdr;
	if ((ptr[0] == 0x7F) && (ptr[1] == 'E') && (ptr[2] == 'L') && (ptr[3] == 'F')) {
		lseek(fd, elfhdr.e_shoff + elfhdr.e_shentsize, SEEK_SET);			// Seek to ilife section header
		read(fd, (unsigned char *) &sechdr, sizeof(sechdr));
	} else {
		close(fd);
		return K_ERR_FILE;
	}

	lseek(fd, sechdr.sh_offset, SEEK_SET);							// Seek to ilife section
	//memset(header, 0, sizeof(header));
	//read(fd, (unsigned char *) header, sizeof(header));
	//if (strncmp((char *) header, "ILIFE", sizeof(header)) != 0) {
	//	close(fd);
	//	return K_ERR_HEADER;
	//}

	lseek(fd, (sechdr.sh_offset + K_SIGN_OFFSET), SEEK_SET);			// Seek to signature offset
	read(fd, signature, sizeof(signature));

	//Get signature from serial no.
	//... encrypt serial no. using key
	//memcpy(signature_cal, "12345678", 8);
	mbedtls_des3_context ctx1, ctx2;

	//Decrypt MCU unique id signature.
	mbedtls_des3_init(&ctx1);
	mbedtls_des3_set3key_dec(&ctx1, a_rand);
	for (int i = 0; i < SIGNATURE_SIZE; i += 8)
		mbedtls_des3_crypt_ecb(&ctx1, a_esno + i, signature_mcu + i);

#if VERIFY_DEBUG
	//TODO: Remove this unique id signature printout
	printf("Unique ID..........:\n");
	printf("Coordinates:\t%u\n", (void *) signature_mcu);
	printf("Wafer num:\t%uhh\n", (char *) signature_mcu + 4);
	printf("Uart transfered encrypted signature: ");
	for (int i = 0; i < SIGNATURE_SIZE; i++)
		printf("%x ", ((unsigned char *) a_esno)[i] );
	printf("\n");

	printf("Decoded signature:\t");
	for (int i = 0; i < SIGNATURE_SIZE; i++)
		printf("%02x ", ((unsigned char *) signature_mcu)[i] );
	printf("\n");
#endif

	//Got unique id signature, now encrypt with primary key
	mbedtls_des3_init(&ctx2);
	mbedtls_des3_set3key_enc(&ctx2, primary_key);
	for (int i = 0; i < SIGNATURE_SIZE; i += 8)
		mbedtls_des3_crypt_ecb(&ctx2, signature_mcu + i, signature_cal + i);

#if VERIFY_DEBUG
	printf("Final encrypted signature is: ");
	for (int i=0; i < SIGNATURE_SIZE; i++)
		printf("%02x ", ((unsigned char*)signature_cal)[i] );
	printf("\n");
#endif

	mbedtls_des3_free(&ctx1);
	mbedtls_des3_free(&ctx2);

	//Check virgin
	if (memcmp(signature, origin_key, sizeof(signature)) == 0) {

#if VERIFY_DEBUG
		printf("Virgin pattern found!\n");
#endif

		lseek(fd, (sechdr.sh_offset + K_SIGN_OFFSET), SEEK_SET);		// Seek to signature offset
		if (write(fd, signature_cal, sizeof(signature_cal)) != sizeof(signature_cal)) {

#if VERIFY_DEBUG
			printf("Update signature error\n");
#endif

			close(fd);
			return K_ERR_UPDATE;
		}

		close(fd);

#if VERIFY_DEBUG
		printf("File signature updated to: ");
		for (int i=0; i < SIGNATURE_SIZE; i++)
			printf("%02x ", ((unsigned char *) signature_cal)[i] );
		printf("\n");
#endif

		return K_SIGN_WR;
	} else {
		//No virgin pattern detected, check signature in file.
		close(fd);

#if VERIFY_DEBUG
		printf("File signature already exist: ");
		for (int i=0; i < SIGNATURE_SIZE; i++)
			printf("%x ", ((unsigned char*)signature)[i] );
		printf("\n");
#endif

		// Verify signature
		if (memcmp(signature, signature_cal, sizeof(signature)) == 0)
			return K_SIGN_CORRECT;

		return K_ERR_SIGN;
	}
}

int verify_key(void)
{
	/*************************0. Check signature of MCU **************/
	int 	retries = 0, sequence_number;
	uint8_t	key[ENCRYPTION_KEY_SIZE], encID[SIGNATURE_SIZE];

	const int		key_length = ENCRYPTION_KEY_SIZE;
	const int		max_retries = 3;
	unsigned char	result = 0;

	std::random_device	rd;
	std::mt19937		random_number_engine(rd()); // pseudorandom number generator

	std::uniform_int_distribution<uint8_t>	dice_distribution(0, 255);

	sequence_number = dice_distribution(random_number_engine) * 256 + dice_distribution(random_number_engine);

	while (!serial.isReady()) {
		// Init for serial.
		if (!serial.init("/dev/ttyS2", 115200))
			printf("%s %d: Serial init failed!!", __FUNCTION__, __LINE__);
	}

	while (1) {
		while (retries < max_retries) {

#if VERIFY_DEBUG
			printf("Sending a key:  ");
#endif
			for (int i = 0; i < ENCRYPTION_KEY_SIZE; i++) {
				key[i] = (uint8_t) dice_distribution(random_number_engine);

#if VERIFY_DEBUG
				printf("0x%x ", key[i]);
				if (i == ENCRYPTION_KEY_SIZE - 1)
					printf("\n");
#endif

			}
			mbedtls_des_key_set_parity(key);
			if (control_get_sign(key, encID, key_length, sequence_number) > 0) {

#if VERIFY_DEBUG
				printf("Got encrypted signature:  ");
					for (int i = 0; i < SIGNATURE_SIZE; i++) {
					printf("0x%x ", encID[i]);
				}
				printf("\n");
#endif

				result = set_sign(encID, key);

#if VERIFY_DEBUG
				printf("check_sign status: 0x%x(%d)\n", result, (int8_t) result);
#endif

			}
			if (result == K_SIGN_CORRECT || result == K_SIGN_WR)
				break;

			retries++;
			sequence_number++;
		}

		if (result == K_SIGN_CORRECT || result == K_SIGN_WR)
			break;

		//No K_SIGN_CORRECT detected. If max_retries is exceeded, it goes into an infinite loop.
		usleep(500);
	}
	system("sync");
}
#endif

int main(int argc , char **argv)
{
	bool verify_ok = true;

#if VERIFY_CPU_ID
	memset(proc_file, 0, sizeof(char) * 14);
	memset(cpuid, 0, sizeof(char) * 33);

	if (process_proc() == 0) {
		return 1;
	}

	if (process_bin() == 0) {
		return 1;
	}
#endif

#if VERIFY_KEY
	verify_key();
#endif

	return 0;
}
