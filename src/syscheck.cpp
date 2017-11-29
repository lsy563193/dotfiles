#include <fcntl.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <linux/elf.h>

#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>

#include <random>

#include "config.h"
#include "des.h"
#include "serial.h"

#define	TARGET_FILE "/opt/ros/indigo/lib/pp/pp"

using namespace std;

const unsigned char primary_key[24] = {0x36, 0xa1, 0xfc, 0xa6, 0xed, 0x29, 0x77, 0x42,
										0xa5, 0xc3, 0x44, 0x4f, 0x50, 0xb, 0x57, 0x6e,
										0x19, 0xe6, 0x4e, 0xc0, 0xf4, 0x2e, 0xba, 0xff};

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

#define K_SIGN_OFFSET				0x00	// 0x0C
#define SIGNATURE_SIZE				16
#define ENCRYPTION_KEY_SIZE			24

#define DUMMY_DOWNLINK_OFFSET		2
#define KEY_DOWNLINK_OFFSET			9
#define SEQUENCE_DOWNLINK_OFFSET	7

#define DUMMY_DOWNLINK_LENGTH		5
#define SEQUENCE_DOWNLINK_LENGTH	2
#define KEY_DOWNLINK_LENGTH			8

#define KEY_UPLINK_OFFSET			36
#define KEY_UPLINK_LENGTH			16

#define CMD_UPLINK_OFFSET			53

#define	CMD_KEY1					0x40
#define	CMD_KEY2					0x41
#define	CMD_KEY3					0x42
#define	CMD_ID						0x43

#define CMD_ACK						0x23
#define CMD_NCK						0x25

#define RECEI_LEN					57
#define SEND_LEN					21

enum {
	K_SIGN_CORRECT	= 1,
	K_SIGN_WR		= 0,
	K_ERR_INPUT		= -1,
	K_ERR_FILE		= -2,
	K_ERR_HEADER	= -3,
	K_ERR_UPDATE	= -4,
	K_ERR_SIGN		= -5
};

enum {
	CTL_WHEEL_LEFT_HIGH		= 2,
	CTL_WHEEL_LEFT_LOW		= 3,
	CTL_WHEEL_RIGHT_HIGH	= 4,
	CTL_WHEEL_RIGHT_LOW		= 5,
	CTL_VACCUM_PWR			= 6,
	CTL_BRUSH_LEFT			= 7,
	CTL_BRUSH_RIGHT			= 8,
	CTL_BRUSH_MAIN			= 9,
	CTL_BUZZER				= 10,
	CTL_MAIN_PWR			= 11,
	CTL_CHARGER				= 12,
	CTL_LED_RED				= 13,
	CTL_LED_GREEN			= 14,
	CTL_OMNI_RESET			= 15,
	CTL_GYRO				= 16,
	CTL_CMD					= 17,
	CTL_CRC					= 18,
};

const unsigned char origin_key[16] = {0xF8, 0x8E, 0xC9, 0xEA, 0xC8, 0x3D, 0xA5, 0x67, 0x6B, 0xC1, 0xAE, 0x9A, 0x12, 0xF0, 0xB4, 0x7F};

uint8_t receiStream[RECEI_LEN] = {0xaa, 0x55, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
									0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
									0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
									0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
									0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
									0x00, 0x00, 0x00, 0x00, 0x00, 0xcc, 0x33};

uint8_t sendStream[SEND_LEN] = {0xaa, 0x55, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0xcc, 0x33};

int control_get_sign(uint8_t* key, uint8_t* sign, uint8_t key_length, int sequence_number)
{
	int		num_send_packets = key_length / KEY_DOWNLINK_LENGTH;
	uint8_t	ptr[RECEI_LEN], buf[SEND_LEN];

	//Set random seed.
	//srand(time(NULL));
	//Send random key to robot.
	for (int i = 0; i < num_send_packets; i++) {
		//Populate dummy.
		for (int j = 0; j < DUMMY_DOWNLINK_LENGTH; j++)
			sendStream[j + DUMMY_DOWNLINK_OFFSET] = (uint8_t) (rand() % 256);

		//Populate Sequence number.
		for (int j = 0; j < SEQUENCE_DOWNLINK_LENGTH; j++)
			sendStream[j + SEQUENCE_DOWNLINK_OFFSET] = (uint8_t) ((sequence_number >> 8 * j) % 256);

		//Populate key.
#if VERIFY_DEBUG
		printf("appending key: ");
#endif

		for (int k = 0; k < KEY_DOWNLINK_LENGTH; k++) {
			sendStream[k + KEY_DOWNLINK_OFFSET] = key[i * KEY_DOWNLINK_LENGTH + k];

#if VERIFY_DEBUG
			printf("%02x ", sendStream[k + KEY_DOWNLINK_OFFSET]);
			if (k == KEY_DOWNLINK_LENGTH - 1)
				printf("\n");
#endif

		}

		//Fill command field
		switch (i) {
			case 0:
				sendStream[SEND_LEN - 4] = CMD_KEY1;
				break;
			case 1:
				sendStream[SEND_LEN - 4] = CMD_KEY2;
				break;
			case 2:
				sendStream[SEND_LEN - 4] = CMD_KEY3;
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


			memcpy(buf, sendStream, sizeof(uint8_t) * SEND_LEN);
			buf[CTL_CRC] = calcBufCrc8((char *)buf, SEND_LEN - 3);
			serial_write(SEND_LEN, buf);

#if VERIFY_DEBUG
			printf("sending data to robot: i: %d\n", i);
			for (int j = 0; j < SEND_LEN; j++) {
				printf("%02x ", buf[j]);
			}
			printf("\n");
#endif

			while (counter < 200) {

				ret = serial_read(1, ptr);
				if (ptr[0] != 0xAA)
					continue;

				ret = serial_read(1, ptr);
				if (ptr[0] != 0x55)
					continue;

				ret = serial_read(RECEI_LEN - 2, ptr);
				if (RECEI_LEN - 2!= ret) {

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
				for (int j = 0; j < RECEI_LEN - 2; j++) {
					printf("%02x ", ptr[j]);
				}
				printf("\n");
#endif

				if(ptr[CMD_UPLINK_OFFSET - 2] ==  CMD_NCK)   //robot received bronen packet
					continue;

				if(ptr[CMD_UPLINK_OFFSET - 2] == CMD_ACK) {	//set finished
					//printf("Downlink command ACKed!!\n");
					sendStream[CTL_CMD] = 0;                              //clear command byte
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
			ret = serial_read(1, ptr);
			if (ptr[0] != 0xAA)
				continue;

			ret = serial_read(1, ptr);
			if (ptr[0] != 0x55)
					continue;

			ret = serial_read(RECEI_LEN - 2, ptr);

#if VERIFY_DEBUG
			printf("%s %d: %d %d %d\n", __FUNCTION__, __LINE__, ret, RECEI_LEN - 2, counter);
#endif

			if (RECEI_LEN - 2!= ret) { 
				usleep(100);
				counter++;
			} else {
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
			sendStream[CTL_CMD] = CMD_ACK;
			for (int k = 0; k < 20; k++) {
				memcpy(buf, sendStream, sizeof(uint8_t) * SEND_LEN);
				buf[CTL_CRC] = calcBufCrc8((char *)buf, SEND_LEN - 3);
				serial_write(SEND_LEN, buf);

				usleep(500);
			}
			sendStream[CTL_CMD] = 0;

#if VERIFY_DEBUG
			printf("%s %d: exit\n", __FUNCTION__, __LINE__);
#endif

			return KEY_UPLINK_LENGTH;
		}
		usleep(500);
	}
	sendStream[CTL_CMD] = 0;

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

	serial_init("/dev/ttyS2", 115200);

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

