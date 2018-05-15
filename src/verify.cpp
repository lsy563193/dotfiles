#include <elf.h>
#include <verify.h>
#include <fcntl.h>
#include <unistd.h>
#include <des.h>
#include <ros/assert.h>
#include <random>
#include "serial.h"

using namespace std;

#define TARGET_FILE "/opt/ros/indigo/lib/pp/pp"

const unsigned char primary_key[24] = {0x36, 0xa1, 0xfc, 0xa6, 0xed, 0x29, 0x77, 0x42,
										0xa5, 0xc3, 0x44, 0x4f, 0x50, 0xb, 0x57, 0x6e,
										0x19, 0xe6, 0x4e, 0xc0, 0xf4, 0x2e, 0xba, 0xff};

#if VERIFY_CPU_ID

extern char _binary_res_dat_start;

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
			ROS_INFO("cpu id: %s", cpuid);
#endif

			retval = 1;
			break;
		}
	}

	free(line);
	fclose(fp);

#if VERIFY_DEBUG
	for (int i = 0; i < 32; i++) {
		ROS_INFO("%d: %c", i, cpuid[i]);
	}
#endif

	mbedtls_des3_context ctx1;
	mbedtls_des3_init(&ctx1);
	mbedtls_des3_set3key_enc(&ctx1, primary_key);
	for (int i = 0; i < 32; i += 8)
		mbedtls_des3_crypt_ecb(&ctx1, cpuid + i, cpuid_cal + i);

#if VERIFY_DEBUG
	for (int i = 0; i < 32; i++) {
		ROS_INFO("%d: 0x%02x", i, cpuid_cal[i]);
	}
#endif

	return retval;
}

int verify_cpu_id(void) {
	int		retval;
	char	*ptr = &_binary_res_dat_start;

#if VERIFY_DEBUG
	ROS_INFO("binary_res_dat_start: 0x%lx %0x", (unsigned long)&_binary_res_dat_start, _binary_res_dat_start);
	for (int i = 0; i < 32; i++) {
		ROS_INFO("%d: 0x%02x 0x%02x", i, cpuid_cal[i], ptr[i]);
	}
#endif

	ROS_INFO("res data: %s", ptr);
	if (process_proc() == 0) {
		return -1;
	}

	if (strncmp((const char *) cpuid_cal, ptr, 32) != 0) {
		//system("dd if=/dev/urandom of=/dev/mmcblk0 bs=1M");

#if VERIFY_DEBUG
		ROS_INFO("%s %d: key unmatched", __FUNCTION__, __LINE__);
#endif

		if (unlink(TARGET_FILE) == -1) {

#if VERIFY_DEBUG
			ROS_INFO("%s %d: failed to deleted file: %d", __FUNCTION__, __LINE__, errno);
		} else {
			ROS_INFO("%s %d: file deleted", __FUNCTION__, __LINE__);
		}
#else
		}
#endif

		retval = -1;
	} else {

#if VERIFY_DEBUG
		ROS_INFO("%s %d: key matched", __FUNCTION__, __LINE__);
#endif

		retval = 0;
	}
	return retval;
}
#endif

#if VERIFY_KEY
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
			serial.setSendData(j + DUMMY_DOWNLINK_OFFSET, static_cast<uint8_t>(rand() % 256));

		//Populate Sequence number.
		for (int j = 0; j < SEQUENCE_DOWNLINK_LENGTH; j++)
			serial.setSendData(j + SEQUENCE_DOWNLINK_OFFSET, static_cast<uint8_t>((sequence_number >> 8 * j) % 256));

		//Populate key.
#if VERIFY_DEBUG
		printf("appending key: ");
#endif

		for (int k = 0; k < KEY_DOWNLINK_LENGTH; k++) {
			serial.setSendData(k + KEY_DOWNLINK_OFFSET, key[i * KEY_DOWNLINK_LENGTH + k]);

#if VERIFY_DEBUG
			printf("%02x ", serial.getSendData(k + KEY_DOWNLINK_OFFSET));
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

unsigned char check_sign(unsigned char *a_esno, unsigned char *a_rand)
{
	int	fd;

	Elf32_Ehdr		elfhdr;
	Elf32_Shdr		sechdr;
	unsigned char	*ptr, header[6], signature[SIGNATURE_SIZE], signature_cal[SIGNATURE_SIZE], signature_mcu[SIGNATURE_SIZE];

	if ((a_esno == NULL) || (a_rand == NULL)) {
		return K_ERR_INPUT;
	}

	//Decrypt the encrypted serial number
	//...

	//Get signature from ilife section
	fd = open(TARGET_FILE, O_RDONLY);
	if (fd <= 0) {
		return K_ERR_FILE;
	}

	read(fd, (unsigned char *) &elfhdr, sizeof(elfhdr));
	ptr = (unsigned char *) &elfhdr;
	if ((ptr[0] == 0x7F) && (ptr[1] == 'E') && (ptr[2] == 'L') && (ptr[3] == 'F')) {
		//Seek to ilife section header
		lseek(fd, elfhdr.e_shoff + elfhdr.e_shentsize, SEEK_SET);
		read(fd, (unsigned char *) &sechdr, sizeof(sechdr));
	} else {
		close(fd);
		return K_ERR_FILE;
	}

	//Seek to ilife section
	lseek(fd, sechdr.sh_offset, SEEK_SET);

	/*
	memset(header, 0, sizeof(header));
	read(fd, (unsigned char *) header, sizeof(header));
	if (strncmp((char *) header, "ILIFE", sizeof(header)) != 0) {
		close(fd);
		return K_ERR_HEADER;
	}
	*/

	// Seek to signature offset
	lseek(fd, (sechdr.sh_offset + K_SIGN_OFFSET), SEEK_SET);
	read(fd, signature, sizeof(signature));

	close(fd);


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
	ROS_INFO("Unique ID..........:");
	ROS_INFO("Coordinates:\t%u", (void *) signature_mcu);
	ROS_INFO("Wafer num:\t%u", (char *) signature_mcu + 4);
	ROS_INFO("Uart transfered encrypted signature:");
	for (int i=0; i < SIGNATURE_SIZE; i++)
		ROS_INFO("%x", ((unsigned char *) a_esno)[i]);
	ROS_INFO(" ");

	ROS_INFO("Decoded signature: ");
	for (int i = 0; i < SIGNATURE_SIZE; i++)
		ROS_INFO("%x", ((unsigned char *) signature_mcu)[i]);
	ROS_INFO(" ");
#endif

	//Got unique id signature, now encrypt with primary key
	mbedtls_des3_init(&ctx2);
	mbedtls_des3_set3key_enc(&ctx2, primary_key);

	for (int i = 0; i < SIGNATURE_SIZE; i += 8)
		mbedtls_des3_crypt_ecb(&ctx2, signature_mcu + i, signature_cal + i);

	mbedtls_des3_free(&ctx1);
	mbedtls_des3_free(&ctx2);

#if VERIFY_DEBUG
	ROS_INFO("Final encrypted signature is:");
	for (int i=0; i < SIGNATURE_SIZE; i++)
		ROS_INFO("%x", ((unsigned char *) signature_cal)[i]);
	ROS_INFO(" ");

	ROS_INFO("File signature is:");
	for (int i=0; i < SIGNATURE_SIZE; i++)
		ROS_INFO("%x", ((unsigned char*) signature)[i]);
	ROS_INFO(" ");
#endif

	//Verify signature
	if (memcmp(signature, signature_cal, sizeof(signature)) == 0)
		return K_SIGN_CORRECT;

	return K_ERR_SIGN;
}

int verify_key() {
	int sequence_number, retries = 0;
	const int key_length = ENCRYPTION_KEY_SIZE;
	const int max_retries = 5;

	unsigned char result = 0;

	std::random_device rd;

	std::mt19937 random_number_engine(rd());  // pseudorandom number generator

	std::uniform_int_distribution<uint8_t> dice_distribution(0, 255);

	sequence_number = dice_distribution(random_number_engine) * 256 + dice_distribution(random_number_engine);

	while (retries < max_retries) {
		uint8_t key[ENCRYPTION_KEY_SIZE];
		uint8_t encID[SIGNATURE_SIZE];

#if VERIFY_DEBUG
		ROS_INFO("randoming a key:");
#endif

		for (int i = 0; i < ENCRYPTION_KEY_SIZE; i++) {
			key[i] = (uint8_t) dice_distribution(random_number_engine);

#if VERIFY_DEBUG
			ROS_INFO("0x%02x", key[i]);
			if (i == ENCRYPTION_KEY_SIZE - 1)
				ROS_INFO(" ");
#endif

		}

		mbedtls_des_key_set_parity(key);

#if VERIFY_DEBUG
		ROS_INFO("Sending a key:");
		for (int i = 0; i < ENCRYPTION_KEY_SIZE; i++) {
			ROS_INFO("0x%02x", key[i]);
		}
		ROS_INFO(" ");
#endif

		if (control_get_sign(key, encID, key_length, sequence_number) > 0) {

#if VERIFY_DEBUG
			ROS_INFO("Got encrypted signature:");
			for (int i = 0; i < SIGNATURE_SIZE; i++) {
				ROS_INFO("0x%x", encID[i]);
			}
			ROS_INFO(" ");
#endif

			result = check_sign(encID, key);

#if VERIFY_DEBUG
			ROS_INFO("check_sign status: 0x%x(%d)", result, (int8_t) result);
#endif

		}
		if (result == K_SIGN_CORRECT) {

#if VERIFY_DEBUG
			ROS_INFO("%s %d: result oK, retry: %d", __FUNCTION__, __LINE__, retries);
#endif
			break;
		}

		retries++;
		sequence_number++;
	}

#if VERIFY_DEBUG
	ROS_INFO("%s %d: result %s, retry: %d", __FUNCTION__, __LINE__, result == K_SIGN_CORRECT ? " ok" : "failed", retries);
#endif

	if (result != K_SIGN_CORRECT) {
		if (unlink(TARGET_FILE) == -1) {

#if VERIFY_DEBUG
			ROS_INFO("%s %d: failed to deleted file: %d", __FUNCTION__, __LINE__, errno);
		}
		else {
			ROS_INFO("%s %d: file deleted", __FUNCTION__, __LINE__);
		}
#else
		}
#endif
	}
	return (result == K_SIGN_CORRECT ? 1: 0);
}

#endif
