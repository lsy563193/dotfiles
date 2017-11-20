//
// Created by root on 11/17/17.
//

#ifndef PP_CONTROLLER_H
#define PP_CONTROLLER_H

#include<boost/thread.hpp>
#include "robotbase.h"
#include "movement.h"
#include "crc8.h"
#include "serial.h"

extern uint8_t g_send_stream[SEND_LEN];
extern boost::mutex g_send_stream_mutex;

void set_send_flag(void);

void reset_send_flag(void);


class Controller {
public:
	void set(uint8_t type, uint8_t val) {
		set_send_flag();
		if (type >= CTL_WHEEL_LEFT_HIGH && type <= CTL_GYRO) {
			g_send_stream[type] = val;
		}
		reset_send_flag();
	}

	uint8_t get(uint8_t seq) {
		uint8_t tmp_data;
		g_send_stream_mutex.lock();
		tmp_data = g_send_stream[seq];
		g_send_stream_mutex.unlock();
		return tmp_data;
	}

	int get_sign(uint8_t *key, uint8_t *sign, uint8_t key_length, int sequence_number) {
		int num_send_packets = key_length / KEY_DOWNLINK_LENGTH;
		uint8_t ptr[RECEI_LEN], buf[SEND_LEN];

		//Set random seed.
		//srand(time(NULL));
		//Send random key to robot.
		for (int i = 0; i < num_send_packets; i++) {
			//Populate dummy.
			for (int j = 0; j < DUMMY_DOWNLINK_LENGTH; j++)
				set(j + DUMMY_DOWNLINK_OFFSET, (uint8_t) (rand() % 256));

			//Populate Sequence number.
			for (int j = 0; j < SEQUENCE_DOWNLINK_LENGTH; j++)
				set(j + DUMMY_DOWNLINK_OFFSET, (uint8_t) ((sequence_number >> 8 * j) % 256));

			//Populate key.
#if VERIFY_DEBUG
			printf("appending key: ");
#endif

			for (int k = 0; k < KEY_DOWNLINK_LENGTH; k++) {
				set(k + KEY_DOWNLINK_OFFSET, key[i * KEY_DOWNLINK_LENGTH + k]);

#if VERIFY_DEBUG
				printf("%02x ", g_send_stream[k + KEY_DOWNLINK_OFFSET]);
				if (k == KEY_DOWNLINK_LENGTH - 1)
					printf("\n");
#endif

			}

			//Fill command field
			switch (i) {
				case 0:
					set(SEND_LEN - 4, CMD_KEY1);
					break;
				case 1:
					set(SEND_LEN - 4, CMD_KEY2);
					break;
				case 2:
					set(SEND_LEN - 4, CMD_KEY3);
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
				memcpy(buf, g_send_stream, sizeof(uint8_t) * SEND_LEN);
				g_send_stream_mutex.unlock();
				buf[CTL_CRC] = calc_buf_crc8(buf, SEND_LEN - 3);
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
						set(CTL_CMD, 0x00);
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
				set(CTL_CMD, CMD_ACK);
				for (int k = 0; k < 20; k++) {
					g_send_stream_mutex.lock();
					memcpy(buf, g_send_stream, sizeof(uint8_t) * SEND_LEN);
					g_send_stream_mutex.unlock();
					buf[CTL_CRC] = calc_buf_crc8(buf, SEND_LEN - 3);
					serial_write(SEND_LEN, buf);

					usleep(500);
				}
				set(CTL_CMD, 0x00);

#if VERIFY_DEBUG
				printf("%s %d: exit\n", __FUNCTION__, __LINE__);
#endif

				return KEY_UPLINK_LENGTH;
			}
			usleep(500);
		}
		set(CTL_CMD, 0x00);

#if VERIFY_DEBUG
		printf("%s %d: exit\n", __FUNCTION__, __LINE__);
#endif

		return -1;
	}

	void set_status(uint8_t val) {
		set(CTL_MAIN_PWR, val & 0xff);
	}

uint8_t get_status()
{
	return get(CTL_MAIN_PWR);
}

private:
};

extern Controller controller;

#endif //PP_CONTROLLER_H
