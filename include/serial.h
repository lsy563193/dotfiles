#ifndef __SERIAL_H__
#define __SERIAL_H__

#include <stdint.h>
#include <thread>
#include <boost/thread.hpp>
#include "config.h"
#include "termios.h"

#define RECEI_LEN 57
#define SEND_LEN 21
#define	CTL_WHEEL_LEFT_HIGH 2
#define	CTL_WHEEL_LEFT_LOW  3
#define	CTL_WHEEL_RIGHT_HIGH  4
#define	CTL_WHEEL_RIGHT_LOW 5
#define	CTL_VACCUM_PWR 6
#define	CTL_BRUSH_LEFT 7
#define	CTL_BRUSH_RIGHT 8
#define	CTL_BRUSH_MAIN 9
#define	CTL_BUZZER 10
#define	CTL_MAIN_PWR 11
#define	CTL_CHARGER 12
#define	CTL_LED_RED 13
#define	CTL_LED_GREEN 14

#define REC_LW_S_H 2//left wheel speed high byte
#define REC_LW_S_L 3
#define REC_RW_S_H 4//right wheel speed high byte
#define REC_RW_S_L 5

#define REC_ANGLE_H 6 //angle high byte
#define REC_ANGLE_L 7
#define REC_ANGLE_V_H 8//angle velocity high byte
#define REC_ANGLE_V_L 9

#define REC_LW_C_H 10//left wheel current high byte
#define REC_LW_C_L 11
#define REC_RW_C_H 12
#define REC_RW_C_L 13

#define REC_L_WALL_H 14 //left wall sensor high byte
#define REC_L_WALL_L 15

#define REC_L_OBS_H 16 //left obs high byte
#define REC_L_OBS_L 17
#define REC_F_OBS_H 18//front obs high byte
#define REC_F_OBS_L 19
#define REC_R_OBS_H 20//right obs high byte
#define REC_R_OBS_L 21

#define REC_R_WALL_H 22//right wall sensor high byte
#define REC_R_WALL_L 23//low byte

#define REC_BUMPER 24//bumper

#define REC_REMOTE_IR 25//remote ir control

#define REC_CHARGE_STUB_4 26//charge stub signal byte 4
#define REC_CHARGE_STUB_3 27//byte 3
#define REC_CHARGE_STUB_2 28//byte 2
#define REC_CHARGE_STUB_1 29//byte 1

#define REC_VISUAL_WALL_H 30//visual wall high byte
#define REC_VISUAL_WALL_L 31

#define REC_KEY 32//key on robot

#define REC_CHARGE_STATE 33

#define REC_WATER_TANK 34

#define REC_BAT_V 35//battery voltage

#define REC_L_CLIFF_H 36//left cliff high byte
#define REC_L_CLIFF_L 37
#define REC_F_CLIFF_H 38//front cliff high byte
#define REC_F_CLIFF_L 39
#define REC_R_CLIFF_H 40//right cliff high byte
#define REC_R_CLIFF_L 41

#define REC_CL_OC 42//clean tools over current

#define REC_GYRO_DYMC 43//gyro dynamic

#define REC_OMNI_W_H 44//omni wheel count value high byte
#define REC_OMNI_W_L 45

#define REC_XACC_H 46//x acceleration high byte
#define REC_XACC_L 47
#define REC_YACC_H 48
#define REC_YACC_L 49
#define REC_ZACC_H 50
#define REC_ZACC_L 51

#define REC_PLAN 52//set clean plan

#define CTL_OMNI_RESET 15
#define CTL_GYRO 16
#define CTL_CMD				17
#define CTL_CRC				18

#define DUMMY_DOWNLINK_OFFSET		2
#define KEY_DOWNLINK_OFFSET			9
#define SEQUENCE_DOWNLINK_OFFSET	7

#define DUMMY_DOWNLINK_LENGTH		5
#define SEQUENCE_DOWNLINK_LENGTH	2
#define KEY_DOWNLINK_LENGTH			8

#define KEY_UPLINK_OFFSET			36
#define KEY_UPLINK_LENGTH			16

#define CMD_UPLINK_OFFSET			53

#define CMD_KEY1					0x40
#define CMD_KEY2					0x41
#define CMD_KEY3					0x42
#define CMD_ID						0x43

#define CMD_ACK						0x23
#define CMD_NCK						0x25

#define DI		0x07

extern pthread_mutex_t serial_data_ready_mtx;
extern pthread_cond_t serial_data_ready_cond;

extern boost::mutex g_send_stream_mutex;
extern boost::mutex g_receive_stream_mutex;

class Serial
{
public:
	Serial();
	~Serial(){};

	bool init(const char* port,int baudrate);

	int close();

	int flush();

	void sleep(void);

	void wakeUp(void);

	bool isSleep(void);

	bool is_ready();

	int write(uint8_t len, uint8_t *buf);

	int read(int len,uint8_t *buf);

	void setSendData(uint8_t seq, uint8_t val);

	uint8_t getSendData(uint8_t seq);

	uint8_t getReceiveData(uint8_t seq);

	void setReceiveData(uint8_t (&buf)[RECEI_LEN]);

	int get_sign(uint8_t *key, uint8_t *sign, uint8_t key_length, int sequence_number);

	void setCleanMode(uint8_t val);

	uint8_t getCleanMode();

	void init_crc8(void);

	void crc8(uint8_t *crc, const uint8_t m);

	uint8_t calc_buf_crc8(const uint8_t *inBuf, uint32_t inBufSz);


	uint8_t receive_stream[RECEI_LEN]={		0xaa,0x55,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
											0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
											0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
											0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
											0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
											0x00,0x00,0x00,0x00,0x00,0xcc,0x33};
	uint8_t send_stream[SEND_LEN]={0xaa,0x55,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x64,0x00,0x02,0x00,0x00,0xcc,0x33};

private:

	bool sleep_status_;

	int	crport_fd_;
	bool serial_init_done_;
	struct termios orgopt_, curopt_;
	int bardrate_;

	// For crc8
	int made_table_ = 0;
	uint8_t crc8_table_[256];	/* 8-bit table */

};

extern Serial serial;
#endif
