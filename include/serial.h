#ifndef __SERIAL_H__
#define __SERIAL_H__

#include <stdint.h>
#include <thread>
#include <boost/thread.hpp>
#include "config.h"
#include "termios.h"

// For control stream.
#define SEND_LEN 21

// Two bytes for stream header.
#define CTL_HEADER_1 0
#define CTL_HEADER_2 1

// Two bytes for controlling left wheel.
#define	CTL_WHEEL_LEFT_HIGH 2
#define	CTL_WHEEL_LEFT_LOW  3

// Two bytes for controlling right wheel.
#define	CTL_WHEEL_RIGHT_HIGH  4
#define	CTL_WHEEL_RIGHT_LOW 5

// One byte for controlling vacuum PWM.
#define	CTL_VACCUM_PWR 6

// One byte for controlling left brush PWM.
#define	CTL_BRUSH_LEFT 7

// One byte for controlling right brush PWM.
#define	CTL_BRUSH_RIGHT 8

// One byte for controlling main brush PWM.
#define	CTL_BRUSH_MAIN 9

// One byte for controlling beeper.
#define	CTL_BEEPER 10

// One byte for sending clean mode.
#define	CTL_CLEAN_MODE 11

// One byte for controlling charge status.
#define	CTL_CHARGER 12

// One byte for controlling red led.
#define	CTL_LED_RED 13

// One byte for controlling green led.
#define	CTL_LED_GREEN 14

// One byte for mix command.
// bit 0 for wifi led controlling.
// bit 1 for vacuum exception resume control.
// bit 2 for vacuum exception resume status
// bit 3-7 reserved.
#define CTL_MIX 15

// One byte for controlling gyro.
// bit 0 for switch of dynamic adjustment.
// bit 1 for switch of gyro.
// bit 2-7 reserved.
#define CTL_GYRO 16

// One byte for key validation.
#define CTL_KEY_VALIDATION 17

// One byte for crc checking.
#define CTL_CRC 18

// Two bytes for stream trailer.
#define CTL_TRAILER_1 19
#define CTL_TRAILER_2 20

// For receive stream.
#define RECEI_LEN 44

// Two bytes for stream header.
#define REC_HEADER_1 0
#define REC_HEADER_2 1

// Two bytes for left wheel speed
#define REC_WHEEL_L_SPEED_H 2
#define REC_WHEEL_L_SPEED_L 3
// Two bytes for right wheel speed
#define REC_WHEEL_R_SPEED_H 4
#define REC_WHEEL_R_SPEED_L 5
// One byte for wheel cliff switch status
#define REC_WHEEL_CLIFF 6

// One byte for gyro calibration status
#define REC_GYRO_CALIBRATION 7
// Two bytes for gyro angle value
#define REC_ANGLE_H 8
#define REC_ANGLE_L 9
// Two bytes for gyro angle velocity value
#define REC_ANGLE_V_H 10
#define REC_ANGLE_V_L 11
// Two bytes for gyro x acceleration value
#define REC_XACC_H 12
#define REC_XACC_L 13
// Two bytes for gyro y acceleration value
#define REC_YACC_H 14
#define REC_YACC_L 15
// Two bytes for gyro z acceleration value
#define REC_ZACC_H 16
#define REC_ZACC_L 17

// Two bytes for left wall sensor value
#define REC_L_WALL_H 18
#define REC_L_WALL_L 19
// Two bytes for right wall sensor value
#define REC_R_WALL_H 20
#define REC_R_WALL_L 21

// Two bytes for left OBS sensor value
#define REC_L_OBS_H 22
#define REC_L_OBS_L 23
// Two bytes for front OBS sensor value
#define REC_F_OBS_H 24
#define REC_F_OBS_L 25
// Two bytes for right OBS sensor value
#define REC_R_OBS_H 26
#define REC_R_OBS_L 27

// One byte for bumper status and cliff triggered status.
// bit 0 for right cliff.
// bit 1 for front cliff.
// bit 2 for right cliff.
// bit 3 reserved.
// bit 4 for right bumper.
// bit 5 for left bumper.
// bit 6 reserved.
// bit 7 reserved.
#define REC_BUMPER_AND_CLIFF 28

// One byte for remote controller signal.
// bit 0 for remote spot.
// bit 1 for remote follow wall.
// bit 2 for remote home.
// bit 3 for remote clean.
// bit 4 for remote max.
// bit 5 for remote right.
// bit 6 for remote left.
// bit 7 for remote forward.
#define REC_REMOTE 29

// Four bytes for rcon receiving charger signal.
// bit 0 for rcon left receiving charger right signal.
// bit 1 for rcon left receiving charger top signal.
// bit 2 for rcon left receiving charger left signal.
// bit 3 reserved.
// bit 4 for rcon back left receiving charger right signal.
// bit 5 for rcon back left receiving charger top signal.
// bit 6 for rcon back left receiving charger left signal.
// bit 7 reserved.
#define REC_RCON_CHARGER_4 30
// bit 0 for rcon front left receiving charger right signal.
// bit 1 for rcon front left receiving charger top signal.
// bit 2 for rcon front left receiving charger left signal.
// bit 3 reserved.
// bit 4 for rcon front left 2 receiving charger right signal.
// bit 5 for rcon front left 2 receiving charger top signal.
// bit 6 for rcon front left 2 receiving charger left signal.
// bit 7 reserved.
#define REC_RCON_CHARGER_3 31
// bit 0 for rcon front right 2 receiving charger right signal.
// bit 1 for rcon front right 2 receiving charger top signal.
// bit 2 for rcon front right 2 receiving charger left signal.
// bit 3 reserved.
// bit 4 for rcon front right receiving charger right signal.
// bit 5 for rcon front right receiving charger top signal.
// bit 6 for rcon front right receiving charger left signal.
// bit 7 reserved.
#define REC_RCON_CHARGER_2 32
// bit 0 for rcon back right receiving charger right signal.
// bit 1 for rcon back right receiving charger top signal.
// bit 2 for rcon back right receiving charger left signal.
// bit 3 reserved.
// bit 4 for rcon right receiving charger right signal.
// bit 5 for rcon right receiving charger top signal.
// bit 6 for rcon right receiving charger left signal.
// bit 7 reserved.
#define REC_RCON_CHARGER_1 33

// Two bytes for rcon receiving virtual wall signal.
// bit 0 for rcon back right receiving virtual wall code signal.
// bit 1 for rcon right receiving virtual wall code signal.
// bit 2 for rcon front right 2 receiving virtual wall code signal.
// bit 3 for rcon front right receiving virtual wall code signal.
// bit 4 for rcon front left receiving virtual wall code signal.
// bit 5 for rcon front left 2 receiving virtual wall code signal.
// bit 6 for rcon left receiving virtual wall code signal.
// bit 7 for rcon back left right receiving virtual wall code signal.
#define REC_VISUAL_WALL_H 34
// bit 0 for rcon back right receiving virtual wall top signal.
// bit 1 for rcon right receiving virtual wall top signal.
// bit 2 for rcon front right 2 receiving virtual wall top signal.
// bit 3 for rcon front right receiving virtual wall top signal.
// bit 4 for rcon front left receiving virtual wall top signal.
// bit 5 for rcon front left 2 receiving virtual wall top signal.
// bit 6 for rcon left receiving virtual wall top signal.
// bit 7 for rcon back left right receiving virtual wall top signal.
#define REC_VISUAL_WALL_L 35

// One byte for mix status.
// bit 0 for key clean.
// bit 1/2 for plan status.
// bit 3 for water tank status.
// bit 4/5/6 for charge status.
// bit 7 reserved.
#define REC_MIX_BYTE 36

// One byte for battery voltage.
#define REC_BATTERY 37

// One byte for vacuum exception resume status.
#define REC_VACUUM_EXCEPTION_RESUME 38

// One byte for over current signal.
// bit 0 for vacuum over current.
// bit 1 for right brush over current.
// bit 2 for main brush over current.
// bit 3 for left brush over current.
// bit 4 for right wheel over current.
// bit 5 for left wheel over current.
// bit 6 for water tank over current.
// bit 7 reserved.
#define REC_OC 39

// One byte for key validation.
#define REC_KEY_VALIDATION 40

// One byte for crc checking.
#define REC_CRC 41

// Two bytes for stream trailer.
#define REC_TRAILER_1 42
#define REC_TRAILER_2 43


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

	//										   1    2    3    4    5    6    7    8    9   10
	uint8_t receive_stream[RECEI_LEN]={		0xaa,0x55,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
											0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
											0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
											0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
											0x00,0x00,0xcc,0x33};
	uint8_t send_stream[SEND_LEN]={0xaa,0x55,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x64,0x00,0x02,0x00,0x00,0xcc,0x33};

	void receive_routine_cb();

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
