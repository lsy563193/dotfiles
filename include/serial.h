#ifndef __SERIAL_H__
#define __SERIAL_H__

#include <stdint.h>
#include <thread>
#include <boost/thread.hpp>
#include "config.h"
#include "termios.h"

// For control stream.
#define SEND_LEN 26

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

// One byte for work mode.
#define	CTL_WORK_MODE 11

// One byte for controlling charge status.
#define	CTL_CHARGER 12

// One byte for controlling red led.
#define	CTL_LED_RED 13

// One byte for controlling green led.
#define	CTL_LED_GREEN 14

// One byte for mix command.
// bit 0 for wifi led controlling.
// bit 1 for vacuum exception resume control.
// bit 2 for switch of dynamic adjustment.
// bit 3 for switch of gyro.
// bit 4-7 reserved.
#define CTL_MIX 15

// One byte for controlling water tank.
// bit 0 for switch of pump.
// bit 1-7 for controlling swing motor PWM.
#define CTL_WATER_TANK 16

// One byte for IR control and test step.
// bit 0-5 for test step.
// bit 6-7 for IR control.
#define CTL_IR_CTRL 17

// Two bytes for IR error code display.
#define CTL_IR_ERROR_CODE_H 18
#define CTL_IR_ERROR_CODE_L 19

// Two bytes for IR content display.
#define CTL_IR_CONTENT_H 20
#define CTL_IR_CONTENT_L 21

// One byte for key validation.
#define CTL_KEY_VALIDATION 22

// One byte for crc checking.
#define CTL_CRC 23

// Two bytes for stream trailer.
#define CTL_TRAILER_1 24
#define CTL_TRAILER_2 25

// ------------For functional test--------------
#define CTL_TESTING_STAGE 2
#define CTL_ERROR_CODE_HIGH 3
#define CTL_ERROR_CODE_LOW 4
#define CTL_CURRENT_DATA_HIGH 5
#define CTL_CURRENT_DATA_LOW 6
// For message type
// 0 for normal message
// 1 for error message
#define CTL_MESSAGE_TYPE 7
// For motors test mode
// 0 for idle mode
// 1 for stall mode
#define CTL_LEFT_WHEEL_TEST_MODE 6
#define CTL_RIGHT_WHEEL_TEST_MODE 7
#define CTL_LEFT_BRUSH_TEST_MODE 2
#define CTL_MAIN_BRUSH_TEST_MODE 3
#define CTL_RIGHT_BRUSH_TEST_MODE 4
#define CTL_VACUUM_TEST_MODE 7
// For Charger Connected Status
// 0 for no charger connected
// 1 for already connect charger
#define CTL_CHARGER_CINNECTED_STATUS 3
// Is on fixture
#define CTL_IS_FIXTURE 2
// ------------For functional test end--------------

// For receive stream.
#define REC_LEN 46

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

// One byte for work mode.
#define REC_WORK_MODE 38

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

// One byte for left wheel encoder.
#define REC_LEFT_WHEEL_ENCODER 40

// One byte for right wheel encoder.
#define REC_RIGHT_WHEEL_ENCODER 41

// One byte for key validation.
#define REC_KEY_VALIDATION 42

// One byte for crc checking.
#define REC_CRC 43

// Two bytes for stream trailer.
#define REC_TRAILER_1 44
#define REC_TRAILER_2 45

// ------------------------------work mode--------------------------------------
#define NORMAL_SLEEP_MODE 		0
#define BATTERY_FULL_SLEEP_MODE 1
#define WORK_MODE 				2
#define IDLE_MODE 				3
#define CHARGE_MODE 			4
#define FUNC_SERIAL_TEST_MODE		5
#define FUNC_ELECTRICAL_AND_LED_TEST_MODE		6
#define FUNC_OBS_TEST_MODE		7
#define FUNC_BUMPER_TEST_MODE		8
#define FUNC_CLIFF_TEST_MODE		9
#define FUNC_RCON_TEST_MODE		10
#define FUNC_WATER_TANK_TEST_MODE	11
#define FUNC_WHEELS_TEST_MODE	12
#define FUNC_BRUSHES_TEST_MODE		13
#define FUNC_VACUUM_TEST_MODE		14
#define FUNC_CHARGE_CURRENT_TEST_MODE		15
#define DESK_TEST_CURRENT_MODE		16 // For checking current
#define DESK_TEST_MOVEMENT_MODE		17
#define DESK_TEST_WRITE_BASELINE_MODE	18
#define GYRO_TEST_MODE		19
#define LIFE_TEST_MODE		20
#define WATER_TANK_TEST_MODE	21
#define R16_AND_LIDAR_TEST_MODE	22

// ------------------------------work mode end--------------------------------------

// -----------------------------For DESK_TEST_CURRENT_MODE and LIFE_TEST_MODE ------------------------------------

// Two bytes for right brush current.
#define REC_M_BRUSH_CUNT_H 6
#define REC_M_BRUSH_CUNT_L 7
// Two bytes for left cliff value.
#define REC_L_CLIFF_H 12
#define REC_L_CLIFF_L 13
// Two bytes for front cliff value.
#define REC_F_CLIFF_H 14
#define REC_F_CLIFF_L 15
// Two bytes for right cliff value
#define REC_R_CLIFF_H 16
#define REC_R_CLIFF_L 17
// Two bytes for left wheel current.
#define REC_L_WHEEL_CUNT_H 18
#define REC_L_WHEEL_CUNT_L 19
// Two bytes for right wheel current.
#define REC_R_WHEEL_CUNT_H 20
#define REC_R_WHEEL_CUNT_L 21
// Two bytes for robot total current.
#define REC_ROBOT_CUNT_H 22
#define REC_ROBOT_CUNT_L 23
// Two bytes for left brush current.
#define REC_L_BRUSH_CUNT_H 24
#define REC_L_BRUSH_CUNT_L 25
// Two bytes for right brush current.
#define REC_R_BRUSH_CUNT_H 26
#define REC_R_BRUSH_CUNT_L 27
// Two bytes for vacuum current.
#define REC_VACUUM_CURRENT_H 30
#define REC_VACUUM_CURRENT_L 31
// Two bytes for water pump current.
#define REC_WATER_PUMP_CURRENT_H 32
#define REC_WATER_PUMP_CURRENT_L 33

// -----------------------------For DESK_TEST_CURRENT_MODE and LIFE_TEST_MODE end------------------------------------

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

class Serial
{
public:
	Serial();
	~Serial();

	bool init(const std::string port,int baudrate);

	int close();

	int flush();

	bool isReady();

	void isMainBoardSleep(bool val)
	{
		is_main_board_sleep_ = val;
	}

	bool isMainBoardSleep() const
	{
		return is_main_board_sleep_;
	}

	int write(uint8_t *buf, uint8_t len);

	int read(uint8_t *buf, int len);

	void resetSendStream(void);

	void setSendData(uint8_t seq, uint8_t val);

	uint8_t getSendData(uint8_t seq);

	//int get_sign(uint8_t *key, uint8_t *sign, uint8_t key_length, int sequence_number);

	void setWorkMode(uint8_t val);

	void initCrc8(void);

	void crc8(uint8_t *crc, const uint8_t m);

	uint8_t calBufCrc8(const uint8_t *inBuf, uint32_t inBufSz);

	//										   1    2    3    4    5    6    7    8    9   10
	uint8_t receive_stream[REC_LEN]={		0xaa,0x55,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
											0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
											0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
											0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
											0x00,0x00,0x00,0x00,0xcc,0x33};
	//										   1    2    3    4    5    6    7    8    9   10
	uint8_t send_stream[SEND_LEN]={			0xaa,0x55,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
											0x00,0x00,0x00,0x00,0x64,0x00,0x02,0x00,0x00,0x00,
											0x00,0x00,0x00,0x00,0xcc,0x33};

	void receive_routine_cb();

	void sendData();

	void send_routine_cb();

	void debugReceivedStream(const uint8_t *buf);

	void debugSendStream(const uint8_t *buf);

private:

	bool is_main_board_sleep_{};

	int	crport_fd_;
	bool serial_port_ready_;
	struct termios orgopt_, curopt_;
	int bardrate_;
	std::string port_{};

	// For crc8
	int made_table_ = 0;
	uint8_t crc8_table_[256];	/* 8-bit table */
};

extern Serial serial;
#endif
