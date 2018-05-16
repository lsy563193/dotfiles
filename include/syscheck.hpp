#ifndef __SYSCHECK_H__
#define __SYSCHECK_H__

#include <stdint.h>
#include <thread>
//#include <boost/thread.hpp>
#include "config.h"
#include "termios.h"

namespace SERIAL{
	enum CTL{
		// Two bytes for stream header.
						CTL_HEADER_1 = 0,
		CTL_HEADER_2 = 1,

		// Two bytes for controlling left wheel.
						CTL_WHEEL_LEFT_HIGH = 2,
		CTL_WHEEL_LEFT_LOW = 3,

		// Two bytes for controlling right wheel.
						CTL_WHEEL_RIGHT_HIGH = 4,
		CTL_WHEEL_RIGHT_LOW = 5,

		// One byte for controlling vacuum PWM.
						CTL_VACCUM_PWR = 6,

		// One byte for controlling left brush PWM.
						CTL_BRUSH_LEFT = 7,

		// One byte for controlling right brush PWM.
						CTL_BRUSH_RIGHT = 8,

		// One byte for controlling main brush PWM.
						CTL_BRUSH_MAIN = 9,

		// One byte for controlling beeper.
						CTL_BEEPER = 10,

		// One byte for work mode.
						CTL_WORK_MODE = 11,

		// One byte for controlling charge status.
						CTL_CHARGER = 12,

		// One byte for controlling red led.
						CTL_LED_RED = 13,

		// One byte for controlling green led.
						CTL_LED_GREEN = 14,

		// One byte for mix command.
		// bit 0 for wifi led controlling.
		// bit 1 for vacuum exception resume control.
		// bit 2 for switch of dynamic adjustment.
		// bit 3 for switch of gyro.
		// bit 4 for switch of obs.
		// bit 5-7 reserved.
						CTL_MIX = 15,

		// One byte for controlling water tank.
		// bit 0-6 for controlling swing motor PWM.
		// bit 7 for switch of pump.
						CTL_WATER_TANK = 16,

		// One byte for IR control and test step.
		// bit 0-5 for test step.
		// bit 6-7 for IR control.
						CTL_IR_CTRL = 17,

		// Two bytes for IR error code display.
						CTL_IR_ERROR_CODE_H = 18,
		CTL_IR_ERROR_CODE_L = 19,

		// Two bytes for IR content display.
						CTL_IR_CONTENT_H = 20,
		CTL_IR_CONTENT_L = 21,

		//send appiontment time to bottom board
		//[15:14]
		//[13:0]
						CTL_APPOINTMENT_H = 22,
		CTL_APPOINTMENT_L = 23,

		// One byte for key validation.
						CTL_KEY_VALIDATION = 24,

		// One byte for crc checking.
						CTL_CRC = 25,

		// Two bytes for stream trailer.
						CTL_TRAILER_1 = 26,
		CTL_TRAILER_2 = 27,

		// For control stream.
						SEND_LEN = 28,
	};

	enum REC{
		// Two bytes for stream header.
						REC_HEADER_1 = 0,
		REC_HEADER_2 = 1,

		// Two bytes for left wheel speed
						REC_WHEEL_L_SPEED_H = 2,
		REC_WHEEL_L_SPEED_L = 3,
		// Two bytes for right wheel speed
						REC_WHEEL_R_SPEED_H = 4,
		REC_WHEEL_R_SPEED_L = 5,
		// One byte for wheel cliff switch status
						REC_WHEEL_CLIFF = 6,

		// One byte for gyro calibration status
						REC_GYRO_CALIBRATION = 7,
		// Two bytes for gyro angle value
						REC_ANGLE_H = 8,
		REC_ANGLE_L = 9,
		// Two bytes for gyro angle velocity value
						REC_ANGLE_V_H = 10,
		REC_ANGLE_V_L = 11,
		// Two bytes for gyro x acceleration value
						REC_XACC_H = 12,
		REC_XACC_L = 13,
		// Two bytes for gyro y acceleration value
						REC_YACC_H = 14,
		REC_YACC_L = 15,
		// Two bytes for gyro z acceleration value
						REC_ZACC_H = 16,
		REC_ZACC_L = 17,

		// Two bytes for left wall sensor value
						REC_L_WALL_H = 18,
		REC_L_WALL_L = 19,
		// Two bytes for right wall sensor value
						REC_R_WALL_H = 20,
		REC_R_WALL_L = 21,

		// Two bytes for left OBS sensor value
						REC_L_OBS_H = 22,
		REC_L_OBS_L = 23,
		// Two bytes for front OBS sensor value
						REC_F_OBS_H = 24,
		REC_F_OBS_L = 25,
		// Two bytes for right OBS sensor value
						REC_R_OBS_H = 26,
		REC_R_OBS_L = 27,

		// One byte for bumper status and cliff triggered status.
		// bit 0 for right cliff.
		// bit 1 for front cliff.
		// bit 2 for right cliff.
		// bit 3 reserved.
		// bit 4 for right bumper.
		// bit 5 for left bumper.
		// bit 6 reserved.
		// bit 7 reserved.
						REC_BUMPER_AND_CLIFF = 28,

		// One byte for remote controller signal.
		// bit 0 for remote spot.
		// bit 1 for remote follow wall.
		// bit 2 for remote home.
		// bit 3 for remote clean.
		// bit 4 for remote max.
		// bit 5 for remote right.
		// bit 6 for remote left.
		// bit 7 for remote forward.
						REC_REMOTE = 29,

		// Four bytes for rcon receiving charger signal.
		// bit 0 for rcon left receiving charger right signal.
		// bit 1 for rcon left receiving charger top signal.
		// bit 2 for rcon left receiving charger left signal.
		// bit 3 reserved.
		// bit 4 for rcon back left receiving charger right signal.
		// bit 5 for rcon back left receiving charger top signal.
		// bit 6 for rcon back left receiving charger left signal.
		// bit 7 reserved.
						REC_RCON_CHARGER_4 = 30,
		// bit 0 for rcon front left receiving charger right signal.
		// bit 1 for rcon front left receiving charger top signal.
		// bit 2 for rcon front left receiving charger left signal.
		// bit 3 reserved.
		// bit 4 for rcon front left 2 receiving charger right signal.
		// bit 5 for rcon front left 2 receiving charger top signal.
		// bit 6 for rcon front left 2 receiving charger left signal.
		// bit 7 reserved.
						REC_RCON_CHARGER_3 = 31,
		// bit 0 for rcon front right 2 receiving charger right signal.
		// bit 1 for rcon front right 2 receiving charger top signal.
		// bit 2 for rcon front right 2 receiving charger left signal.
		// bit 3 reserved.
		// bit 4 for rcon front right receiving charger right signal.
		// bit 5 for rcon front right receiving charger top signal.
		// bit 6 for rcon front right receiving charger left signal.
		// bit 7 reserved.
						REC_RCON_CHARGER_2 = 32,
		// bit 0 for rcon back right receiving charger right signal.
		// bit 1 for rcon back right receiving charger top signal.
		// bit 2 for rcon back right receiving charger left signal.
		// bit 3 reserved.
		// bit 4 for rcon right receiving charger right signal.
		// bit 5 for rcon right receiving charger top signal.
		// bit 6 for rcon right receiving charger left signal.
		// bit 7 reserved.
						REC_RCON_CHARGER_1 = 33,

		// Two bytes for rcon receiving virtual wall signal.
		// bit 0 for rcon back right receiving virtual wall code signal.
		// bit 1 for rcon right receiving virtual wall code signal.
		// bit 2 for rcon front right 2 receiving virtual wall code signal.
		// bit 3 for rcon front right receiving virtual wall code signal.
		// bit 4 for rcon front left receiving virtual wall code signal.
		// bit 5 for rcon front left 2 receiving virtual wall code signal.
		// bit 6 for rcon left receiving virtual wall code signal.
		// bit 7 for rcon back left right receiving virtual wall code signal.
						REC_VISUAL_WALL_H = 34,
		// bit 0 for rcon back right receiving virtual wall top signal.
		// bit 1 for rcon right receiving virtual wall top signal.
		// bit 2 for rcon front right 2 receiving virtual wall top signal.
		// bit 3 for rcon front right receiving virtual wall top signal.
		// bit 4 for rcon front left receiving virtual wall top signal.
		// bit 5 for rcon front left 2 receiving virtual wall top signal.
		// bit 6 for rcon left receiving virtual wall top signal.
		// bit 7 for rcon back left right receiving virtual wall top signal.
						REC_VISUAL_WALL_L = 35,

		// One byte for mix status.
		// bit 0 for key clean.
		// bit 1/2 for plan status.
		// bit 3 for water tank status.
		// bit 4/5/6 for charge status.
		// bit 7 reserved.
						REC_MIX_BYTE = 36,

		// One byte for battery voltage.
						REC_BATTERY = 37,

		// One byte for work mode.
						REC_WORK_MODE = 38,

		// One byte for over current signal.
		// bit 0 for vacuum over current.
		// bit 1 for right brush over current.
		// bit 2 for main brush over current.
		// bit 3 for left brush over current.
		// bit 4 for right wheel over current.
		// bit 5 for left wheel over current.
		// bit 6 for water tank over current.
		// bit 7 reserved.
						REC_OC = 39,

		// One byte for left wheel encoder.
						REC_LEFT_WHEEL_ENCODER = 40,

		// One byte for right wheel encoder.
						REC_RIGHT_WHEEL_ENCODER = 41,
		// appintment time in 15mins
						REC_APPOINTMENT_TIME = 42,
		//real time in 1mins
						REC_REALTIME_H = 43,
		REC_REALTIME_L = 44,

		// One byte for key validation.
						REC_KEY_VALIDATION = 45,

		// One byte for crc checking.
						REC_CRC = 46,

		// Two bytes for stream trailer.
						REC_TRAILER_1 = 47,
		REC_TRAILER_2 = 48,
		// For receive stream.
						REC_LEN = 49,

	};//end enum REC
}//end namespace serial

#define DI		0x07

using namespace SERIAL;

class Serial
{
public:
	Serial();
	~Serial();

	bool init(const std::string port,int baudrate);

	int close();

	int flush();

	bool isReady();

	int write(uint8_t *buf, uint8_t len);

	int read(uint8_t *buf, int len);

	void setSendData(uint8_t seq, uint8_t val);

	void initCrc8(void);

	void crc8(uint8_t *crc, const uint8_t m);

	uint8_t calBufCrc8(const uint8_t *inBuf, uint32_t inBufSz);

	//										   1    2    3    4    5    6    7    8    9   10
	uint8_t receive_stream[REC_LEN]={		0xaa,0x55,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
																			 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
																			 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
																			 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
																			 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xcc,0x33};
	//										   1    2    3    4    5    6    7    8    9   10
	uint8_t send_stream[SEND_LEN]={			0xaa,0x55,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
																			 0x00,0x00,0x00,0x00,0x64,0x10,0x02,0x00,0x00,0x00,
																			 0x00,0x00,0x00,0x00,0x00,0x00,0xcc,0x33};

	void sendData();

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
