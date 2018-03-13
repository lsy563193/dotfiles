//
// Created by root on 11/20/17.
//

#ifndef PP_ERROR_H
#define PP_ERROR_H

#include "stdint.h"

// Definition for error code.
#define SERIAL_ERROR 		((uint16_t)3001)
#define RAM_ERROR 			((uint16_t)3002)
#define FLASH_ERROR 		((uint16_t)3003)
#define LIDAR_ERROR 		((uint16_t)3004)
#define LIDAR_BUMPER_ERROR 	((uint16_t)3005)
//#define MAIN_BOARD_ERROR 	((uint16_t)3006)
#define BASELINE_VOLTAGE_ERROR				(uint16_t)2101
#define BATTERY_ERROR						(uint16_t)2301
#define BATTERY_LOW							(uint16_t)2302
#define BASELINE_CURRENT_ERROR				(uint16_t)2401
#define BASELINE_CURRENT_LOW				(uint16_t)2402
#define LEFT_OBS_ERROR						(uint16_t)101
#define FRONT_OBS_ERROR						(uint16_t)102
#define RIGHT_OBS_ERROR						(uint16_t)103
#define LEFT_WALL_ERROR						(uint16_t)104
#define RIGHT_WALL_ERROR					(uint16_t)105
#define OBS_ENABLE_ERROR					(uint16_t)106
#define LEFT_BUMPER_ERROR					(uint16_t)201
#define RIGHT_BUMPER_ERROR					(uint16_t)202
#define LEFT_CLIFF_ERROR					(uint16_t)301
#define FRONT_CLIFF_ERROR					(uint16_t)302
#define RIGHT_CLIFF_ERROR					(uint16_t)303
#define LEFT_WHEEL_SW_ERROR					(uint16_t)304
#define RIGHT_WHEEL_SW_ERROR				(uint16_t)305
#define BLRCON_ERROR						(uint16_t)401
#define LRCON_ERROR							(uint16_t)402
#define FL2RCON_ERROR						(uint16_t)403
#define FLRCON_ERROR						(uint16_t)404
#define FRRCON_ERROR						(uint16_t)405
#define FR2RCON_ERROR						(uint16_t)406
#define RRCON_ERROR							(uint16_t)407
#define BRRCON_ERROR						(uint16_t)408
#define LEFT_WHEEL_FORWARD_CURRENT_ERROR	(uint16_t)501
#define LEFT_WHEEL_FORWARD_PWM_ERROR		(uint16_t)502
#define LEFT_WHEEL_FORWARD_ENCODER_FAIL		(uint16_t)503
#define LEFT_WHEEL_FORWARD_ENCODER_ERROR	(uint16_t)504
#define LEFT_WHEEL_BACKWARD_CURRENT_ERROR	(uint16_t)505
#define LEFT_WHEEL_BACKWARD_PWM_ERROR		(uint16_t)506
#define LEFT_WHEEL_BACKWARD_ENCODER_FAIL	(uint16_t)507
#define LEFT_WHEEL_BACKWARD_ENCODER_ERROR	(uint16_t)508
#define LEFT_WHEEL_STALL_ERROR				(uint16_t)509
#define RIGHT_WHEEL_FORWARD_CURRENT_ERROR	(uint16_t)601
#define RIGHT_WHEEL_FORWARD_PWM_ERROR		(uint16_t)602
#define RIGHT_WHEEL_FORWARD_ENCODER_FAIL	(uint16_t)603
#define RIGHT_WHEEL_FORWARD_ENCODER_ERROR	(uint16_t)604
#define RIGHT_WHEEL_BACKWARD_CURRENT_ERROR	(uint16_t)605
#define RIGHT_WHEEL_BACKWARD_PWM_ERROR		(uint16_t)606
#define RIGHT_WHEEL_BACKWARD_ENCODER_FAIL	(uint16_t)607
#define RIGHT_WHEEL_BACKWARD_ENCODER_ERROR	(uint16_t)608
#define RIGHT_WHEEL_STALL_ERROR				(uint16_t)609
#define	LEFT_BRUSH_CURRENT_ERROR			(uint16_t)701
#define LEFT_BRUSH_STALL_ERROR				(uint16_t)702
#define	RIGHT_BRUSH_CURRENT_ERROR			(uint16_t)801
#define RIGHT_BRUSH_STALL_ERROR				(uint16_t)802
#define	MAIN_BRUSH_CURRENT_ERROR			(uint16_t)901
#define MAIN_BRUSH_STALL_ERROR				(uint16_t)902
#define VACUUM_CURRENT_ERROR					(uint16_t)1001
#define VACUUM_PWM_ERROR							(uint16_t)1002
#define VACUUM_ENCODER_FAIL						(uint16_t)1003
#define VACUUM_ENCODER_ERROR					(uint16_t)1004
#define VACUUM_STALL_ERROR						(uint16_t)1005
#define CHARGE_PWM_ERROR						(uint16_t)1101
#define CHARGE_CURRENT_ERROR				(uint16_t)1102

typedef enum {
	ERROR_CODE_NONE,
	ERROR_CODE_LEFTWHEEL,
	ERROR_CODE_RIGHTWHEEL,
	ERROR_CODE_LEFTBRUSH,
	ERROR_CODE_RIGHTBRUSH,
	ERROR_CODE_PICKUP,
	ERROR_CODE_CLIFF,
	ERROR_CODE_BUMPER,
	ERROR_CODE_STUCK,
	ERROR_CODE_MAINBRUSH,
	ERROR_CODE_VACUUM,
	ERROR_CODE_WATERTANK,
	ERROR_CODE_BTA,
	ERROR_CODE_OBS,
	ERROR_CODE_BATTERYLOW,
	ERROR_CODE_DUSTBIN,
	ERROR_CODE_GYRO,
	ERROR_CODE_ENCODER,
	ERROR_CODE_SLAM,
	ERROR_CODE_LIDAR,
	ERROR_CODE_TEST,
	ERROR_CODE_TEST_NULL
}ErrorType;

class Error {
public:
	Error()
	{
		error_code_ = ERROR_CODE_NONE;
	}
	void set(ErrorType code)
	{
		error_code_ = code;
	}

	ErrorType get()
	{
		return error_code_;
	}

	void alarm(void);

	bool clear(uint8_t code);

private:
	ErrorType error_code_;
};

extern Error error;
#endif //PP_ERROR_H
