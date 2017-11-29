//
// Created by root on 11/20/17.
//

#ifndef PP_ERROR_H
#define PP_ERROR_H

#include "mathematics.h"

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
	ERROR_CODE_FAN_H,
	ERROR_CODE_WATERTANK,
	ERROR_CODE_BTA,
	ERROR_CODE_OBS,
	ERROR_CODE_BATTERYLOW,
	ERROR_CODE_DUSTBIN,
	ERROR_CODE_GYRO,
	ERROR_CODE_ENCODER,
	ERROR_CODE_SLAM,
	ERROR_CODE_LASER,
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
