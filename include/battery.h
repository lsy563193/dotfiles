//
// Created by root on 11/17/17.
//

#ifndef PP_BATTERY_H
#define PP_BATTERY_H

#include <clean_mode.h>
#include <pp/x900sensor.h>

class Battery {
public:
	uint8_t isFull(void);

	uint8_t isReadyToClean(void);

	uint16_t getVoltage()
	{
		return voltage_;
	}

	void setVoltage(uint16_t val)
	{
		voltage_ = val;
	}

private:
		uint16_t voltage_;
};

extern Battery battery;

#endif //PP_BATTERY_H
