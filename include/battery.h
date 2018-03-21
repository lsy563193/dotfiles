//
// Created by root on 11/17/17.
//

#ifndef PP_BATTERY_H
#define PP_BATTERY_H

#include "mathematics.h"

class Battery {
public:
	const uint16_t BATTERY_VOL_MAX = 1660;
	const uint16_t BATTERY_VOL_MIN = 1260;
	bool isFull(void);

	bool isReadyToClean(void);

	bool shouldGoHome(void);

	bool isReadyToResumeCleaning(void);

	bool isLow(void);

	uint16_t getVoltage()
	{
		return voltage_;
	}
	uint8_t getPercent();
	void setVoltage(uint16_t val)
	{
		voltage_ = val;
	}

private:
	uint16_t voltage_;
};

extern Battery battery;

#endif //PP_BATTERY_H
