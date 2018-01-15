//
// Created by root on 11/17/17.
//

#ifndef PP_BATTERY_H
#define PP_BATTERY_H

#include "mathematics.h"

class Battery {
public:
	bool isFull(void);

	bool isReadyToClean(void);

	bool shouldGoHome(void);

	bool isReadyToResumeCleaning(void);

	bool isLow(void);

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
