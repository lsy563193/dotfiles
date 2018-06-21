//
// Created by root on 11/17/17.
//

#ifndef PP_BATTERY_H
#define PP_BATTERY_H

#include <cstdint>

class Battery {
public:
	bool isFull();

	void setIsFull(bool is_full)
	{
		is_full_ = is_full;
	}

	uint16_t getVoltage()
	{
//		return 1280;
		return voltage_;
	}

	uint8_t getPercent();

	void setVoltage(uint16_t val);

	void forceUpdate();

private:

	bool isReadyToClean();

	bool shouldGoHome();

	bool isReadyToMove();

	const uint16_t BATTERY_VOL_MAX = 1660;
	const uint16_t BATTERY_VOL_MIN = 1260;

	uint16_t voltage_{0};
	double update_time_stamp_{0};
	bool force_update_{false};
	bool is_full_{false};
};

extern Battery battery;

#endif //PP_BATTERY_H
