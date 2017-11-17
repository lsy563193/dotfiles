//
// Created by root on 11/17/17.
//

#ifndef PP_BATTERY_H
#define PP_BATTERY_H

#include <clean_mode.h>
#include <pp/x900sensor.h>
extern pp::x900sensor sensor;
class Battery {
public:
uint8_t is_full(void)
{
	return (get_voltage() > BATTERY_FULL_VOLTAGE);
}

uint8_t is_ready_to_clean(void)
{
	uint16_t limit;
	if (cm_get() == Clean_Mode_Charging)
	{
		limit = BATTERY_READY_TO_CLEAN_VOLTAGE + 60;
	} else
	{
		limit = BATTERY_READY_TO_CLEAN_VOLTAGE;
	}
	//ROS_INFO("%s %d: Battery limit is %d.", __FUNCTION__, __LINE__, limit);
	// Check if battary is lower than the low battery go home voltage value.
	if (get_voltage() >= limit)
	{
		return 1;
	}
	return 0;
}

uint16_t get_voltage()
{
	return sensor.batv * 10;
}

private:
	uint16_t voltage_;
};

extern Battery battery;

#endif //PP_BATTERY_H
