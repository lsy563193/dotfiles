//
// Created by root on 11/17/17.
//

#include "pp.h"
#include "battery.h"

Battery battery;

uint8_t Battery::isFull(void)
{
	return (getVoltage() > BATTERY_FULL_VOLTAGE);
}

uint8_t Battery::isReadyToClean(void)
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
	if (getVoltage() >= limit)
	{
		return 1;
	}
	return 0;
}

