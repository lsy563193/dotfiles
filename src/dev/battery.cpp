//
// Created by root on 11/17/17.
//

#include "ros/ros.h"
#include "battery.h"
#include "config.h"

Battery battery;

bool Battery::isFull()
{
	return (getVoltage() >= BATTERY_FULL_VOLTAGE);
}

bool Battery::isReadyToClean()
{
	// Check if battary is lower than the low battery go home voltage value.
	return (getVoltage() >= BATTERY_READY_TO_CLEAN_VOLTAGE);
}

bool Battery::isLow()
{
	return (getVoltage() <= LOW_BATTERY_STOP_VOLTAGE);
}

bool Battery::isReadyToResumeCleaning()
{
	// Check if battary is lower than the low battery go home voltage value.
	return (getVoltage() >= RESUME_CLEANING_VOLTAGE);
}

bool Battery::shouldGoHome()
{
	// Check if battary is lower than the low battery go home voltage value.
	return (getVoltage() < LOW_BATTERY_GO_HOME_VOLTAGE);
}

uint8_t Battery::getPercent()
{
	if(getVoltage() >= BATTERY_VOL_MIN && getVoltage() <= BATTERY_VOL_MAX)
	  return (uint8_t)((getVoltage() - BATTERY_VOL_MIN) *100 /(BATTERY_VOL_MAX - BATTERY_VOL_MIN));
}

void Battery::setVoltage(uint16_t val)
{
	double time_now = ros::Time::now().toSec();
	if (time_now - update_time_stamp_ > 30 || force_update_ || voltage_ == 0)
	{
		voltage_ = val;
		update_time_stamp_ = time_now;
		if (force_update_)
			force_update_ = false;
		ROS_WARN("%s %d: Update Battery as %.1fv.", __FUNCTION__, __LINE__,
				 static_cast<float>(battery.getVoltage() / 100.0));
	}
}

