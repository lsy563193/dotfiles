//
// Created by root on 11/17/17.
//

#include <charger.h>
#include "ros/ros.h"
#include "battery.h"
#include "config.h"
#include "log.h"
#include <robot.hpp>

Battery battery;

bool Battery::isFull()
{
	return is_full_;
}

bool Battery::isReadyToClean()
{
	// Check if battary is lower than the low battery go home voltage value.
	return (getVoltage() >= BATTERY_READY_TO_CLEAN_VOLTAGE);
//	return (getVoltage() >= 1600);
}

bool Battery::isReadyToMove()
{
	return (getVoltage() >= LOW_BATTERY_STOP_VOLTAGE);
//	return (getVoltage() >= 1490);
}

bool Battery::shouldGoHome()
{
	// Check if battary is lower than the low battery go home voltage value.
	return (getVoltage() < LOW_BATTERY_GO_HOME_VOLTAGE);
//	return (getVoltage() < 1540);
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

		if (!robot::instance()->batteryTooLowToClean() && !isReadyToClean())
		{
			ROS_WARN("%s %d: Battery too low to clean.", __FUNCTION__, __LINE__);
			robot::instance()->setBatteryTooLowToClean(true);
		}
		else if (!robot::instance()->batteryTooLowToMove() && !isReadyToMove())
		{
			ROS_WARN("%s %d: Battery too low to move.", __FUNCTION__, __LINE__);
			robot::instance()->setBatteryTooLowToMove(true);
		}
		else if (!robot::instance()->batteryLowForGoingHome() && shouldGoHome())
		{
			ROS_WARN("%s %d: Battery low for going home.", __FUNCTION__, __LINE__);
			robot::instance()->setBatteryLowForGoingHome(true);
		}

		float bv = static_cast<float>(voltage_ / 100.0);
		ROS_WARN("%s %d: Battery %.1fv.", __FUNCTION__, __LINE__, bv);
	}
}

void Battery::forceUpdate()
{
	force_update_ = true;
	// Sleep for 20ms for robot updating battery voltage.
	usleep(20000);
}

void Battery::updateForLowBattery()
{
	if (robot::instance()->batteryTooLowToClean() && isReadyToClean())
	{
		ROS_WARN("%s %d: Battery ready to clean.", __FUNCTION__, __LINE__);
		robot::instance()->setBatteryTooLowToClean(false);
	} else if (robot::instance()->batteryTooLowToMove() && isReadyToMove())
	{
		ROS_WARN("%s %d: Battery ready to move.", __FUNCTION__, __LINE__);
		robot::instance()->setBatteryTooLowToMove(false);
	} else if (robot::instance()->batteryLowForGoingHome() && !shouldGoHome())
	{
		ROS_WARN("%s %d: Battery up and no need to go home.", __FUNCTION__, __LINE__);
		robot::instance()->setBatteryLowForGoingHome(false);
	}
}

