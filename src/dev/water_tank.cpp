//
// Created by austin on 18-3-13.
//

#include <battery.h>
#include "water_tank.hpp"
#include "serial.h"

WaterTank water_tank;

bool WaterTank::checkEquipment()
{
	normalOperate();
	usleep(150000);
	if (getStatus())
		is_equipped_ = true;

//	printf("watertank%d\n", is_equipped_);
	ROS_INFO("%s %d: Robot is %scarrying a water tank.", __FUNCTION__, __LINE__, is_equipped_ ? "" : "not ");

	return is_equipped_;
}

void WaterTank::normalOperate()
{
	operation_ = true;
	checkBatterySetPWM();
	serial.setSendData(CTL_WATER_TANK, static_cast<uint8_t>(0x80 | pwm_));
	check_battery_time_stamp_ = ros::Time::now().toSec();
}

void WaterTank::stop()
{
	operation_ = false;
	serial.setSendData(CTL_WATER_TANK, 0x00);
}

void WaterTank::checkBatterySetPWM()
{
	auto current_battery_voltage_ = battery.getVoltage();
	float percentage = static_cast<float>(FULL_OPERATE_VOLTAGE_FOR_WATER_TANK) /
					   static_cast<float>(current_battery_voltage_);
	pwm_ = static_cast<uint8_t>(percentage * 100);
}

void WaterTank::updatePWM()
{
	if (operation_ && ros::Time::now().toSec() - check_battery_time_stamp_ > 60)
	{
		checkBatterySetPWM();
		serial.setSendData(CTL_WATER_TANK, static_cast<uint8_t>(0x80 | pwm_));
		check_battery_time_stamp_ = ros::Time::now().toSec();
	}
	// else no need to init.
}
