//
// Created by austin on 18-3-13.
//

#include <battery.h>
#include "water_tank.hpp"
#include "serial.h"

WaterTank water_tank;

WaterTank::WaterTank()
{
	setMode(PUMP_LOW);
}

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
	pump_switch_ = 0x80;
	last_pump_time_stamp_ = 0;
	serial.setSendData(CTL_WATER_TANK, static_cast<uint8_t>(pump_switch_ | pwm_));
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
	if (operation_ && status_ && ros::Time::now().toSec() - check_battery_time_stamp_ > 60)
	{
		checkBatterySetPWM();
		serial.setSendData(CTL_WATER_TANK, static_cast<uint8_t>(pump_switch_ | pwm_));
		check_battery_time_stamp_ = ros::Time::now().toSec();
	}

	if (operation_ && status_ && ros::Time::now().toSec() - last_pump_time_stamp_ > pump_time_interval_)
	{
		pump_switch_ = 0x80;
		pump_cnt_++;
		if (pump_cnt_ <= pump_max_cnt_)
		{
			serial.setSendData(CTL_WATER_TANK, static_cast<uint8_t>(pump_switch_ | pwm_));
			ROS_INFO("%s %d: Open up pump, cnt: %d.", __FUNCTION__, __LINE__, pump_cnt_);
		}
		else
		{
			pump_switch_ = 0x00;
			pump_cnt_ = 0;
			last_pump_time_stamp_ = ros::Time::now().toSec();
			serial.setSendData(CTL_WATER_TANK, static_cast<uint8_t>(pump_switch_ | pwm_));
			ROS_INFO("%s %d: Turn off pump.", __FUNCTION__, __LINE__);
		}
	}
}

void WaterTank::setMode(uint8_t mode)
{
	switch (mode)
	{
		case PUMP_HIGH:
			pump_time_interval_ = 8 * (20 * pump_max_cnt_ / 50.0);
			break;
		case PUMP_MID:
			pump_time_interval_ = 12 * (20 * pump_max_cnt_ / 50.0);
			break;
		default: // case PUMP_LOW:
			pump_time_interval_ = 15 * (20 * pump_max_cnt_ / 50.0);
			break;
	}
}

