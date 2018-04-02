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

bool WaterTank::checkEquipment(bool is_stop_water_tank)
{
	if (getStatus(water_tank))
		return true;

	open(water_tank);//open water tank to detect if it is equipped
	usleep(150000);
	auto status = getEquimentStatus();
	if(!status || is_stop_water_tank)
		stop(water_tank);

	ROS_INFO("%s %d: Robot is %scarrying a water tank.", __FUNCTION__, __LINE__, status? "" : "not ");
	return status;
}

void WaterTank::open(int equipment)
{
	switch(equipment){
		case water_tank:
			setWaterTankPWM();
			ROS_ERROR("%s %d: Open water tank", __FUNCTION__, __LINE__);
			break;
		case pump:
			pump_pwm_ = 0x80;
			last_pump_time_stamp_ = 0;
			ROS_ERROR("%s %d: Open pump", __FUNCTION__, __LINE__);
			break;
		case tank_pump:
			setWaterTankPWM();
			pump_pwm_ = 0x80;
			last_pump_time_stamp_ = 0;
			ROS_ERROR("%s %d: Open pump and water tank", __FUNCTION__, __LINE__);
			break;
	}
	serial.setSendData(CTL_WATER_TANK, static_cast<uint8_t>(water_tank_pwm_|pump_pwm_));
	if (equipment == tank_pump)
	{
		setStatus(water_tank,true);
		setStatus(pump,true);
	}
	setStatus(equipment,true);
}

void WaterTank::stop(int equipment)
{
	switch(equipment){
		case water_tank:
			serial.setSendData(CTL_WATER_TANK, static_cast<uint8_t>(pump_pwm_ & 0x80));
			ROS_ERROR("%s %d: close water tank", __FUNCTION__, __LINE__);
			break;
		case pump:
			serial.setSendData(CTL_WATER_TANK, static_cast<uint8_t>(water_tank_pwm_ & 0x7f));
			ROS_ERROR("%s %d: close pump", __FUNCTION__, __LINE__);
			break;
		case tank_pump:
			serial.setSendData(CTL_WATER_TANK, 0x00);
			ROS_ERROR("%s %d: close pump and water tank", __FUNCTION__, __LINE__);
			break;
	}
	if (equipment == tank_pump) {
		setStatus(water_tank, false);
		setStatus(pump, false);
	}
	setStatus(equipment,false);
}

void WaterTank::setWaterTankPWM()
{
	auto current_battery_voltage_ = battery.getVoltage();
	float percentage = static_cast<float>(FULL_OPERATE_VOLTAGE_FOR_WATER_TANK) /
										 static_cast<float>(current_battery_voltage_);
	water_tank_pwm_ = static_cast<uint8_t>(percentage * 100);
	check_battery_time_stamp_ = ros::Time::now().toSec();
}

void WaterTank::updatePWM()
{
	if (getStatus(water_tank) && ros::Time::now().toSec() - check_battery_time_stamp_ > 60)
	{
		setWaterTankPWM();
		serial.setSendData(CTL_WATER_TANK, static_cast<uint8_t>(pump_pwm_ | water_tank_pwm_));
		ROS_INFO("%s %d: Update for water tank.", __FUNCTION__, __LINE__);
	}

	if (getStatus(pump) && ros::Time::now().toSec() - last_pump_time_stamp_ > pump_time_interval_)
	{
		pump_cnt_++;
		if (pump_cnt_ <= pump_max_cnt_)
		{
			pump_pwm_ = 0x80;
			serial.setSendData(CTL_WATER_TANK, static_cast<uint8_t>(pump_pwm_ | water_tank_pwm_));
			ROS_INFO("%s %d: Open up pump, cnt: %d.", __FUNCTION__, __LINE__, pump_cnt_);
		}
		else
		{
			pump_pwm_ = 0x00;
			pump_cnt_ = 0;
			last_pump_time_stamp_ = ros::Time::now().toSec();
			serial.setSendData(CTL_WATER_TANK, static_cast<uint8_t>(pump_pwm_ | water_tank_pwm_));
			ROS_INFO("%s %d: Turn off pump.", __FUNCTION__, __LINE__);
		}
	}
}

void WaterTank::setMode(uint8_t mode)
{
	switch (mode)
	{
		case PUMP_HIGH:
			mode_ = PUMP_HIGH;
			pump_time_interval_ = 8 * (20 * pump_max_cnt_ / 50.0);
			break;
		case PUMP_MID:
			mode_ = PUMP_MID;
			pump_time_interval_ = 12 * (20 * pump_max_cnt_ / 50.0);
			break;
		default: // case PUMP_LOW:
			mode_ = PUMP_LOW;
			pump_time_interval_ = 15 * (20 * pump_max_cnt_ / 50.0);
			break;
	}
}

