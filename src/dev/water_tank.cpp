//
// Created by austin on 18-3-13.
//

#include <battery.h>
#include "water_tank.hpp"
#include "serial.h"

WaterTank water_tank;
WaterTank::WaterTank()
{
	setPumpMode(PUMP_LOW);
	setSwingMotorMode(SWING_MOTOR_HIGH);
}

bool WaterTank::checkEquipment()
{
	if (getStatus(swing_motor))
		return true;

	auto last_tank_mode_ = getSwingMotorEquipmentStatus();
	setSwingMotorMode(SWING_MOTOR_HIGH);
	open(operate_option::swing_motor);//open water tank to detect if it is equipped
	usleep(150000);
	auto status = getSwingMotorEquipmentStatus();
	if(!status)
		stop(operate_option::swing_motor_and_pump);
	setSwingMotorMode(last_tank_mode_);

	ROS_INFO("%s %d: Robot is %scarrying a water tank.", __FUNCTION__, __LINE__, status? "" : "not ");
	return status;
}

void WaterTank::open(int equipment)
{
	switch(equipment){
		case operate_option::swing_motor:
			setWaterTankPWM();
			ROS_ERROR("%s %d: Open water tank", __FUNCTION__, __LINE__);
			break;
		case operate_option::pump:
			pump_pwm_ = 0x80;
			last_pump_time_stamp_ = 0;
			ROS_ERROR("%s %d: Open pump", __FUNCTION__, __LINE__);
			break;
		case operate_option::swing_motor_and_pump:
			setWaterTankPWM();
			pump_pwm_ = 0x80;
			last_pump_time_stamp_ = 0;
			ROS_ERROR("%s %d: Open pump and water tank", __FUNCTION__, __LINE__);
			break;
	}
	serial.setSendData(CTL_WATER_TANK, static_cast<uint8_t>(swing_motor_pwm_|pump_pwm_));
	if (equipment == operate_option::swing_motor_and_pump)
	{
		setStatus(operate_option::swing_motor,true);
		setStatus(operate_option::pump,true);
	}
	setStatus(equipment,true);
}

void WaterTank::stop(int operate_option)
{
	switch(operate_option){
		case swing_motor:
			serial.setSendData(CTL_WATER_TANK, static_cast<uint8_t>(pump_pwm_ & 0x80));
			ROS_INFO("%s %d: close water tank", __FUNCTION__, __LINE__);
			break;
		case pump:
			serial.setSendData(CTL_WATER_TANK, static_cast<uint8_t>(swing_motor_pwm_ & 0x7f));
			ROS_INFO("%s %d: close pump", __FUNCTION__, __LINE__);
			break;
		case swing_motor_and_pump:
			serial.setSendData(CTL_WATER_TANK, 0x00);
			ROS_INFO("%s %d: close pump and water tank", __FUNCTION__, __LINE__);
			break;
	}
	if (operate_option == swing_motor_and_pump) {
		setStatus(swing_motor, false);
		setStatus(pump, false);
	}
	setStatus(operate_option,false);
}

void WaterTank::setWaterTankPWM()
{
	auto current_battery_voltage_ = battery.getVoltage();
	float percentage = static_cast<float>(swing_motor_operate_voltage_) /
										 static_cast<float>(current_battery_voltage_);
	swing_motor_pwm_ = static_cast<uint8_t>(percentage * 100);
	check_battery_time_stamp_ = ros::Time::now().toSec();
}

void WaterTank::updatePWM()
{
	if (getStatus(swing_motor) && (ros::Time::now().toSec() - check_battery_time_stamp_ > 60 || is_swing_motor_mode_change_))
	{
		if(is_swing_motor_mode_change_)
			is_swing_motor_mode_change_ = false;
		setWaterTankPWM();
		serial.setSendData(CTL_WATER_TANK, static_cast<uint8_t>(pump_pwm_ | swing_motor_pwm_));
		ROS_INFO("%s %d: Update for water tank.", __FUNCTION__, __LINE__);
	}

	if (getStatus(pump) && ros::Time::now().toSec() - last_pump_time_stamp_ > pump_time_interval_)
	{
		pump_cnt_++;
		if (pump_cnt_ <= pump_max_cnt_)
		{
			pump_pwm_ = 0x80;
			serial.setSendData(CTL_WATER_TANK, static_cast<uint8_t>(pump_pwm_ | swing_motor_pwm_));
			ROS_INFO("%s %d: Open up pump, cnt: %d.", __FUNCTION__, __LINE__, pump_cnt_);
		}
		else
		{
			pump_pwm_ = 0x00;
			pump_cnt_ = 0;
			last_pump_time_stamp_ = ros::Time::now().toSec();
			serial.setSendData(CTL_WATER_TANK, static_cast<uint8_t>(pump_pwm_ | swing_motor_pwm_));
			ROS_INFO("%s %d: Turn off pump.", __FUNCTION__, __LINE__);
		}
	}
}

void WaterTank::setPumpMode(uint8_t mode)
{
	switch (mode)
	{
		case PUMP_HIGH:
			pump_mode_ = PUMP_HIGH;
			pump_time_interval_ = 8 * (20 * pump_max_cnt_ / 50.0);
			break;
		case PUMP_MID:
			pump_mode_ = PUMP_MID;
			pump_time_interval_ = 12 * (20 * pump_max_cnt_ / 50.0);
			break;
		default: // case PUMP_LOW:
			pump_mode_ = PUMP_LOW;
			pump_time_interval_ = 15 * (20 * pump_max_cnt_ / 50.0);
			break;
	}
}

void WaterTank::setSwingMotorMode(uint8_t mode) {
	is_swing_motor_mode_change_ = true;
	switch(mode){
		case SWING_MOTOR_LOW:
			ROS_INFO("%s, %d: Swing motor mode set to low.",__FUNCTION__,__LINE__);
			swing_motor_operate_voltage_ = LOW_OPERATE_VOLTAGE_FOR_SWING_MOTOR;
			break;
		case SWING_MOTOR_HIGH:
			ROS_INFO("%s, %d: Swing motor mode set to high.",__FUNCTION__,__LINE__);
			swing_motor_operate_voltage_ = FULL_OPERATE_VOLTAGE_FOR_SWING_MOTOR;
			break;
		default:
			ROS_ERROR("%s, %d: setSwingMotorMode error.",__FUNCTION__,__LINE__);
			break;
	}
}

int WaterTank::getSwingMotorMode()
{
	if (swing_motor_operate_voltage_ == LOW_OPERATE_VOLTAGE_FOR_SWING_MOTOR)
		return SWING_MOTOR_LOW;
	return SWING_MOTOR_HIGH;
}

