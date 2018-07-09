//
// Created by austin on 18-3-13.
//

#include <battery.h>
#include "water_tank.hpp"
#include "serial.h"
#include "ros/ros.h"

WaterTank water_tank;
WaterTank::WaterTank()
{
	setCurrentPumpMode(PUMP_LOW);
	setCurrentSwingMotorMode(SWING_MOTOR_HIGH);
}

bool WaterTank::checkEquipment()
{
	if (getStatus(swing_motor))
		return true;

	auto saved_swing_motor_mode = getCurrentSwingMotorMode();
	setCurrentSwingMotorMode(SWING_MOTOR_HIGH);
	open(operate_option::swing_motor);//open water tank to detect if it is equipped
	usleep(150000);
	auto status = getSwingMotorEquipmentStatus();
	if(!status)
		stop(operate_option::swing_motor_and_pump);
	setCurrentSwingMotorMode(saved_swing_motor_mode);

	ROS_INFO("%s %d: Robot is %scarrying a water tank.", __FUNCTION__, __LINE__, status? "" : "not ");
	return status;
}

void WaterTank::open(int _operate_option)
{
	switch(_operate_option){
		case operate_option::swing_motor:
			setSwingMotorPWM();
			ROS_WARN("%s %d: Swing motor.", __FUNCTION__, __LINE__);
			break;
		case operate_option::pump:
			pump_pwm_ = 0x80;
			last_pump_time_stamp_ = 0;
			ROS_WARN("%s %d: Pump.", __FUNCTION__, __LINE__);
			break;
		case operate_option::swing_motor_and_pump:
			setSwingMotorPWM();
			pump_pwm_ = 0x80;
			last_pump_time_stamp_ = 0;
			ROS_WARN("%s %d: Swing motor and pump.", __FUNCTION__, __LINE__);
			break;
	}
	serial.setSendData(CTL_WATER_TANK, static_cast<uint8_t>(swing_motor_pwm_|pump_pwm_));
	if (_operate_option == operate_option::swing_motor_and_pump)
	{
		setStatus(operate_option::swing_motor,true);
		setStatus(operate_option::pump,true);
	}
	else
		setStatus(_operate_option,true);
}

void WaterTank::stop(int operate_option)
{
	switch(operate_option){
		case swing_motor:
			serial.setSendData(CTL_WATER_TANK, static_cast<uint8_t>(pump_pwm_ & 0x80));
			swing_motor_pwm_ = 0;
			ROS_WARN("%s %d: Swing motor.", __FUNCTION__, __LINE__);
			break;
		case pump:
			serial.setSendData(CTL_WATER_TANK, static_cast<uint8_t>(swing_motor_pwm_ & 0x7f));
			pump_pwm_ = 0;
			ROS_WARN("%s %d: Pump.", __FUNCTION__, __LINE__);
			break;
		case swing_motor_and_pump:
			serial.setSendData(CTL_WATER_TANK, 0x00);
			pump_pwm_ = 0;
			swing_motor_pwm_ = 0;
			ROS_WARN("%s %d: Swing motor and pump.", __FUNCTION__, __LINE__);
			break;
	}
	if (operate_option == swing_motor_and_pump) {
		setStatus(swing_motor, false);
		setStatus(pump, false);
	}
	setStatus(operate_option,false);
}

void WaterTank::setSwingMotorPWM()
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
		setSwingMotorPWM();
		serial.setSendData(CTL_WATER_TANK, static_cast<uint8_t>(pump_pwm_ | swing_motor_pwm_));
		ROS_INFO("%s %d: Update for swing motor.", __FUNCTION__, __LINE__);
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

void WaterTank::setCurrentPumpMode(int mode)
{
	switch (mode)
	{
		case PUMP_HIGH:
			ROS_INFO("%s, %d: High.",__FUNCTION__,__LINE__);
			current_pump_mode_ = PUMP_HIGH;
			pump_time_interval_ = 8 * (20 * pump_max_cnt_ / 50.0);
			break;
		case PUMP_MIDDLE:
			ROS_INFO("%s, %d: Middle.",__FUNCTION__,__LINE__);
			current_pump_mode_ = PUMP_MIDDLE;
			pump_time_interval_ = 12 * (20 * pump_max_cnt_ / 50.0);
			break;
		default: // case PUMP_LOW:
			ROS_INFO("%s, %d: Low.",__FUNCTION__,__LINE__);
			current_pump_mode_ = PUMP_LOW;
			pump_time_interval_ = 15 * (20 * pump_max_cnt_ / 50.0);
			break;
	}
}

void WaterTank::setUserSetPumpMode(int mode)
{
	switch (mode)
	{
		case PUMP_HIGH:
			ROS_INFO("%s, %d: High.",__FUNCTION__,__LINE__);
			user_set_pump_mode_ = PUMP_HIGH;
			break;
		case PUMP_MIDDLE:
			ROS_INFO("%s, %d: Middle.",__FUNCTION__,__LINE__);
			user_set_pump_mode_ = PUMP_MIDDLE;
			break;
		default: // case PUMP_LOW:
			ROS_INFO("%s, %d: Low.",__FUNCTION__,__LINE__);
			user_set_pump_mode_ = PUMP_LOW;
			break;
	}
}

void WaterTank::setCurrentSwingMotorMode(int mode) {
	is_swing_motor_mode_change_ = true;
	switch(mode){
		case SWING_MOTOR_LOW:
			ROS_INFO("%s, %d: Low.",__FUNCTION__,__LINE__);
			current_swing_motor_mode_ = mode;
			swing_motor_operate_voltage_ = LOW_OPERATE_VOLTAGE_FOR_SWING_MOTOR;
			break;
		case SWING_MOTOR_HIGH:
			ROS_INFO("%s, %d: High.",__FUNCTION__,__LINE__);
			current_swing_motor_mode_ = mode;
			swing_motor_operate_voltage_ = FULL_OPERATE_VOLTAGE_FOR_SWING_MOTOR;
			break;
		default:
			ROS_ERROR("%s, %d: Error.",__FUNCTION__,__LINE__);
			break;
	}
}

void WaterTank::setUserSwingMotorMode(int mode)
{
	switch(mode){
		case SWING_MOTOR_LOW:
			ROS_INFO("%s, %d: Low.",__FUNCTION__,__LINE__);
			user_set_swing_motor_mode_ = mode;
			break;
		case SWING_MOTOR_HIGH:
			ROS_INFO("%s, %d: High.",__FUNCTION__,__LINE__);
			user_set_swing_motor_mode_ = mode;
			break;
		default:
			ROS_ERROR("%s, %d: Error.",__FUNCTION__,__LINE__);
			break;
	}
}

void WaterTank::setStatus(int _operate_option, bool status)
{
	if (_operate_option == swing_motor)
		swing_motor_switch_ = status;
	else if (_operate_option == pump)
		pump_switch_ = status;
}

bool WaterTank::getStatus(int _operate_option)
{
	if (_operate_option == pump)
		return pump_switch_;
	else
		return swing_motor_switch_;
}

void WaterTank::slowOperateSwingMotor()
{
	setCurrentSwingMotorMode(swing_motor_mode::SWING_MOTOR_HIGH);
	open(operate_option::swing_motor);
}

void WaterTank::fullOperateSwingMotor()
{
	setCurrentSwingMotorMode(swing_motor_mode::SWING_MOTOR_HIGH);
	open(operate_option::swing_motor);
}

void WaterTank::slowOperatePump()
{
}

void WaterTank::fullOperatePump()
{
}


