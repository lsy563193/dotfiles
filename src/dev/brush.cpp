//
// Created by root on 11/17/17.
//

#include <ros/ros.h>
#include <battery.h>
#include <event_manager.h>
#include "brush.h"
#include "serial.h"

Brush brush;

Brush::Brush(void)
{
	brush_status_ = brush_stop;

	is_main_oc_ = false;

	check_battery_time_stamp_ = 0;
}

void Brush::operate()
{
	if (brush_status_ != brush_stop && (side_brush_oc_status_[left] || side_brush_oc_status_[right] || is_main_oc_))
		return;

	switch (brush_status_)
	{
		case brush_slow:
		case brush_normal:
			checkBatterySetPWM();
			setPWM(normal_PWM, normal_PWM, normal_PWM);
			break;
		case brush_max:
			setPWM(100, 100, 100);
			break;
		default: // brush_stop
			setPWM(0, 0, 0);
			break;
	}
}

void Brush::slowOperate(void)
{
	brush_status_ = brush_slow;
	operate();
	ROS_INFO("%s %d: Brush set to slow.", __FUNCTION__, __LINE__);
}

void Brush::normalOperate(void)
{
	brush_status_ = brush_normal;
	operate();
	check_battery_time_stamp_ = ros::Time::now().toSec();
	ROS_INFO("%s %d: Brush set to normal.", __FUNCTION__, __LINE__);
}

void Brush::fullOperate()
{
	brush_status_ = brush_max;
	operate();
	ROS_INFO("%s %d: Brush set to max.", __FUNCTION__, __LINE__);
}

void Brush::stop(void)
{
	brush_status_ = brush_stop;
	operate();
	resume_count_[left] = 0;
	resume_count_[right] = 0;
	ROS_INFO("%s %d: Brush set to stop.", __FUNCTION__, __LINE__);
}

void Brush::mainBrushResume()
{
	brush_status_ = brush_normal;
	checkBatterySetPWM();
	setPWM(0, 0, normal_PWM);
	ROS_INFO("%s %d: Main Brush set to normal.", __FUNCTION__, __LINE__);
}

void Brush::updatePWM()
{
	if ((brush_status_ == brush_normal || brush_status_ == brush_slow)
		&& ros::Time::now().toSec() - check_battery_time_stamp_ > 60)
	{
		operate();
		check_battery_time_stamp_ = ros::Time::now().toSec();
	}
	// else no need to init.
}

void Brush::checkBatterySetPWM()
{
	auto current_battery_voltage_ = battery.getVoltage();
	auto operate_voltage_ = brush_status_ == brush_slow ? SLOW_OPERATE_VOLTAGE_FOR_BRUSH : FULL_OPERATE_VOLTAGE_FOR_BRUSH;
	float percentage = static_cast<float>(operate_voltage_) /
					   static_cast<float>(current_battery_voltage_);
	normal_PWM = static_cast<uint8_t>(percentage * 100);
}

void Brush::setLeftBrushPWM(uint8_t PWM)
{
	PWM = PWM < 100 ? PWM : 100;
	serial.setSendData(CTL_BRUSH_LEFT, PWM & 0xff);
}

void Brush::setRightBrushPWM(uint8_t PWM)
{
	PWM = PWM < 100 ? PWM : 100;
	serial.setSendData(CTL_BRUSH_RIGHT, PWM & 0xff);
}

void Brush::setMainBrushPWM(uint8_t PWM)
{
	// Set main brush PWM, the value of PWM should be in range (0, 100).
	PWM = PWM < 100 ? PWM : 100;
	serial.setSendData(CTL_BRUSH_MAIN, PWM & 0xff);
}

void Brush::setPWM(uint8_t L, uint8_t R, uint8_t M)
{
	setLeftBrushPWM(L);
	setRightBrushPWM(R);
	setMainBrushPWM(M);
}

bool Brush::checkLeftBrushTwined()
{
	return checkBrushTwined(left);
}

bool Brush::checkRightBrushTwined()
{
	return checkBrushTwined(right);
}

bool Brush::checkBrushTwined(uint8_t brush_indicator)
{
	if (!brush.isOn())
		return false;

	switch (resume_stage_[brush_indicator])
	{
		case 0:
		{
			if (side_brush_oc_status_[brush_indicator])
			{
				ROS_WARN("%s %d: Detect %s brush over current, stop for 5s.", __FUNCTION__, __LINE__,
						 brush_indicator == left ? "left" : "right");
				resume_stage_[brush_indicator]++;
				resume_start_time_[brush_indicator] = ros::Time::now().toSec();
			}
			break;
		}
		case 1:
		{
			if (brush_indicator == left)
				brush.setLeftBrushPWM(0);
			else
				brush.setRightBrushPWM(0);
			// Stop side brush for 5s.
			if (ros::Time::now().toSec() - resume_start_time_[brush_indicator] > 5)
			{
				resume_start_time_[brush_indicator] = ros::Time::now().toSec();
				resume_stage_[brush_indicator]++;
				ROS_WARN("%s %d: Fully operate %s brush for 5s.", __FUNCTION__, __LINE__,
						 brush_indicator == left ? "left" : "right");
			}
			break;
		}
		case 2:
		{
			// Try fully operating side brush for 5s.
			if (brush_indicator == left)
				brush.setLeftBrushPWM(100);
			else
				brush.setRightBrushPWM(100);

			if (ros::Time::now().toSec() - resume_start_time_[brush_indicator] > 1 && side_brush_oc_status_[brush_indicator])
			{
				if (++resume_count_[brush_indicator] > 3)
				{
					ROS_WARN("%s %d: %s brush resume failed.", __FUNCTION__, __LINE__,
							 brush_indicator == left ? "Left" : "Right");
					resume_count_[brush_indicator] = 0;
					resume_stage_[brush_indicator] = 0;
					return true;
				}
				else
					resume_stage_[brush_indicator] = 0;
			}
			if (ros::Time::now().toSec() - resume_start_time_[brush_indicator] > 5)
			{
				resume_count_[brush_indicator] = 0;
				resume_stage_[brush_indicator] = 0;
				operate();
				ROS_WARN("%s %d: %s brush resume succeeded.", __FUNCTION__, __LINE__,
						 brush_indicator == left ? "Left" : "Right");
			}
			break;
		}
		default:
			resume_stage_[brush_indicator] = 0;
			break;
	}
	return false;
}
