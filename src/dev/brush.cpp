//
// Created by root on 11/17/17.
//

#include <ros/ros.h>
#include <battery.h>
#include <event_manager.h>
#include "brush.h"
#include "serial.h"

Brush brush;

void Brush::operate()
{
	if (resume_stage_[left] == 0 && resume_stage_[right] == 0 && !is_main_oc_)
	{
		switch (side_brush_status_)
		{
			case brush_slow:
			case brush_normal:
				checkBatterySetSideBrushPWM();
				break;
			case brush_max:
				side_brush_PWM_ = 100;
				break;
			default: // brush_stop
				side_brush_PWM_ = 0;
				break;
		}
	}

	if (!is_main_oc_)
	{
		switch (main_brush_status_)
		{
			case brush_slow:
			case brush_normal:
				checkBatterySetMainBrushPWM();
				break;
			case brush_max:
				main_brush_PWM_ = 100;
				break;
			default: // brush_stop
				main_brush_PWM_ = 0;
				break;
		}
	}

	setPWM(side_brush_PWM_, side_brush_PWM_, main_brush_PWM_);
}

void Brush::slowOperate()
{
	side_brush_status_ = brush_slow;
	if (!block_main_brush_low_operation_)
		main_brush_status_ = brush_slow;
	operate();
	ROS_WARN("%s %d: All brush.", __FUNCTION__, __LINE__);
}

void Brush::normalOperate()
{
	side_brush_status_ = brush_normal;
	main_brush_status_ = brush_normal;
	operate();
	check_battery_time_stamp_ = ros::Time::now().toSec();
	ROS_WARN("%s %d: All brush.", __FUNCTION__, __LINE__);
}

void Brush::fullOperate()
{
	side_brush_status_ = brush_max;
	main_brush_status_ = brush_max;
	operate();
	ROS_WARN("%s %d: All brush.", __FUNCTION__, __LINE__);
}

void Brush::stop()
{
	side_brush_status_ = brush_stop;
	main_brush_status_ = brush_stop;
//	operate();
	side_brush_PWM_ = 0;
	main_brush_PWM_ = 0;
	setPWM(side_brush_PWM_, side_brush_PWM_, main_brush_PWM_);
	resume_count_[left] = 0;
	resume_stage_[left] = 0;
	resume_count_[right] = 0;
	resume_stage_[right] = 0;
	ROS_WARN("%s %d: All brush.", __FUNCTION__, __LINE__);
}

void Brush::stopForMainBrushResume()
{
	side_brush_status_ = brush_stop;
	side_brush_PWM_ = 0;
	main_brush_PWM_ = 0;
	setPWM(side_brush_PWM_, side_brush_PWM_, main_brush_PWM_);
	// Update the check battery time stamp in case it will update PWM during self resume.
	check_battery_time_stamp_ = ros::Time::now().toSec();
	ROS_WARN("%s %d: Main brush.", __FUNCTION__, __LINE__);
}

void Brush::mainBrushResume()
{
	side_brush_PWM_ = 0;
	main_brush_PWM_ = 100;
	setPWM(side_brush_PWM_, side_brush_PWM_, main_brush_PWM_);
	ROS_INFO("%s %d: Main Brush start resuming.", __FUNCTION__, __LINE__);
}

void Brush::updatePWM()
{
	if (ros::Time::now().toSec() - check_battery_time_stamp_ > 60)
	{
		operate();
		check_battery_time_stamp_ = ros::Time::now().toSec();
	}
	// else no need to init.
}

void Brush::checkBatterySetMainBrushPWM()
{
	auto current_battery_voltage_ = battery.getVoltage();
	auto operate_voltage_ = main_brush_status_ == brush_slow ? SLOW_OPERATE_VOLTAGE_FOR_MAIN_BRUSH
														: NORMAL_OPERATE_VOLTAGE_FOR_MAIN_BRUSH;
	float percentage = static_cast<float>(operate_voltage_) /
					   static_cast<float>(current_battery_voltage_);
	main_brush_PWM_ = static_cast<uint8_t>(percentage * 100);
}

void Brush::checkBatterySetSideBrushPWM()
{
	auto current_battery_voltage_ = battery.getVoltage();
	auto operate_voltage_ = side_brush_status_ == brush_slow ? SLOW_OPERATE_VOLTAGE_FOR_SIDE_BRUSH
														: NORMAL_OPERATE_VOLTAGE_FOR_SIDE_BRUSH;
	float percentage = static_cast<float>(operate_voltage_) /
					   static_cast<float>(current_battery_voltage_);
	side_brush_PWM_ = static_cast<uint8_t>(percentage * 100);
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
	if (!brush.isSideBrushOn() || is_main_oc_)
		return false;

	switch (resume_stage_[brush_indicator])
	{
		case 0:
		{
			if (side_brush_oc_status_[brush_indicator])
				resume_stage_[brush_indicator]++;
			else
				break;
		}
		case 1:
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
		case 2:
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
		case 3:
		{
			// Try fully operating side brush for 5s.
			if (brush_indicator == left)
				brush.setLeftBrushPWM(100);
			else
				brush.setRightBrushPWM(100);

			if (ros::Time::now().toSec() - resume_start_time_[brush_indicator] > 1.5 && side_brush_oc_status_[brush_indicator])
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

void Brush::updateSideBrushTime(uint32_t addition_time)
{
	side_brush_operation_time_ += addition_time;
	ROS_INFO("%s %d: Update side brush operation time to %ds(%dh).", __FUNCTION__, __LINE__,
			 side_brush_operation_time_, side_brush_operation_time_ / 3600);
}

void Brush::updateMainBrushTime(uint32_t addition_time)
{
	main_brush_operation_time_ += addition_time;
	ROS_INFO("%s %d: Update main brush operation time to %ds(%dh).", __FUNCTION__, __LINE__,
			 main_brush_operation_time_, main_brush_operation_time_ / 3600);
}

void Brush::resetSideBrushTime()
{
	side_brush_operation_time_ = 0;
	ROS_INFO("%s %d: Reset side brush operation time to 0.", __FUNCTION__, __LINE__);
}

void Brush::resetMainBrushTime()
{
	main_brush_operation_time_ = 0;
	ROS_INFO("%s %d: Reset main brush operation time to 0.", __FUNCTION__, __LINE__);
}

void Brush::blockMainBrushSlowOperation()
{
	block_main_brush_low_operation_ = true;
	ROS_WARN("%s %d: Block main brush from slow operation(For carpet oc problems).", __FUNCTION__, __LINE__);
}
