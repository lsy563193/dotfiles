//
// Created by root on 11/17/17.
//

#include <ros/ros.h>
#include <battery.h>
#include "brush.h"
#include "serial.h"

Brush brush;

Brush::Brush(void)
{
	brush_status_ = brush_stop;

	is_left_oc_ = false;
	is_right_oc_ = false;
	is_main_oc_ = false;

	oc_left_cnt_ = 0;
	oc_main_cnt_ = 0;
	oc_right_cnt_ = 0;

	check_battery_time_stamp_ = 0;
}

void Brush::slowOperate(void)
{
	brush_status_ = brush_slow;
	setPWM(20, 20, 20);
	check_battery_time_stamp_ = ros::Time::now().toSec();
	ROS_INFO("%s %d: Brush set to slow.", __FUNCTION__, __LINE__);
}

void Brush::normalOperate(void)
{
	brush_status_ = brush_normal;
	checkBatterySetPWM();
	setPWM(normal_PWM, normal_PWM, normal_PWM);
	check_battery_time_stamp_ = ros::Time::now().toSec();
	ROS_INFO("%s %d: Brush set to normal.", __FUNCTION__, __LINE__);
}

void Brush::fullOperate()
{
	brush_status_ = brush_max;
	setPWM(100, 100, 100);
	check_battery_time_stamp_ = ros::Time::now().toSec();
	ROS_INFO("%s %d: Brush set to max.", __FUNCTION__, __LINE__);
}

void Brush::stop(void)
{
	brush_status_ = brush_stop;
	setPWM(0, 0, 0);
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
	if (brush_status_ == brush_normal && ros::Time::now().toSec() - check_battery_time_stamp_ > 60)
	{
		checkBatterySetPWM();
		setPWM(normal_PWM, normal_PWM, normal_PWM);
		check_battery_time_stamp_ = ros::Time::now().toSec();
	}
	// else no need to init.
}

void Brush::checkBatterySetPWM()
{
	auto current_battery_voltage_ = battery.getVoltage();
	float percentage = static_cast<float>(FULL_OPERATE_VOLTAGE_FOR_BRUSH) /
					   static_cast<float>(current_battery_voltage_);
	normal_PWM = static_cast<uint8_t>(percentage * 100);
}

void Brush::setPWM(uint8_t L, uint8_t R, uint8_t M)
{
	// Set left and right brush PWM, the value of L/R should be in range (0, 100).
	L = L < 100 ? L : 100;
	serial.setSendData(CTL_BRUSH_LEFT, L & 0xff);
	R = R < 100 ? R : 100;
	serial.setSendData(CTL_BRUSH_RIGHT, R & 0xff);

	// Set main brush PWM, the value of PWM should be in range (0, 100).
	M = M < 100 ? M : 100;
	serial.setSendData(CTL_BRUSH_MAIN, M & 0xff);
}

uint8_t Brush::leftIsStall(void)
{
	return 0;
/*
	static time_t time_left_brush;
	static uint8_t left_error_counter = 0;
	static uint8_t left_status = 1;
	*//*---------------------------------Left Brush Stall---------------------------------*//*
	if (reset_left_oc_) {
		left_error_counter = 0;
		left_status = 1;
		reset_left_oc_ = false;
		//ROS_WARN("%s %d: Reset left brush.", __FUNCTION__, __LINE__);
		return 0;
	}

	switch (left_status) {
		case 1: {
			if (getLeftOc()) {
				if (oc_left_cnt_ < 200)
					oc_left_cnt_++;
			}
			else
				oc_left_cnt_ = 0;

			if (oc_left_cnt_ > 10) {
				*//*-----Left Brush is stall, stop the brush-----*//*
				setPWM(0, right_pwm_, main_pwm_);
				left_status = 2;
				time_left_brush = time(NULL);
				ROS_WARN("%s %d: Stop the brush for 5s.", __FUNCTION__, __LINE__);
			}
			break;
		}
		case 2: {
			*//*-----brush should stop for 5s-----*//*
			if ((time(NULL) - time_left_brush) >= 5) {
				// Then restart brush and let it fully operated.
				setPWM(100, right_pwm_, main_pwm_);
				left_status = 3;
				time_left_brush = time(NULL);
				ROS_WARN("%s %d: Fully operate the brush for 5s.", __FUNCTION__, __LINE__);
			}
			break;
		}

		case 3: {
			if (getLeftOc()) {
				if (oc_left_cnt_ < 200)
					oc_left_cnt_++;
			}
			else {
				oc_left_cnt_ = 0;
			}

			if (oc_left_cnt_ > 10) {
				*//*-----Brush is still stall, stop the brush and increase error counter -----*//*
				setPWM(0, right_pwm_, main_pwm_);
				left_status = 2;
				time_left_brush = time(NULL);
				left_error_counter++;
				if (left_error_counter > 2) {
					left_status = 1;
					oc_left_cnt_ = 0;
					left_error_counter = 0;
					return 1;
				}
				break;
			}
			else {
				ROS_DEBUG("%s %d: Fully operate the brush for 5s.", __FUNCTION__, __LINE__);
				if ((time(NULL) - time_left_brush) >= 5) {
					ROS_WARN("%s %d: Restore from fully operated.", __FUNCTION__, __LINE__);
					*//*-----brush is in max is_max_clean_state_ more than 5s, turn to normal is_max_clean_state_ and reset error counter-----*//*
					setPWM(left_pwm_, right_pwm_, main_pwm_);
					left_status = 1;
					oc_left_cnt_ = 0;
					left_error_counter = 0;
				}
			}
			break;
		}
	}
	return 0;*/
}

uint8_t Brush::rightIsStall(void)
{
	return 0;
/*	static time_t time_right_brush;
	static uint8_t right_error_counter = 0;
	static uint8_t right_status = 1;
	*//*---------------------------------Right Brush Stall---------------------------------*//*

	if (reset_right_oc_) {
		right_error_counter = 0;
		right_status = 1;
		reset_right_oc_ = false;
		//ROS_WARN("%s %d: Reset right brush.", __FUNCTION__, __LINE__);
		return 0;
	}

	switch (right_status) {
		case 1: {
			if (getRightOc()) {
				if (oc_right_cnt_ < 200)
					oc_right_cnt_++;
			}
			else
				oc_right_cnt_ = 0;

			if (oc_right_cnt_ > 10) {
				*//*-----Right Brush is stall, stop the brush-----*//*
				setPWM(left_pwm_, 0, main_pwm_);
				right_status = 2;
				time_right_brush = time(NULL);
				ROS_WARN("%s %d: Stop the brush for 5s.", __FUNCTION__, __LINE__);
			}
			break;
		}
		case 2: {
			*//*-----brush should stop for 5s-----*//*
			if ((time(NULL) - time_right_brush) >= 5) {
				// Then restart brush and let it fully operated.
				setPWM(left_pwm_, 100, main_pwm_);
				right_status = 3;
				time_right_brush = time(NULL);
				ROS_WARN("%s %d: Fully operate the brush for 5s.", __FUNCTION__, __LINE__);
			}
			break;
		}

		case 3: {
			if (getRightOc()) {
				if (oc_right_cnt_ < 200)
					oc_right_cnt_++;
			}
			else {
				oc_right_cnt_ = 0;
			}

			if (oc_right_cnt_ > 10) {
				*//*-----Brush is still stall, stop the brush and increase error counter -----*//*
				setPWM(left_pwm_, 0, main_pwm_);
				right_status = 2;
				time_right_brush = time(NULL);
				right_error_counter++;
				if (right_error_counter > 2) {
					right_status = 1;
					oc_right_cnt_ = 0;
					right_error_counter = 0;
					return 1;
				}
				break;
			}
			else {
				ROS_DEBUG("%s %d: Fully operate the brush for 5s.", __FUNCTION__, __LINE__);
				if ((time(NULL) - time_right_brush) >= 5) {
					ROS_WARN("%s %d: Restore from fully operated.", __FUNCTION__, __LINE__);
					*//*-----brush is in max is_max_clean_state_ more than 5s, turn to normal is_max_clean_state_ and reset error counter-----*//*
					setPWM(left_pwm_, right_pwm_, main_pwm_);
					right_status = 1;
					oc_right_cnt_ = 0;
					right_error_counter = 0;
				}
			}
			break;
		}
	}
	return 0;*/
}

