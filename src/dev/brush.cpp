//
// Created by root on 11/17/17.
//

#include "pp.h"
#include "brush.h"
#include "serial.h"

Brush brush;

Brush::Brush(void)
{
	left_pwm_ = 0;
	right_pwm_ = 0;
	main_pwm_ = 0;

	reset_left_oc_ = false;
	reset_right_oc_ = false;
	reset_main_oc_ = false;

	is_left_oc_ = false;
	is_right_oc_ = false;
	is_main_oc_ = false;

	oc_left_cnt_ = 0;
	oc_main_cnt_ = 0;
	oc_right_cnt_ = 0;
}

void Brush::stop(void)
{
	setSidePwm(0, 0);
	setMainPwm(0);
}

void Brush::setMainPwm(uint16_t PWM) {
	// Set main brush PWM, the value of PWM should be in range (0, 100).
	PWM = PWM < 100 ? PWM : 100;
	serial.setSendData(CTL_BRUSH_MAIN, PWM & 0xff);
}

void Brush::setSidePwm(uint16_t L, uint16_t R) {
	// Set left and right brush PWM, the value of L/R should be in range (0, 100).
	L = L < 100 ? L : 100;
	left_pwm_ = L;
	serial.setSendData(CTL_BRUSH_LEFT, L & 0xff);
	R = R < 100 ? R : 100;
	right_pwm_ = R;
	serial.setSendData(CTL_BRUSH_RIGHT, R & 0xff);
}

void Brush::setLeftPwm(uint16_t L) {
	L = L < 100 ? L : 100;
	serial.setSendData(CTL_BRUSH_LEFT, L & 0xff);
}

void Brush::setRightPwm(uint16_t R) {
	R = R < 100 ? R : 100;
	serial.setSendData(CTL_BRUSH_RIGHT, R & 0xff);
}

uint8_t Brush::leftIsStall(void) {
	static time_t time_left_brush;
	static uint8_t left_error_counter = 0;
	static uint8_t left_status = 1;
	/*---------------------------------Left Brush Stall---------------------------------*/
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
				/*-----Left Brush is stall, stop the brush-----*/
				setLeftPwm(0);
				left_status = 2;
				time_left_brush = time(NULL);
				ROS_WARN("%s %d: Stop the brush for 5s.", __FUNCTION__, __LINE__);
			}
			break;
		}
		case 2: {
			/*-----brush should stop for 5s-----*/
			if ((time(NULL) - time_left_brush) >= 5) {
				// Then restart brush and let it fully operated.
				setLeftPwm(100);
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
				/*-----Brush is still stall, stop the brush and increase error counter -----*/
				setLeftPwm(0);
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
					/*-----brush is in max mode_ more than 5s, turn to normal mode_ and reset error counter-----*/
					setLeftPwm(left_pwm_);
					left_status = 1;
					oc_left_cnt_ = 0;
					left_error_counter = 0;
				}
			}
			break;
		}
	}
	return 0;
}

uint8_t Brush::rightIsStall(void) {
	static time_t time_right_brush;
	static uint8_t right_error_counter = 0;
	static uint8_t right_status = 1;
	/*---------------------------------Right Brush Stall---------------------------------*/

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
				/*-----Right Brush is stall, stop the brush-----*/
				setRightPwm(0);
				right_status = 2;
				time_right_brush = time(NULL);
				ROS_WARN("%s %d: Stop the brush for 5s.", __FUNCTION__, __LINE__);
			}
			break;
		}
		case 2: {
			/*-----brush should stop for 5s-----*/
			if ((time(NULL) - time_right_brush) >= 5) {
				// Then restart brush and let it fully operated.
				setRightPwm(100);
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
				/*-----Brush is still stall, stop the brush and increase error counter -----*/
				setRightPwm(0);
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
					/*-----brush is in max mode_ more than 5s, turn to normal mode_ and reset error counter-----*/
					setRightPwm(right_pwm_);
					right_status = 1;
					oc_right_cnt_ = 0;
					right_error_counter = 0;
				}
			}
			break;
		}
	}
	return 0;
}

