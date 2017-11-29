//
// Created by root on 11/17/17.
//

#ifndef PP_BRUSH_H
#define PP_BRUSH_H

#include "robotbase.h"
#include "serial.h"
extern pp::x900sensor sensor;

class Brush {
public:
	Brush() {
		l_pwm = 0;
		r_pwm = 0;
		reset_loc = false;
		reset_roc = false;

		oc_left_cnt = 0;
		oc_main_cnt = 0;
		oc_right_cnt = 0;
	}

	void stop()
	{
		set_side_pwm(0, 0);
		set_main_pwm(0);
	}
	void set_main_pwm(uint16_t PWM) {
		// Set main brush PWM, the value of PWM should be in range (0, 100).
		PWM = PWM < 100 ? PWM : 100;
		serial.setSendData(CTL_BRUSH_MAIN, PWM & 0xff);
	}

	void set_side_pwm(uint16_t L, uint16_t R) {
		// Set left and right brush PWM, the value of L/R should be in range (0, 100).
		L = L < 100 ? L : 100;
		l_pwm = L;
		serial.setSendData(CTL_BRUSH_LEFT, L & 0xff);
		R = R < 100 ? R : 100;
		r_pwm = R;
		serial.setSendData(CTL_BRUSH_RIGHT, R & 0xff);
	}

	void set_left_pwm(uint16_t L) {
		L = L < 100 ? L : 100;
		serial.setSendData(CTL_BRUSH_LEFT, L & 0xff);
	}

	void set_right_pwm(uint16_t R) {
		R = R < 100 ? R : 100;
		serial.setSendData(CTL_BRUSH_RIGHT, R & 0xff);
	}

	uint8_t left_is_stall(void) {
		static time_t time_lbrush;
		static uint8_t lerror_counter = 0;
		static uint8_t left_status = 1;
		/*---------------------------------Left Brush Stall---------------------------------*/
		if (reset_loc) {
			lerror_counter = 0;
			left_status = 1;
			reset_loc = false;
			//ROS_WARN("%s %d: Reset left brush.", __FUNCTION__, __LINE__);
			return 0;
		}

		switch (left_status) {
			case 1: {
				if (getLbrushOc()) {
					if (oc_left_cnt < 200)
						oc_left_cnt++;
				}
				else
					oc_left_cnt = 0;

				if (oc_left_cnt > 10) {
					/*-----Left Brush is stall, stop the brush-----*/
					set_left_pwm(0);
					left_status = 2;
					time_lbrush = time(NULL);
					ROS_WARN("%s %d: Stop the brush for 5s.", __FUNCTION__, __LINE__);
				}
				break;
			}
			case 2: {
				/*-----brush should stop for 5s-----*/
				if ((time(NULL) - time_lbrush) >= 5) {
					// Then restart brush and let it fully operated.
					set_left_pwm(100);
					left_status = 3;
					time_lbrush = time(NULL);
					ROS_WARN("%s %d: Fully operate the brush for 5s.", __FUNCTION__, __LINE__);
				}
				break;
			}

			case 3: {
				if (getLbrushOc()) {
					if (oc_left_cnt < 200)
						oc_left_cnt++;
				}
				else {
					oc_left_cnt = 0;
				}

				if (oc_left_cnt > 10) {
					/*-----Brush is still stall, stop the brush and increase error counter -----*/
					set_left_pwm(0);
					left_status = 2;
					time_lbrush = time(NULL);
					lerror_counter++;
					if (lerror_counter > 2) {
						left_status = 1;
						oc_left_cnt = 0;
						lerror_counter = 0;
						return 1;
					}
					break;
				}
				else {
					ROS_DEBUG("%s %d: Fully operate the brush for 5s.", __FUNCTION__, __LINE__);
					if ((time(NULL) - time_lbrush) >= 5) {
						ROS_WARN("%s %d: Restore from fully operated.", __FUNCTION__, __LINE__);
						/*-----brush is in max mode_ more than 5s, turn to normal mode_ and reset error counter-----*/
						set_left_pwm(l_pwm);
						left_status = 1;
						oc_left_cnt = 0;
						lerror_counter = 0;
					}
				}
				break;
			}
		}
		return 0;
	}

	uint8_t right_is_stall(void) {
		static time_t time_rbrush;
		static uint8_t rerror_counter = 0;
		static uint8_t right_status = 1;
		/*---------------------------------Left Brush Stall---------------------------------*/

		if (reset_roc) {
			rerror_counter = 0;
			right_status = 1;
			reset_roc = false;
			//ROS_WARN("%s %d: Reset right brush.", __FUNCTION__, __LINE__);
			return 0;
		}

		switch (right_status) {
			case 1: {
				if (getRbrushOc()) {
					if (oc_right_cnt < 200)
						oc_right_cnt++;
				}
				else
					oc_right_cnt = 0;

				if (oc_right_cnt > 10) {
					/*-----Left Brush is stall, stop the brush-----*/
					set_right_pwm(0);
					right_status = 2;
					time_rbrush = time(NULL);
					ROS_WARN("%s %d: Stop the brush for 5s.", __FUNCTION__, __LINE__);
				}
				break;
			}
			case 2: {
				/*-----brush should stop for 5s-----*/
				if ((time(NULL) - time_rbrush) >= 5) {
					// Then restart brush and let it fully operated.
					set_right_pwm(100);
					right_status = 3;
					time_rbrush = time(NULL);
					ROS_WARN("%s %d: Fully operate the brush for 5s.", __FUNCTION__, __LINE__);
				}
				break;
			}

			case 3: {
				if (getRbrushOc()) {
					if (oc_right_cnt < 200)
						oc_right_cnt++;
				}
				else {
					oc_right_cnt = 0;
				}

				if (oc_right_cnt > 10) {
					/*-----Brush is still stall, stop the brush and increase error counter -----*/
					set_right_pwm(0);
					right_status = 2;
					time_rbrush = time(NULL);
					rerror_counter++;
					if (rerror_counter > 2) {
						right_status = 1;
						oc_right_cnt = 0;
						rerror_counter = 0;
						return 1;
					}
					break;
				}
				else {
					ROS_DEBUG("%s %d: Fully operate the brush for 5s.", __FUNCTION__, __LINE__);
					if ((time(NULL) - time_rbrush) >= 5) {
						ROS_WARN("%s %d: Restore from fully operated.", __FUNCTION__, __LINE__);
						/*-----brush is in max mode_ more than 5s, turn to normal mode_ and reset error counter-----*/
						set_right_pwm(r_pwm);
						right_status = 1;
						oc_right_cnt = 0;
						rerror_counter = 0;
					}
				}
				break;
			}
		}
		return 0;
	}

	bool getLbrushOc() const {
		return sensor.lbrush_oc;
	}

	bool getRbrushOc() const {
		return sensor.rbrush_oc;
	}

	bool getMbrushOc() const {
		return sensor.mbrush_oc;
	}

	uint8_t oc_left_cnt;
	uint8_t oc_main_cnt;
	uint8_t oc_right_cnt;
private:

//Value for saving SideBrush_PWM
	uint16_t l_pwm;
	uint16_t r_pwm;

	bool reset_loc;
	bool reset_roc;
};
extern Brush brush;

#endif //PP_BRUSH_H
