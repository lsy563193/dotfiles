//
// Created by root on 11/17/17.
//

#ifndef PP_CLIFF_H
#define PP_CLIFF_H

#include "movement.h"
#include "map.h"
#include <pp/x900sensor.h>
extern pp::x900sensor sensor;

class Cliff {
public:
	Cliff() {
		left_trig_value_ = CLIFF_LIMIT;
		front_trig_value_ = CLIFF_LIMIT;
		right_trig_value_ = CLIFF_LIMIT;
		g_left_baseline_ = 100;
		g_front_baseline_ = 100;
		g_right_baseline_ = 100;
	};

	uint8_t get_status(void) {
		uint8_t status = 0x00;

		if (get_left() < get_left_trig_value_())
			status |= BLOCK_LEFT;

		if (get_front() < get_front_trig_value_())
			status |= BLOCK_FRONT;

		if (get_right() < get_right_trig_value_())
			status |= BLOCK_RIGHT;

		//if (status != 0x00){
		//	ROS_WARN("%s %d: Return Cliff status:%x.", __FUNCTION__, __LINE__, status);
		//	beep_for_command(true);
		//}
		return status;
	}

	int16_t get_front_trig_value_(void) {
		return front_trig_value_;
	}

	int16_t get_left_trig_value_(void) {
		return left_trig_value_;
	}

	int16_t get_right_trig_value_(void) {
		return right_trig_value_;
	}

	int16_t get_front(void) {
		return sensor.fcliff;
	}

	int16_t get_left(void) {
		return sensor.lcliff;
	}

	int16_t get_right(void) {
		return sensor.rcliff;
	}

private:
	int16_t left_trig_value_;
	int16_t front_trig_value_;
	int16_t right_trig_value_;
	int16_t g_left_baseline_;
	int16_t g_front_baseline_;
	int16_t g_right_baseline_;

};
extern Cliff cliff;

#endif //PP_CLIFF_H
