//
// Created by root on 11/17/17.
//

#ifndef PP_OBS_H
#define PP_OBS_H

#include "map.h"

#include <pp/x900sensor.h>

extern pp::x900sensor sensor;
class Obs {
public:
	Obs() {
		left_trig_value = 100;
		front_trig_value = 100;
		right_trig_value = 100;
		g_left_baseline = 100;
		g_front_baseline = 100;
		g_right_baseline = 100;
	};

//--------------------------------------Obs Dynamic adjust----------------------
	void dynamic_base(uint16_t count);

	int16_t get_front_trig_value(void) {
		return front_trig_value;
	}

	int16_t get_left_trig_value(void) {
		return left_trig_value;
	}

	int16_t get_right_trig_value(void) {
		return right_trig_value;
	}

	int16_t get_front_baseline(void) {
		return g_front_baseline;
	}

	int16_t get_left_baseline(void) {
		return g_left_baseline;
	}

	int16_t get_right_baseline(void) {
		return g_right_baseline;
	}

//	uint8_t get_status(int16_t left_offset, int16_t front_offset, int16_t right_offset) {
		uint8_t get_status(int16_t left_offset = 0, int16_t front_offset = 0, int16_t right_offset = 0){
		uint8_t status = 0;

		if (get_left() > get_left_trig_value() + left_offset)
			status |= BLOCK_LEFT;

		if (get_front() > get_front_trig_value() + front_offset)
			status |= BLOCK_FRONT;

		if (get_right() > get_right_trig_value() + right_offset)
			status |= BLOCK_RIGHT;

		return status;
	}

	int16_t get_front(void) {
		return sensor.f_obs;
	}

	int16_t get_left(void) {
		return sensor.l_obs;
	}

	int16_t get_right(void) {
		return sensor.r_obs;
	}

	bool is_wall_front(void) {
		auto status = (get_front() > get_front_trig_value());
		return  status;
	}

	friend	void obs_dynamic_base(uint16_t count);
private:

int16_t left_trig_value;
int16_t front_trig_value;
int16_t right_trig_value;
int16_t g_left_baseline;
int16_t g_front_baseline;
int16_t g_right_baseline;

};

void obs_dynamic_base(uint16_t count);
extern Obs obs;

#endif //PP_OBS_H
