//
// Created by root on 11/17/17.
//

#ifndef PP_CLIFF_H
#define PP_CLIFF_H

#include "mathematics.h"

#define CLIFF_LIMIT  60

class Cliff
{
public:
	Cliff() {
		left_trig_value_ = CLIFF_LIMIT;
		front_trig_value_ = CLIFF_LIMIT;
		right_trig_value_ = CLIFF_LIMIT;
		left_value_ = 0;
		front_value_ = 0;
		right_value_ = 0;
	};

	uint8_t get_status(void);

	int16_t getFrontTrigValue(void) {
		return front_trig_value_;
	}

	int16_t getLeftTrigValue(void) {
		return left_trig_value_;
	}

	int16_t getRightTrigValue(void) {
		return right_trig_value_;
	}

	int16_t getFront(void) {
		return front_value_;
	}

	void setFront(int16_t value)
	{
		front_value_ = value;
	}

	int16_t getLeft(void) {
		return left_value_;
	}

	void setLeft(int16_t value)
	{
		left_value_ = value;
	}

	int16_t getRight(void) {
		return right_value_;
	}

	void setRight(int16_t value)
	{
		right_value_ = value;
	}

private:
	int16_t left_trig_value_;
	int16_t front_trig_value_;
	int16_t right_trig_value_;
	int16_t left_value_;
	int16_t front_value_;
	int16_t right_value_;

};
extern Cliff cliff;

#endif //PP_CLIFF_H
