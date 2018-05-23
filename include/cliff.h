//
// Created by root on 11/17/17.
//

#ifndef PP_CLIFF_H
#define PP_CLIFF_H

#include <cstdint>

class Cliff
{
public:
	Cliff() {
		left_status_ = 0;
		front_status_ = 0;
		right_status_ = 0;
	};

	uint8_t getStatus();

	// For status.
	bool getFront(void) {
		return front_status_;
	}

	void setFront(bool value)
	{
		front_status_ = value;
	}

	bool getLeft(void) {
		return left_status_;
	}

	void setLeft(bool value)
	{
		left_status_ = value;
	}

	bool getRight(void) {
		return right_status_;
	}

	void setRight(bool value)
	{
		right_status_ = value;
	}

	// For value.

	int16_t getFrontValue(void) {
		return front_value_;
	}

	void setFrontValue(int16_t value)
	{
		front_value_ = value;
	}

	int16_t getLeftValue(void) {
		return left_value_;
	}

	void setLeftValue(int16_t value)
	{
		left_value_ = value;
	}

	int16_t getRightValue(void) {
		return right_value_;
	}

	void setRightValue(int16_t value)
	{
		right_value_ = value;
	}

private:
	bool left_status_;
	bool front_status_;
	bool right_status_;

	int16_t left_value_;
	int16_t front_value_;
	int16_t right_value_;
};
extern Cliff cliff;

#endif //PP_CLIFF_H
