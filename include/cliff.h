//
// Created by root on 11/17/17.
//

#ifndef PP_CLIFF_H
#define PP_CLIFF_H

#include "mathematics.h"

class Cliff
{
public:
	Cliff() {
		left_value_ = 0;
		front_value_ = 0;
		right_value_ = 0;
	};

	uint8_t getStatus(void);

	bool getFront(void) {
		return front_value_;
	}

	void setFront(bool value)
	{
		front_value_ = value;
	}

	bool getLeft(void) {
		return left_value_;
	}

	void setLeft(bool value)
	{
		left_value_ = value;
	}

	bool getRight(void) {
		return right_value_;
	}

	void setRight(bool value)
	{
		right_value_ = value;
	}

private:
	bool left_value_;
	bool front_value_;
	bool right_value_;

};
extern Cliff cliff;

#endif //PP_CLIFF_H
