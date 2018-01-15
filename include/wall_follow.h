//
// Created by root on 11/20/17.
//

#ifndef PP_WALL_FOLLOW_H
#define PP_WALL_FOLLOW_H


#include <cstdint>
#include <cfloat>

class WallFollow {
public:

	void set_base(int8_t dir, int32_t data) {
		if (dir == 0) {
			left_baseline = data;
		}
		else {
			right_baseline = data;
		}
	}

	int32_t get_base(int8_t dir) {
		if (dir == 0) {
			return left_baseline;
		}
		else {
			return right_baseline;
		}
	}

	int32_t get_adc(int8_t dir)
	{
		if (dir == 0)
		{
			return (int32_t) getLeft();
		} else
		{
			return (int32_t) getRight();
		}
	}


	void dynamic_base(uint32_t Cy);

	int16_t getLeft() const
	{
		return left_ - left_baseline;
	}

	void setLeft(int16_t val)
	{
		left_ = val;
	}

	int16_t getRight() const
	{
		return right_ - right_baseline;
	}

	void setRight(int16_t val)
	{
		right_ = val;
	}

private:
	int16_t left_{0};
	int16_t right_{0};
	// Value for wall sensor offset.
	int16_t left_baseline = 50;
	int16_t right_baseline = 50;

};

extern WallFollow wall;

#endif //PP_WALL_FOLLOW_H
