//
// Created by root on 11/17/17.
//

#ifndef PP_TILT_H
#define PP_TILT_H

#include "gyro.h"

class Tilt {
public:
Tilt()
{
	g_status = 0;
}

uint8_t check()
{
	//todo Change the method of getting the acc data, now data is from gyro instance.
	static uint16_t front_count = 0;
	static uint16_t left_count = 0;
	static uint16_t right_count = 0;
	static uint16_t z_count = 0;
	uint8_t tmp_status = 0;

	if (g_enable)
	{
		if (gyro.getXAcc() - gyro.getInitXAcc() > FRONT_TILT_LIMIT)
		{
			front_count += 2;
			//ROS_WARN("%s %d: front(%d)\tfront init(%d), front cnt(%d).", __FUNCTION__, __LINE__, gyro.getXAcc(), gyro.getInitXAcc(), front_count);
		}
		else
		{
			if (front_count > 0)
				front_count--;
			else
				front_count = 0;
		}
		if (gyro.getYAcc() - gyro.getInitYAcc() > LEFT_TILT_LIMIT)
		{
			left_count++;
			//ROS_WARN("%s %d: left(%d)\tleft init(%d), left cnt(%d).", __FUNCTION__, __LINE__, gyro.getYAcc(), gyro.getInitYAcc(), left_count);
		}
		else
		{
			if (left_count > 0)
				left_count--;
			else
				left_count = 0;
		}
		if (gyro.getRight() - gyro.getRightInit() > RIGHT_TILT_LIMIT)
		{
			right_count++;
			//ROS_WARN("%s %d: right(%d)\tright init(%d), right cnt(%d).", __FUNCTION__, __LINE__, gyro.getRight(), gyro.getRightInit(), right_count);
		}
		else
		{
			if (right_count > 0)
				right_count--;
			else
				right_count = 0;
		}
		if (abs(gyro.getZAcc() - gyro.getInitZAcc()) > DIF_TILT_Z_VAL)
		{
			z_count++;
			//ROS_WARN("%s %d: z(%d)\tzi(%d).", __FUNCTION__, __LINE__, robot::instance()->getZAcc(), robot::instance()->getInitZAcc());
		}
		else
		{
			if (z_count > 1)
				z_count -= 2;
			else
				z_count = 0;
		}

		//if (left_count > 7 || front_count > 7 || right_count > 7 || z_count > 7)
			//ROS_WARN("%s %d: count left:%d, front:%d, right:%d, z:%d", __FUNCTION__, __LINE__, left_count, front_count, right_count, z_count);

		if (front_count + left_count + right_count + z_count > TILT_COUNT_REACH)
		{
			ROS_INFO("\033[47;34m" "%s,%d,robot tilt !!" "\033[0m",__FUNCTION__,__LINE__);
			if (left_count > TILT_COUNT_REACH / 3)
				tmp_status |= TILT_LEFT;
			if (right_count > TILT_COUNT_REACH / 3)
				tmp_status |= TILT_RIGHT;

			if (front_count > TILT_COUNT_REACH / 3 || !tmp_status)
				tmp_status |= TILT_FRONT;
			set_status(tmp_status);
			front_count /= 3;
			left_count /= 3;
			right_count /= 3;
			z_count /= 3;
		}
		else if (front_count + left_count + right_count + z_count < TILT_COUNT_REACH / 4)
			set_status(0);
	}
	else{
		front_count = 0;
		left_count = 0;
		right_count = 0;
		z_count = 0;
		set_status(0);
	}

	return tmp_status;
}

void set_status(uint8_t status)
{
	g_status = status;
}

uint8_t get_status()
{
	return g_status;
}

bool isEnable()
	{
		return g_enable;
	}

	bool enable(bool val)
	{g_enable = val;}

private:
	uint8_t g_status;
	bool g_enable;
};
extern Tilt tilt;

#endif //PP_TILT_H
