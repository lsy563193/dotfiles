//
// Created by lsy563193 on 11/29/17.
//
#include "pp.h"

void SelfCheckRegulator::adjustSpeed(uint8_t bumper_jam_state)
{
	uint8_t left_speed;
	uint8_t right_speed;
	if (ev.oc_suction)
		left_speed = right_speed = 0;
	else if (ev.oc_wheel_left || ev.oc_wheel_right)
	{
		if (ev.oc_wheel_right) {
			wheel.setDirectionRight();
		} else {
			wheel.setDirectionLeft();
		}
		left_speed = 30;
		right_speed = 30;
	}
	else if (ev.cliff_jam)
	{
		wheel.setDirBackward();
		left_speed = right_speed = 18;
	}
	else if (ev.bumper_jam)
	{
		switch (bumper_jam_state)
		{
			case 1:
			case 2:
			case 3:
			{
				// Quickly move back for a distance.
				wheel.setDirBackward();
				left_speed = right_speed = RUN_TOP_SPEED;
				break;
			}
			case 4:
			{
				// Quickly turn right for 90 degrees.
				wheel.setDirectionRight();
				left_speed = right_speed = RUN_TOP_SPEED;
				break;
			}
			case 5:
			{
				// Quickly turn left for 180 degrees.
				wheel.setDirectionLeft();
				left_speed = right_speed = RUN_TOP_SPEED;
				break;
			}
		}
	}
	else if(g_slip_cnt>=2)
	{
		if(g_slip_cnt <3)
			wheel.setDirectionLeft();
		else if(g_slip_cnt <4)
			wheel.setDirectionRight();
		left_speed = right_speed = ROTATE_TOP_SPEED;
	}
	else if (ev.lidar_stuck)
	{
		wheel.setDirBackward();
		left_speed = right_speed = 2;
	}

	wheel.setPidTargetSpeed(left_speed, right_speed);
}

