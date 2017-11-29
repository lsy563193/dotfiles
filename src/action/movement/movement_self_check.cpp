//
// Created by root on 11/29/17.
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
			wheel.set_dir_right();
		} else {
			wheel.set_dir_left();
		}
		left_speed = 30;
		right_speed = 30;
	}
	else if (ev.cliff_jam)
	{
		wheel.set_dir_backward();
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
				wheel.set_dir_backward();
				left_speed = right_speed = RUN_TOP_SPEED;
				break;
			}
			case 4:
			{
				// Quickly turn right for 90 degrees.
				wheel.set_dir_right();
				left_speed = right_speed = RUN_TOP_SPEED;
				break;
			}
			case 5:
			{
				// Quickly turn left for 180 degrees.
				wheel.set_dir_left();
				left_speed = right_speed = RUN_TOP_SPEED;
				break;
			}
		}
	}
//	else if(g_omni_notmove)
//	{
		//wheel.set_dir_backward();
		//left_speed = right_speed = RUN_TOP_SPEED;
//	}
	else if(g_slip_cnt>=2)
	{
		if(g_slip_cnt <3)
			wheel.set_dir_left();
		else if(g_slip_cnt <4)
			wheel.set_dir_right();
		left_speed = right_speed = ROTATE_TOP_SPEED;
	}
	else if (ev.laser_stuck)
	{
		wheel.set_dir_backward();
		left_speed = right_speed = 2;
	}

	wheel.set_speed(left_speed, right_speed);
}

