//
// Created by austin on 17-12-8.
//

#include <pp.h>
#include "arch.hpp"
#include "dev.h"

MovementStay::MovementStay()
{
	vacuum.setMode(Vac_Save);
	brush.setSidePwm(50, 50);
	brush.setMainPwm(30);
	ROS_INFO("%s %d: Start movement stay.", __FUNCTION__, __LINE__);
}

MovementStay::~MovementStay()
{
	ROS_INFO("%s %d: End movement stay.", __FUNCTION__, __LINE__);
}


bool MovementStay::isFinish()
{

	if (ev.remote_direction_forward || ev.remote_direction_left ||
		ev.remote_direction_right)
		return true;

	return false;
}

void MovementStay::adjustSpeed(int32_t &left_speed, int32_t &right_speed)
{
	left_speed = right_speed = 0;
}
