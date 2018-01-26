//
// Created by austin on 17-12-8.
//

#include <movement.hpp>
#include <move_type.hpp>
#include <event_manager.h>
#include "dev.h"

MovementStay::MovementStay()
{
	ROS_INFO("%s %d: Start movement stay.", __FUNCTION__, __LINE__);
	start_timer_ = ros::Time::now().toSec();
	timeout_interval_ = 15;
}

MovementStay::~MovementStay()
{
	ROS_INFO("%s %d: End movement stay.", __FUNCTION__, __LINE__);
}


bool MovementStay::isFinish()
{
	ev.bumper_triggered = bumper.getStatus();
	ev.cliff_triggered = cliff.getStatus();

	return ev.remote_direction_forward ||
		   ev.remote_direction_left ||
		   ev.remote_direction_right ||
		   ev.bumper_triggered ||
		   ev.cliff_triggered ||
		   isTimeUp();
}

void MovementStay::adjustSpeed(int32_t &left_speed, int32_t &right_speed)
{
	left_speed = right_speed = 0;
}
