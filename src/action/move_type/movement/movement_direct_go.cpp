//
// Created by austin on 17-12-8.
//

#include <movement.hpp>
#include <move_type.hpp>
#include <event_manager.h>
#include "dev.h"

MovementDirectGo::MovementDirectGo()
{
	direct_go_time_stamp_ = ros::Time::now().toSec();
	c_rcon.resetStatus();
	ROS_INFO("%s %d: Start movement direct go.", __FUNCTION__, __LINE__);
}

MovementDirectGo::~MovementDirectGo()
{
	wheel.stop();
	ROS_INFO("%s %d: End movement direct go.", __FUNCTION__, __LINE__);
}


bool MovementDirectGo::isFinish()
{
	ev.bumper_triggered = bumper.getStatus();
	ev.cliff_triggered = cliff.getStatus();
	ev.rcon_triggered = c_rcon.getForwardTop();

	return ev.remote_direction_forward ||
		   ev.remote_direction_left ||
		   ev.remote_direction_right ||
		   ev.bumper_triggered ||
		   ev.cliff_triggered ||
		   ev.rcon_triggered ||
		   ros::Time::now().toSec() - direct_go_time_stamp_ > 5;
}

void MovementDirectGo::adjustSpeed(int32_t &left_speed, int32_t &right_speed)
{
	wheel.setDirectionForward();
	if (obs.getStatus() > 0)
	{
		speed_--;
		check_limit(speed_, LINEAR_MIN_SPEED, LINEAR_MAX_SPEED);
		left_speed = right_speed = speed_;
	}
	else
	{
		speed_++;
		check_limit(speed_, LINEAR_MIN_SPEED, LINEAR_MAX_SPEED);
		left_speed = right_speed = speed_;
	}
}
