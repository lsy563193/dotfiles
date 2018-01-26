//
// Created by austin on 17-12-8.
//

#include <movement.hpp>
#include "dev.h"

MovementRemoteDirectGo::MovementRemoteDirectGo()
{
	timeout_interval_ = 5;
	c_rcon.resetStatus();
	ROS_INFO("%s %d: Start movement direct go.", __FUNCTION__, __LINE__);
}

MovementRemoteDirectGo::~MovementRemoteDirectGo()
{
	wheel.stop();
	ROS_INFO("%s %d: End movement direct go.", __FUNCTION__, __LINE__);
}


bool MovementRemoteDirectGo::isFinish()
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
		   isTimeUp();
}

void MovementRemoteDirectGo::adjustSpeed(int32_t &left_speed, int32_t &right_speed)
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
