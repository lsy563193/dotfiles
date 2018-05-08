//
// Created by austin on 17-12-8.
//

#include <movement.hpp>
#include "dev.h"

MovementDirectGo::MovementDirectGo(bool slow_down, float timeout)
{
	timeout_interval_ = timeout;
	c_rcon.resetStatus();
	slow_down_ = slow_down;
	ROS_WARN("%s %d: Start movement direct go.", __FUNCTION__, __LINE__);
}

MovementDirectGo::~MovementDirectGo()
{
	wheel.stop();
	ROS_WARN("%s %d: End movement direct go.", __FUNCTION__, __LINE__);
}


bool MovementDirectGo::isFinish()
{
	ev.bumper_triggered = bumper.getStatus();
	ev.cliff_triggered = cliff.getStatus();
	ev.rcon_status = c_rcon.getForwardTop();

	return ev.remote_direction_forward ||
		   ev.remote_direction_left ||
		   ev.remote_direction_right ||
		   ev.bumper_triggered ||
		   ev.cliff_triggered ||
		   ev.rcon_status ||
		   isTimeUp();
}

void MovementDirectGo::adjustSpeed(int32_t &left_speed, int32_t &right_speed)
{
	wheel.setDirectionForward();
	if (slow_down_ && obs.getStatus() > 0)
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
