//
// Created by lsy563193 on 12/20/17.
//


#include <movement.hpp>
#include <move_type.hpp>
#include "dev.h"
MovementStraight::MovementStraight()
{
	timeout_interval_ = 0.2;
	ROS_WARN("%s %d: Start movement Straight, timeout(%f)s.", __FUNCTION__, __LINE__, timeout_interval_);
}

MovementStraight::~MovementStraight()
{
//	wheel.stop();
	ROS_WARN("%s %d: End movement Straight.", __FUNCTION__, __LINE__);
}


bool MovementStraight::isFinish()
{
	return isTimeUp() || sp_mt_->isFinishForward();
}

void MovementStraight::adjustSpeed(int32_t &left_speed, int32_t &right_speed) {
	wheel.setDirectionForward();
	auto tramp = ros::Time::now().toSec() - start_timer_;
//	ROS_INFO("%s,%d tramp(%f),",__FUNCTION__, __LINE__,tramp);
	if (tramp < (timeout_interval_ / 3)) {
		if (left_speed < 8)
			left_speed = right_speed += 1;
		else
			left_speed = right_speed = 8;
	}
	else if (tramp < (2 * timeout_interval_ / 3)) {
		if (left_speed < 8)
			left_speed = right_speed = 8;
		if (left_speed < 13)
			left_speed = right_speed += 1;
		else
			left_speed = right_speed = 13;
	}
	else {
		if (left_speed < 13)
			left_speed = right_speed = 13;
		if (left_speed < 18)
			left_speed = right_speed += 1;
		else
			left_speed = right_speed = 18;
	}
}
