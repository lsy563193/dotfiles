//
// Created by austin on 17-12-8.
//

#include <pp.h>
#include "arch.hpp"
#include "dev.h"

MovementDirectGo::MovementDirectGo()
{
	direct_go_time_stamp_ = ros::Time::now().toSec();
	ROS_INFO("%s %d: Start movement direct go.", __FUNCTION__, __LINE__);
}

MovementDirectGo::~MovementDirectGo()
{
	wheel.stop();
	ROS_INFO("%s %d: End movement direct go.", __FUNCTION__, __LINE__);
}


bool MovementDirectGo::isFinish()
{

	return ev.remote_direction_forward ||
		   ev.remote_direction_left ||
		   ev.remote_direction_right ||
		   bumper.get_status() ||
		   cliff.get_status() ||
		   ros::Time::now().toSec() - direct_go_time_stamp_ > 5;

}

void MovementDirectGo::adjustSpeed(int32_t &left_speed, int32_t &right_speed)
{
	wheel.setDirectionForward();
	left_speed = right_speed = LINEAR_MAX_SPEED;
}
