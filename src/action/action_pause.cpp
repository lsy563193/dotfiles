//
// Created by austin on 17-12-11.
//

#include <odom.h>
#include <pp.h>
#include "arch.hpp"

ActionPause::ActionPause()
{
	wheel.stop();
	brush.stop();
	vacuum.stop();

	lidar.motorCtrl(OFF);

	start_timer_ = ros::Time::now().toSec();
	timeout_interval_ = IDLE_TIMEOUT;
	pause_pose_.setX(odom.getX());
	pause_pose_.setY(odom.getY());
	ROS_INFO("%s %d: Enter action pause.", __FUNCTION__, __LINE__);
}

ActionPause::~ActionPause()
{

}

bool ActionPause::isFinish()
{
	return false;
}

bool ActionPause::isExit()
{
	if (two_points_distance_double(pause_pose_.getX(), pause_pose_.getY(), odom.getX(), odom.getY()) > 0.1)
	{
		// Robot moved too far
		ROS_WARN("%s %d: Robot is moved during pause.", __FUNCTION__, __LINE__);
		return true;
	}

	return false;
}


void ActionPause::run()
{
	// Just do nothing...
}
