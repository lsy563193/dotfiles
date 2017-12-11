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

	if(lidar.isScanOriginalReady())
	{
		lidar.motorCtrl(OFF);
		lidar.setScanOriginalReady(0);
	}

	pause_start_time_ = ros::Time::now().toSec();
	pause_pose_.setX(odom.getX());
	pause_pose_.setY(odom.getY());
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

bool ActionPause::isTimeUp()
{
	if (ros::Time::now().toSec() - pause_start_time_ > 600)
		// Robot pause for too long, directly change to sleep mode.
		return true;

	return false;
}
void ActionPause::run()
{
	// Just do nothing...
}
