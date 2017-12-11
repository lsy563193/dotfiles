//
// Created by austin on 17-12-11.
//

#include <odom.h>
#include <pp.h>
#include "arch.hpp"

ActionPause::ActionPause()
{
	pause_start_time_ = ros::Time::now().toSec();
	pause_pose_.setX(odom.getX());
	pause_pose_.setY(odom.getY());
	pause_pose_.setAngle(odom.getAngle());
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
		// Robot moved too far
		return true;

	if (ros::Time::now().toSec() - pause_start_time_ > 600)
		return true;

	return false;
}

void ActionPause::run()
{
	// Just do nothing...
}
