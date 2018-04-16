//
// Created by austin on 17-12-11.
//

#include <odom.h>
#include <dev.h>
#include "action.hpp"
#include "mode.hpp"

ActionPause::ActionPause()
{
	wheel.stop();
	brush.stop();
	vacuum.stop();
	water_tank.stop(WaterTank::tank_pump);

	lidar.motorCtrl(OFF);

	start_timer_ = ros::Time::now().toSec();
	timeout_interval_ = IDLE_TIMEOUT;
	pause_pose_.setX(odom.getOriginX());
	pause_pose_.setY(odom.getOriginY());

	s_wifi.setWorkMode(Mode::md_idle);
	s_wifi.taskPushBack(S_Wifi::ACT::ACT_UPLOAD_STATUS);
	ROS_INFO("%s %d: Enter action pause.", __FUNCTION__, __LINE__);
}

ActionPause::~ActionPause()
{

	s_wifi.setWorkMode(Mode::cm_navigation);
	s_wifi.taskPushBack(S_Wifi::ACT::ACT_UPLOAD_STATUS);
}

bool ActionPause::isFinish()
{
	return false;
}

bool ActionPause::isExit()
{
	if (two_points_distance_double(pause_pose_.getX(), pause_pose_.getY(), odom.getOriginX(), odom.getOriginY()) > 0.1)
	{
		// Robot moved too far
		ROS_WARN("%s %d: Robot is moved during pause.", __FUNCTION__, __LINE__);
		return true;
	}

	return false;
}


void ActionPause::run()
{
	if (appmt_obj.shouldUpdateIdleTimer())
	{
		appmt_obj.resetUpdateIdleTimerFlag();
		start_timer_ = ros::Time::now().toSec();
		ROS_INFO("%s %d: Action Pause start timer is reset.", __FUNCTION__, __LINE__);
	}
}
