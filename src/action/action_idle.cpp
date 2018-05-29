//
// Created by lsy563193 on 11/30/17.
//


#include <action.hpp>
#include <error.h>
#include "dev.h"

#define ERROR_ALARM_TIMES 5
#define ERROR_ALARM_INTERVAL 10

ActionIdle::ActionIdle()
{
	timeout_interval_ = IDLE_TIMEOUT*1.0;
	ROS_WARN("%s %d: Start action idle. timeout %.0fs.", __FUNCTION__, __LINE__,timeout_interval_);
}

ActionIdle::~ActionIdle()
{
	ROS_WARN("%s %d: Exit action idle.", __FUNCTION__, __LINE__);
}

bool ActionIdle::isFinish()
{
	return false;
}

void ActionIdle::run()
{
	// Just wait...
	if (robot_error.get() && error_alarm_cnt_ < ERROR_ALARM_TIMES
		&& ros::Time::now().toSec() - error_alarm_time_ > ERROR_ALARM_INTERVAL)
	{
		robot_error.alarm();
		error_alarm_time_ = ros::Time::now().toSec();
		error_alarm_cnt_++;
	}

	if (appmt_obj.shouldUpdateIdleTimer())
	{
		appmt_obj.resetUpdateIdleTimerFlag();
		start_timer_ = ros::Time::now().toSec();
		ROS_INFO("%s %d: Action idle start timer is reset.", __FUNCTION__, __LINE__);
	}
}

bool ActionIdle::isTimeUp()
{
	if (IAction::isTimeUp())
	{
		ROS_WARN("%s %d: Timeout(%fs).", __FUNCTION__, __LINE__, timeout_interval_);
		return true;
	}
	return false;
}
