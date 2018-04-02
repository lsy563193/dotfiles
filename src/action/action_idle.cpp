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
	ROS_INFO("%s %d: Start action idle. timeout %f", __FUNCTION__, __LINE__,timeout_interval_);

}

ActionIdle::~ActionIdle()
{
	ROS_INFO("%s %d: Exit action idle.", __FUNCTION__, __LINE__);
}

bool ActionIdle::isFinish()
{
	return false;
}

void ActionIdle::run()
{
	// Just wait...
	if (error.get() && error_alarm_cnt_ < ERROR_ALARM_TIMES
		&& ros::Time::now().toSec() - error_alarm_time_ > ERROR_ALARM_INTERVAL)
	{
		error.alarm();
		error_alarm_time_ = ros::Time::now().toSec();
		error_alarm_cnt_++;
	}

}

bool ActionIdle::isTimeUp()
{
	if (IAction::isTimeUp())
	{
		ROS_INFO("%s %d: Timeout(%fs).", __FUNCTION__, __LINE__, timeout_interval_);
		return true;
	}
	return false;
}
