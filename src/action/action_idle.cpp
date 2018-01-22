//
// Created by lsy563193 on 11/30/17.
//


#include <action.hpp>
#include <error.h>
#include "dev.h"

ActionIdle::ActionIdle()
{
	ROS_INFO("%s %d: Start action idle.", __FUNCTION__, __LINE__);
	if (error.get())
		led.set_mode(LED_FLASH, LED_RED);
	else
		led.set_mode(LED_BREATH, LED_GREEN);

	timeout_interval_ = IDLE_TIMEOUT;
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
}

bool ActionIdle::isTimeUp()
{
	if (IAction::isTimeUp())
	{
		ROS_INFO("%s %d: Timeout(%ds).", __FUNCTION__, __LINE__, timeout_interval_);
		return true;
	}
	return false;
}
