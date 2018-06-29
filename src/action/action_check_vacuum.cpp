//
// Created by austin on 17-12-21.
//

#include <event_manager.h>
#include "action.hpp"
#include "dev.h"

ActionCheckVacuum::ActionCheckVacuum()
{
	ROS_WARN("%s %d: Start." , __FUNCTION__, __LINE__);
	vacuum.setForUserSetMaxMode(false);
	vacuum.setSpeedByUserSetMode();
}

bool ActionCheckVacuum::isFinish()
{
	if (ev.key_clean_pressed)
	{
		ev.key_clean_pressed = false;
		return true;
	}

	return false;
}

bool ActionCheckVacuum::isExit()
{
	return false;
}

void ActionCheckVacuum::run()
{
	// Just...
}
