//
// Created by austin on 17-12-21.
//

#include <event_manager.h>
#include "action.hpp"
#include "dev.h"

ActionCheckVacuum::ActionCheckVacuum()
{
	ROS_INFO("%s %d: Starting action check vacuum." , __FUNCTION__, __LINE__);
	vacuum.setForMaxMode(false);
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
	vacuum.setSpeedByMode();
}
