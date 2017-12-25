//
// Created by austin on 17-12-21.
//

#include "arch.hpp"
#include "dev.h"

ActionCheckVacuum::ActionCheckVacuum()
{
	ROS_INFO("%s %d: Starting action check vacuum." , __FUNCTION__, __LINE__);
	vacuum.bldcSpeed(80);
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
	vacuum.bldcSpeed(80);
}
