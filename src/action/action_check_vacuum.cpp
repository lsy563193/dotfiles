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
