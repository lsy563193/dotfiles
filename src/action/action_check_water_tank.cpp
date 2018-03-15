//
// Created by austin on 18-3-14.
//

#include "ros/ros.h"
#include "action.hpp"

ActionCheckWaterTank::ActionCheckWaterTank()
{
	ROS_INFO("%s %d: Starting action check water tank." , __FUNCTION__, __LINE__);
	water_tank.normalOperate();
}
