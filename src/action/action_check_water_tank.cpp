//
// Created by austin on 18-3-14.
//

#include <water_tank.hpp>
#include "ros/ros.h"
#include "action.hpp"

ActionCheckWaterTank::ActionCheckWaterTank()
{
	ROS_INFO("%s %d: Starting action check water tank." , __FUNCTION__, __LINE__);
	if(water_tank.checkEquipment(false))
		water_tank.open(WaterTank::tank_pump);
}
