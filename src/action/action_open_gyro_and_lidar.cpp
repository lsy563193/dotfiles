//
// Created by pierre on 18-5-17.
//

#include <action.hpp>
#include "lidar.hpp"

ActionOpenGyroAndLidar::ActionOpenGyroAndLidar() {
	ROS_WARN("%s %d: Enter.", __FUNCTION__, __LINE__);
	lidar.motorCtrl(ON);
}

ActionOpenGyroAndLidar::~ActionOpenGyroAndLidar()
{
	ROS_WARN("%s %d: Exit.", __FUNCTION__, __LINE__);
}

