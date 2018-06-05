//
// Created by pierre on 18-5-17.
//

#include <action.hpp>
#include "lidar.hpp"

ActionOpenGyroAndLidar::ActionOpenGyroAndLidar() {
	ROS_WARN("%s %d: Enter action open gyro and lidar.", __FUNCTION__, __LINE__);
	lidar.motorCtrl(ON);
}

