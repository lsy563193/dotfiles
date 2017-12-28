//
// Created by lsy563193 on 11/30/17.
//
#include "pp.h"
#include "arch.hpp"

ActionOpenLidar::ActionOpenLidar() {
	ROS_INFO("%s %d: Enter action open lidar.", __FUNCTION__, __LINE__);
	wheel.stop();

	lidar.motorCtrl(ON);
	lidar.setScanOriginalReady(0);
//	ROS_INFO("%s,%d:add ActionOpenLidar,sp_action_(%d)",__FUNCTION__, __LINE__,sp_action_);

}

bool ActionOpenLidar::isFinish(){
	if (lidar.isScanOriginalReady() == 1)
	{
		ROS_INFO("%s %d: Action open lidar succeed.", __FUNCTION__, __LINE__);
		return true;
	}

	return false;
}

void ActionOpenLidar::run() {
	wheel.setPidTargetSpeed(0, 0);
}

//IAction* ActionOpenLidar::setNextAction() {
//	if ((g_is_manual_pause || g_is_low_bat_pause) && slam.isMapReady()) {
//		 return new ActionOpenGyroNav;
//	}
//	else {
//		return new ActionAlign;
//	}
//}
