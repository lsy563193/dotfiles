//
// Created by lsy563193 on 11/30/17.
//
#include "dev.h"
#include "action.hpp"

ActionOpenLidar::ActionOpenLidar()
{
	ROS_WARN("%s %d: Enter.", __FUNCTION__, __LINE__);
	wheel.stop();

	lidar.motorCtrl(ON);
	lidar.setScanOriginalReady(0);

	timeout_interval_ = 15;

}

ActionOpenLidar::~ActionOpenLidar()
{
	ROS_WARN("%s %d: Exit.", __FUNCTION__, __LINE__);
}

bool ActionOpenLidar::isFinish(){
	if (lidar.isScanOriginalReady() == 1)
	{
		ROS_WARN("%s %d: Action open lidar succeed.", __FUNCTION__, __LINE__);
		lidar.setLidarStuckCheckingEnable(true);
		return true;
	}

	return false;
}

void ActionOpenLidar::run() {
	wheel.setPidTargetSpeed(0, 0);
}

bool ActionOpenLidar::isTimeUp()
{
	if (IAction::isTimeUp())
	{
		ROS_ERROR("%s %d: Open lidar timeout", __FUNCTION__, __LINE__);
		return true;
	}

	return false;
}

//IAction* ActionOpenLidar::setNextAction() {
//	if ((g_is_manual_pause || g_is_low_bat_pause) && slam.isMapReady()) {
//		 return new ActionOpenGyroNav;
//	}
//	else {
//		return new ActionAlign;
//	}
//}
