//
// Created by lsy563193 on 11/30/17.
//
#include "pp.h"
#include "arch.hpp"

ActionOpenLidar::ActionOpenLidar() {
	wheel.stop();

	vacuum.setMode(Vac_Save);
	brush.setSidePwm(50, 50);
	brush.setMainPwm(30);

	lidar.motorCtrl(ON);
	lidar.setScanOriginalReady(0);
//	ROS_INFO("%s,%d:add ActionOpenLidar,sp_action_(%d)",__FUNCTION__, __LINE__,sp_action_);

}

bool ActionOpenLidar::isFinish(){
	return (lidar.isScanOriginalReady() == 1);
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
