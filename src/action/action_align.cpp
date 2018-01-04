//
// Created by lsy563193 on 11/30/17.
//
#include "pp.h"
#include "arch.hpp"

ActionAlign::ActionAlign() {
	wheel.stop();
	lidar.startAlign();
//	ROS_INFO("%s,%d:add ActionOpenLidar,sp_action_(%d)",__FUNCTION__, __LINE__,sp_action_);
}

bool ActionAlign::isFinish() {
	if (lidar.alignTimeOut()) {
		return true;
	}
	if (lidar.alignFinish()) {
		float align_angle = lidar.alignAngle();
		odom.setAngleOffset(-align_angle);
		ROS_INFO("%s %d: align_angle angle (%f).", __FUNCTION__, __LINE__, align_angle);
		return true;
	}
	return false;
}

void ActionAlign::run() {
	wheel.setPidTargetSpeed(0, 0);
	if(lidar.lidarGetFitLine(0,359,-1.0,3.0,&align_angle,&distance,isLeft,true))
	{
		lidar.alignAngle(static_cast<float>(align_angle));
	}
}

//IAction* ActionAlign::setNextAction() {
//	return new ActionOpenSlam;
//}
