//
// Created by lsy563193 on 11/30/17.
//
#include "pp.h"
#include "arch.hpp"

ActionAlign::ActionAlign() {
	wheel.stop();

	vacuum.setMode(Vac_Save);
	brush.setSidePwm(50, 50);
	brush.setMainPwm(30);

	lidar.startAlign();
//	ROS_INFO("%s,%d:add ActionOpenLidar,sp_action_(%d)",__FUNCTION__, __LINE__,sp_action_);

}

bool ActionAlign::isFinish() {
	if (lidar.alignTimeOut()) {
		return true;
	}
	if (lidar.alignFinish()) {
		float align_angle = lidar.alignAngle();
		align_angle += (float) (LIDAR_THETA / 10);
		odom.setAngleOffset(align_angle);
		ROS_INFO("%s %d: align_angle angle (%f).", __FUNCTION__, __LINE__, align_angle);
		return true;
	}
	return false;
}

void ActionAlign::run() {
	wheel.setPidTargetSpeed(0, 0);
	std::vector<LineABC> lines;
	float align_angle = 0.0;
	if (lidar.findLines(&lines, true)) {
		if (lidar.getAlignAngle(&lines, &align_angle))
			lidar.alignAngle(align_angle);
	}
}

//IAction* ActionAlign::setNextAction() {
//	return new ActionOpenSlam;
//}
