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

	lidar.motorCtrl(ON);
	lidar.setScanOriginalReady(0);
//	ROS_INFO("%s,%d:add ActionOpenLidar,sp_action_(%d)",__FUNCTION__, __LINE__,sp_action_);

}

bool ActionAlign::isFinish() {
	if (lidar.alignTimeOut()) {
		return true;
	}
	if (lidar.alignFinish()) {
		PP_INFO();
		float align_angle = lidar.alignAngle();
		PP_INFO();
		align_angle += (float) (LIDAR_THETA / 10);
		PP_INFO();
		robot::instance()->offsetAngle(align_angle);
		PP_INFO();
		g_homes[0].TH = -(int16_t) (align_angle);
		PP_INFO();
		ROS_INFO("%s %d: align_angle angle (%f), g_homes[0].TH (%d).", __FUNCTION__, __LINE__, align_angle, g_homes[0].TH);
		usleep(230000);
		PP_INFO();
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
