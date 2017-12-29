//
// Created by lsy563193 on 11/30/17.
//

#include "pp.h"
#include "arch.hpp"

ActionOpenSlam::ActionOpenSlam() {
	ROS_INFO("%s %d: Enter action open slam.", __FUNCTION__, __LINE__);
	robot::instance()->setTfReady(false);
	robot::instance()->setBaselinkFrameType(SLAM_POSITION_SLAM_ANGLE);
	// Wait for a while to make sure the scan data and odom are both updated.
	// It is important, otherwise, align will failed and slam start with a correction just as align angle.
	usleep(230000);
	slam.start();
}

bool ActionOpenSlam::isFinish(){
	return slam.isMapReady() && robot::instance()->isTfReady();
}

void ActionOpenSlam::run() {
//	printf("\n gyro.waitForOn()\n");
//	wheel.setPidTargetSpeed(0, 0);
}

//IAction* ActionOpenSlam::setNextAction() {
//	return new ActionOpenGyroNav;
//}


