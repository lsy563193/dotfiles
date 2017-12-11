//
// Created by lsy563193 on 11/30/17.
//

#include "pp.h"
#include "arch.hpp"

ActionOpenSlam::ActionOpenSlam() {
	PP_INFO();
	robot::instance()->setTfReady(false);
	robot::instance()->setBaselinkFrameType(Map_Position_Map_Angle);
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


