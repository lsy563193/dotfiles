//
// Created by lsy563193 on 11/30/17.
//

#include "pp.h"
#include "arch.hpp"

ActionOpenSlam::ActionOpenSlam() {
	if (!(g_is_manual_pause || g_resume_cleaning)) {
		robot::instance()->setTfReady(false);
		robot::instance()->setBaselinkFrameType(Map_Position_Map_Angle);
		slam.start();
	}
	else
		robot::instance()->setBaselinkFrameType(Map_Position_Map_Angle);
}

bool ActionOpenSlam::isFinish(){
	return slam.isMapReady() && robot::instance()->isTfReady();
}

void ActionOpenSlam::run() {
//	printf("\n gyro.waitForOn()\n");
//	wheel.setPidTargetSpeed(0, 0);
}

//IAction* ActionOpenSlam::getNextAction() {
//	return new ActionOpenGyroNav;
//}


