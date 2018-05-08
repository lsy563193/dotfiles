//
// Created by lsy563193 on 11/30/17.
//

#include <robot.hpp>
#include <slam.h>
#include "dev.h"
#include "action.hpp"

ActionOpenSlam::ActionOpenSlam() {
	ROS_WARN("%s %d: Enter action open slam.", __FUNCTION__, __LINE__);
	robot::instance()->setTfReady(false);
}

ActionOpenSlam::~ActionOpenSlam()
{
	if (!isFinish())
		slam.stop();
}

bool ActionOpenSlam::isFinish(){
/*	 Wait for a while to make sure the scan data and odom are both updated.
	 It is important, otherwise, align will failed and slam start with a correction just as align angle.*/
	if (ros::Time::now().toSec() - start_timer_ <= wait_slam_timer_) {
//		ROS_WARN("%s %d: Waiting for slam", __FUNCTION__, __LINE__);
		return false;
	}
	if (!is_slam_start_) {
		robot::instance()->setBaselinkFrameType(SLAM_POSITION_SLAM_ANGLE);
		slam.start();
		is_slam_start_ = true;
	}
	if (slam.isMapReady() && robot::instance()->isTfReady())
	{
		// Wait a while to make sure robot has used new base link framework.
		usleep(20000);
		return true;
	}
	return false;
}

void ActionOpenSlam::run() {
//	printf("\n gyro.waitForOn()\n");
//	wheel.setPidTargetSpeed(0, 0);
}

//IAction* ActionOpenSlam::setNextAction() {
//	return new ActionOpenGyroNav;
//}


