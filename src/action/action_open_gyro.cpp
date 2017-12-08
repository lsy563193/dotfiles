//
// Created by lsy563193 on 11/29/17.
//

#include "pp.h"
#include "arch.hpp"

//IAction::IAction(Mode* p_mode) {
//	sp_mode_ = p_mode;
//}

ActionOpenGyro::ActionOpenGyro(){

	// Operate on gyro.
	gyro.setOff();
	usleep(30000);
	gyro.reOpen();

}

bool ActionOpenGyro::isFinish(){
	ROS_INFO("%s,%d", __FUNCTION__, __LINE__);
	return (gyro.isOn());
}

void ActionOpenGyro::run() {
	ROS_INFO("%s,%d", __FUNCTION__, __LINE__);
	wheel.setPidTargetSpeed(0, 0);
	gyro.waitForOn();
}

//ActionOpenGyroNav::ActionOpenGyroNav() {

//}
