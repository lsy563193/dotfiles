//
// Created by lsy563193 on 11/29/17.
//

#include "pp.h"
#include "arch.hpp"

//IAction::IAction(Mode* p_mode) {
//	sp_mode_ = p_mode;
//}

ActionOpenGyro::ActionOpenGyro(){

	PP_INFO();
	// Operate on gyro.
	gyro.setOff();
	usleep(30000);
	gyro.reOpen();

}

bool ActionOpenGyro::isFinish(){
	return (gyro.isOn());
}

void ActionOpenGyro::run() {
	wheel.setPidTargetSpeed(0, 0);
	gyro.waitForOn();
}

//ActionOpenGyroNav::ActionOpenGyroNav() {

//}

