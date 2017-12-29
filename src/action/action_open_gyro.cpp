//
// Created by lsy563193 on 11/29/17.
//

#include "pp.h"
#include "arch.hpp"

//IAction::IAction(Mode* p_mode) {
//	sp_mode_ = p_mode;
//}

ActionOpenGyro::ActionOpenGyro()
{
	ROS_INFO("%s %d: Enter action open gyro.", __FUNCTION__, __LINE__);
	gyro.reOpen();
}

bool ActionOpenGyro::isFinish()
{
	if (gyro.isOn())
		ROS_INFO("%s %d: Open gyro succeeded.", __FUNCTION__, __LINE__);

	return (gyro.isOn());
}

void ActionOpenGyro::run()
{
	wheel.setPidTargetSpeed(0, 0);
	gyro.waitForOn();
}
