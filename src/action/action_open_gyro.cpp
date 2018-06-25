//
// Created by lsy563193 on 11/29/17.
//

#include "dev.h"
#include "action.hpp"

//IAction::IAction(Mode* p_mode) {
//	sp_mode_ = p_mode;
//}

ActionOpenGyro::ActionOpenGyro()
{
	ROS_WARN("%s %d: Enter.", __FUNCTION__, __LINE__);
	gyro.reOpen();
}

ActionOpenGyro::~ActionOpenGyro()
{
	ROS_WARN("%s %d: Exit.", __FUNCTION__, __LINE__);
}

bool ActionOpenGyro::isFinish()
{
	if (gyro.isOn())
	{
		ROS_WARN("%s %d: Open gyro succeeded.", __FUNCTION__, __LINE__);
		gyro.setAccInitData();
		gyro.setAngleVOffset();
		gyro.setAngleROffset();
		gyro.resetKalmanParam();
		gyro.setTiltCheckingEnable(true);
		if(gyro.getErrorCount() >= set_brush_stop_count_) // Brush should be reopened if gyro opens succeed after it opens failed above 4 times
			brush.slowOperate();
		return true;
	}
	return false;
}

void ActionOpenGyro::run()
{
	wheel.setPidTargetSpeed(0, 0);
	gyro.waitForOn();

	auto gyro_error_count = gyro.getErrorCount();
	if(gyro_error_count == set_brush_slow_count_ && !had_set_brush_slow_)
	{
		ROS_WARN("%s,%d: Open gyro failed 2 times, set brush to slow");
		brush.slowOperate();
		had_set_brush_slow_ = true;
	} else if(gyro_error_count == set_brush_stop_count_ && !had_set_brush_stop_)
	{
		brush.stop();
		had_set_brush_stop_ = true;
		ROS_WARN("%s,%d: Open gyro failed 4 times, set brush to stop");
	}
}

