//
// Created by lsy563193 on 12/25/17.
//

#include <movement.hpp>
#include <event_manager.h>
#include "dev.h"
MovementForwardTurn::MovementForwardTurn(bool is_left) : is_left_(is_left) {
	ROS_WARN("%s %d, Enter Movement forward turn.",__FUNCTION__,__LINE__);
}

void MovementForwardTurn::adjustSpeed(int32_t &left_speed, int32_t &right_speed)
{
	wheel.setDirectionForward();
		if (is_left_) {
		left_speed = 8;
		right_speed = 31;
	}
	else {
		left_speed = 31;
		right_speed = 8;
	}
}

bool MovementForwardTurn::isFinish()
{
		// Robot should move back for these cases.
	ev.bumper_triggered = bumper.getStatus();
	ev.cliff_triggered = cliff.getStatus();
	ev.tilt_triggered = gyro.getTiltCheckingStatus();
	ev.slip_triggered= lidar.isRobotSlip();

	if (ev.bumper_triggered || ev.cliff_triggered || ev.tilt_triggered || ev.slip_triggered)
	{
		ROS_WARN("%s, %d,MovementForwardTurn, ev.bumper_triggered(\033[32m%d\033[0m) ev.cliff_triggered(\033[32m%d\033[0m) "
											 "ev.tilt_triggered(\033[32m%d\033[0m) ev.slip_triggered(\033[32m%d\033[0m)."
				, __FUNCTION__, __LINE__,ev.bumper_triggered,ev.cliff_triggered,ev.tilt_triggered,ev.slip_triggered);
		return true;
	}

	return false;

}
