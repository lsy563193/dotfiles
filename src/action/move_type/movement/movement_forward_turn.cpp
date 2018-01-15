//
// Created by lsy563193 on 12/25/17.
//

#include <movement.hpp>
#include "dev.h"
MovementForwardTurn::MovementForwardTurn(bool is_left) : is_left_(is_left) {
	ROS_INFO("%s,%d",__FUNCTION__,__LINE__);
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

	if (ev.bumper_triggered || ev.cliff_triggered || ev.tilt_triggered || ev.robot_slip)
	{
		ROS_WARN("%s, %d,MovementForwardTurn, ev.bumper_triggered(\033[32m%d\033[0m) ev.cliff_triggered(\033[32m%d\033[0m) ev.tilt_triggered(\033[32m%d\033[0m) ev.robot_slip(\033[32m%d\033[0m)."
				, __FUNCTION__, __LINE__,ev.bumper_triggered,ev.cliff_triggered,ev.tilt_triggered,ev.robot_slip);
		return true;
	}

	return false;

}
