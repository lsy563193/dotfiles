//
// Created by lsy563193 on 11/30/17.
//
#include "dev.h"
#include "action.hpp"

ActionAlign::ActionAlign() {
	timeout_interval_ = 3;
	ROS_INFO("%s %d: Start action align, timeout(%f)s.",__FUNCTION__, __LINE__, timeout_interval_);

	wheel.stop();
	lidar.startAlign();
}

bool ActionAlign::isFinish()
{
	if (isTimeUp())
		return true;

	if (lidar.alignFinish())
	{
		float align_angle = lidar.alignAngle();
		odom.setRadianOffset(-align_angle);
		ROS_INFO("%s %d: align_angle angle (%f).", __FUNCTION__, __LINE__, align_angle);
		return true;
	}
	return false;
}

void ActionAlign::run() {
	wheel.setPidTargetSpeed(0, 0);
	if(lidar.lidarGetFitLine(degree_to_radian(0), degree_to_radian(359), -1.0, 3.0, &align_angle, &distance, isLeft, true))
	{
		lidar.alignAngle(static_cast<float>(align_angle));
	}
}

bool ActionAlign::isTimeUp()
{
	if (IAction::isTimeUp())
	{
		ROS_WARN("%s %d: Align timeout", __FUNCTION__, __LINE__);
		return true;
	}

	return false;
}

//IAction* ActionAlign::setNextAction() {
//	return new ActionOpenSlam;
//}
