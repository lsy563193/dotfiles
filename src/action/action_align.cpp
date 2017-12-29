//
// Created by lsy563193 on 11/30/17.
//
#include "pp.h"
#include "arch.hpp"

ActionAlign::ActionAlign() {
	wheel.stop();

	lidar.startAlign();
//	ROS_INFO("%s,%d:add ActionOpenLidar,sp_action_(%d)",__FUNCTION__, __LINE__,sp_action_);

	timeout_interval_ = 3;
}

bool ActionAlign::isFinish()
{
	if (isTimeUp())
		return true;

	if (lidar.alignFinish())
	{
		float align_angle = lidar.alignAngle();
		align_angle += (float) (LIDAR_THETA / 10);
		odom.setAngleOffset(-align_angle);
		ROS_INFO("%s %d: align_angle angle (%f).", __FUNCTION__, __LINE__, align_angle);
		return true;
	}
	return false;
}

void ActionAlign::run() {
	wheel.setPidTargetSpeed(0, 0);
	std::vector<LineABC> lines;
	float align_angle = 0.0;
	if (lidar.findLines(&lines, true)) {
		if (lidar.getAlignAngle(&lines, &align_angle))
			lidar.alignAngle(align_angle);
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
