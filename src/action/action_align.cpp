//
// Created by lsy563193 on 11/30/17.
//
#include "dev.h"
#include "action.hpp"

ActionAlign::ActionAlign() {
	timeout_interval_ = 5;
	wait_laser_timer_ = 1.0;
	ROS_WARN("%s %d: Start action align, timeout(%f)s.",__FUNCTION__, __LINE__, timeout_interval_);

	wheel.stop();
	lidar.startAlign();
}

bool ActionAlign::isFinish()
{
	if (isTimeUp())
		return true;

	if (lidar.isAlignFinish())
	{
		float align_radian = lidar.alignRadian();
		odom.setRadianOffset(-align_radian);
//		ROS_ERROR("!!!!!!!!!!!!!!!!!odom rad offset: %f", odom.getRadianOffset());
//		ROS_INFO("%s %d: align_radian angle (%f).", __FUNCTION__, __LINE__, radian_to_degree(align_radian));
		return true;
	}

	return false;
}

void ActionAlign::run()
{
	wheel.setPidTargetSpeed(0, 0);
	if (ros::Time::now().toSec() - start_timer_ <= wait_laser_timer_) {
//		ROS_WARN("%s %d: Waiting for stable laser", __FUNCTION__, __LINE__);
		return;
	}

	if(!lidar.isAlignFinish() &&
					lidar.getFitLine(degree_to_radian(0), degree_to_radian(359), -1.0, 3.0, &align_radian, &distance, isLeft,
													 true))
	{
		lidar.alignRadian(static_cast<float>(ranged_radian(align_radian)));
		lidar.setAlignFinish();
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