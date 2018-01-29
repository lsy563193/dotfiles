//
// Created by lsy563193 on 11/29/17.
//

#include "dev.h"
#include "action.hpp"
ActionBackFromCharger::ActionBackFromCharger()
{
	ROS_INFO("%s %d, Init action back from charger.", __FUNCTION__, __LINE__);

	wheel.setDirectionBackward();
	// This time out interval is just for checking whether the switch is on.
	timeout_interval_ = 1;
};

ActionBackFromCharger::~ActionBackFromCharger()
{
	ROS_INFO("%s %d, Finish action back from charger.", __FUNCTION__, __LINE__);
}

bool ActionBackFromCharger::isFinish() {
	static Vector2<float> tmp_pose(odom.getX(),odom.getY());
	const float BACK_DIST = 0.5f;
	double distance = two_points_distance_double(tmp_pose.GetX(), tmp_pose.GetY(), odom.getX(), odom.getY());
	if(distance >= BACK_DIST)
		return true;
	else
		return false;
}

void ActionBackFromCharger::run() {
	wheel.setPidTargetSpeed(20, 20);
}
