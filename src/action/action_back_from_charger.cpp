//
// Created by lsy563193 on 11/29/17.
//

#include "dev.h"
#include "action.hpp"
Odom charger_pose;
ActionBackFromCharger::ActionBackFromCharger()
{
	ROS_INFO("%s %d, Init action back from charger.", __FUNCTION__, __LINE__);
//	path_set_home(nav_map.getCurrCell());

	vacuum.setTmpMode(Vac_Normal);
	brush.normalOperate();
	wheel.setDirectionBackward();
	charger_pose.setX(odom.getX());
	charger_pose.setY(odom.getY());
};

ActionBackFromCharger::~ActionBackFromCharger()
{
	ROS_INFO("%s %d, Finish action back from charger.", __FUNCTION__, __LINE__);
}

bool ActionBackFromCharger::isFinish() {
	return  (two_points_distance_double(charger_pose.getX(), charger_pose.getY(), odom.getX(), odom.getY()) > 0.5);
}

void ActionBackFromCharger::run() {
	wheel.setPidTargetSpeed(20, 20);
}

//IAction* ActionBackFromCharger::setNextAction() {
//	return new ActionOpenLidar;
//}
