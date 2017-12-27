//
// Created by lsy563193 on 11/29/17.
//

#include "pp.h"
#include "arch.hpp"
Odom charger_pose;
ActionBackFromCharger::ActionBackFromCharger()
{
	path_set_home(nav_map.getCurrPoint());

	vacuum.setMode(Vac_Normal, false);
	brush.normalOperate();
	wheel.setDirBackward();
	charger_pose.setX(odom.getX());
	charger_pose.setY(odom.getY());
};

bool ActionBackFromCharger::isFinish() {
	return  (two_points_distance_double(charger_pose.getX(), charger_pose.getY(), odom.getX(), odom.getY()) > 0.5);
}

void ActionBackFromCharger::run() {
	wheel.setPidTargetSpeed(20, 20);
}

//IAction* ActionBackFromCharger::setNextAction() {
//	return new ActionOpenLidar;
//}
