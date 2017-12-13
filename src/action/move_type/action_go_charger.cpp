//
// Created by lsy563193 on 12/4/17.
//
#include "pp.h"
#include "arch.hpp"

//bool ActionGoCharger::isFinish() {
//	return false;
//}

//IAction *ActionGoCharger::setNextAction() {
//	return nullptr;
//}

ActionGoCharger::ActionGoCharger() {
	ROS_INFO("%s,%d: mt_is_go_to_charger", __FUNCTION__, __LINE__);
	turn_target_angle_ = ranged_angle(robot::instance()->getPoseAngle());
}
