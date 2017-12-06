//
// Created by austin on 17-12-3.
//

#include <event_manager.h>
#include <pp.h>
#include "arch.hpp"

CleanModeNav::CleanModeNav()
{
	event_manager_register_handler(this);
//	IAction::setMode(this);
	ROS_INFO("%s %d:this(%d)", __FUNCTION__, __LINE__,this);
	IAction::setActionIndex(IAction::ac_open_gyro);
	ROS_INFO("%s %d:this(%d)", __FUNCTION__, __LINE__,this);
	sp_action_.reset(new ActionOpenGyro());
	ROS_INFO("%s %d:", __FUNCTION__, __LINE__);
	sp_action_->registerMode(this);
	ROS_INFO("%s %d:this(%d)", __FUNCTION__, __LINE__,this);
}

IAction* CleanModeNav::getNextActionOpenGyro() {

	ROS_INFO("%s,%d", __FUNCTION__, __LINE__);
	if (charger.isOnStub()) {
		IAction::setActionIndex(IAction::ac_back_form_charger);
		return new ActionBackFromCharger;
	}
	else {
		IAction::setActionIndex(IAction::ac_open_lidar);
		return new ActionOpenLidar;
	}
}

IAction *CleanModeNav::getNextActionOpenSlam() {

	if(isFinish())
		return nullptr;

	return sp_action_.get();
}
