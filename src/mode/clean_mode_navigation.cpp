//
// Created by austin on 17-12-3.
//

#include <event_manager.h>
#include <pp.h>
#include <mode/mode.hpp>
#include "ros/ros.h"

CleanModeNav::CleanModeNav()
{
	event_manager_register_handler(this);
//	IAction::setMode(this);
	IAction::setNext(IAction::ac_open_gyro);
	sp_action_.reset(new ActionOpenGyro(this));
//	ROS_INFO("%s %d:", __FUNCTION__, __LINE__);
	ROS_INFO("%s %d:this(%d)", __FUNCTION__, __LINE__,this);
}

bool CleanModeNav::updateAction() {
	if(sp_state_ != nullptr)
	{

	}
	else{
		Mode::updateAction();
	}
}

IAction* CleanModeNav::getNextActionOpenGyro() {

	ROS_INFO("%s,%d", __FUNCTION__, __LINE__);
	if (charger.isOnStub()) {
		IAction::setNext(IAction::ac_back_form_charger);
		return new ActionBackFromCharger;
	}
	else {
		IAction::setNext(IAction::ac_open_lidar);
		return new ActionOpenLidar;
	}
}
