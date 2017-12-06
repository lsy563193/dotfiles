//
// Created by austin on 17-12-3.
//

#include <event_manager.h>
#include <pp.h>
#include "arch.hpp"

CleanModeNav::CleanModeNav()
{
	register_events();
//	setMode(this);
	ROS_INFO("%s %d:this(%d)", __FUNCTION__, __LINE__,this);
	sp_action_.reset(new ActionOpenGyro());
	ROS_INFO("%s %d:", __FUNCTION__, __LINE__);
	action_i_ = ac_open_gyro;
	ROS_INFO("%s %d:this(%d)", __FUNCTION__, __LINE__,this);
//	sp_action_->registerMode(this);
//	ROS_INFO("%s %d:this(%d)", __FUNCTION__, __LINE__,this);
}

IAction *CleanModeNav::getNextAction() {
	if(action_i_ == ac_open_gyro) {
		ROS_INFO("%s,%d", __FUNCTION__, __LINE__);
		if (charger.isOnStub()) {
			action_i_ = ac_back_form_charger;
			return new ActionBackFromCharger;
		}
		else {
			action_i_ = ac_open_lidar;
			return new ActionOpenLidar;
		}
	}
	else if(action_i_ == ac_back_form_charger) {
		ROS_INFO("%s,%d", __FUNCTION__, __LINE__);
		action_i_ = ac_open_lidar;
		return new ActionOpenLidar;
	}
	else if(action_i_ == ac_open_lidar) {
		ROS_INFO("%s,%d", __FUNCTION__, __LINE__);
		action_i_ = ac_align;
		return new ActionAlign;
	}
	else if(action_i_ == ac_align)
	{
		ROS_INFO("%s,%d",__FUNCTION__, __LINE__);
		action_i_ = ac_open_slam;
		return new ActionOpenSlam;
	}
	else if(action_i_ == ac_open_slam) {
		if (isFinish())
			return nullptr;

		return sp_action_.get();
	}
	return nullptr;
}

State *CleanModeNav::getNextState() {
	state_i_ = ac_null;
	return nullptr;
}

void CleanModeNav::register_events(void)
{
	event_manager_register_handler(this);
	event_manager_set_enable(true);
}

