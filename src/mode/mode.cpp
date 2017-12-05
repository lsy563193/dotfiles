//
// Created by lsy563193 on 12/4/17.
//

#include <pp.h>
#include "action.hpp"
#include <mode/mode.hpp>

boost::shared_ptr<IAction> Mode::sp_action_ = nullptr;

Mode *Mode::run() {
	ROS_INFO("%s %d:this(%d)", __FUNCTION__, __LINE__,this);
	bool eh_status_now = false, eh_status_last = false;

	while (ros::ok())
	{
		ROS_INFO("%s %d:", __FUNCTION__, __LINE__);
		if (event_manager_check_event(&eh_status_now, &eh_status_last) == 1) {
			usleep(100);
			continue;
		}

		ROS_INFO("%s %d:this(%d)", __FUNCTION__, __LINE__,this);
		if (isExit())
		{
			ROS_INFO("%s %d:", __FUNCTION__, __LINE__);
			return p_next_clean_mode_;
		}
		ROS_INFO("%s %d:", __FUNCTION__, __LINE__);
		if (! updateAction())
			return p_next_clean_mode_;

		ROS_INFO("%s %d:", __FUNCTION__, __LINE__);
		sp_action_->run();
		ROS_INFO("%s %d:", __FUNCTION__, __LINE__);
	}
}

bool Mode::isExit()
{
	ROS_INFO("%s %d:", __FUNCTION__, __LINE__);
	return false;
}

bool Mode::updateAction() {
	if (sp_action_->isFinish()) {
		ROS_INFO("%s,%d",__FUNCTION__, __LINE__);
		sp_action_.reset(sp_action_->getNextAction());
		return sp_action_ != nullptr;
	}
	return true;
}

IAction* Mode::getNextActionOpenGyro()
{
	ROS_INFO("%s,%d",__FUNCTION__, __LINE__);
	IAction::setNext(IAction::ac_null);
	return nullptr;
}

IAction *Mode::getNextActionBackFromCharger() {
	ROS_INFO("%s,%d",__FUNCTION__, __LINE__);
	IAction::setNext(IAction::ac_open_lidar);
	return new ActionOpenLidar;
}

IAction *Mode::getNextActionOpenLidar() {
	ROS_INFO("%s,%d",__FUNCTION__, __LINE__);
	IAction::setNext(IAction::ac_align);
	return new ActionAlign;
}

IAction *Mode::getNextActionAlign() {
	ROS_INFO("%s,%d",__FUNCTION__, __LINE__);
	IAction::setNext(IAction::ac_open_slam);
	return new ActionOpenSlam;
}

IAction *Mode::getNextActionOpenSlam() {
	ROS_INFO("%s,%d",__FUNCTION__, __LINE__);
	IAction::setNext(IAction::ac_null);
	return nullptr ;
}

//IAction *Mode::get() {
//	return new ActionOpenLidar;
//}
