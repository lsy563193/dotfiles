//
// Created by austin on 17-12-3.
//

#include <event_manager.h>
#include "ros/ros.h"
#include "clean_mode_new/clean_mode_navigation_new.hpp"
#include "action_new/action_open_gyro_new.hpp"

NavigationModeNew::NavigationModeNew()
{
	sp_action_ = new NavOpenGyroAction();
	ROS_INFO("%s %d:", __FUNCTION__, __LINE__);
}

bool NavigationModeNew::eventHandle(CleanModeNew **p_sp_next_clean_mode_, PathAlgorithm **p_sp_path_, Action **p_sp_action_)
{
	if (ev.key_clean_pressed)
	{
		ROS_INFO("%s %d: update mode", __FUNCTION__, __LINE__);
		*p_sp_next_clean_mode_ = new NavigationModeNew();
		ev.key_clean_pressed = false;
		return true;
	}

	return false;
}

bool NavigationModeNew::updateAction(CleanModeNew **p_sp_next_clean_mode_, PathAlgorithm **p_sp_path_, Action **p_sp_action_)
{
	if ((*p_sp_action_)->isFinishAndUpdateAction())
	{
		ROS_INFO("%s %d: finish", __FUNCTION__, __LINE__);
		Action* new_action = (*p_sp_action_)->getNextAction();
		delete *p_sp_action_;
		*p_sp_action_ = new_action;
	}
	return false;
}

bool NavigationModeNew::updatePath(CleanModeNew **p_sp_next_clean_mode_, PathAlgorithm **p_sp_path_, Action **p_sp_action_)
{
	return false;
}

