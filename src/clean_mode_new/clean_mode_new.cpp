//
// Created by austin on 17-12-3.
//

#include "ros/ros.h"
#include "event_manager.h"
#include "clean_mode_new/clean_mode_new.hpp"

CleanModeNew* CleanModeNew::run()
{
	ROS_INFO("%s %d:", __FUNCTION__, __LINE__);
	bool eh_status_now = false, eh_status_last = false;

	while (ros::ok())
	{
		ROS_INFO("%s %d:", __FUNCTION__, __LINE__);
		if (event_manager_check_event(&eh_status_now, &eh_status_last) == 1) {
			usleep(100);
			continue;
		}

		ROS_INFO("%s %d:", __FUNCTION__, __LINE__);
		if (eventHandle(&sp_next_clean_mode_, &sp_path_algorithm_, &sp_action_))
			return sp_next_clean_mode_;
		ROS_INFO("%s %d:", __FUNCTION__, __LINE__);
		if (updatePath(&sp_next_clean_mode_, &sp_path_algorithm_, &sp_action_))
			return sp_next_clean_mode_;
		ROS_INFO("%s %d:", __FUNCTION__, __LINE__);
		if (updateAction(&sp_next_clean_mode_, &sp_path_algorithm_, &sp_action_))
			return sp_next_clean_mode_;

		ROS_INFO("%s %d:", __FUNCTION__, __LINE__);
		sp_action_->action();
		ROS_INFO("%s %d:", __FUNCTION__, __LINE__);
	}
}

