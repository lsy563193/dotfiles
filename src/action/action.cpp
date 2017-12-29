//
// Created by lsy563193 on 11/29/17.
//

#include "pp.h"
#include "arch.hpp"

//static uint32_t IAction::g_wf_start_timer = 0;
//static uint32_t IAction::g_wf_diff_timer = 0;

IAction::IAction():start_timer_(ros::Time::now().toSec()) {

}

bool IAction::isTimeUp()
{
	if (timeout_interval_ == 0)
		return false;
//	PP_INFO();
//	ROS_INFO("tramp(%f),interval(%f)",ros::Time::now().toSec() - start_timer_,timeout_interval_);
	return (ros::Time::now().toSec() - start_timer_ > timeout_interval_);
}

bool IAction::isExit()
{
	return false;
}

