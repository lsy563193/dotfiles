//
// Created by lsy563193 on 11/29/17.
//

#include "pp.h"
#include "arch.hpp"

//static uint32_t IAction::g_wf_start_timer = 0;
//static uint32_t IAction::g_wf_diff_timer = 0;

bool IAction::isTimeUp()
{
	if (interval_ == 0)
		return false;
	return (ros::Time::now().toSec() - start_timer_ > interval_);
}

bool IAction::isExit()
{
	return false;
}
