//
// Created by lsy563193 on 12/4/17.
//

#include <dev.h>
#include <event_manager.h>
#include "mode.hpp"
#include "robotbase.h"
boost::shared_ptr<IAction> Mode::sp_action_ = nullptr;
//IAction* Mode::sp_action_ = nullptr;

void Mode::run()
{
//	ROS_INFO("%s %d: Mode start running.", __FUNCTION__, __LINE__);
	bool eh_status_now = false, eh_status_last = false;

	while (ros::ok() && !core_thread_stop)
	{
//		PP_INFO();
		if (event_manager_check_event(&eh_status_now, &eh_status_last) == 1) {
			usleep(100);
			continue;
		}

//		PP_INFO();
		if (isExit())
		{
			PP_INFO();
			return;
		}
//		PP_INFO();
		if (isFinish())
			return;

//		PP_INFO();
		sp_action_->run();
//		PP_INFO();
	}
}

bool Mode::isExit()
{
//	ROS_INFO("%s,%s,%d",__FILE__,__FUNCTION__, __LINE__);
	return false;
}

bool Mode::isFinish()
{
	return sp_action_->isFinish();
}

void Mode::setNextMode(int next_mode)
{
	next_mode_i_ = next_mode;
}

int Mode::getNextMode()
{
	return next_mode_i_;
}

bool Mode::isExceptionTriggered()
{
	return ev.bumper_jam || ev.cliff_jam || ev.oc_wheel_left || ev.oc_wheel_right
		   || ev.oc_suction || ev.lidar_stuck || ev.robot_stuck;
}

