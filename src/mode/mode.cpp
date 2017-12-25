//
// Created by lsy563193 on 12/4/17.
//

#include <pp.h>
#include <arch.hpp>

boost::shared_ptr<IAction> Mode::sp_action_ = nullptr;
//IAction* Mode::sp_action_ = nullptr;

void Mode::run()
{
	PP_INFO();
	bool eh_status_now = false, eh_status_last = false;

	while (ros::ok())
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

