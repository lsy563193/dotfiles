//
// Created by lsy563193 on 12/4/17.
//

#include <pp.h>
#include <arch.hpp>

boost::shared_ptr<IAction> Mode::sp_action_ = nullptr;
//IAction* Mode::sp_action_ = nullptr;

void Mode::run() {
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
		if (Mode::isFinish())
		{
//			PP_INFO();
			return;
		}

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

bool Mode::isFinish() {
	if (sp_action_->isFinish()) {
		PP_INFO();
		sp_action_.reset(getNextAction());
		PP_INFO();
		if(sp_action_ == nullptr)
		{
			PP_INFO();
			return true;
		}
		PP_INFO();
	}
//	PP_INFO();
	return false;
}

//IAction *Mode::get() {
//	return new ActionOpenLidar;
//}

void Mode::setNextMode(int next_mode)
{
	next_mode_i_ = next_mode;
}

int Mode::getNextMode()
{
	return next_mode_i_;
}
