//
// Created by lsy563193 on 12/4/17.
//

#include <pp.h>
#include <arch.hpp>

boost::shared_ptr<IAction> Mode::sp_action_ = nullptr;

void Mode::run() {
	ROS_INFO("%s,%s,%d",__FILE__,__FUNCTION__, __LINE__);
	bool eh_status_now = false, eh_status_last = false;

	while (ros::ok())
	{
		ROS_INFO("%s,%s,%d",__FILE__,__FUNCTION__, __LINE__);
		if (event_manager_check_event(&eh_status_now, &eh_status_last) == 1) {
			usleep(100);
			continue;
		}

		ROS_INFO("%s,%s,%d",__FILE__,__FUNCTION__, __LINE__);
		if (isExit())
		{
			ROS_INFO("%s,%s,%d",__FILE__,__FUNCTION__, __LINE__);
			return;
		}
		ROS_INFO("%s,%s,%d",__FILE__,__FUNCTION__, __LINE__);
		if (Mode::isFinish())
			return;

		ROS_INFO("%s,%s,%d",__FILE__,__FUNCTION__, __LINE__);
		sp_action_->run();
		ROS_INFO("%s,%s,%d",__FILE__,__FUNCTION__, __LINE__);
	}
}

bool Mode::isExit()
{
	ROS_INFO("%s,%s,%d",__FILE__,__FUNCTION__, __LINE__);
	return false;
}

bool Mode::isFinish() {
	if (sp_action_->isFinish()) {
		ROS_INFO("%s,%s,%d",__FILE__,__FUNCTION__, __LINE__);
		sp_action_.reset(getNextAction());
		if(sp_action_ == nullptr)
			return true;
	}
	return false;
}

//IAction *Mode::get() {
//	return new ActionOpenLidar;
//}
