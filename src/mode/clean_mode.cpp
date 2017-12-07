//
// Created by lsy563193 on 17-12-3.
//

#include "pp.h"
#include "arch.hpp"

boost::shared_ptr<State> ACleanMode::sp_state_ = nullptr;

bool ACleanMode::isFinish() {
	ROS_INFO("%s,%s,%d",__FILE__,__FUNCTION__, __LINE__);
	if(sp_state_ == nullptr)
		sp_state_.reset(getNextState());

	if(sp_state_->isFinish(this))
	{
		ROS_INFO("%s,%s,%d",__FILE__,__FUNCTION__, __LINE__);
		sp_state_.reset(getNextState());
		ROS_INFO("%s,%s,%d",__FILE__,__FUNCTION__, __LINE__);
		if(sp_state_ == nullptr)
			return true;
	}
	ROS_INFO("%s,%s,%d",__FILE__,__FUNCTION__, __LINE__);
	return false;
}
