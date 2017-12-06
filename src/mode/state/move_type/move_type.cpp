//
// Created by lsy563193 on 12/4/17.
//

//#include "pp.h"
#include "arch.hpp"

boost::shared_ptr<IAction> IMoveType::sp_movement_ = nullptr;

bool IMoveType::isFinish() {
	ROS_INFO("%s,%d",__FUNCTION__, __LINE__);
	if(sp_movement_ == nullptr || sp_movement_->isFinish())
	{
		ROS_INFO("%s,%d",__FUNCTION__, __LINE__);
		sp_movement_.reset(getNextAction());
		if(sp_movement_ == nullptr)
			return true;
	}
	ROS_INFO("%s,%d",__FUNCTION__, __LINE__);
	return false;
}

