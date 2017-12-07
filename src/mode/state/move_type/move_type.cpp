//
// Created by lsy563193 on 12/4/17.
//

//#include "pp.h"
#include "arch.hpp"

boost::shared_ptr<IAction> IMoveType::sp_movement_ = nullptr;

bool IMoveType::isFinish(ACleanMode* p_mode_) {
	ROS_INFO("%s,%s,%d",__FILE__,__FUNCTION__, __LINE__);
	if(sp_movement_ == nullptr)
		sp_movement_.reset(p_mode_->getNextMovement());

	if(sp_movement_->isFinish())
	{
		ROS_INFO("%s,%s,%d",__FILE__,__FUNCTION__, __LINE__);
		sp_movement_.reset(p_mode_->getNextMovement());
		if(sp_movement_ == nullptr)
			return true;
	}
	ROS_INFO("%s,%s,%d",__FILE__,__FUNCTION__, __LINE__);
	return false;
}

