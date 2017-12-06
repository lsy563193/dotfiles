//
// Created by lsy563193 on 12/4/17.
//

//#include "pp.h"
#include "arch.hpp"

boost::shared_ptr<IAction> IMoveType::sp_movement_ = nullptr;

bool IMoveType::isFinish() {
	if(sp_movement_ == nullptr)
	{
//		sp_movement_->registerMode(sp_mode_);
//		sp_movement_.reset(new MovementForward);
	}
	if(sp_movement_->isFinish())
	{
		sp_movement_.reset(getNextAction());
		if(sp_movement_ == nullptr)
			return true;
	}
	return false;
}

