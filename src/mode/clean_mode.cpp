//
// Created by lsy563193 on 17-12-3.
//

#include "pp.h"
#include "arch.hpp"

boost::shared_ptr<State> ACleanMode::sp_state_ = nullptr;

bool ACleanMode::isFinish() {
	if(sp_state_ == nullptr)
	{
//		sp_state_->registerMode(this);
//		sp_state_.reset(new StateClean());
	}

	if(sp_state_->isFinish())
	{
		sp_state_.reset(getNextState());
		if(sp_state_ == nullptr)
			return true;
	}
	return false;
}
