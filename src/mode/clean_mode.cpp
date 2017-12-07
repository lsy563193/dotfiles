//
// Created by lsy563193 on 17-12-3.
//

#include "pp.h"
#include "arch.hpp"

boost::shared_ptr<State> ACleanMode::sp_state_ = nullptr;
boost::shared_ptr<IMoveType> ACleanMode::sp_move_type_ = nullptr;
//boost::shared_ptr<IMovement> ACleanMode::sp_movement_ = nullptr;

bool ACleanMode::isFinish() {
	PP_INFO();
	if(sp_state_ == nullptr)
		sp_state_.reset(getNextState());

	if(sp_state_->isFinish(this,sp_move_type_.get(),sp_action_.get() ,action_i_))
	{
		PP_INFO();
		sp_state_.reset(getNextState());
		PP_INFO();
		if(sp_state_ == nullptr)
			return true;
	}
	PP_INFO();
	return false;
}
