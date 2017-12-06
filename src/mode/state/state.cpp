//
// Created by lsy563193 on 12/4/17.
//
#include <arch.hpp>
//int State::action_i_= st_null;
boost::shared_ptr<IMoveType> State::sp_move_type_ = nullptr;

bool State::isFinish() {
	if(sp_move_type_ == nullptr)
	{
//		sp_move_type_->registerMode(sp_mode_);
		sp_move_type_.reset(new MoveTypeLinear);
	}

	if(sp_move_type_->isFinish())
	{
		sp_move_type_.reset(getNextMoveType());
		if(sp_move_type_ == nullptr)
			return true;
	}
	return false;
}
