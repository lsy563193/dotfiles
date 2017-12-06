//
// Created by lsy563193 on 12/4/17.
//
#include <arch.hpp>
//int State::action_i_= st_null;
boost::shared_ptr<IMoveType> State::sp_move_type_ = nullptr;

bool State::isFinish() {
	ROS_INFO("%s,%d",__FUNCTION__, __LINE__);
	if(sp_move_type_ == nullptr)
		sp_move_type_.reset(getNextMoveType());
	if(sp_move_type_->isFinish())
	{
		ROS_INFO("%s,%d",__FUNCTION__, __LINE__);
		sp_move_type_.reset(getNextMoveType());
		if(sp_move_type_ == nullptr)
			return true;
	}
	return false;
}
