//
// Created by lsy563193 on 12/4/17.
//
#include <arch.hpp>

extern MapDirection g_old_dir;
//int State::action_i_= st_null;
boost::shared_ptr<IMoveType> State::sp_move_type_ = nullptr;

bool State::isFinish(ACleanMode* p_mode) {
	ROS_INFO("%s,%s,%d",__FILE__,__FUNCTION__, __LINE__);
	if(sp_move_type_ == nullptr)
		sp_move_type_.reset(p_mode->getNextMoveType(nav_map.getCurrCell(),g_old_dir));
	if(sp_move_type_->isFinish(p_mode))
	{
		ROS_INFO("%s,%s,%d",__FILE__,__FUNCTION__, __LINE__);
		sp_move_type_.reset(p_mode->getNextMoveType(nav_map.getCurrCell(),g_old_dir));
		if(sp_move_type_ == nullptr)
			return true;
	}
	ROS_INFO("%s,%s,%d",__FILE__,__FUNCTION__, __LINE__);
	return false;
}
