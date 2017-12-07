//
// Created by lsy563193 on 12/4/17.
//
#include <arch.hpp>

extern MapDirection g_old_dir;
//int State::action_i_= st_null;

bool State::isFinish(ACleanMode* p_mode, IMoveType* p_move_type,IAction* p_action, int& action_i) {
	PP_INFO();
	if(p_move_type == nullptr)
		p_move_type = p_mode->getNextMoveType(nav_map.getCurrCell(),g_old_dir);
	if(p_move_type->isFinish(p_mode, p_action, action_i))
	{
		PP_INFO();
		p_move_type = p_mode->getNextMoveType(nav_map.getCurrCell(),g_old_dir);
		if(p_move_type == nullptr)
			return true;
	}
	PP_INFO();
	return false;
}
