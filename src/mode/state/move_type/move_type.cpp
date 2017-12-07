//
// Created by lsy563193 on 12/4/17.
//

//#include "pp.h"
#include "arch.hpp"

//boost::shared_ptr<IAction> IMoveType::sp_movement_ = nullptr;

bool IMoveType::isFinish(ACleanMode* p_mode_,IAction* p_action,  int& action_i) {
	PP_INFO();
	if (action_i != Mode::ac_movement_turn &&
			action_i != Mode::ac_movement_back &&
			action_i != Mode::ac_movement_forward &&
			action_i != Mode::ac_movement_follow_wall
					)
		action_i = p_mode_->getNextMovement();

	if (p_action->isFinish()) {
		PP_INFO();
		action_i = p_mode_->getNextMovement();
		if (action_i == Mode::ac_null)
			return true;
	}
	PP_INFO();
	return false;
}

