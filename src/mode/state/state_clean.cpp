//
// Created by lsy563193 on 12/4/17.
//

#include "pp.h"
#include "arch.hpp"

StateClean::StateClean() {
	led.set_mode(LED_STEADY, LED_GREEN);
//	sp_move_type_.reset(new IMoveType);
}

//bool StateClean::isFinish() {
//	return path_next_nav(sp_action_->, path);
//}

State *StateClean::setNextState() {
//	auto path = sp_mode_->generatePath(nav_map,nav_map.getCurrCell(),g_old_dir);
//	if(path.empty()) {
//		ROS_INFO("%s%d:", __FUNCTION__, __LINE__);
//		if (sp_mode_->checkTrapped(nav_map, nav_map.getCurrCell())) {
//			return new StateTrapped;
//		}
//		else {
//			return new StateGoHomePoint;
//		}
//	}
	return this;

}

