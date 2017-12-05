//
// Created by lsy563193 on 12/4/17.
//

#include "pp.h"
#include "state.hpp"

StateClean::StateClean() {
	led.set_mode(LED_STEADY, LED_GREEN);
//	p_mt_.reset(new IMoveType);
}

bool StateClean::isFinish() {
//	return path_next_nav(start, path);
}

State *StateClean::setNextState() {
//	ROS_INFO("%s%d:", __FUNCTION__, __LINE__);
//	if (isTrapped()) {
//		return new StateTrapped;
////		path.push_back(g_virtual_target);
//	}
//	else {
//		return new StateGoHomePoint;
////		setNext(CS_GO_HOME_POINT);
////		return true;
//	}
}

