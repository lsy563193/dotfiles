//
// Created by lsy563193 on 12/4/17.
//
#include "pp.h"
#include "state.hpp"

StateSelfCheck::StateSelfCheck() {

}

bool StateSelfCheck::isFinish() {
	return false;
}

State *StateSelfCheck::setNextState() {
	return State::setNextState();
}
