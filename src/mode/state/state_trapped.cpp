//
// Created by lsy563193 on 12/4/17.
//
#include "pp.h"
#include "state.hpp"

StateTrapped::StateTrapped() {

}

bool StateTrapped::isFinish() {
	return false;
}

State *StateTrapped::setNextState() {
	return State::setNextState();
}
