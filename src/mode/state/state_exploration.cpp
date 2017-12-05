//
// Created by lsy563193 on 12/4/17.
//
#include "pp.h"
#include "state.hpp"

StateExploration::StateExploration() {

}

bool StateExploration::isFinish() {
	return false;
}

State *StateExploration::setNextState() {
	return State::setNextState();
}

