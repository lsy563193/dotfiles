//
// Created by lsy563193 on 12/4/17.
//


#include "pp.h"
#include "state.hpp"


StateTmpSpot::StateTmpSpot() {

}

bool StateTmpSpot::isFinish() {
	return false;
}

State *StateTmpSpot::setNextState() {
	return State::setNextState();
}
