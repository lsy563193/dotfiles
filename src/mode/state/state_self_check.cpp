//
// Created by lsy563193 on 12/4/17.
//
#include "pp.h"
#include "arch.hpp"

StateSelfCheck::StateSelfCheck() {

}

State *StateSelfCheck::getNextState() {
//	return State::setNextState();
	return this;
}
