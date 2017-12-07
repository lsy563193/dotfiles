//
// Created by austin on 17-12-7.
//

#include "arch.hpp"
#include "dev.h"

ActionTurnForCharger::ActionTurnForCharger()
{

}

bool ActionTurnForCharger::isFinish()
{
	return charger.getChargeStatus();
}

void ActionTurnForCharger::run()
{
	//Turn left

	//Turn right

}

