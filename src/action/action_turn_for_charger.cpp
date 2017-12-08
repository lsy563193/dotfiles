//
// Created by austin on 17-12-7.
//

#include "arch.hpp"
#include "dev.h"

ActionTurnForCharger::ActionTurnForCharger()
{
	led.set_mode(LED_STEADY, LED_ORANGE);
	start_turning_time_stamp_ = time(NULL);
	ROS_INFO("%s %d: Start turn for charger action.", __FUNCTION__, __LINE__);
}

ActionTurnForCharger::~ActionTurnForCharger()
{
	ROS_INFO("%s %d: End turn for charger action.", __FUNCTION__, __LINE__);
}

bool ActionTurnForCharger::isFinish()
{
	return charger.getChargeStatus() || (difftime(time(NULL), start_turning_time_stamp_) > 3);
}

void ActionTurnForCharger::run()
{
	// todo: Add turning movement to this function.
	//Turn left

	//Turn right

}

