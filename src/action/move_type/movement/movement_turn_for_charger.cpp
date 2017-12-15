//
// Created by austin on 17-12-7.
//

#include "pp.h"
#include "arch.hpp"
#include "dev.h"

MovementTurnForCharger::MovementTurnForCharger()
{
	led.set_mode(LED_STEADY, LED_ORANGE);
	start_turning_time_stamp_ = ros::Time::now().toSec();
	turn_right_finish_ = false;
	ROS_INFO("%s %d: Start movement turn for charger.", __FUNCTION__, __LINE__);
}

MovementTurnForCharger::~MovementTurnForCharger()
{
	wheel.stop();
	ROS_INFO("%s %d: End movement turn for charger.", __FUNCTION__, __LINE__);
}

bool MovementTurnForCharger::isFinish()
{
	return charger.getChargeStatus() || (ros::Time::now().toSec() - start_turning_time_stamp_ > 3);
}

void MovementTurnForCharger::adjustSpeed(int32_t &left_speed, int32_t &right_speed)
{
	if (ros::Time::now().toSec() - start_turning_time_stamp_ > 1)
		turn_right_finish_ = true;

	if (turn_right_finish_)
	{
		wheel.setDirectionRight();
		left_speed = right_speed = 5;
	}
	else
	{
		wheel.setDirectionLeft();
		left_speed = right_speed = 5;
	}
}
