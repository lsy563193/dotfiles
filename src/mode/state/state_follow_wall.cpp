//
// Created by lsy563193 on 12/4/17.
//

#include <mode.hpp>

#include <robot_timer.h>
#include <water_tank.hpp>
#include "key_led.h"

void StateFolllowWall::init() {
	if(water_tank.checkEquipment(false))
		water_tank.open(WaterTank::tank_pump);
	robot_timer.initTrapTimer();
	key_led.setMode(LED_STEADY, getMode()->isExpMode() ? LED_ORANGE : LED_GREEN);
	ROS_INFO("%s %d: Enter state follow wall.", __FUNCTION__, __LINE__);
}

//bool StateFolllowWall::isFinish() {
//	return false;
//}

