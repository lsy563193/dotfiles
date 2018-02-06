//
// Created by lsy563193 on 12/4/17.
//

#include <mode.hpp>

#include <robot_timer.h>
#include <robot.hpp>
#include "key_led.h"

void StateFolllowWall::init() {
	robot_timer.initTrapTimer();
	key_led.setMode(LED_FLASH, LED_GREEN, 300);
}

//bool StateFolllowWall::isFinish() {
//	return false;
//}

