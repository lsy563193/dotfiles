//
// Created by lsy563193 on 12/4/17.
//

#include <mode.hpp>

#include <robot_timer.h>
#include "key_led.h"

void StateTrapped::init() {
	robot_timer.initTrapTimer();
	led.setMode(LED_FLASH, LED_GREEN, 300);
}

//bool StateTrapped::isFinish() {
//	return false;
//}

