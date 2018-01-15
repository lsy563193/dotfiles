//
// Created by lsy563193 on 12/4/17.
//
#include <robot_timer.h>
#include "key_led.h"
#include "arch.hpp"

void StateTrapped::init() {
	robot_timer.initTrapTimer();
	led.set_mode(LED_FLASH, LED_GREEN, 300);
}

//bool StateTrapped::isFinish() {
//	return false;
//}

