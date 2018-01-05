//
// Created by lsy563193 on 12/4/17.
//
#include "pp.h"
#include "arch.hpp"

void StateTrapped::init() {
	robot_timer.initTrapTimer();
	led.set_mode(LED_FLASH, LED_GREEN, 300);
}

//bool StateTrapped::isFinish() {
//	return false;
//}

