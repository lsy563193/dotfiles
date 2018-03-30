//
// Created by lsy563193 on 1/2/18.
//

#include <mode.hpp>
#include <robot.hpp>

#include "key_led.h"

void StateInit::init() {
	if(Mode::next_mode_i_ == Mode::cm_exploration)
		key_led.setMode(LED_STEADY, LED_ORANGE, 600);
	else
		key_led.setMode(LED_STEADY, LED_GREEN, 600);
}

