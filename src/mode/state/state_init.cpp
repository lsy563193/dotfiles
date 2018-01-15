//
// Created by lsy563193 on 1/2/18.
//

#include <mode.hpp>

#include "key_led.h"

void StateInit::init() {
	led.set_mode(LED_FLASH, LED_GREEN, 1000);
}

