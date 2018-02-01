//
// Created by lsy563193 on 1/2/18.
//

#include <mode.hpp>

#include "key_led.h"

void StateInit::init() {
	key_led.setMode(LED_FLASH, LED_GREEN, 600);
}

