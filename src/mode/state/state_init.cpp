//
// Created by lsy563193 on 1/2/18.
//
#include "key_led.h"
#include "arch.hpp"

void StateInit::init() {
	led.set_mode(LED_FLASH, LED_GREEN, 1000);
}

