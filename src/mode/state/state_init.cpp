//
// Created by lsy563193 on 1/2/18.
//
#include "pp.h"
#include "arch.hpp"

void StateInit::update() {
	led.set_mode(LED_FLASH, LED_GREEN, 1000);
}

