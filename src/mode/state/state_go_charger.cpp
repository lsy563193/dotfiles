//
// Created by lsy563193 on 12/4/17.
//
#include "pp.h"
#include "arch.hpp"

void StateGoCharger::update() {
	gyro.TiltCheckingEnable(false); //disable tilt detect
	led.set_mode(LED_STEADY, LED_ORANGE);
}
