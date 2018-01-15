//
// Created by lsy563193 on 12/4/17.
//
#include "gyro.h"
#include "key_led.h"
#include "arch.hpp"

void StateGoCharger::init() {
	gyro.TiltCheckingEnable(false); //disable tilt detect
	led.set_mode(LED_STEADY, LED_ORANGE);
}
