//
// Created by lsy563193 on 12/4/17.
//

#include <mode.hpp>

#include "gyro.h"
#include "key_led.h"

void StateGoCharger::init() {
	gyro.TiltCheckingEnable(false); //disable tilt detect
	key_led.setMode(LED_STEADY, LED_ORANGE);
}
