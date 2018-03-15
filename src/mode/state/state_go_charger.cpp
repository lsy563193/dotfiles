//
// Created by lsy563193 on 12/4/17.
//

#include <mode.hpp>
#include <brush.h>

#include "gyro.h"
#include "key_led.h"

void StateGoCharger::init() {
	gyro.setTiltCheckingEnable(false); //disable tilt detect
	brush.slowOperate();
	key_led.setMode(LED_STEADY, LED_ORANGE);
}
