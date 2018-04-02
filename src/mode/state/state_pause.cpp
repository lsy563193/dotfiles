//
// Created by lsy563193 on 12/4/17.
//

#include <mode.hpp>
#include <robot.hpp>
#include "key_led.h"
#include <error.h>

void StatePause::init() {
	if (error.get())
		key_led.setMode(LED_STEADY, LED_RED);
	else if (robot::instance()->isBatteryLow())
		key_led.setMode(LED_BREATH, LED_ORANGE);
	else
		key_led.setMode(LED_BREATH, LED_GREEN);
}
//bool StateFolllowWall::isFinish() {
//	return false;
//}

