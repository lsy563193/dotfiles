//
// Created by lsy563193 on 12/4/17.
//

#include <mode.hpp>
#include "key_led.h"

void StatePause::init() {
	led.setMode(LED_BREATH, LED_GREEN);
}

//bool StateTrapped::isFinish() {
//	return false;
//}

