//
// Created by lsy563193 on 12/4/17.
//

#include <mode.hpp>

#include "key_led.h"

void ExceptionResume::init() {
	led.set_mode(LED_STEADY, LED_GREEN);
}
