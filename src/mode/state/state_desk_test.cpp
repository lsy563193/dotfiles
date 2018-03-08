//
// Created by austin on 18-3-7.
//

#include "state.hpp"
#include "key_led.h"

void StateDeskTest::init() {
	key_led.setMode(LED_STEADY, LED_GREEN);
}
