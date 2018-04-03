//
// Created by austin on 18-3-7.
//

#include <water_tank.hpp>
#include "state.hpp"
#include "key_led.h"

void StateTest::init() {
	key_led.setMode(LED_STEADY, LED_ORANGE);
}
