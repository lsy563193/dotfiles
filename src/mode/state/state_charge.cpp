//
// Created by lsy563193 on 12/4/17.
//

#include <mode.hpp>
#include <key_led.h>

void StateCharge::init() {
    key_led.setMode(LED_STEADY, LED_ORANGE);
}

//bool StateFolllowWall::isFinish() {
//	return false;
//}

