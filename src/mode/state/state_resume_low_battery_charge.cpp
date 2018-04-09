//
// Created by lsy563193 on 12/4/17.
//

#include <mode.hpp>
#include "key_led.h"

void StateResumeLowBatteryCharge::init() {
	key_led.setMode(LED_STEADY, LED_GREEN);
	ROS_INFO("%s %d: Enter state resume low battery charge.", __FUNCTION__, __LINE__);
}

//bool StateFolllowWall::isFinish() {
//	return false;
//}

