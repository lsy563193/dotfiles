//
// Created by austin on 18-3-7.
//

#include "state.hpp"
#include "key_led.h"
#include "ros/ros.h"

void StateTest::init() {
	key_led.setMode(LED_STEADY, LED_ORANGE);

	ROS_INFO("%s %d: Enter state test.", __FUNCTION__, __LINE__);
}
