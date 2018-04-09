//
// Created by lsy563193 on 12/4/17.
//

#include <mode.hpp>
#include <key_led.h>

void StateCharge::init()
{
	key_led.setMode(LED_BREATH, LED_ORANGE);
	ROS_INFO("%s %d: Enter state charge.", __FUNCTION__, __LINE__);
}

//bool StateFolllowWall::isFinish() {
//	return false;
//}

