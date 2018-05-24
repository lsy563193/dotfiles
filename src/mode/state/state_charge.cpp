//
// Created by lsy563193 on 12/4/17.
//

#include <mode.hpp>
#include <key_led.h>
#include <gyro.h>
#include "wifi/wifi.h"

void StateCharge::init()
{
	key_led.setMode(LED_BREATH, LED_ORANGE);
	s_wifi.setWorkMode(Mode::md_charge);
	s_wifi.taskPushBack(S_Wifi::ACT::ACT_UPLOAD_STATUS);
	gyro.setTiltCheckingEnable(false); //disable tilt detect
	ROS_INFO("%s %d: Enter state charge.", __FUNCTION__, __LINE__);
}

//bool StateFolllowWall::isFinish() {
//	return false;
//}

