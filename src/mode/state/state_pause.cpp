//
// Created by lsy563193 on 12/4/17.
//

#include <mode.hpp>
#include <robot.hpp>
#include <error.h>
#include <gyro.h>
#include "key_led.h"
#include "wifi/wifi.h"

void StatePause::init() {
	if (robot_error.get())
		key_led.setMode(LED_STEADY, LED_RED);
	else if (robot::instance()->batteryTooLowToClean())
		key_led.setMode(LED_STEADY, LED_ORANGE);
	else
		key_led.setMode(LED_BREATH, LED_GREEN);

	s_wifi.setWorkMode(Mode::md_idle);
	s_wifi.taskPushBack(S_Wifi::ACT::ACT_UPLOAD_STATUS);
	gyro.setTiltCheckingEnable(false); //disable tilt detect
	ROS_INFO("%s %d: Enter state pause.", __FUNCTION__, __LINE__);
}
//bool StateFolllowWall::isFinish() {
//	return false;
//}

