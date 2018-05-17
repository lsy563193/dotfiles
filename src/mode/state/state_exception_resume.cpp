//
// Created by lsy563193 on 12/4/17.
//

#include <mode.hpp>
#include <gyro.h>

#include "key_led.h"

void StateExceptionResume::init() {
	gyro.setTiltCheckingEnable(false); //disable tilt detect
	ROS_INFO("%s %d: Enter state exception resume.", __FUNCTION__, __LINE__);
}
