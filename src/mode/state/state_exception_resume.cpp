//
// Created by lsy563193 on 12/4/17.
//

#include <mode.hpp>

#include "key_led.h"

void StateExceptionResume::init() {
	if(!sp_cm_->isExpMode())
		key_led.setMode(LED_STEADY, LED_GREEN);
	ROS_INFO("%s %d: Enter state exception resume.", __FUNCTION__, __LINE__);
}
