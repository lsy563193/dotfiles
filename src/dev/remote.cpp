//
// Created by root on 11/17/17.
//

#include <cstdint>
#include "ros/ros.h"
#include "remote.hpp"

Remote remote;

bool Remote::isKeyTrigger(uint8_t key) {
	// Debug
	if (key_status_ > 0)
		ROS_DEBUG("%s, %d press_status_ = %x", __FUNCTION__, __LINE__, key_status_);

	return key_status_ == key;

}

