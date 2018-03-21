//
// Created by root on 11/17/17.
//

#include <cstdint>
#include <ros/ros.h>
#include "remote.hpp"

Remote remote;

bool Remote::isKeyTrigger(uint8_t key) {
	// Debug
	uint8_t key_status = this->get();
	if (key_status > 0)
		ROS_DEBUG("%s, %d press_status_ = %x", __FUNCTION__, __LINE__, key_status);

	return key_status_ == key;

}

