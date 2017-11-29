//
// Created by root on 11/20/17.
//

#include <error.h>
#include "dev.h"

Omni omni;


void Omni::detect() {
	auto rv = wheel.getRightWheelActualSpeed();
	auto lv = wheel.getLeftWheelActualSpeed();
	if (std::abs(rv - lv) <= 0.05 && (rv != 0 && lv != 0)) {
		if (std::abs(getOmniWheel() - last_omni_wheel) == 0) {
			omni_detect_cnt++;
			//ROS_INFO("\033[35m" "omni count %d %f\n" "\033[0m",omni_detect_cnt,std::abs(msg->rw_vel - msg->lw_vel));
			if (omni_detect_cnt >= 150) {
				omni_detect_cnt = 0;
				ROS_INFO("\033[36m" "omni detetced ,wheel speed %f,%f  \n" "\033[0m", rv, lv);
				stop_ = true;
			}
		}
	}
	else {
		//g_omni_notmove = false;
		omni_detect_cnt = 0;
		last_omni_wheel = getOmniWheel();
	}
	if (getOmniWheel() >= 10000) {
		omni.reset();
	}
}
