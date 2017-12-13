//
// Created by lsy563193 on 12/4/17.
//
#include "pp.h"
#include "arch.hpp"



ActionLinear::ActionLinear() {
		turn_target_angle_ = plan_path_.front().TH;
		ROS_INFO("%s,%d: mt_is_linear,turn(%d)", __FUNCTION__, __LINE__,turn_target_angle_);
}
