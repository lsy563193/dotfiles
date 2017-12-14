//
// Created by lsy563193 on 12/4/17.
//
#include "pp.h"
#include "arch.hpp"



ActionLinear::ActionLinear() {
	resetTriggeredValue();
	turn_target_angle_ = sp_cm_->plan_path_.front().TH;
	ROS_INFO("%s,%d: mt_is_linear,turn(%d)", __FUNCTION__, __LINE__, turn_target_angle_);
	movement_i_ = mm_turn;
	sp_movement_.reset(new MovementTurn(turn_target_angle_));
//	ROS_INFO("%s,%d: mt_is_linear,turn(%d)", __FUNCTION__, __LINE__, turn_target_angle_);
//	ROS_ERROR("%s,%d: mt_is_linear,turn(%d)", __FUNCTION__, __LINE__, turn_target_angle_);
	IMovement::sp_mt_ = this;
//	ROS_WARN("%s,%d: mt_is_linear,turn(%d)", __FUNCTION__, __LINE__, turn_target_angle_);
}

//ActionLinear::~ActionLinear() {
//
//}
bool ActionLinear::isFinish() {
//	PP_INFO();
	if (sp_movement_->isFinish()) {
		PP_INFO();
		path_display_path_points(sp_cm_->plan_path_);

		if (movement_i_ == mm_turn) {
			PP_INFO();
			movement_i_ = mm_forward;
			resetTriggeredValue();
			sp_movement_.reset(new MovementForward());
		}
		else if (movement_i_ == mm_forward) {
			PP_INFO();
			nav_map.saveBlocks();
			if (ev.bumper_triggered || ev.cliff_triggered || ev.tilt_triggered) {
				movement_i_ = mm_back;
				sp_movement_.reset(new MovementBack);
			}
			else {
//				resetTriggeredValue();
				return true;
			}
		}
		else {//back
//			resetTriggeredValue();
			return true;
		}
	}
	return false;
}

ActionLinear::~ActionLinear() {
//	PP_WARN();
}
