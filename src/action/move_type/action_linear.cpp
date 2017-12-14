//
// Created by lsy563193 on 12/4/17.
//
#include "pp.h"
#include "arch.hpp"



ActionLinear::ActionLinear() {
	turn_target_angle_ = sp_cm_->plan_path_.front().TH;
	ROS_INFO("%s,%d: mt_is_linear,turn(%d)", __FUNCTION__, __LINE__, turn_target_angle_);
	ROS_INFO("turn_angle", turn_target_angle_);
	movement_i_ = mm_turn;
	sp_movement_.reset(new MovementTurn(turn_target_angle_));
	IMovement::sp_mt_.reset(this);
}

bool ActionLinear::isFinish() {
	if (sp_movement_->isFinish()) {
		PP_INFO();
		path_display_path_points(sp_cm_->plan_path_);

		if (movement_i_ == mm_turn) {
			movement_i_ = mm_forward;
			auto target = GridMap::cellToPoint(sp_cm_->plan_path_.back());
			sp_movement_.reset(new MovementForward());
		}
		else if (movement_i_ == mm_forward) {
			ROS_INFO("mm_forward");
			nav_map.saveBlocks();
			if (ev.bumper_triggered || ev.cliff_triggered || ev.tilt_triggered) {
				movement_i_ = mm_back;
				sp_movement_.reset(new MovementBack);
			}
			else {
				return true;
			}
		}
		else {//back
			return true;
		}
		return false;
	}
}
