//
// Created by lsy563193 on 12/9/17.
//
#include "dev.h"
#include "arch.hpp"

CleanModeSpot::CleanModeSpot() {
	IMoveType::sp_cm_.reset(this);
	speaker.play(VOICE_CLEANING_SPOT);
	clean_path_algorithm_.reset();
	go_home_path_algorithm_.reset();
}

//
//bool CleanModeSpot::setNextAction_() {
//	move_type_i_ = mt_follow_wall_left;
//	return ACleanMode::setNextAction_();
//}

bool CleanModeSpot::mapMark() {
	return false;
}
