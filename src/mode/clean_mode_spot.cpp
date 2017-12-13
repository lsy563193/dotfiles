//
// Created by lsy563193 on 12/9/17.
//
#include "dev.h"
#include "arch.hpp"

CleanModeSpot::CleanModeSpot() {
	IMovement::sp_cm_ = this;
	speaker.play(SPEAKER_CLEANING_SPOT);
	clean_path_algorithm_ = nullptr;
	go_home_path_algorithm_ = nullptr;
}

CleanModeSpot::~CleanModeSpot() {

}
//
//bool CleanModeSpot::setNextAction_() {
//	move_type_i_ = mt_follow_wall_left;
//	return ACleanMode::setNextAction_();
//}

bool CleanModeSpot::map_mark() {
	return false;
}
