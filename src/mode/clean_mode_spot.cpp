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

bool CleanModeSpot::setNextMoveType() {
	move_type_i_ = mt_follow_wall_left;
	return ACleanMode::setNextMoveType();
}

bool CleanModeSpot::map_mark() {
	return false;
}
