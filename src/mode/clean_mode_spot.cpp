//
// Created by lsy563193 on 12/9/17.
//
#include "dev.h"
#include "arch.hpp"

CleanModeSpot::CleanModeSpot() {
	IMovement::sp_cm_ = this;
	speaker.play(SPEAKER_CLEANING_SPOT);
}

CleanModeSpot::~CleanModeSpot() {

}

bool CleanModeSpot::setNextMoveType() {
	return ACleanMode::setNextMoveType();
}

Path_t CleanModeSpot::generatePath(GridMap &map, const Cell_t &curr_cell, const MapDirection &last_dir) {
	return Path_t();
}

bool CleanModeSpot::map_mark() {
	return false;
}
