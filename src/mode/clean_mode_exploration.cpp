//
// Created by pierre on 17-12-20.
//
#include <event_manager.h>
#include <dev.h>
#include <error.h>
#include <map.h>

#include "mode.hpp"

CleanModeExploration::CleanModeExploration()
{
	ROS_INFO("%s %d: Entering Exploration mode\n=========================" , __FUNCTION__, __LINE__);
	speaker.play(VOICE_EXPLORATION_START, false);
	mode_i_ = cm_exploration;
	clean_path_algorithm_.reset(new NavCleanPathAlgorithm());
	IMoveType::sp_mode_ = this; // todo: is this sentence necessary? by Austin
	go_home_path_algorithm_.reset();
	error_marker_.clear();
	clean_map_.mapInit();
}

CleanModeExploration::~CleanModeExploration()
{
}

bool CleanModeExploration::mapMark()
{
	clean_map_.merge(slam_grid_map, true, true, false, false, false, false);
	clean_map_.setCircleMarkers(getPosition(),10,CLEANED,error_marker_);
	resetErrorMarker();

	setBlocks(iterate_point_.dir);
	if(mark_robot_)
		clean_map_.markRobot(CLEAN_MAP);
//	passed_path_.clear();
	return false;
}

// event
void CleanModeExploration::keyClean(bool state_now, bool state_last) {
	ROS_WARN("%s %d: key clean.", __FUNCTION__, __LINE__);

	beeper.beepForCommand(VALID);

	// Wait for key released.
	bool long_press = false;
	while (key.getPressStatus())
	{
		if (!long_press && key.getPressTime() > 3)
		{
			ROS_WARN("%s %d: key clean long pressed.", __FUNCTION__, __LINE__);
			beeper.beepForCommand(VALID);
			long_press = true;
		}
		usleep(20000);
	}

	if (long_press)
		ev.key_long_pressed = true;
	else
		ev.key_clean_pressed = true;
	ROS_WARN("%s %d: Key clean is released.", __FUNCTION__, __LINE__);

	key.resetTriggerStatus();
}

void CleanModeExploration::remoteClean(bool state_now, bool state_last) {
	ROS_WARN("%s %d: remote clean.", __FUNCTION__, __LINE__);

	beeper.beepForCommand(VALID);
	ev.key_clean_pressed = true;
	remote.reset();
}

void CleanModeExploration::overCurrentWheelLeft(bool state_now, bool state_last) {
	ROS_WARN("%s %d: Left wheel oc.", __FUNCTION__, __LINE__);
	ev.oc_wheel_left = true;
}

void CleanModeExploration::overCurrentWheelRight(bool state_now, bool state_last) {
	ROS_WARN("%s %d: Right wheel oc.", __FUNCTION__, __LINE__);
	ev.oc_wheel_right = true;
}

void CleanModeExploration::chargeDetect(bool state_now, bool state_last) {
	ROS_WARN("%s %d: Charge detect!.", __FUNCTION__, __LINE__);
	if (charger.getChargeStatus() >= 1)
	{
		ROS_WARN("%s %d: Set ev.chargeDetect.", __FUNCTION__, __LINE__);
		ev.charge_detect = charger.getChargeStatus();
	}
}

/*void CleanModeExploration::printMapAndPath()
{
	clean_path_algorithm_->displayCellPath(pointsGenerateCells(passed_path_));
	clean_map_.print(CLEAN_MAP,getPosition().toCell().x,getPosition().toCell().y);
}*/

//state GoToCharger
void CleanModeExploration::switchInStateGoToCharger() {
	PP_INFO();
	if (ev.charge_detect && charger.isOnStub()) {
		ev.charge_detect = 0;
		sp_state = nullptr;
		return;
	}
	else
		sp_state = state_exploration;
	sp_state->init();
}

//state Init
void CleanModeExploration::switchInStateInit() {
	PP_INFO();
	action_i_ = ac_null;
	sp_action_ = nullptr;
	sp_state = state_exploration;
	sp_state->init();
}

bool CleanModeExploration::updateActionInStateInit() {
	if (action_i_ == ac_null)
		action_i_ = ac_open_gyro;
	else if (action_i_ == ac_open_gyro) {
		vacuum.setLastMode();
		brush.normalOperate();
		action_i_ = ac_open_lidar;
	}
	else if (action_i_ == ac_open_lidar)
		action_i_ = ac_align;
	else if(action_i_ == ac_align)
		action_i_ = ac_open_slam;
	else // action_open_slam
		return false;

	ACleanMode::genNextAction();
	return true;
}
//state GoHomePoint
void CleanModeExploration::switchInStateGoHomePoint() {
	PP_INFO();
	sp_state = nullptr;
}
/*

bool CleanModeExploration::moveTypeFollowWallIsFinish(IMoveType *p_move_type, bool is_new_cell) {
	if(action_i_ == ac_follow_wall_left || action_i_ == ac_follow_wall_right)
	{
		auto p_mt = dynamic_cast<MoveTypeFollowWall *>(p_move_type);
		return p_mt->isBlockCleared(clean_map_, passed_path_);
	}
	return false;
}
*/

bool CleanModeExploration::markMapInNewCell() {
	if(sp_state == state_folllow_wall)
	{
		mark_robot_ = false;
		mapMark();
		mark_robot_ = true;
	}
	else
		mapMark();
	return true;
}

void CleanModeExploration::resetErrorMarker() {
	//set unclean to map
	auto time = ros::Time::now().toSec();
	for(auto ite = error_marker_.begin();ite != error_marker_.end();ite++){
		if(time - ite->time > 20){
			if(clean_map_.getCell(CLEAN_MAP,ite->x,ite->y) == CLEANED &&
					slam_grid_map.getCell(CLEAN_MAP,ite->x,ite->y) != SLAM_MAP_CLEANABLE)
				clean_map_.setCell(CLEAN_MAP,ite->x,ite->y,UNCLEAN);
			if(error_marker_.empty())
				break;
			error_marker_.erase(ite);
		}
	}
}



