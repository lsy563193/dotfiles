//
// Created by pierre on 17-12-20.
//
#include <event_manager.h>
#include <pp.h>
#include <error.h>
#include <map.h>
#include "arch.hpp"

CleanModeExploration::CleanModeExploration()
{
	speaker.play(VOICE_EXPLORATION_START, false);
	action_i_ = ac_open_gyro;
	mode_i_ = cm_exploration;
	clean_path_algorithm_.reset(new NavCleanPathAlgorithm());
	IMoveType::sp_mode_ = this;
	go_home_path_algorithm_.reset();
	clean_map_.mapInit();
}

CleanModeExploration::~CleanModeExploration()
{
}

bool CleanModeExploration::mapMark()
{
//	PP_WARN();
	clean_map_.mergeFromSlamGridMap(slam_grid_map,true,true);
	clean_map_.setExplorationCleaned();
	clean_map_.setBlocks();
	clean_map_.markRobot(CLEAN_MAP);
	passed_path_.clear();
	return false;
}

bool CleanModeExploration::setNextAction()
{
	PP_INFO();
	//todo action convert
	if (sp_state == state_init)
		return ACleanMode::setNextAction();
	else if(sp_state == state_exploration)
		action_i_ = ac_linear;
	else if(sp_state == state_go_to_charger)
		action_i_ = ac_go_to_charger;
	else if(sp_state == state_go_home_point)
		action_i_ = ac_linear;
	else
		action_i_ = ac_null;
	genNextAction();
	return action_i_ != ac_null;
}
// event
void CleanModeExploration::keyClean(bool state_now, bool state_last) {
	ROS_WARN("%s %d: key clean.", __FUNCTION__, __LINE__);

	beeper.play_for_command(VALID);

	// Wait for key released.
	bool long_press = false;
	while (key.getPressStatus())
	{
		if (!long_press && key.getPressTime() > 3)
		{
			ROS_WARN("%s %d: key clean long pressed.", __FUNCTION__, __LINE__);
			beeper.play_for_command(VALID);
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

	beeper.play_for_command(VALID);
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
	{
		sp_state = state_exploration;
	}
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

//state GoHomePoint
void CleanModeExploration::switchInStateGoHomePoint() {
	PP_INFO();
	sp_state = nullptr;
}

bool CleanModeExploration::MoveTypeFollowWallIsFinish(MoveTypeFollowWall *p_mt) {
	return p_mt->isBlockCleared(clean_map_, passed_path_);
}

