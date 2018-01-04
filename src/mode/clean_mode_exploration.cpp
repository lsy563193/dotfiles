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
	event_manager_register_handler(this);
	event_manager_set_enable(true);
	event_manager_reset_status();
	PP_INFO();
	ROS_INFO("%s %d: Entering Exporation mode\n=========================" , __FUNCTION__, __LINE__);
	speaker.play(VOICE_EXPLORATION_START, false);
	action_i_ = ac_open_gyro;
	clean_path_algorithm_.reset(new NavCleanPathAlgorithm());
	IMoveType::sp_mode_ = this;
}

CleanModeExploration::~CleanModeExploration()
{
	IMoveType::sp_mode_ = nullptr;
	event_manager_set_enable(false);
	wheel.stop();
	brush.stop();
	vacuum.stop();
	lidar.motorCtrl(OFF);
	lidar.setScanLinearReady(0);
	lidar.setScanOriginalReady(0);

	robot::instance()->setBaselinkFrameType(ODOM_POSITION_ODOM_ANGLE);
	slam.stop();
	odom.setAngleOffset(0);

	if(ev.key_clean_pressed || ev.key_long_pressed) {
		speaker.play(VOICE_CLEANING_STOP);
		ROS_WARN("%s %d: Press remote clean.Stop Exploration", __FUNCTION__, __LINE__);
	}
	else if (ev.cliff_all_triggered)
	{
		speaker.play(VOICE_ERROR_LIFT_UP, false);
		speaker.play(VOICE_CLEANING_STOP);
		ROS_WARN("%s %d: Cliff all triggered. Stop Exploration.", __FUNCTION__, __LINE__);
	}
	else if (ev.fatal_quit)
	{
		speaker.play(VOICE_CLEANING_STOP, false);
		error.alarm();
		ROS_WARN("%s %d: fatal_quit. Stop Exploration.", __FUNCTION__, __LINE__);
	}
	else
	{
		speaker.play(VOICE_CLEANING_FINISHED);
		ROS_WARN("%s %d: Finish cleaning.", __FUNCTION__, __LINE__);
	}
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

bool CleanModeExploration::isExit()
{
	if(ev.cliff_all_triggered){
		ROS_WARN("%s %d:.", __FUNCTION__, __LINE__);
		setNextMode(md_idle);
		return true;
	}
	if(ev.key_clean_pressed || ev.key_long_pressed){
		ROS_WARN("%s %d:.", __FUNCTION__, __LINE__);
		setNextMode(md_idle);
		return true;
	}
	return ACleanMode::isExit();
}

bool CleanModeExploration::setNextAction()
{
	PP_INFO();
	//todo action convert
	if (sp_state == state_init)
		return ACleanMode::setNextAction();
	else if(sp_state == state_clean)
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

void CleanModeExploration::cliffAll(bool state_now, bool state_last) {
	ROS_WARN("%s %d: Cliff all.", __FUNCTION__, __LINE__);
	ev.cliff_all_triggered = true;
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

void CleanModeExploration::printMapAndPath()
{
	clean_path_algorithm_->displayCellPath(pointsGenerateCells(passed_path_));
	clean_map_.print(CLEAN_MAP,getPosition().toCell().x,getPosition().toCell().y);
}

// state

bool CleanModeExploration::isStateInitConfirmed() {
	if (action_i_ == ac_open_slam) {
		auto curr = updatePosition();
		passed_path_.push_back(curr);
		home_points_.back().home_point.th = robot::instance()->getWorldPoseAngle();
		PP_INFO();

		sp_state = state_clean;
		sp_state->update();
		action_i_ = ac_null;
	}
	else
		return true;
	return false;
}

bool CleanModeExploration::isStateCleanConfirmed() {
	mapMark();
	PP_INFO();
	old_dir_ = new_dir_;
	ROS_WARN("old_dir_(%d)", old_dir_);
	plan_path_.clear();
	if (ev.rcon_triggered) {
		ROS_WARN("%s,%d:find charge success,convert to go to charge state", __func__, __LINE__);
		sp_state = state_go_to_charger;
		sp_state->update();
		action_i_ = ac_go_to_charger;
		return true;
	}
	else if (clean_path_algorithm_->generatePath(clean_map_, getPosition(), old_dir_, plan_path_)) {
		new_dir_ = (MapDirection) plan_path_.front().th;
		ROS_WARN("new_dir_(%d)", new_dir_);
		plan_path_.pop_front();
		clean_path_algorithm_->displayCellPath(pointsGenerateCells(plan_path_));
		robot::instance()->pubCleanMapMarkers(clean_map_, pointsGenerateCells(plan_path_));
		return true;
	}
	else {
		ROS_WARN("%s,%d:exploration finish,did not find charge", __func__, __LINE__);
		sp_state = state_go_home_point;
		if (go_home_path_algorithm_ == nullptr)
			go_home_path_algorithm_.reset(new GoHomePathAlgorithm(clean_map_, home_points_));
		sp_state->update();
		action_i_ = ac_null;
	}
	return false;
}

bool CleanModeExploration::isStateGoHomePointConfirmed() {
	PP_INFO();
	setNextStateForGoHomePoint(clean_map_);
	return true;
}

bool CleanModeExploration::isStateGoToChargerConfirmed() {
	PP_INFO();
	if (ev.charge_detect && charger.isOnStub()) {
		sp_state = nullptr;
		return true;
	}
	else
		sp_state = state_clean;
	return false;
}
