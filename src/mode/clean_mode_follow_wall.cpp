//
// Created by lsy563193 on 12/9/17.
//

#include <dev.h>
#include "robot.hpp"
#include "dev.h"
#include "mode.hpp"
Cells path_points;

CleanModeFollowWall::CleanModeFollowWall()
{
//	event_manager_register_handler(this);
//	event_manager_set_enable(true);
//	ROS_INFO("%s %d: Entering Follow wall mode\n=========================" , __FUNCTION__, __LINE__);
//	IMoveType::sp_mode_ = this;
//	diff_timer_ = WALL_FOLLOW_TIME;
	speaker.play(VOICE_CLEANING_WALL_FOLLOW, false);
	clean_path_algorithm_.reset(new WFCleanPathAlgorithm);
	go_home_path_algorithm_.reset();
	closed_count_limit_ = 1;
}

CleanModeFollowWall::~CleanModeFollowWall()
{
/*
	IMoveType::sp_mode_ = nullptr;
	event_manager_set_enable(false);

	if (ev.key_clean_pressed)
	{
		speaker.play(VOICE_CLEANING_FINISHED);
		ROS_WARN("%s %d: Key clean pressed. Finish cleaning.", __FUNCTION__, __LINE__);
	}
	else if (ev.cliff_all_triggered)
	{
		speaker.play(VOICE_ERROR_LIFT_UP, false);
		speaker.play(VOICE_CLEANING_STOP);
		ROS_WARN("%s %d: Cliff all triggered. Finish cleaning.", __FUNCTION__, __LINE__);
	}
	else
	{
		speaker.play(VOICE_CLEANING_FINISHED);
		ROS_WARN("%s %d: Finish cleaning.", __FUNCTION__, __LINE__);
	}
*/
}

bool CleanModeFollowWall::mapMark() {
	clean_path_algorithm_->displayPointPath(passed_path_);
	PP_WARN();
	if (isStateGoHomePoint())
	{
		setCleaned(pointsGenerateCells(passed_path_));
		setBlocks(iterate_point_.dir);
	}
	else if (action_i_ == ac_follow_wall_left || action_i_ == ac_follow_wall_right)
	{
		setCleaned(pointsGenerateCells(passed_path_));
		setBlocks(iterate_point_.dir);
		ROS_ERROR("-------------------------------------------------------");
		auto start = *passed_path_.begin();
		passed_path_.erase(std::remove_if(passed_path_.begin(),passed_path_.end(),[&start](Point_t& it){
			return it.toCell() == start.toCell();
		}),passed_path_.end());
		clean_path_algorithm_->displayPointPath(passed_path_);
		ROS_ERROR("-------------------------------------------------------");
		setFollowWall(clean_map_, action_i_ == ac_follow_wall_left, passed_path_);
	}
	clean_map_.markRobot(CLEAN_MAP);
	clean_map_.print(CLEAN_MAP, Cells{getPosition().toCell()});
	passed_path_.clear();
	return false;
}

void CleanModeFollowWall::keyClean(bool state_now, bool state_last)
{
	ROS_WARN("%s %d: key clean.", __FUNCTION__, __LINE__);

	beeper.beepForCommand(VALID);
	wheel.stop();

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

//
//void CleanModeFollowWall::overCurrentWheelLeft(bool state_now, bool state_last)
//{
//	ROS_WARN("%s %d: Left wheel oc.", __FUNCTION__, __LINE__);
//	ev.oc_wheel_left = true;
//}
//
//void CleanModeFollowWall::overCurrentWheelRight(bool state_now, bool state_last)
//{
//	ROS_WARN("%s %d: Right wheel oc.", __FUNCTION__, __LINE__);
//	ev.oc_wheel_right = true;
//}
//

void CleanModeFollowWall::remoteMax(bool state_now, bool state_last)
{
	ROS_WARN("%s %d: Remote max is pressed.", __FUNCTION__, __LINE__);
	if(isStateClean())
	{
		beeper.beepForCommand(VALID);
		if (!water_tank.isEquipped())
			vacuum.switchToNext();
	}
	else if (isStateGoHomePoint() || isStateGoToCharger())
	{
		beeper.beepForCommand(VALID);
		if (!water_tank.isEquipped())
		{
			vacuum.switchToNext();
			vacuum.setTmpMode(Vac_Normal);
		}
	}
	else
		beeper.beepForCommand(INVALID);
	remote.reset();
}
void CleanModeFollowWall::remoteClean(bool state_now, bool state_last)
{
	ROS_WARN("%s %d: remote clean.", __FUNCTION__, __LINE__);

	beeper.beepForCommand(VALID);
	wheel.stop();
	ev.key_clean_pressed = true;
	remote.reset();
}


void CleanModeFollowWall::switchInStateInit() {
	PP_INFO();
	action_i_ = ac_null;
	sp_action_ = nullptr;
	sp_state = state_folllow_wall;
	is_isolate = true;
	is_closed = true;
	closed_count_ = 0;
	isolate_count_ = 0;
	sp_state->init();
	key_led.setMode(LED_STEADY, LED_GREEN);
}

//bool CleanModeFollowWall::moveTypeFollowWallIsFinish(IMoveType *p_mt,bool is_new_cell) {
//	if (ACleanMode::moveTypeFollowWallIsFinish(p_mt, is_new_cell))
//	{
//		ROS_INFO("closed_count_(%d), limit(%d)",p_mt->closed_count_, closed_count_limit_);
//		ROS_WARN("moveTypeFollowWallIsFinish close!!!");
//		return true;
//	}

//	return false;
//}

void CleanModeFollowWall::switchInStateFollowWall() {
	sp_state = state_go_home_point;
	ROS_INFO("%s %d: home_cells_.size(%lu)", __FUNCTION__, __LINE__, home_points_.size());
	speaker.play(VOICE_BACK_TO_CHARGER, true);
	go_home_path_algorithm_.reset();
	go_home_path_algorithm_.reset(new GoHomePathAlgorithm(clean_map_, home_points_, start_point_));
	sp_state->init();
	action_i_ = ac_go_to_charger;
	genNextAction();
}

