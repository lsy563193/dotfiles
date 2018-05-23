//
// Created by lsy563193 on 12/9/17.
//

#include <dev.h>
#include <event_manager.h>
#include "robot.hpp"
#include "dev.h"
#include "mode.hpp"
Cells path_points;

CleanModeFollowWall::CleanModeFollowWall()
{
	ROS_WARN("%s %d: Entering Follow wall mode\n=========================" , __FUNCTION__, __LINE__);
//	event_manager_register_handler(this);
//	event_manager_set_enable(true);
//	ROS_INFO("%s %d: Entering Follow wall mode\n=========================" , __FUNCTION__, __LINE__);
//	IMoveType::sp_mode_ = this;
//	diff_timer_ = WALL_FOLLOW_TIME;
	speaker.play(VOICE_CLEANING_WALL_FOLLOW, false);
	clean_path_algorithm_.reset(new WFCleanPathAlgorithm);
	go_home_path_algorithm_.reset(new GoHomePathAlgorithm());
	closed_count_limit_ = 1;
	mode_i_ = cm_wall_follow;
	s_wifi.setWorkMode(cm_wall_follow);
	s_wifi.taskPushBack(S_Wifi::ACT::ACT_UPLOAD_STATUS);
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
		speaker.play(VOICE_CLEANING_STOP_UNOFFICIAL);
		ROS_WARN("%s %d: Cliff all triggered. Finish cleaning.", __FUNCTION__, __LINE__);
	}
	else
	{
		speaker.play(VOICE_CLEANING_FINISHED);
		ROS_WARN("%s %d: Finish cleaning.", __FUNCTION__, __LINE__);
	}
*/
	ROS_WARN("%s %d: Exit.", __FUNCTION__, __LINE__);
}

bool CleanModeFollowWall::mapMark() {
	displayPointPath(passed_path_);
	PP_WARN();
	if (isStateGoHomePoint())
	{
		setCleaned(pointsGenerateCells(passed_path_));
		setBlocks(iterate_point_->dir);
	}
	else if (action_i_ == ac_follow_wall_left || action_i_ == ac_follow_wall_right)
	{
		setCleaned(pointsGenerateCells(passed_path_));
		setBlocks(iterate_point_->dir);
		ROS_ERROR("-------------------------------------------------------");
		auto start = *passed_path_.begin();
		passed_path_.erase(std::remove_if(passed_path_.begin(),passed_path_.end(),[&start](Point_t& it){
			return it.toCell() == start.toCell();
		}),passed_path_.end());
		displayPointPath(passed_path_);
		ROS_ERROR("-------------------------------------------------------");
		setFollowWall(clean_map_, action_i_ == ac_follow_wall_left, passed_path_);
	}
	clean_map_.markRobot(getPosition().toCell(), CLEAN_MAP);
	clean_map_.print(getPosition().toCell(), CLEAN_MAP, Cells{getPosition().toCell()});
	passed_path_.clear();
	return false;
}

bool CleanModeFollowWall::isExit()
{
	if (ev.remote_follow_wall)
	{
		ROS_WARN("%s %d: Exit for ev.remote_follow_wall.", __FUNCTION__, __LINE__);
		setNextMode(md_idle);
		return true;
	}

	if (s_wifi.receiveIdle())
	{
		ROS_WARN("%s %d: Exit for wifi idle.", __FUNCTION__, __LINE__);
		setNextMode(md_idle);
		return true;
	}

	if (s_wifi.receiveFollowWall())
	{
		ROS_WARN("%s %d: Exit for wifi follow wall.", __FUNCTION__, __LINE__);
		setNextMode(md_idle);
		return true;
	}

	return ACleanMode::isExit();
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
	if(water_tank.getStatus(WaterTank::operate_option::swing_motor)){
		beeper.beepForCommand(INVALID);
	}
	else if(isStateInit() || isStateFollowWall() || isStateGoHomePoint() || isStateGoToCharger())
	{
//		beeper.beepForCommand(VALID);
		vacuum.setForUserSetMaxMode(!vacuum.isUserSetMaxMode());
		ACleanMode::setVacuum();
	}
	remote.reset();
}

void CleanModeFollowWall::remoteClean(bool state_now, bool state_last)
{
	ROS_WARN("%s %d: remote clean.", __FUNCTION__, __LINE__);

//	beeper.beepForCommand(VALID);
	wheel.stop();
	ev.key_clean_pressed = true;
	remote.reset();
}

void CleanModeFollowWall::remoteWallFollow(bool state_now, bool state_last)
{
	ROS_WARN("%s %d: remote wall follow.", __FUNCTION__, __LINE__);

//	beeper.beepForCommand(VALID);
	wheel.stop();
	ev.remote_follow_wall = true;
	remote.reset();
}

void CleanModeFollowWall::chargeDetect(bool state_now, bool state_last)
{
	if (!ev.charge_detect)
	{
		if (isStateGoToCharger())
		{
			ROS_WARN("%s %d: Charge detect!.", __FUNCTION__, __LINE__);
			ev.charge_detect = charger.getChargeStatus();
		}
		else if (charger.isDirected())
		{
			ROS_WARN("%s %d: Charge detect!.", __FUNCTION__, __LINE__);
			ev.charge_detect = charger.getChargeStatus();
			ev.fatal_quit = true;
		}
	}
}

void CleanModeFollowWall::batteryHome(bool state_now, bool state_last)
{
	if (!ev.battery_home && isStateFollowWall())
	{
		ROS_WARN("%s %d: low battery, battery =\033[33m %dmv \033[0m", __FUNCTION__, __LINE__, battery.getVoltage());
		ev.battery_home = true;
	}
}

void CleanModeFollowWall::switchInStateInit() {
	PP_INFO();
	action_i_ = ac_null;
	sp_action_ = nullptr;
	sp_state = state_folllow_wall.get();
	is_isolate = true;
	is_closed = true;
	closed_count_ = 0;
	isolate_count_ = 0;
	sp_state->init();
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
	sp_state = state_go_home_point.get();
	go_home_path_algorithm_->initForGoHomePoint(clean_map_);
	sp_state->init();
	action_i_ = ac_null;
	genNextAction();
}

