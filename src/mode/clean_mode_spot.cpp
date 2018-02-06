//
// Created by lsy563193 on 12/9/17.
//
#include <map.h>
#include "dev.h"
#include "path_algorithm.h"
#include "robot.hpp"
#include "mode.hpp"
CleanModeSpot::CleanModeSpot()
{
	speaker.play(VOICE_CLEANING_SPOT,false);
	clean_path_algorithm_.reset(new SpotCleanPathAlgorithm());
	go_home_path_algorithm_.reset();
}

CleanModeSpot::~CleanModeSpot()
{

}

bool CleanModeSpot::isExit()
{
	if(ev.remote_home)
	{
		//ev.remote_home = false;
		INFO_YELLOW("in spot mode enter exploration");
		setNextMode(cm_exploration);
		return true;
	}
	else if(ev.remote_follow_wall)
	{
		//ev.remote_follow_wall = false;
		INFO_YELLOW("in spot mode enter follow_wall");
		setNextMode(cm_wall_follow);
		return true;
	}
	else if(ev.remote_direction_forward || ev.remote_direction_left || ev.remote_direction_right)
	{
		INFO_YELLOW("in spot mode enter idle");
		setNextMode(md_idle);
		return true;
	}
	return ACleanMode::isExit();
}

bool CleanModeSpot::mapMark()
{
	ROS_INFO("%s,%d,passed_path",__FUNCTION__,__LINE__);
	auto passed_path_cells = pointsGenerateCells(passed_path_);
	clean_path_algorithm_->displayCellPath(passed_path_cells);

	if (action_i_ == ac_linear) {
		PP_INFO();
		clean_map_.setCleaned(pointsGenerateCells(passed_path_));
	}

	if (sp_state == state_folllow_wall)
		clean_map_.markRobot(CLEAN_MAP);
	clean_map_.setBlocks();
	PP_INFO();
	clean_map_.print(CLEAN_MAP, getPosition().toCell().x, getPosition().toCell().y);

	passed_path_.clear();
	return false;
}

void CleanModeSpot::remoteClean(bool state_now, bool state_last)
{
	ev.key_clean_pressed = true;
	beeper.beepForCommand(true);
	remote.reset();
}

void CleanModeSpot::remoteWallFollow(bool state_now, bool state_last)
{
	ev.remote_follow_wall = true;
	beeper.beepForCommand(true);
	INFO_YELLOW("REMOTE FOLLOW_WALL PRESS");
	remote.reset();
}

void CleanModeSpot::keyClean(bool state_now,bool state_last)
{
	INFO_GREEN("key clean press");
	wheel.stop();
	ev.key_clean_pressed = true;
	beeper.beepForCommand(true);
	key.resetTriggerStatus();

}

void CleanModeSpot::overCurrentWheelLeft(bool state_now, bool state_last)
{
	ROS_WARN("%s %d: Left wheel oc.", __FUNCTION__, __LINE__);
	ev.oc_wheel_left = true;
}

void CleanModeSpot::overCurrentWheelRight(bool state_now, bool state_last)
{
	ROS_WARN("%s %d: Right wheel oc.", __FUNCTION__, __LINE__);
	ev.oc_wheel_right = true;
}

void CleanModeSpot::remoteDirectionLeft(bool state_now, bool state_last)
{
	beeper.beepForCommand(VALID);
	ev.remote_direction_left = true;
	ROS_INFO("%s %d: Remote Left.", __FUNCTION__, __LINE__);
	remote.reset();
}

void CleanModeSpot::remoteDirectionRight(bool state_now, bool state_last)
{
	beeper.beepForCommand(VALID);
	ev.remote_direction_right = true;
	ROS_INFO("%s %d: Remote Right.", __FUNCTION__, __LINE__);
	remote.reset();
}

void CleanModeSpot::remoteDirectionForward(bool state_now, bool state_last)
{
	beeper.beepForCommand(VALID);
	ROS_INFO("%s %d: Remote Forward.", __FUNCTION__, __LINE__);
	ev.remote_direction_right = true;

	remote.reset();
}

void CleanModeSpot::switchInStateInit()
{
	action_i_ = ac_null;
	sp_action_ = nullptr;
	sp_state = state_spot;
	sp_state->init();
}

void CleanModeSpot::switchInStateSpot()
{
	action_i_ = ac_null;
	sp_action_ = nullptr;
	sp_state = nullptr;
//	sp_state->init();
}
