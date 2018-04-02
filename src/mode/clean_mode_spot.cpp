//
// Created by lsy563193 on 12/9/17.
//
#include <map.h>
#include <event_manager.h>
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
	if(ev.remote_spot)
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
		setCleaned(pointsGenerateCells(passed_path_));
	}

	if (sp_state == state_folllow_wall.get())
		clean_map_.markRobot(CLEAN_MAP);
	setBlocks(iterate_point_.dir);
	PP_INFO();
	clean_map_.print(CLEAN_MAP, Cells{getPosition().toCell()});

	passed_path_.clear();
	return false;
}

void CleanModeSpot::remoteClean(bool state_now, bool state_last)
{
	ev.key_clean_pressed = true;
	beeper.beepForCommand(true);
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

void CleanModeSpot::remoteSpot(bool state_now, bool state_last)
{
	INFO_GREEN("key spot press");
	wheel.stop();
	ev.remote_spot = true;
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

void CleanModeSpot::switchInStateInit()
{
	action_i_ = ac_null;
	sp_action_ = nullptr;
	sp_state = state_spot.get();
	sp_state->init();
}

void CleanModeSpot::switchInStateSpot()
{
	action_i_ = ac_null;
	sp_action_ = nullptr;
	sp_state = nullptr;
//	sp_state->init();
}
