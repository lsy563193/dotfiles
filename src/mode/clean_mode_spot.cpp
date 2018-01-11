//
// Created by lsy563193 on 12/9/17.
//
#include <map.h>
#include "dev.h"
#include "arch.hpp"
#include "path_algorithm.h"
#include "pp.h"
#include "robot.hpp"

CleanModeSpot::CleanModeSpot()
{
	//speaker.play(VOICE_CLEANING_SPOT,false);
	clean_path_algorithm_.reset(new SpotCleanPathAlgorithm());
	go_home_path_algorithm_.reset();
}

CleanModeSpot::~CleanModeSpot()
{
/*	IMoveType::sp_mode_ = nullptr;
	event_manager_set_enable(false);
	wheel.stop();
	brush.stop();
	vacuum.stop();
	lidar.motorCtrl(OFF);
	lidar.setScanOriginalReady(0);

	robot::instance()->setBaselinkFrameType( ODOM_POSITION_ODOM_ANGLE);
	slam.stop();
	odom.setAngleOffset(0);
	speaker.play(VOICE_CLEANING_STOP,false);*/
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

	if (sp_state == state_trapped)
		clean_map_.markRobot(CLEAN_MAP);
	clean_map_.setBlocks();
	PP_INFO();
	clean_map_.print(CLEAN_MAP, getPosition().toCell().x, getPosition().toCell().y);

	passed_path_.clear();
	return false;
}

bool CleanModeSpot::setNextAction()
{
	if (sp_state == state_init)
		return ACleanMode::setNextAction();
	else if (sp_state == state_clean)
	{
		PP_INFO();
		if(plan_path_.size() >= 2)
			action_i_ = ac_linear;
		else
			action_i_ = ac_null;
	}
	genNextAction();
	return action_i_ != ac_null;
}

void CleanModeSpot::remoteClean(bool state_now, bool state_last)
{
	ev.key_clean_pressed = true;
	beeper.play_for_command(true);
	remote.reset();
}

void CleanModeSpot::switchInStateInit()
{
//	if(action_i_ == ac_open_slam)
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


void CleanModeSpot::keyClean(bool state_now,bool state_last)
{
	INFO_GREEN("key clean press");
	wheel.stop();
	ev.key_clean_pressed = true;
	beeper.play_for_command(true);
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

