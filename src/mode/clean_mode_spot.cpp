//
// Created by lsy563193 on 12/9/17.
//
#include <map.h>
#include <event_manager.h>
#include "dev.h"
CleanModeSpot::CleanModeSpot()
{
	ROS_WARN("%s %d: Entering Spot mode\n=========================" , __FUNCTION__, __LINE__);
	speaker.play(VOICE_CLEANING_SPOT,false);

	clean_path_algorithm_.reset(new SpotCleanPathAlgorithm());
	go_home_path_algorithm_.reset(new GoHomePathAlgorithm());
	mode_i_ = cm_spot;
	s_wifi.setWorkMode(cm_spot);
	s_wifi.taskPushBack(S_Wifi::ACT::ACT_UPLOAD_STATUS);
}

CleanModeSpot::~CleanModeSpot()
{
	ROS_WARN("%s %d: Exit.", __FUNCTION__, __LINE__);
}

bool CleanModeSpot::isExit()
{
	if(ev.remote_spot)
	{
		INFO_YELLOW("in spot mode enter idle");
		setNextMode(md_idle);
		return true;
	}

	if (s_wifi.receiveSpot())
	{
		ROS_WARN("%s %d: Exit for wifi spot.", __FUNCTION__, __LINE__);
		setNextMode(md_idle);
		return true;
	}

	if (s_wifi.receiveIdle())
	{
		ROS_WARN("%s %d: Exit for wifi idle.", __FUNCTION__, __LINE__);
		setNextMode(md_idle);
		return true;
	}

	if (s_wifi.receivePlan1())
	{
		ROS_WARN("%s %d: Exit for wifi plan1.", __FUNCTION__, __LINE__);
		setNextMode(cm_navigation);
		return true;
	}

	return ACleanMode::isExit();
}

bool CleanModeSpot::mapMark()
{
	ROS_INFO("%s,%d,passed_path",__FUNCTION__,__LINE__);
	auto passed_path_cells = pointsGenerateCells(passed_path_);
	displayCellPath(passed_path_cells);

	if (action_i_ == ac_linear) {
//		PP_INFO();
		setCleaned(pointsGenerateCells(passed_path_));
	}

	if (sp_state == state_folllow_wall.get())
		clean_map_.markRobot(getPosition().toCell(), CLEAN_MAP);
	setBlocks(iterate_point_->dir);
	PP_INFO();
	clean_map_.print(getPosition().toCell(), CLEAN_MAP, Cells{getPosition().toCell()});

	passed_path_.clear();
	return false;
}

void CleanModeSpot::remoteClean(bool state_now, bool state_last)
{
	ev.key_clean_pressed = true;
//	beeper.beepForCommand(VALID);
	remote.reset();
}

void CleanModeSpot::keyClean(bool state_now,bool state_last)
{
	INFO_GREEN("key clean press");

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

void CleanModeSpot::remoteSpot(bool state_now, bool state_last)
{
	INFO_GREEN("key spot press");
	wheel.stop();
	ev.remote_spot = true;
//	beeper.beepForCommand(VALID);
	remote.reset();

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

bool CleanModeSpot::updateActionInStateInit() {
	if (action_i_ == ac_null)
		action_i_ = ac_open_gyro_and_lidar;
	else if (action_i_ == ac_open_gyro_and_lidar) {
		action_i_ = ac_open_lidar;
	}
	else if (action_i_ == ac_open_lidar)
		action_i_ = ac_open_slam;
	else // action_open_slam
		return false;

	genNextAction();
	return true;
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
