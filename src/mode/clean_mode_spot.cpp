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
	event_manager_register_handler(this);
	event_manager_set_enable(true);
	IMoveType::sp_mode_ = this;
	speaker.play(VOICE_CLEANING_SPOT,false);
	clean_path_algorithm_.reset(new SpotCleanPathAlgorithm());
	go_home_path_algorithm_.reset();
}

CleanModeSpot::~CleanModeSpot()
{
	IMoveType::sp_mode_ = nullptr;
	event_manager_set_enable(false);
	wheel.stop();
	brush.stop();
	vacuum.stop();
	lidar.motorCtrl(OFF);
	lidar.setScanOriginalReady(0);

	robot::instance()->setBaselinkFrameType( ODOM_POSITION_ODOM_ANGLE);
	slam.stop();
	odom.setAngleOffset(0);
	speaker.play(VOICE_CLEANING_STOP,false);
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

bool CleanModeSpot::isExit()
{
	
	if (ev.fatal_quit || sp_action_->isExit())
	{
		ROS_WARN("%s %d:.", __FUNCTION__, __LINE__);
		setNextMode(md_idle);
		return true;
	}

	if(ev.key_clean_pressed){
		ev.key_clean_pressed = false;
		ROS_WARN("%s %d:.", __FUNCTION__, __LINE__);
		setNextMode(md_idle);
		return true;
	}
	if(ev.cliff_all_triggered) {
		ev.cliff_all_triggered = false;
		ROS_WARN("%s %d:.", __FUNCTION__, __LINE__);
		setNextMode(md_idle);
		return true;
	}
	return ACleanMode::isExit();
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
}

void CleanModeSpot::keyClean(bool state_now,bool state_last)
{
	ev.key_clean_pressed = true;
	beeper.play_for_command(true);
}

void CleanModeSpot::cliffAll(bool state_now, bool state_last)
{
	ev.cliff_all_triggered = true;
}

//state
bool CleanModeSpot::isStateInitUpdateFinish()
{
	bool ret = false;
	if (sp_action_ != nullptr && !sp_action_->isFinish())
		return true;

	sp_action_.reset();//for call ~constitution;
	
	if (action_i_ == ac_null)
	{
		action_i_ = ac_open_gyro;
		INFO_CYAN(ac_open_gyro);
		ret = true;
	} 
	else if (action_i_ == ac_open_gyro)
	{
		INFO_CYAN(ac_open_lidar);
		action_i_ = ac_open_lidar;
		vacuum.setMode(Vac_Max);
		brush.fullOperate();
		ret = true;
	}
	else if (action_i_ == ac_open_lidar)
	{
		INFO_CYAN(ac_open_slam);
		action_i_ = ac_open_slam;
		ret = true;
	}
	else if(action_i_ == ac_open_slam)
	{
		INFO_CYAN(init_finish!!);
		auto curr = updatePosition();
		passed_path_.push_back(curr);

		home_points_.back().home_point.th = curr.th; 
		vacuum.setMode(Vac_Max);
		brush.fullOperate();

		sp_state = state_clean;
		sp_state->update();
		return false;
	}
	genNextAction();
	return ret;
}

bool CleanModeSpot::isStateCleanUpdateFinish()
{
	updatePath(clean_map_);
	if (sp_action_ != nullptr && !sp_action_->isFinish())
		return true;
	sp_action_.reset();//for call ~constitution;
	clean_map_.saveBlocks(action_i_ == ac_linear, sp_state == state_clean);
	mapMark();

	old_dir_ = new_dir_;
	ROS_ERROR("old_dir_(%d)", old_dir_);
	auto cur_point = getPosition();
	ROS_INFO("\033[32m plan_path front (%d,%d),cur point:(%d,%d)\033[0m",plan_path_.front().toCell().x,plan_path_.front().toCell().y,cur_point.toCell().x,cur_point.toCell().y);
	if (clean_path_algorithm_->generatePath(clean_map_, cur_point, old_dir_, plan_path_)) {
		new_dir_ = (MapDirection) plan_path_.front().th;
		ROS_ERROR("new_dir_(%d)", new_dir_);
		PP_INFO();
		clean_path_algorithm_->displayCellPath(pointsGenerateCells(plan_path_));
		plan_path_.pop_front();
		action_i_ = ac_linear;
		genNextAction();
		return true;
	}
	else {
		sp_state = nullptr;
		action_i_ = ac_null;
		return true;
	}
	return false;
}
