//
// Created by lsy563193 on 12/9/17.
//
#include "dev.h"
#include "arch.hpp"
#include "path_algorithm.h"
#include "pp.h"
#include "robot.hpp"

CleanModeSpot::CleanModeSpot()
{
	plan_path_.clear();
	passed_path_.clear();
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

bool CleanModeSpot::isFinish()
{
	return ACleanMode::isFinish();
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

	if (state_i_ == st_trapped)
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

bool CleanModeSpot::setNextState()
{
	PP_INFO();
	bool state_confirm = false;
	while (ros::ok() && !state_confirm)
	{
		if (state_i_ == st_init)
		{
			if (action_i_ == ac_open_slam)
			{
				auto curr = updatePosition();
				passed_path_.push_back(curr);

				home_points_.back().th = robot::instance()->getWorldPoseAngle();
				PP_INFO();
				vacuum.setMode(Vac_Max);
				brush.fullOperate();

				state_i_ = st_clean;
				stateInit(state_i_);
			}
			else
				state_confirm = true;
		}
		else if (isExceptionTriggered())
		{
			ROS_INFO("%s %d: Pass this state switching for exception cases.", __FUNCTION__, __LINE__);
			// Apply for all states.
			// If all these exception cases happens, directly set next action to exception resume action.
			// BUT DO NOT CHANGE THE STATE!!! Because after exception resume it should restore the state.
			action_i_ = ac_null;
			state_confirm = true;
		}
		else if(state_i_ == st_clean)
		{
			PP_INFO();
			old_dir_ = new_dir_;
			ROS_INFO("\033[34mold_dir_(%d)\033[0m", old_dir_);
			auto cur_point = getPosition();
			ROS_INFO("%s,%s,%d,\033[32m plan_path front (%d,%d),cur point:(%d,%d)\033[0m",__FILE__,__FUNCTION__,__LINE__,plan_path_.front().toCell().x,plan_path_.front().toCell().y,cur_point.toCell().x,cur_point.toCell().y);
			if (clean_path_algorithm_->generatePath(clean_map_, cur_point, old_dir_, plan_path_))
			{
				new_dir_ = (MapDirection)plan_path_.front().th;
				ROS_INFO("\033[34mnew_dir_(%d)\033[0m", new_dir_);
				PP_INFO();
				clean_path_algorithm_->displayPointPath(plan_path_);
				plan_path_.pop_front();
				state_confirm = true;
			}
			else
			{
				state_i_ = st_null;
				action_i_ = ac_null;
				state_confirm = true;
			}
		}
	}

	return state_i_ != st_null;
}

bool CleanModeSpot::setNextAction()
{
	if (state_i_ == st_init)
		return ACleanMode::setNextAction();
	else if (state_i_ == st_clean)
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

