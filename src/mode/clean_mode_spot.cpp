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
	event_manager_register_handler(this);
	event_manager_set_enable(true);
	IMoveType::sp_mode_ = this;
	speaker.play(VOICE_CLEANING_SPOT,false);
	clean_path_algorithm_.reset(new SpotCleanPathAlgorithm());
	go_home_path_algorithm_.reset();
	has_aligned_and_open_slam = false;
	nav_map.reset(COST_MAP);
	nav_map.reset(CLEAN_MAP);
	clean_map_ = &nav_map;
}

CleanModeSpot::~CleanModeSpot()
{
	IMoveType::sp_mode_ = nullptr;
	wheel.stop();
	brush.stop();
	vacuum.stop();
	lidar.motorCtrl(OFF);
	lidar.setScanOriginalReady(0);

	robot::instance()->setBaselinkFrameType( ODOM_POSITION_ODOM_ANGLE);
	slam.stop();
	odom.setAngleOffset(0);
}

bool CleanModeSpot::isFinish()
{
	if(action_i_ == ac_open_slam){
		vacuum.setMode(Vac_Max);
		brush.setPWM(50,50,50);
	}
	return ACleanMode::isFinish();
}

bool CleanModeSpot::mapMark()
{
	ROS_INFO("%s,%d,passed_path",__FUNCTION__,__LINE__);
	clean_path_algorithm_->displayPath(passed_path_);
	if (action_i_ == ac_linear) {
		PP_INFO();
		nav_map.setCleaned(passed_path_);
	}

	if (state_i_ == st_trapped)
		nav_map.markRobot(CLEAN_MAP);
	nav_map.setBlocks();
	nav_map.print(CLEAN_MAP, nav_map.getXCell(), nav_map.getYCell());

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
	return false;
}

bool CleanModeSpot::setNextState()
{
	PP_INFO();

	if (!isInitFinished_)
		return true;

	bool state_confirm = false;
	while (ros::ok() && !state_confirm)
	{
		if (state_i_ == st_null)
		{
			auto curr = nav_map.updatePosition();
			passed_path_.push_back(curr);

			home_cells_.back().TH = robot::instance()->getPoseAngle();
			PP_INFO();

			state_i_ = st_clean;
			stateInit(state_i_);
			action_i_ = ac_null;
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
			ROS_INFO("\033old_dir_(%d)", old_dir_);
			if (clean_path_algorithm_->generatePath(nav_map, nav_map.getCurrCell(), old_dir_, plan_path_))
			{
				new_dir_ = (MapDirection)plan_path_.front().TH;
				ROS_ERROR("new_dir_(%d)", new_dir_);
				PP_INFO();
				plan_path_.pop_front();
				clean_path_algorithm_->displayPath(plan_path_);
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
	if (!isInitFinished_)
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

