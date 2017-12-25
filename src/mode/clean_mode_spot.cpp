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
	IMoveType::sp_cm_.reset(this);
	speaker.play(SPEAKER_CLEANING_SPOT);
	usleep(200000);
	vacuum.setMode(Vac_Save);
	brush.setLeftPwm(50);
	brush.setRightPwm(50);
	brush.setMainPwm(50);
	clean_path_algorithm_.reset(new SpotCleanPathAlgorithm());
	go_home_path_algorithm_.reset();
	has_aligned_and_open_slam = false;
}

CleanModeSpot::~CleanModeSpot()
{
	wheel.stop();
	brush.stop();
	vacuum.stop();
	lidar.motorCtrl(OFF);
	lidar.setScanOriginalReady(0);

	robot::instance()->setBaselinkFrameType(Odom_Position_Odom_Angle);
	slam.stop();
	odom.setAngleOffset(0);
}

bool CleanModeSpot::isFinish()
{
	return ACleanMode::isFinish();
}

bool CleanModeSpot::mapMark()
{

	clean_path_algorithm_->displayPath(passed_path_);
	if (action_i_ == ac_linear) {
		PP_INFO()
		nav_map.setCleaned(passed_path_);
	}

	if (state_i_ == st_trapped)
		nav_map.markRobot(CLEAN_MAP);

	nav_map.setBlocks();
	PP_INFO()
	nav_map.print(CLEAN_MAP, nav_map.getXCell(), nav_map.getYCell());

	passed_path_.clear();
	return false;
}

bool CleanModeSpot::isExit()
{
	if (action_i_ == ac_pause && sp_action_->isTimeUp())
	{
		ROS_WARN("%s %d:.", __FUNCTION__, __LINE__);
		setNextMode(md_sleep);
		return true;
	}

	if (action_i_ == ac_pause && sp_action_->isExit())
	{
		ROS_WARN("%s %d:.", __FUNCTION__, __LINE__);
		setNextMode(md_idle);
		return true;
	}

	if (ev.fatal_quit || ev.key_long_pressed || ev.cliff_all_triggered || sp_action_->isExit())
	{
		ROS_WARN("%s %d:.", __FUNCTION__, __LINE__);
		setNextMode(md_idle);
		return true;
	}

	if (ev.charge_detect >= 3)
	{
		ROS_WARN("%s %d: Exit for directly charge.", __FUNCTION__, __LINE__);
		setNextMode(md_charge);
		return true;
	}

	return false;
}

bool CleanModeSpot::setNextAction()
{	
	if(isInitState()){	
		PP_INFO();
		if (action_i_ == ac_open_gyro)
		{
			PP_INFO();
			if (charger.isOnStub())
				action_i_ = ac_back_form_charger;
			else
				action_i_ = ac_open_lidar;
		}
		else if (action_i_ == ac_open_lidar)
		{
			if (!has_aligned_and_open_slam)
				action_i_ = ac_align;
			else
				action_i_ = ac_null;
		}
		else if (action_i_ == ac_align)
			action_i_ = ac_open_slam;
		genNextAction();
	}
	else{

		if (action_i_ == ac_open_slam)
		{
			has_aligned_and_open_slam == true;
			PP_INFO();
		}
		if (state_i_ == st_clean)
		{
			PP_INFO();
			if(plan_path_.size() >= 2)
				action_i_ = ac_linear;
			else
				action_i_ = ac_null;
		}
		genNextAction();
	}
	return action_i_ != ac_null;
}
