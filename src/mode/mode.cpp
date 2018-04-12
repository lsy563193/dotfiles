//
// Created by lsy563193 on 12/4/17.
//

#include <dev.h>
#include <event_manager.h>
#include "mode.hpp"

boost::shared_ptr<IAction> Mode::sp_action_ = nullptr;

int Mode::next_mode_i_{};
//IAction* Mode::sp_action_ = nullptr;

void Mode::run()
{
	ROS_INFO("%s %d: Mode start running.", __FUNCTION__, __LINE__);
	bool eh_status_now = false, eh_status_last = false;

	while (ros::ok() && !core_thread_kill)
	{
//		PP_INFO();
		if (event_manager_check_event(&eh_status_now, &eh_status_last) == 1) {
			usleep(100);
			continue;
		}

//		PP_INFO();
		if (isExit())
		{
//			PP_INFO();
			return;
		}
//		PP_INFO();
		if (isFinish())
			return;

//		PP_INFO();
		sp_action_->run();
//		PP_INFO();
	}
}

bool Mode::isExit()
{
//	ROS_INFO("%s,%s,%d",__FILE__,__FUNCTION__, __LINE__);
	return false;
}

bool Mode::isFinish()
{
	return sp_action_->isFinish();
}

void Mode::setNextMode(int next_mode)
{
	next_mode_i_ = next_mode;
}

int Mode::getNextMode()
{
	return next_mode_i_;
}

bool Mode::isExceptionTriggered()
{
	if((ev.right_wheel_cliff || ev.left_wheel_cliff) && wheel_cliff_triggered_time_ == DBL_MAX)
		wheel_cliff_triggered_time_ = ros::Time::now().toSec();
	if(ros::Time::now().toSec() - wheel_cliff_triggered_time_ > 0.1)
	{
		if(wheel.getRightWheelCliffStatus() || wheel.getLeftWheelCliffStatus())
		{
			is_wheel_cliff_triggered = true;
			ROS_WARN("%s,%d,Enter exception by wheel cliff triggered over 100ms",__FUNCTION__,__LINE__);
		}
		else
		{
			ev.left_wheel_cliff = false;
			ev.right_wheel_cliff = false;
		}
		wheel_cliff_triggered_time_ = DBL_MAX;
	}

	return ev.bumper_jam || ev.lidar_bumper_jam || ev.cliff_jam || ev.cliff_all_triggered || ev.oc_wheel_left || ev.oc_wheel_right
						 || ev.oc_vacuum || ev.lidar_stuck || ev.robot_stuck || ev.oc_brush_main || ev.robot_slip || is_wheel_cliff_triggered;
}

void Mode::genNextAction()
{
	INFO_GREEN("before genNextAction");

	switch (action_i_) {
		case ac_open_gyro :
			sp_action_.reset(new ActionOpenGyro);
			break;
		case ac_back_from_charger :
			sp_action_.reset(new ActionBackFromCharger);
			break;
		case ac_open_lidar :
			sp_action_.reset(new ActionOpenLidar);
			break;
		case ac_align :
			sp_action_.reset(new ActionAlign);
			break;
		case ac_open_slam :
			sp_action_.reset(new ActionOpenSlam);
			break;
		case ac_pause :
			sp_action_.reset(new ActionPause);
			break;
		case ac_go_to_charger :
			sp_action_.reset(new MoveTypeGoToCharger);
			break;
		case ac_exception_resume :
			sp_action_.reset(new MovementExceptionResume);
			break;
		case ac_charge :
			sp_action_.reset(new MovementCharge);
			break;
		case ac_check_bumper :
			sp_action_.reset(new ActionCheckBumper);
			break;
		case ac_bumper_hit_test :
			sp_action_.reset(new MoveTypeBumperHitTest);
			break;
		case ac_check_vacuum :
			sp_action_.reset(new ActionCheckVacuum);
			break;
		case ac_remote :
			sp_action_.reset(new MoveTypeRemote());
			break;
		case ac_desk_test:
			sp_action_.reset(new MoveTypeDeskTest());
			break;
		case ac_gyro_test:
			sp_action_.reset(new MoveTypeGyroTest());
			break;
		case ac_water_tank_test:
			sp_action_.reset(new ActionCheckWaterTank());
			break;
		case ac_life_test:
			sp_action_.reset(new ActionLifeCheck());
			break;
		case ac_r16_test:
			sp_action_.reset(new ActionR16Test());
			break;
		default : //case ac_null :
			sp_action_.reset();
			break;
	}
	INFO_GREEN("after genNextAction");
}

