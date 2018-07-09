//
// Created by lsy563193 on 12/4/17.
//

#include <robot.hpp>
#include <vacuum.h>
#include <water_tank.hpp>
#include <event_manager.h>
#include "mode.hpp"
#include "action.hpp"
#include "movement.hpp"
#include "move_type.hpp"
#include "log.h"

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
	if (ev.bumper_jam)
		ROS_WARN("%s %d: Bumper jam.", __FUNCTION__, __LINE__);
	if (ev.lidar_bumper_jam)
		ROS_WARN("%s %d: Lidar bumper jam.", __FUNCTION__, __LINE__);
	if (ev.cliff_jam)
		ROS_WARN("%s %d: Cliff jam.", __FUNCTION__, __LINE__);
	if (ev.tilt_jam)
		ROS_WARN("%s %d: Tilt jam.", __FUNCTION__, __LINE__);
	if (ev.cliff_all_triggered)
		ROS_WARN("%s %d: Cliff all triggered.", __FUNCTION__, __LINE__);
	if (ev.oc_wheel_left)
		ROS_WARN("%s %d: Left wheel oc.", __FUNCTION__, __LINE__);
	if (ev.oc_wheel_right)
		ROS_WARN("%s %d: Right wheel oc.", __FUNCTION__, __LINE__);
	if (ev.oc_vacuum)
		ROS_WARN("%s %d: Vacuum oc.", __FUNCTION__, __LINE__);
	if (ev.lidar_stuck)
		ROS_WARN("%s %d: Lidar stuck.", __FUNCTION__, __LINE__);
	if (ev.robot_stuck)
	{
		current_action_i_ = action_i_;
		ROS_WARN("%s %d: Robot stuck.", __FUNCTION__, __LINE__);
	}
	if (ev.oc_brush_main)
		ROS_WARN("%s %d: Main brush oc.", __FUNCTION__, __LINE__);
	if (ev.left_wheel_cliff || ev.right_wheel_cliff)
		ROS_WARN("%s %d: Wheel cliff triggered.", __FUNCTION__, __LINE__);
	if (ev.gyro_error)
		ROS_WARN("%s %d: Gyro error.", __FUNCTION__, __LINE__);
	if (ev.cliff_turn)
		ROS_WARN("%s %d: Cliff turn.", __FUNCTION__, __LINE__);

	return ev.bumper_jam || ev.lidar_bumper_jam || ev.cliff_jam || ev.tilt_jam || ev.cliff_all_triggered ||
		   ev.oc_wheel_left || ev.oc_wheel_right || ev.oc_vacuum || ev.lidar_stuck || ev.robot_stuck ||
		   ev.oc_brush_main || ev.left_wheel_cliff || ev.right_wheel_cliff || ev.gyro_error || ev.cliff_turn;
}

void Mode::genNextAction()
{
	INFO_GREEN("before genNextAction");

	switch (action_i_) {
		case ac_open_gyro :
			sp_action_.reset(new ActionOpenGyro);
			break;
		case ac_open_gyro_and_lidar :
			sp_action_.reset(new ActionOpenGyroAndLidar);
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
			sp_action_.reset(new MovementExceptionResume(current_action_i_));
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
			ROS_INFO("%s %d: Reset action to null.", __FUNCTION__, __LINE__);
			break;
	}
	INFO_GREEN("after genNextAction");
}

void Mode::wifiSetWaterTank()
{
	// Just for protection that water tank should not be working.
	if (water_tank.getStatus(WaterTank::operate_option::swing_motor))
		water_tank.stop(WaterTank::operate_option::swing_motor);
	if (water_tank.getStatus(WaterTank::operate_option::pump))
		water_tank.stop(WaterTank::operate_option::pump);
}

void Mode::setVacuum()
{
	// Just for protection that vacuum should not be working.
	if (vacuum.isOn())
		vacuum.stop();
}
