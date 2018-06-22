//
// Created by austin on 17-12-8.
//

#include <event_manager.h>
#include <robot.hpp>
#include <move_type.hpp>
#include <gyro.h>
#include <rcon.h>
#include <key.h>
#include <remote.hpp>
#include <wifi/wifi.h>
#include <wheel.hpp>
#include <brush.h>
#include <water_tank.hpp>
#include <vacuum.h>
#include <battery.h>
#include <speaker.h>
#include <beeper.h>
#include <charger.h>
#include "mode.hpp"
#include "appointment.h"

ModeRemote::ModeRemote()
{//use dynamic then you can limit using derived class member
	system("turbo_cpu.sh");
	ROS_WARN("%s %d: Entering remote mode\n=========================" , __FUNCTION__, __LINE__);
	event_manager_register_handler(this);
	event_manager_set_enable(true);

	mode_i_ = md_remote;
	IMoveType::sp_mode_ = this;

	serial.setWorkMode(WORK_MODE);
	if (gyro.isOn())
	{
		sp_state = st_clean.get();
		sp_state->init();
		action_i_ = ac_remote;
	}
	else
	{
		sp_state = st_init.get();
		sp_state->init();
		action_i_ = ac_open_gyro;
	}
	genNextAction();
	key.resetTriggerStatus();
	c_rcon.resetStatus();
	remote.reset();
	appmt_obj.resetPlanStatus();
	event_manager_reset_status();

	remote_mode_time_stamp_ = ros::Time::now().toSec();

	s_wifi.setWorkMode(md_remote);
	s_wifi.taskPushBack(S_Wifi::ACT::ACT_UPLOAD_STATUS);
	s_wifi.resetReceivedWorkMode();
}

ModeRemote::~ModeRemote()
{
	event_manager_set_enable(false);
	sp_action_.reset();

	wheel.stop();
	brush.stop();
	brush.unblockMainBrushSlowOperation();
	vacuum.stop();
	water_tank.stop(WaterTank::operate_option::swing_motor_and_pump);

	// Wait for battery recovery from operating motors.
	usleep(200000);
	battery.forceUpdate();
	gyro.setTiltCheckingEnable(false);
	if (ev.cliff_all_triggered)
		speaker.play(VOICE_ERROR_LIFT_UP);
	ROS_INFO("%s %d: Exit remote mode.", __FUNCTION__, __LINE__);
}

bool ModeRemote::isExit()
{
	if (ev.key_clean_pressed || ev.remote_spot || ev.remote_home || ev.remote_follow_wall)
	{
		ROS_WARN("%s %d: Exit to idle mode.", __FUNCTION__, __LINE__);
		setNextMode(md_idle);
		return true;
	}

	if(ev.fatal_quit)
	{
		ROS_WARN("%s %d: Exit to idle mode by fatal quit.", __FUNCTION__, __LINE__);
		setNextMode(md_idle);
		return true;
	}

/*
	if (ev.cliff_all_triggered)
	{
		ROS_WARN("%s %d: Exit to idle mode.", __FUNCTION__, __LINE__);
		speaker.play(VOICE_ERROR_LIFT_UP);
		setNextMode(md_idle);
		return true;
	}
*/

	if (ev.charge_detect)
	{
		ROS_WARN("%s %d: Exit to charge mode.", __FUNCTION__, __LINE__);
		setNextMode(md_charge);
		return true;
	}

	if (ev.battery_low)
	{
		ROS_WARN("%s %d: Exit to idle mode for low battery(%.2fV).", __FUNCTION__, __LINE__, battery.getVoltage() / 100.0);
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

	if (s_wifi.receiveHome())
	{
		ROS_WARN("%s %d: Exit for wifi home.", __FUNCTION__, __LINE__);
		setNextMode(cm_exploration);
		return true;
	}

	if (s_wifi.receiveSpot())
	{
		ROS_WARN("%s %d: Exit for wifi spot.", __FUNCTION__, __LINE__);
		setNextMode(cm_spot);
		return true;
	}

	if (s_wifi.receiveFollowWall())
	{
		ROS_WARN("%s %d: Exit for wifi follow wall.", __FUNCTION__, __LINE__);
		setNextMode(cm_wall_follow);
		return true;
	}

	return false;
}

bool ModeRemote::isFinish()
{
	if ((action_i_ != ac_exception_resume) && isExceptionTriggered())
	{
		ROS_WARN("%s %d: Exception triggered.", __FUNCTION__, __LINE__);
		action_i_ = ac_exception_resume;
		genNextAction();
	}

	if (sp_action_->isFinish())
	{
		PP_INFO();
		action_i_ = getNextAction();
		genNextAction();
		if (sp_action_ == nullptr)
		{
			setNextMode(md_idle);
			return true;
		}
	}

	return false;
}

int ModeRemote::getNextAction()
{
	if (action_i_ == ac_exception_resume && !ev.fatal_quit)
	{
		if (gyro.isOn())
		{
			sp_state = st_clean.get();
			sp_state->init();
			return ac_remote;
		}
		else
			return ac_open_gyro;
	}
	else if(action_i_ == ac_open_gyro)
	{
		sp_state = st_clean.get();
		sp_state->init();
		return ac_remote;
	}

	return ac_null;
}

void ModeRemote::remoteClean(bool state_now, bool state_last)
{
	ROS_WARN("%s %d: remote clean.", __FUNCTION__, __LINE__);
	beeper.beepForCommand(VALID);
	ev.key_clean_pressed = true;
	remote.reset();
}

void ModeRemote::remoteDirectionForward(bool state_now, bool state_last)
{
	ROS_WARN("%s %d: remote forward.", __FUNCTION__, __LINE__);
	beeper.beepForCommand(VALID);
	ev.remote_direction_forward = true;
	if(action_i_ == ac_open_gyro)
		MoveTypeRemote::forwardStart();
	remote.reset();
}

void ModeRemote::remoteDirectionLeft(bool state_now, bool state_last)
{
	ROS_WARN("%s %d: remote left.", __FUNCTION__, __LINE__);
	beeper.beepForCommand(VALID);
	ev.remote_direction_left = true;
	if (action_i_ == ac_open_gyro)
		MoveTypeRemote::leftStart();
	remote.reset();
}

void ModeRemote::remoteDirectionRight(bool state_now, bool state_last)
{
	ROS_WARN("%s %d: remote right.", __FUNCTION__, __LINE__);
	beeper.beepForCommand(VALID);
	ev.remote_direction_right = true;
	if (action_i_ == ac_open_gyro)
		MoveTypeRemote::rightStart();
	remote.reset();
}

void ModeRemote::remoteMax(bool state_now, bool state_last)
{
	ROS_WARN("%s %d: Remote max is pressed.", __FUNCTION__, __LINE__);
	if(water_tank.getStatus(WaterTank::operate_option::swing_motor)){
		beeper.beepForCommand(INVALID);
	}
	else{
		beeper.beepForCommand(VALID);
		vacuum.setForUserSetMaxMode(!vacuum.isUserSetMaxMode());
		setVacuum();
	}
	remote.reset();
}

void ModeRemote::keyClean(bool state_now, bool state_last)
{
	ROS_WARN("%s %d: key clean.", __FUNCTION__, __LINE__);

	beeper.beepForCommand(VALID);

	// Wait for key released.
	while (key.getPressStatus())
		usleep(20000);

	ev.key_clean_pressed = true;
	ROS_WARN("%s %d: Key clean is released.", __FUNCTION__, __LINE__);

	key.resetTriggerStatus();
}

void ModeRemote::chargeDetect(bool state_now, bool state_last)
{
	ROS_WARN("%s %d: Charge detect.", __FUNCTION__, __LINE__);
	ev.charge_detect = charger.getChargeStatus();
}

void ModeRemote::remoteWallFollow(bool state_now, bool state_last)
{
	ROS_WARN("%s %d: remote wall follow.", __FUNCTION__, __LINE__);
	beeper.beepForCommand(VALID);
	ev.remote_follow_wall = true;
	remote.reset();
}

void ModeRemote::remoteSpot(bool state_now, bool state_last)
{
	ROS_WARN("%s %d: remote spot.", __FUNCTION__, __LINE__);
	beeper.beepForCommand(VALID);
	ev.remote_spot = true;
	remote.reset();
}

void ModeRemote::remoteHome(bool state_now, bool state_last)
{
	ROS_WARN("%s %d: remote home.", __FUNCTION__, __LINE__);
	beeper.beepForCommand(VALID);
	ev.remote_home = true;
	remote.reset();
}

void ModeRemote::cliffAll(bool state_now, bool state_last)
{
	ROS_WARN("%s %d: Cliff all.", __FUNCTION__, __LINE__);

	ev.cliff_all_triggered = true;
}

void ModeRemote::wifiSetWaterTank()
{
	if (!water_tank.getStatus(WaterTank::operate_option::swing_motor))
		return;

	auto user_set_swing_motor_mode = water_tank.getUserSetSwingMotorMode();
	if (water_tank.getCurrentSwingMotorMode() != user_set_swing_motor_mode)
		water_tank.setCurrentSwingMotorMode(user_set_swing_motor_mode);

	auto user_set_pump_mode = water_tank.getUserSetPumpMode();
	if (water_tank.getStatus(WaterTank::operate_option::pump) &&
		water_tank.getCurrentPumpMode() != user_set_pump_mode)
		water_tank.setCurrentPumpMode(user_set_pump_mode);
}

void ModeRemote::setVacuum()
{
	if (water_tank.getStatus(WaterTank::operate_option::swing_motor))
		return;

	auto user_set_max_mode = vacuum.isUserSetMaxMode();
	if (vacuum.isCurrentMaxMode() != user_set_max_mode)
	{
		vacuum.setSpeedByUserSetMode();
		speaker.play(vacuum.isCurrentMaxMode() ? VOICE_VACCUM_MAX : VOICE_VACUUM_NORMAL);
	}
}

void ModeRemote::batteryLow(bool state_now, bool state_last)
{
	if (!ev.battery_low)
	{
		ROS_WARN("%s %d: Low battery(%.2fV).", __FUNCTION__, __LINE__, battery.getVoltage() / 100.0);
		speaker.play(VOICE_BATTERY_LOW, false);
		ev.battery_low = true;
	}
}
