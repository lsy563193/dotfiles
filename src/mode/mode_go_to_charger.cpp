//
//	Created by lsy on 17-12-20.
//

#include <event_manager.h>
#include <serial.h>
#include <wheel.hpp>
#include <brush.h>
#include <vacuum.h>
#include <water_tank.hpp>
#include <gyro.h>
#include <key.h>
#include <beeper.h>
#include <remote.hpp>
#include <charger.h>
#include "speaker.h"
#include "wifi/wifi.h"
#include "mode.hpp"
#include "cliff.h"

ModeGoToCharger::ModeGoToCharger()
{
	ROS_WARN("%s, %d: Entering go to charger mode\n=========================", __FUNCTION__, __LINE__);
	event_manager_register_handler(this);
	event_manager_reset_status();
	event_manager_set_enable(true);

	serial.setWorkMode(WORK_MODE);

	speaker.play(VOICE_GO_HOME_MODE, false);
	sp_state = st_init.get();
	sp_state->init();
	sp_action_.reset(new ActionOpenGyro);
	action_i_ = ac_open_gyro;
	mode_i_ = md_go_to_charger;
	IMoveType::sp_mode_ = this;
	s_wifi.setWorkMode(cm_exploration);
	s_wifi.taskPushBack(S_Wifi::ACT::ACT_UPLOAD_STATUS);
	s_wifi.resetReceivedWorkMode();
}

ModeGoToCharger::~ModeGoToCharger()
{
	event_manager_set_enable(false);
	wheel.stop();
	brush.stop();
	brush.unblockMainBrushSlowOperation();
	vacuum.stop();
	water_tank.stop(WaterTank::operate_option::swing_motor_and_pump);
	gyro.setTiltCheckingEnable(false);
	if (ev.cliff_all_triggered)
		speaker.play(VOICE_ERROR_LIFT_UP);
	ROS_WARN("%s %d: Exit.", __FUNCTION__, __LINE__);
}

bool ModeGoToCharger::isExit()
{
	if(ev.key_clean_pressed)
	{
		ROS_WARN("%s %d:.", __FUNCTION__, __LINE__);
		setNextMode(md_idle);
		ev.cliff_all_triggered = false;
		ev.key_clean_pressed = false;
		return true;
	}

	if(ev.fatal_quit)
	{
		ROS_WARN("%s %d: Exit to idle mode by fatal quit.", __FUNCTION__, __LINE__);
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
		setNextMode(md_idle);
		return true;
	}

	return false;
}

bool ModeGoToCharger::isFinish()
{
	if ((action_i_ != ac_exception_resume) && isExceptionTriggered())
	{
		ROS_WARN("%s %d: Exception triggered.", __FUNCTION__, __LINE__);
		action_i_ = ac_exception_resume;
		genNextAction();
	}

	if(sp_action_->isFinish())
	{
//		PP_INFO();
		action_i_ = getNextAction();
		genNextAction();
		if(sp_action_ == nullptr)
		{
			if(ev.charge_detect)
				setNextMode(md_charge);
			else
				setNextMode(md_idle);
			return true;
		}
	}

	return false;
}

int ModeGoToCharger::getNextAction()
{
//	PP_INFO();
	if (action_i_ == ac_exception_resume && !ev.fatal_quit)
	{
		if (gyro.isOn())
		{
			sp_state = st_init.get();
			sp_state->init();
			return ac_go_to_charger;
		}
		else
			return ac_open_gyro;
	}
	else if (action_i_ == ac_open_gyro)
	{
		sp_state = st_init.get();
		sp_state->init();
		return ac_go_to_charger;
	}
	return ac_null;
}

void ModeGoToCharger::keyClean(bool state_now, bool state_last)
{
	ROS_WARN("%s %d: key clean.", __FUNCTION__, __LINE__);

	beeper.beepForCommand(VALID);
	wheel.stop();

	// Wait for key released.
	while (key.getPressStatus())
		usleep(20000);
	ev.key_clean_pressed = true;
	ROS_WARN("%s %d: Key clean is released.", __FUNCTION__, __LINE__);

	key.resetTriggerStatus();
}

void ModeGoToCharger::overCurrentWheelLeft(bool state_now, bool state_last)
{
	ROS_WARN("%s %d: Left wheel oc.", __FUNCTION__, __LINE__);
	ev.oc_wheel_left = true;
}

void ModeGoToCharger::overCurrentWheelRight(bool state_now, bool state_last)
{
	ROS_WARN("%s %d: Right wheel oc.", __FUNCTION__, __LINE__);
	ev.oc_wheel_right = true;
}

void ModeGoToCharger::remoteClean(bool state_now, bool state_last)
{
	ROS_WARN("%s %d: remote clean.", __FUNCTION__, __LINE__);

	beeper.beepForCommand(VALID);
	wheel.stop();
	ev.key_clean_pressed = true;
	remote.reset();
}

void ModeGoToCharger::cliffAll(bool state_now, bool state_last)
{
	auto cliff_all_start_time_ = ros::Time::now().toSec();
	ROS_INFO("%s,%d Wait 0.2s to confirm if cliffAll triggered",__FUNCTION__,__LINE__);
	while (ros::Time::now().toSec() - cliff_all_start_time_ < 0.2)
		wheel.stop();

	if (cliff.getStatus() == BLOCK_ALL)
	{
		ROS_WARN("%s %d: Cliff all.", __FUNCTION__, __LINE__);
		ev.cliff_all_triggered = true;
	}
}

void ModeGoToCharger::chargeDetect(bool state_now, bool state_last)
{
	if (charger.isDirected())
	{
		ROS_WARN("%s %d: Set ev.chargeDetect.", __FUNCTION__, __LINE__);
		ev.charge_detect = charger.getChargeStatus();
	}

}

void ModeGoToCharger::wifiSetWaterTank()
{
	// DO NOT CHANGE THE SWING MOTOR AND PUMP!
}

void ModeGoToCharger::setVacuum()
{
	// DO NOT CHANGE THE VACUUM!
}

