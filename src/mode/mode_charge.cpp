//
// Created by austin on 17-12-6.
//

#include <event_manager.h>
#include <key.h>
#include <rcon.h>
#include <remote.hpp>
#include <wifi/wifi.h>
#include <charger.h>
#include <speaker.h>
#include <battery.h>
#include <beeper.h>
#include <water_tank.hpp>
#include <vacuum.h>
#include "error.h"
#include "mode.hpp"
#include <robot.hpp>
#include "appointment.h"

ModeCharge::ModeCharge()
{
	ROS_WARN("%s %d: Entering Charge mode\n=========================" , __FUNCTION__, __LINE__);
	system("unturbo_cpu.sh");

	robot::instance()->setBatterLow(false);
	robot::instance()->setBatterLow2(false);
	key.resetTriggerStatus();
	c_rcon.resetStatus();
	remote.reset();
	appmt_obj.resetPlanStatus();
	event_manager_register_handler(this);
	event_manager_reset_status();
	event_manager_set_enable(true);
	sp_action_.reset(new MovementCharge);
	action_i_ = ac_charge;
	mode_i_ = md_charge;
	serial.setWorkMode(CHARGE_MODE);
	s_wifi.resetReceivedWorkMode();
	plan_activated_status_ = false;
	sp_state = state_charge.get();
	sp_state->init();
	IMoveType::sp_mode_ = this;
}

ModeCharge::~ModeCharge()
{
	event_manager_set_enable(false);
	sp_action_.reset();
	ROS_WARN("%s %d: Exit charge mode.", __FUNCTION__, __LINE__);
}

bool ModeCharge::isExit()
{
	if (plan_activated_status_)
	{
		if (charger.isDirected())
		{
			ROS_WARN("%s %d: Plan not activated not valid because of charging with adapter.", __FUNCTION__, __LINE__);
			sp_state = state_charge.get();
			sp_state->init();
			speaker.play(VOICE_PLEASE_PULL_OUT_THE_PLUG, false);
			sp_action_.reset(new MovementCharge);
			plan_activated_status_ = false;
			return false;
		}

		if (robot_error.get() != ERROR_CODE_NONE)
		{
			if (robot_error.clear(robot_error.get()))
				ROS_WARN("%s %d: Clear the error %x.", __FUNCTION__, __LINE__, robot_error.get());
			else
			{
				sp_state = state_charge.get();
				sp_state->init();
				robot_error.alarm(false);
				sp_action_.reset(new MovementCharge);
				plan_activated_status_ = false;
				ROS_WARN("%s %d: Error exists, so cancel the appointment.", __FUNCTION__, __LINE__);
				return false;
			}
		}

		if (!battery.isReadyToClean())
		{
			ROS_WARN("%s %d: Plan not activated not valid because of battery not ready to clean.", __FUNCTION__,
					 __LINE__);
			sp_state = state_charge.get();
			sp_state->init();
			speaker.play(VOICE_BATTERY_LOW, false);
			sp_action_.reset(new MovementCharge);
			plan_activated_status_ = false;
			return false;
		} else
		{
			ROS_WARN("%s %d: Charge mode receives plan, change to navigation mode.", __FUNCTION__, __LINE__);
			setNextMode(cm_navigation);
			if (action_i_ == ac_charge)
			{
				auto p_movement_charge = boost::dynamic_pointer_cast<MovementCharge>(sp_action_);
				if (p_movement_charge->stillCharging())
					charger.enterNavFromChargeMode(true);
			}
			ACleanMode::plan_activation_ = true;
			return true;
		}
	}

	if (ev.key_clean_pressed)
	{
		if (isStateSleep())
		{
			sp_state = state_charge.get();
			sp_state->init();
			sp_action_.reset(new MovementCharge);
			ev.key_clean_pressed = false;
		} else if (!battery.isReadyToClean())
		{
			ROS_WARN("%s %d: Battery not ready to clean.", __FUNCTION__, __LINE__);
			sp_state = state_charge.get();
			sp_state->init();
			speaker.play(VOICE_BATTERY_LOW, false);
			sp_action_.reset(new MovementCharge);
			ev.key_clean_pressed = false;
			return false;
		} else if (charger.isDirected())
		{
			ROS_WARN("%s %d: Charging with adapter.", __FUNCTION__, __LINE__);
			sp_state = state_charge.get();
			sp_state->init();
			speaker.play(VOICE_PLEASE_PULL_OUT_THE_PLUG, false);
			sp_action_.reset(new MovementCharge);
			ev.key_clean_pressed = false;
			return false;
		} else
		{
			ROS_WARN("%s %d: Charge mode receives remote clean or key clean, change to navigation mode.", __FUNCTION__,
					 __LINE__);
			setNextMode(cm_navigation);
			if (action_i_ == ac_charge)
			{
				auto p_action = boost::dynamic_pointer_cast<MovementCharge>(sp_action_);
				if (p_action->stillCharging())
					charger.enterNavFromChargeMode(true);
			}
			return true;
		}
	}

	if (s_wifi.receivePlan1())
	{
		if (!battery.isReadyToClean())
		{
			ROS_WARN("%s %d: Battery not ready to clean.", __FUNCTION__, __LINE__);
			sp_state = state_charge.get();
			sp_state->init();
			speaker.play(VOICE_BATTERY_LOW);
			sp_action_.reset(new MovementCharge);
			s_wifi.resetReceivedWorkMode();
			return false;
		} else if (charger.isDirected())
		{
			ROS_WARN("%s %d: Charging with adapter.", __FUNCTION__, __LINE__);
			sp_state = state_charge.get();
			sp_state->init();
			speaker.play(VOICE_PLEASE_PULL_OUT_THE_PLUG, false);
			sp_action_.reset(new MovementCharge);
			s_wifi.resetReceivedWorkMode();
			return false;
		} else
		{
			ROS_WARN("%s %d: Charge mode receives wifi plan1, change to navigation mode.", __FUNCTION__, __LINE__);
			setNextMode(cm_navigation);
			if (action_i_ == ac_charge)
			{
				auto p_action = boost::dynamic_pointer_cast<MovementCharge>(sp_action_);
				if (p_action->stillCharging())
					charger.enterNavFromChargeMode(true);
			}
			return true;
		}
	}

	return false;
}

bool ModeCharge::isFinish()
{
	if (isStateCharge())
	{
		auto p_movement_charge = boost::dynamic_pointer_cast<MovementCharge>(sp_action_);
		if (p_movement_charge->batteryFullAndSleep())
		{
			sp_state = state_sleep.get();
			sp_state->init();
		}
	}

	if (sp_action_->isFinish())
	{
		setNextMode(md_idle);
		return true;
	}

	return false;
}

void ModeCharge::remoteClean(bool state_now, bool state_last)
{
	ROS_WARN("%s %d: Receive remote key clean.", __FUNCTION__, __LINE__);
	ev.key_clean_pressed = true;
	beeper.beepForCommand(VALID);
	remote.reset();
}

void ModeCharge::keyClean(bool state_now, bool state_last)
{
	ROS_WARN("%s %d: Receive key clean.", __FUNCTION__, __LINE__);
	ev.key_clean_pressed = true;
	beeper.beepForCommand(VALID);

	// Wait for key released.
	while (key.getPressStatus())
		usleep(20000);
	ROS_WARN("%s %d: Key clean is released.", __FUNCTION__, __LINE__);

	key.resetTriggerStatus();
}

void ModeCharge::remotePlan(bool state_now, bool state_last)
{
	if (!plan_activated_status_ && appmt_obj.getPlanStatus() > 2)
	{
		appmt_obj.resetPlanStatus();
		appmt_obj.timesUp();
		INFO_YELLOW("Plan activated.");
		// Sleep for 30ms for M0 setting the next plan status.
		usleep(30000);
//		appmt_obj.resetPlanStatus();
//		// Sleep for 30ms for M0 resetting the plan status.
//		usleep(30000);
		plan_activated_status_ = true;
	}
	else
	{
		if (isStateCharge())
			EventHandle::remotePlan(state_now, state_last);
	}
}

bool ModeCharge::allowRemoteUpdatePlan()
{
	return isStateCharge();
}

void ModeCharge::remoteKeyHandler(bool state_now, bool state_last)
{
	if (isStateCharge())
	{
		if (remote.isKeyTrigger(REMOTE_MAX))
		{
			ROS_WARN("%s %d: Receive remote max.", __FUNCTION__, __LINE__);
			if (water_tank.getStatus(WaterTank::operate_option::swing_motor))
				beeper.beepForCommand(INVALID);
			else
			{
				beeper.beepForCommand(VALID);
				vacuum.setForUserSetMaxMode(!vacuum.isUserSetMaxMode());
				speaker.play(vacuum.isUserSetMaxMode() ? VOICE_VACCUM_MAX : VOICE_VACUUM_NORMAL);
				setVacuum();
			}
		}
		else
			ROS_WARN("%s %d: Receive %d but not valid during charge.", __FUNCTION__, __LINE__);
	}
	else
		ROS_WARN("%s %d: Receive %d but not valid during fake sleep.", __FUNCTION__, __LINE__);
	remote.reset();
}
