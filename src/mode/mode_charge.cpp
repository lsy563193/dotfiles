//
// Created by austin on 17-12-6.
//

#include <robot.hpp>
#include "error.h"
#include "dev.h"
#include "mode.hpp"
#include "appointment.h"

ModeCharge::ModeCharge()
{
	ROS_INFO("%s %d: Entering Charge mode\n=========================" , __FUNCTION__, __LINE__);

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
	plan_activated_status_ = false;
	sp_state = state_charge.get();
	sp_state->init();
	IMoveType::sp_mode_ = this;
}

ModeCharge::~ModeCharge()
{
	event_manager_set_enable(false);
	sp_action_.reset();
	ROS_INFO("%s %d: Exit charge mode.", __FUNCTION__, __LINE__);
}

bool ModeCharge::isExit()
{
	if (ev.key_clean_pressed || plan_activated_status_)
	{
		if (plan_activated_status_)
		{
			if (error.get() != ERROR_CODE_NONE)
			{
				if (error.clear(error.get()))
				{
					ROS_WARN("%s %d: Clear the error %x.", __FUNCTION__, __LINE__, error.get());
//					speaker.play(VOICE_CLEAR_ERROR_UNOFFICIAL, false);
				} else
				{
//					speaker.play(VOICE_CANCEL_APPOINTMENT_UNOFFICIAL, false);
					error.alarm();
				}
			}

			if (error.get() != ERROR_CODE_NONE)
				ROS_INFO("%s %d: Error exists, so cancel the appointment.", __FUNCTION__, __LINE__);
			else if (cliff.getStatus() & (BLOCK_LEFT | BLOCK_FRONT | BLOCK_RIGHT))
			{
				ROS_WARN("%s %d: Plan not activated not valid because of robot lifted up.", __FUNCTION__, __LINE__);
				speaker.play(VOICE_ERROR_LIFT_UP);
			} else if (!battery.isReadyToClean())
			{
				ROS_WARN("%s %d: Plan not activated not valid because of battery not ready to clean.", __FUNCTION__, __LINE__);
				speaker.play(VOICE_BATTERY_LOW);
			} else if (charger.isDirected())
			{
				ROS_WARN("%s %d: Plan not activated not valid because of charging with adapter.", __FUNCTION__, __LINE__);
				speaker.play(VOICE_PLEASE_PULL_OUT_THE_PLUG);
			} else{
				ROS_WARN("%s %d: Charge mode receives plan, change to navigation mode.", __FUNCTION__, __LINE__);
				setNextMode(cm_navigation);
				ACleanMode::plan_activation_ = true;
				return true;
			}
			plan_activated_status_ = false;
		}
		else
		{
			ROS_WARN("%s %d: Charge mode receives remote clean or clean key, change to navigation mode.", __FUNCTION__, __LINE__);
			setNextMode(cm_navigation);
			return true;
		}
	}
	return false;
}

bool ModeCharge::isFinish()
{
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
	if (charger.isDirected())
	{
		beeper.beepForCommand(INVALID);
		speaker.play(VOICE_PLEASE_PULL_OUT_THE_PLUG);
		ROS_WARN("%s %d: change charge mode to navigation mode by remote clean failed because of charging with adapter.", __FUNCTION__, __LINE__);
	}
	else
	{
		ev.key_clean_pressed = true;
		beeper.beepForCommand(VALID);
	}
	remote.reset();
}

void ModeCharge::keyClean(bool state_now, bool state_last)
{
	ROS_WARN("%s %d: Receive key clean.", __FUNCTION__, __LINE__);
	if (charger.isDirected())
	{
		beeper.beepForCommand(INVALID);
		speaker.play(VOICE_PLEASE_PULL_OUT_THE_PLUG);
		ROS_WARN("%s %d: change charge mode to navigation mode by key clean failed because of charging with adapter.", __FUNCTION__, __LINE__);
	}
	else
	{
		ev.key_clean_pressed = true;
		beeper.beepForCommand(VALID);
	}

	// Wait for key released.
	while (key.getPressStatus())
		usleep(20000);
	ROS_WARN("%s %d: Key clean is released.", __FUNCTION__, __LINE__);

	key.resetTriggerStatus();
}

void ModeCharge::remotePlan(bool state_now, bool state_last)
{
	if (appmt_obj.getPlanStatus() == 1)
	{
		beeper.beepForCommand(VALID);
		speaker.play(VOICE_APPOINTMENT_DONE);
		ROS_WARN("%s %d: Plan received.", __FUNCTION__, __LINE__);
	}
	else if (appmt_obj.getPlanStatus() == 2)
	{
		beeper.beepForCommand(VALID);
		speaker.play(VOICE_APPOINTMENT_DONE);
//		speaker.play(VOICE_CANCEL_APPOINTMENT_UNOFFICIAL);
		ROS_WARN("%s %d: Plan cancel received.", __FUNCTION__, __LINE__);
	}
	else if (appmt_obj.getPlanStatus() == 3)
	{
		ROS_WARN("%s %d: Plan activated.", __FUNCTION__, __LINE__);
		// Sleep for 50ms cause the status 3 will be sent for 3 times.
		usleep(50000);
		plan_activated_status_ = true;
	}
	appmt_obj.resetPlanStatus();
}
