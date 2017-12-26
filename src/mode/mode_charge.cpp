//
// Created by austin on 17-12-6.
//

#include "pp.h"
#include <arch.hpp>
#include "dev.h"

ModeCharge::ModeCharge()
{
	ROS_INFO("%s %d: Entering Charge mode\n=========================" , __FUNCTION__, __LINE__);

	key.resetTriggerStatus();
	c_rcon.resetStatus();
	remote.reset();
	robot_timer.resetPlanStatus();
	event_manager_register_handler(this);
	event_manager_reset_status();
	event_manager_set_enable(true);
	sp_action_.reset(new MovementCharge);
	action_i_ = ac_charge;
	serial.setCleanMode(Clean_Mode_Charging);

	plan_activated_status_ = false;
	PP_INFO();
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
		ROS_WARN("%s %d:.", __FUNCTION__, __LINE__);
		setNextMode(cm_navigation);
		return true;
	}

	return false;
}

bool ModeCharge::isFinish()
{
	if (sp_action_->isFinish())
	{
		// todo: Temperary not quit charging if full.
		if (charger.isOnStub() && battery.isFull())
		{
			led.set_mode(LED_STEADY, LED_GREEN);
			return false;
		}

		setNextMode(md_idle);
		return true;
	}

	return false;
}

void ModeCharge::remoteClean(bool state_now, bool state_last)
{
	ev.key_clean_pressed = true;
	ROS_WARN("%s %d: Waked up by remote key clean.", __FUNCTION__, __LINE__);
}

void ModeCharge::keyClean(bool state_now, bool state_last)
{
	ROS_WARN("%s %d: Waked up by key clean.", __FUNCTION__, __LINE__);
	ev.key_clean_pressed = true;

	// Wake up serial so it can beep.
	serial.wakeUp();
	beeper.play_for_command(VALID);

	// Wait for key released.
	while (key.getPressStatus())
		usleep(20000);
	ROS_WARN("%s %d: Key clean is released.", __FUNCTION__, __LINE__);

	key.resetTriggerStatus();
}

void ModeCharge::remotePlan(bool state_now, bool state_last)
{
	if (robot_timer.getPlanStatus() == 3)
	{
		ROS_WARN("%s %d: Plan activated.", __FUNCTION__, __LINE__);
		if (error.get() != ERROR_CODE_NONE)
		{
			ROS_INFO("%s %d: Error exists, so cancel the appointment.", __FUNCTION__, __LINE__);
			error.alarm();
			speaker.play(VOICE_CANCEL_APPOINTMENT);
		}
		else if(cliff.get_status() & (BLOCK_LEFT|BLOCK_FRONT|BLOCK_RIGHT))
		{
			ROS_WARN("%s %d: Plan not activated not valid because of robot lifted up.", __FUNCTION__, __LINE__);
			speaker.play(VOICE_ERROR_LIFT_UP);
			speaker.play(VOICE_CANCEL_APPOINTMENT);
		}
		else if (!battery.isReadyToClean())
		{
			ROS_WARN("%s %d: Plan not activated not valid because of battery not ready to clean.", __FUNCTION__, __LINE__);
			speaker.play(VOICE_BATTERY_LOW);
			speaker.play(VOICE_CANCEL_APPOINTMENT);
		}
		else if (charger.getChargeStatus() == 4)
		{
			ROS_WARN("%s %d: Plan not activated not valid because of charging with adapter.", __FUNCTION__, __LINE__);
			// todo: speaker.play(???);
			speaker.play(VOICE_CANCEL_APPOINTMENT);
		}
		else
		{
			// Sleep for 50ms cause the status 3 will be sent for 3 times.
			usleep(50000);
			plan_activated_status_ = true;
		}
		robot_timer.resetPlanStatus();
	}
}
