//
// Created by austin on 17-12-5.
//

#include <error.h>
#include <event_manager.h>
#include "dev.h"
#include "mode.hpp"

ModeSleep::ModeSleep()
{
	ROS_INFO("%s %d: Entering Sleep mode\n=========================" , __FUNCTION__, __LINE__);
	event_manager_register_handler(this);
	event_manager_reset_status();
	event_manager_set_enable(true);
	action_i_ = ac_sleep;

	key.resetTriggerStatus();
	c_rcon.resetStatus();
	remote.reset();
	robot_timer.resetPlanStatus();

	plan_activated_status_ = false;
	sp_action_.reset(new ActionSleep);
    sp_state = st_sleep.get();
    sp_state->init();

}

ModeSleep::~ModeSleep()
{
	event_manager_set_enable(false);
	sp_action_.reset();
	// Wait 1.5s to avoid gyro can't open if switch to navigation is_max_clean_state_ too soon after waking up.
	usleep(1500000);
	robot::instance()->wake_up_time_ = time(NULL);
	ROS_INFO("%s %d: Exit sleep mode.", __FUNCTION__, __LINE__);
}

bool ModeSleep::isExit()
{
	if ((ev.key_clean_pressed || plan_activated_status_) && !serial.isMainBoardSleep())
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
				//speaker.play(???);
//				speaker.play(VOICE_CANCEL_APPOINTMENT_UNOFFICIAL);
			} else{
				ROS_WARN("%s %d: Sleep mode receives plan, change to navigation mode.", __FUNCTION__, __LINE__);
				setNextMode(cm_navigation);
				ACleanMode::plan_activation_ = true;
				return true;
			}

			ROS_WARN("%s %d: Sleep mode change to idle mode.", __FUNCTION__, __LINE__);
			plan_activated_status_ = false;
			setNextMode(md_idle);
			return true;
		}
		else
		{
			ROS_WARN("%s %d: Sleep mode receives remote clean or clean key, change to idle mode.", __FUNCTION__, __LINE__);
			setNextMode(md_idle);
			return true;
		}
	}

	if (ev.charge_detect && !serial.isMainBoardSleep())
	{
		ROS_WARN("%s %d: Charge detected, switch to charge mode.", __FUNCTION__, __LINE__);
		setNextMode(md_charge);
		return true;
	}

	if (ev.rcon_status && !serial.isMainBoardSleep())
	{
		ROS_WARN("%s %d: Rcon detected, switch to go to charger mode.", __FUNCTION__, __LINE__);
		setNextMode(md_go_to_charger);
		return true;
	}

	return false;
}

bool ModeSleep::isFinish()
{
	return false;
}

void ModeSleep::remoteClean(bool state_now, bool state_last)
{
	if (serial.isMainBoardSleep() && !ev.key_clean_pressed)
	{
		ev.key_clean_pressed = true;
		ROS_WARN("%s %d: Waked up by remote key clean.", __FUNCTION__, __LINE__);
		serial.setWorkMode(IDLE_MODE);
//		beeper.beepForCommand(VALID);
	}

	remote.reset();
}

void ModeSleep::keyClean(bool state_now, bool state_last)
{
	if (serial.isMainBoardSleep() && !ev.key_clean_pressed)
	{
		ROS_WARN("%s %d: Waked up by key clean.", __FUNCTION__, __LINE__);
		// Wake up main board.
		serial.setWorkMode(IDLE_MODE);

		ev.key_clean_pressed = true;
		ROS_WARN("%s %d: Key clean is released.", __FUNCTION__, __LINE__);
	}

	key.resetTriggerStatus();
}

void ModeSleep::chargeDetect(bool state_now, bool state_last)
{
	if (serial.isMainBoardSleep())
	{
		ROS_WARN("%s %d: Waked up by detect charge.", __FUNCTION__, __LINE__);
		ev.charge_detect = charger.getChargeStatus();
		serial.setWorkMode(CHARGE_MODE);
	}
}

void ModeSleep::rcon(bool state_now, bool state_last)
{
	if (serial.isMainBoardSleep())
	{
		ROS_WARN("%s %d: Waked up by rcon signal.", __FUNCTION__, __LINE__);
		ev.rcon_status = c_rcon.getAll();
		serial.setWorkMode(WORK_MODE);
	}
	c_rcon.resetStatus();
}

void ModeSleep::remotePlan(bool state_now, bool state_last)
{
	if (serial.isMainBoardSleep() && !plan_activated_status_ && robot_timer.getPlanStatus() == 3)
	{
		ROS_WARN("%s %d: Waked up by plan.", __FUNCTION__, __LINE__);
		plan_activated_status_ = true;
		serial.setWorkMode(WORK_MODE);
	}
	robot_timer.resetPlanStatus();
}

