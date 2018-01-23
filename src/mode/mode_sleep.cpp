//
// Created by austin on 17-12-5.
//

#include <error.h>
#include <event_manager.h>
#include "dev.h"
#include "robotbase.h"
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
}

ModeSleep::~ModeSleep()
{
	event_manager_set_enable(false);
	sp_action_.reset();
	// Wait 1.5s to avoid gyro can't open if switch to navigation mode_ too soon after waking up.
	usleep(1500000);
	ROS_INFO("%s %d: Exit sleep mode.", __FUNCTION__, __LINE__);
}

bool ModeSleep::isExit()
{
	if ((ev.key_clean_pressed || plan_activated_status_) && !serial.isMainBoardSleep())
	{
		ROS_WARN("%s %d: Clean key pressed, switch to idle mode.", __FUNCTION__, __LINE__);
		setNextMode(md_idle);
		return true;
	}

	if (ev.charge_detect && !serial.isMainBoardSleep())
	{
		ROS_WARN("%s %d: Charge detected, switch to charge mode.", __FUNCTION__, __LINE__);
		setNextMode(md_charge);
		return true;
	}

	if (ev.rcon_triggered && !serial.isMainBoardSleep())
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
	if (serial.isMainBoardSleep())
	{
		if (!ev.key_clean_pressed)
		{
			ev.key_clean_pressed = true;
			ROS_WARN("%s %d: Waked up by remote key clean.", __FUNCTION__, __LINE__);
			serial.setMainBoardMode(IDLE_MODE);
//			beeper.play_for_command(VALID);
		}
	}

	remote.reset();
}

void ModeSleep::keyClean(bool state_now, bool state_last)
{
	if (serial.isMainBoardSleep())
	{
		if (!ev.key_clean_pressed)
		{
			ROS_WARN("%s %d: Waked up by key clean.", __FUNCTION__, __LINE__);

			// Wake up serial so it can beep.
			serial.setMainBoardMode(IDLE_MODE);
//			beeper.play_for_command(VALID);

			// Wait for key released.
//			while (key.getPressStatus())
//				usleep(20000);
			ev.key_clean_pressed = true;
			ROS_WARN("%s %d: Key clean is released.", __FUNCTION__, __LINE__);

		}
	}

	key.resetTriggerStatus();
}

void ModeSleep::chargeDetect(bool state_now, bool state_last)
{
	if (serial.isMainBoardSleep())
	{
		ROS_WARN("%s %d: Waked up by detect charge.", __FUNCTION__, __LINE__);
		ev.charge_detect = charger.getChargeStatus();
		serial.setMainBoardMode(CHARGE_MODE);
	}
}

void ModeSleep::rcon(bool state_now, bool state_last)
{
	if (serial.isMainBoardSleep())
	{
		ROS_WARN("%s %d: Waked up by rcon signal.", __FUNCTION__, __LINE__);
		ev.rcon_triggered = c_rcon.getAll();
		serial.setMainBoardMode(WORK_MODE);
	}
	c_rcon.resetStatus();
}

void ModeSleep::remotePlan(bool state_now, bool state_last)
{
	if (serial.isMainBoardSleep())
	{
		if (!plan_activated_status_)
		{
			ROS_WARN("%s %d: Waked up by plan.", __FUNCTION__, __LINE__);
			if (robot_timer.getPlanStatus() == 3)
			{
				if (error.get() != ERROR_CODE_NONE)
				{
					ROS_WARN("%s %d: Error exists, so cancel the appointment.", __FUNCTION__, __LINE__);
					error.alarm();
					speaker.play(VOICE_CANCEL_APPOINTMENT);
				} else if (cliff.getStatus() & (BLOCK_LEFT | BLOCK_FRONT | BLOCK_RIGHT))
				{
					ROS_WARN("%s %d: Plan not activated not valid because of robot lifted up.", __FUNCTION__, __LINE__);
					speaker.play(VOICE_ERROR_LIFT_UP_CANCEL_APPOINTMENT);
				} else if (!battery.isReadyToClean())
				{
					ROS_WARN("%s %d: Plan not activated not valid because of robot battery not ready to clean.",
							 __FUNCTION__,
							 __LINE__);
					speaker.play(VOICE_BATTERY_LOW_CANCEL_APPOINTMENT);
				} else
				{
					serial.setMainBoardMode(WORK_MODE);
					plan_activated_status_ = true;
				}
			}
		}
	}
	robot_timer.resetPlanStatus();
}

