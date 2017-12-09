//
// Created by austin on 17-12-5.
//

#include <error.h>
#include <pp.h>
#include "dev.h"
#include "arch.hpp"

ModeSleep::ModeSleep()
{
	ROS_INFO("%s %d: Switch to sleep mode.", __FUNCTION__, __LINE__);
	event_manager_register_handler(this);
	event_manager_reset_status();
	event_manager_set_enable(true);

	sp_action_.reset(new ActionSleep);
	action_i_ = ac_sleep;
	serial.setCleanMode(Clean_Mode_Sleep);
	usleep(30000);
	serial.sleep();


	key.resetTriggerStatus();
	c_rcon.resetStatus();
	remote.reset();
	robot_timer.resetPlanStatus();

	plan_activated_status_ = false;
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
	if (ev.charge_detect)
	{
		ROS_WARN("%s %d:.", __FUNCTION__, __LINE__);
		serial.wakeUp();
		setNextMode(md_charge);
		return true;
	}

	if (ev.key_clean_pressed || plan_activated_status_)
	{
		ROS_WARN("%s %d:.", __FUNCTION__, __LINE__);
		serial.wakeUp();
		setNextMode(md_idle);
		return true;
	}

	if (ev.rcon_triggered)
	{
//		ROS_WARN("%s %d:.", __FUNCTION__, __LINE__);
//		serial.wakeUp();
//		setNextMode(md_go_to_charger);
//		return true;
	}

	return false;
}

bool ModeSleep::isFinish()
{
	return false;
}

IAction* ModeSleep::getNextAction()
{
	return nullptr;
}

void ModeSleep::remote_clean(bool state_now, bool state_last)
{
	ev.key_clean_pressed = true;
	ROS_WARN("%s %d: Waked up by remote key clean.", __FUNCTION__, __LINE__);
}

void ModeSleep::key_clean(bool state_now, bool state_last)
{
	ROS_WARN("%s %d: Waked up by key clean.", __FUNCTION__, __LINE__);

	// Wake up serial so it can beep.
	serial.wakeUp();
	beeper.play_for_command(VALID);

	// Wait for key released.
	while (key.getPressStatus())
		usleep(20000);
	ev.key_clean_pressed = true;
	ROS_WARN("%s %d: Key clean is released.", __FUNCTION__, __LINE__);

	key.resetTriggerStatus();
}

void ModeSleep::charge_detect(bool state_now, bool state_last)
{
	ROS_WARN("%s %d: Detect charge!", __FUNCTION__, __LINE__);
	ev.charge_detect = charger.getChargeStatus();
}

void ModeSleep::rcon(bool state_now, bool state_last)
{
//	ROS_WARN("%s %d: Waked up by rcon signal.", __FUNCTION__, __LINE__);
//	ev.rcon_triggered = c_rcon.getStatus();
//	c_rcon.resetStatus();
//	serial.wakeUp();
}

void ModeSleep::remote_plan(bool state_now, bool state_last)
{
	ROS_WARN("%s %d: Waked up by plan.", __FUNCTION__, __LINE__);
	if (robot_timer.getPlanStatus() == 3)
	{
		if (error.get() != ERROR_CODE_NONE)
		{
			ROS_WARN("%s %d: Error exists, so cancel the appointment.", __FUNCTION__, __LINE__);
			error.alarm();
			speaker.play(SPEAKER_CANCEL_APPOINTMENT);
		}
		else if(cliff.get_status() & (BLOCK_LEFT|BLOCK_FRONT|BLOCK_RIGHT))
		{
			ROS_WARN("%s %d: Plan not activated not valid because of robot lifted up.", __FUNCTION__, __LINE__);
			speaker.play(SPEAKER_ERROR_LIFT_UP);
			speaker.play(SPEAKER_CANCEL_APPOINTMENT);
		}
		else if(!battery.isReadyToClean())
		{
			ROS_WARN("%s %d: Plan not activated not valid because of robot battery not ready to clean.", __FUNCTION__, __LINE__);
			speaker.play(SPEAKER_BATTERY_LOW);
			speaker.play(SPEAKER_CANCEL_APPOINTMENT);
		}
		else
		{
			plan_activated_status_ = true;
		}
	}
	robot_timer.resetPlanStatus();
}
