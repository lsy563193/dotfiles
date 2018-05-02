//
// Created by austin on 17-12-5.
//

#include <error.h>
#include <event_manager.h>
#include "dev.h"
#include "mode.hpp"
#include "appointment.h"

#define RCON_TRIGGER_INTERVAL 180

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
	appmt_obj.resetPlanStatus();
	s_wifi.setWorkMode(md_sleep);
	s_wifi.taskPushBack(S_Wifi::ACT::ACT_UPLOAD_STATUS);
	s_wifi.resetReceivedWorkMode();

	sp_state = st_sleep.get();
	sp_state->init();
	plan_activated_status_ = false;

	//sp_action_.reset(new ActionSleep);
	if (!charger.getChargeStatus() && battery.isReadyToClean())
		fake_sleep_ = true;

	sp_action_.reset(new ActionSleep(fake_sleep_));
}

ModeSleep::~ModeSleep()
{
	//s_wifi.taskPushBack(S_Wifi::ACT::ACT_RESUME);
	event_manager_set_enable(false);
	sp_action_.reset();
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
				ROS_WARN("%s %d: Plan not activated not valid because of battery not ready to clean.", __FUNCTION__,
						 __LINE__);
				speaker.play(VOICE_BATTERY_LOW);
			} else if (charger.isDirected())
			{
				ROS_WARN("%s %d: Plan not activated not valid because of charging with adapter.", __FUNCTION__,
						 __LINE__);
				speaker.play(VOICE_PLEASE_PULL_OUT_THE_PLUG, false);
				speaker.play(VOICE_BATTERY_CHARGE);
//				speaker.play(VOICE_CANCEL_APPOINTMENT_UNOFFICIAL);
			} else
			{
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
			ROS_WARN("%s %d: Sleep mode receives remote clean or clean key, change to idle mode.", __FUNCTION__,
					 __LINE__);
			setNextMode(md_idle);
			return true;
		}
	}

	if ((s_wifi.receivePlan1() || s_wifi.receiveHome() || s_wifi.receiveSpot()) && !serial.isMainBoardSleep())
	{
		if (!readyToClean())
		{
			ROS_WARN("%s %d: Change to idle mode.", __FUNCTION__, __LINE__);
			setNextMode(md_idle);
		}
		else if (s_wifi.receivePlan1())
		{
			ROS_WARN("%s %d: Sleep mode receives wifi plan1, change to navigation mode.", __FUNCTION__, __LINE__);
			setNextMode(cm_navigation);
		} else if (s_wifi.receiveHome())
		{
			ROS_WARN("%s %d: Sleep mode receives wifi home, change to exploration mode.", __FUNCTION__, __LINE__);
			setNextMode(cm_exploration);
		} else if (s_wifi.receiveSpot())
		{
			ROS_WARN("%s %d: Sleep mode receives wifi spot, change to spot mode.", __FUNCTION__, __LINE__);
			setNextMode(cm_spot);
		}

		return true;
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
	if (fake_sleep_ && !battery.isReadyToClean())
	{
		ROS_INFO("%s %d: Battery too low, enter low power sleep.", __FUNCTION__, __LINE__);
		auto sp_action_sleep = boost::dynamic_pointer_cast<ActionSleep>(sp_action_);
		sp_action_sleep->lowPowerSleep();
		fake_sleep_ = false;
	}
	return false;
}

void ModeSleep::remoteClean(bool state_now, bool state_last)
{
	if ((fake_sleep_ || serial.isMainBoardSleep()) && !ev.key_clean_pressed)
	{
		ev.key_clean_pressed = true;
		ROS_WARN("%s %d: Waked up by remote key clean.", __FUNCTION__, __LINE__);
		serial.setWorkMode(IDLE_MODE);
		key_led.setMode(LED_STEADY, LED_GREEN);
	}

	remote.reset();
}

void ModeSleep::keyClean(bool state_now, bool state_last)
{
	if ((fake_sleep_ || serial.isMainBoardSleep()) && !ev.key_clean_pressed)
	{
		ROS_WARN("%s %d: Waked up by key clean.", __FUNCTION__, __LINE__);
		// Wake up main board.
		serial.setWorkMode(IDLE_MODE);

		ev.key_clean_pressed = true;
		ROS_WARN("%s %d: Key clean is released.", __FUNCTION__, __LINE__);
		key_led.setMode(LED_STEADY, LED_GREEN);
	}

	key.resetTriggerStatus();
}

void ModeSleep::chargeDetect(bool state_now, bool state_last)
{
	if (fake_sleep_ || serial.isMainBoardSleep())
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
		key_led.setMode(LED_STEADY, LED_ORANGE);
	}
	else if (fake_sleep_)
	{
		if (error.get() == ERROR_CODE_NONE)
		{
			auto time_for_now_ = ros::Time::now().toSec();
//	ROS_WARN("%s %d: rcon signal. first: %lf, last: %lf, now: %lf", __FUNCTION__, __LINE__, first_time_seen_charger, last_time_seen_charger, time_for_now);
			if (time_for_now_ - last_time_seen_charger_ > 60.0)
			{
				/*---more than 1 min haven't seen charger, reset first_time_seen_charger---*/
				first_time_seen_charger_ = time_for_now_;
			} else
			{
				/*---received charger signal continuously, check if more than 3 mins---*/
				if (time_for_now_ - first_time_seen_charger_ > RCON_TRIGGER_INTERVAL)
					ev.rcon_status = c_rcon.getAll();
			}
			last_time_seen_charger_ = time_for_now_;
		}
	}
	c_rcon.resetStatus();
}

void ModeSleep::remotePlan(bool state_now, bool state_last)
{
	if ((fake_sleep_ || serial.isMainBoardSleep()) && !plan_activated_status_ && appmt_obj.getPlanStatus() > 2)
	{
		appmt_obj.resetPlanStatus();
		appmt_obj.timesUp();
		INFO_YELLOW("Plan activated.");
		serial.setWorkMode(WORK_MODE);
		key_led.setMode(LED_FLASH, LED_GREEN);
		plan_activated_status_ = true;
	}
	else
		EventHandle::remotePlan(state_now, state_last);
}

bool ModeSleep::allowRemoteUpdatePlan()
{
	return !fake_sleep_;
}

void ModeSleep::remoteKeyHandler(bool state_now, bool state_last)
{
	ROS_INFO("%s %d: Receive remote:%d but not valid for fake sleep.", __FUNCTION__, __LINE__, remote.get());
	remote.reset();
}

bool ModeSleep::readyToClean()
{
	if (!battery.isReadyToClean())
	{
		ROS_WARN("%s %d: Battery not ready to clean(Not reach %4dmV).", __FUNCTION__,
				 __LINE__, BATTERY_READY_TO_CLEAN_VOLTAGE);
		speaker.play(VOICE_BATTERY_LOW, false);
		return false;
	}
	else if (cliff.getStatus() & (BLOCK_LEFT | BLOCK_FRONT | BLOCK_RIGHT))
	{
		ROS_WARN("%s %d: Robot lifted up.", __FUNCTION__, __LINE__);
		speaker.play(VOICE_ERROR_LIFT_UP, false);
		return false;
	}
	else if (robot::instance()->isBatteryLow2())
	{
		ROS_WARN("%s %d: Battery level low %4dmV(limit in %4dmV)", __FUNCTION__, __LINE__, battery.getVoltage(), (int)LOW_BATTERY_STOP_VOLTAGE);
		sp_state->init();
		beeper.beepForCommand(INVALID);
		speaker.play(VOICE_BATTERY_LOW, false);
		return false;
	}

	return true;
}

