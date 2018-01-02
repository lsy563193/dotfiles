#include <event_manager.h>
#include <pp.h>
#include "arch.hpp"
#include "error.h"

ModeIdle::ModeIdle()
{
	ROS_INFO("%s %d: Entering Idle mode\n=========================" , __FUNCTION__, __LINE__);
	register_events();
	sp_action_.reset(new ActionIdle);
	action_i_ = ac_idle;

	key.resetTriggerStatus();
	c_rcon.resetStatus();
	remote.reset();
	robot_timer.resetPlanStatus();
	event_manager_reset_status();

	plan_activated_status_ = false;
	ROS_INFO("%s %d: Current battery voltage \033[32m%5.2f V\033[0m.", __FUNCTION__, __LINE__, (float)battery.getVoltage()/100.0);

	/*---reset values for rcon handle---*/
	first_time_seen_charger_ = 0.0;
	last_time_seen_charger_ = 0.0;
}

ModeIdle::~ModeIdle()
{
	event_manager_set_enable(false);
	sp_action_.reset();
	ROS_INFO("%s %d: Exit idle mode.", __FUNCTION__, __LINE__);
}

bool ModeIdle::isExit()
{
	if(ev.key_clean_pressed || plan_activated_status_)
	{
		if (ev.key_clean_pressed && bumper.getLeft())
		{
			ROS_WARN("%s %d:.", __FUNCTION__, __LINE__);
			setNextMode(cm_test);
			return true;
		}
		ROS_WARN("%s %d:.", __FUNCTION__, __LINE__);
		setNextMode(cm_navigation);
		return true;
	}

	if(ev.remote_wallfollow)
	{
		ROS_WARN("%s %d:.", __FUNCTION__, __LINE__);
		setNextMode(cm_wall_follow);
		return true;
	}

	if(ev.remote_spot)
	{
		ROS_WARN("%s %d:.", __FUNCTION__, __LINE__);
		setNextMode(cm_spot);
		return true;
	}

	if(ev.remote_home)
	{
		ROS_WARN("%s %d:.", __FUNCTION__, __LINE__);
		setNextMode(cm_exploration);
		return true;
	}
	if (ev.key_long_pressed)
	{
		ROS_WARN("%s %d:.", __FUNCTION__, __LINE__);
		setNextMode(md_sleep);
		return true;
	}

	if (ev.charge_detect)
	{
		ROS_WARN("%s %d:.", __FUNCTION__, __LINE__);
		setNextMode(md_charge);
		return true;
	}

	if (ev.remote_direction_forward || ev.remote_direction_left || ev.remote_direction_right)
	{
		ROS_WARN("%s %d:.", __FUNCTION__, __LINE__);
		setNextMode(md_remote);
		return true;
	}

	if (ev.rcon_triggered)
	{
		ROS_WARN("%s %d:.", __FUNCTION__, __LINE__);
		setNextMode(md_go_to_charger);
		return true;
	}

	return false;
}

void ModeIdle::register_events()
{
	event_manager_register_handler(this);
	event_manager_set_enable(true);
}

void ModeIdle::remoteKeyHandler(bool state_now, bool state_last)
{
	ROS_WARN("%s %d: Remote key %x has been pressed.", __FUNCTION__, __LINE__, remote.get());
	//g_robot_stuck = false;

	if (error.get())
	{
		if (remote.isKeyTrigger(REMOTE_CLEAN))
		{
			ROS_WARN("%s %d: Clear the error %x.", __FUNCTION__, __LINE__, error.get());
			if (error.clear(error.get()))
			{
				beeper.play_for_command(VALID);
				led.set_mode(LED_BREATH, LED_GREEN);
				speaker.play(VOICE_CLEAR_ERROR);
			}
			else
			{
				beeper.play_for_command(INVALID);
				error.alarm();
			}
		}
		else
		{
			ROS_WARN("%s %d: Remote key %x not valid because of error %d.", __FUNCTION__, __LINE__, remote.get(), error.get());
			error.alarm();
			beeper.play_for_command(INVALID);
		}
	}
	else if (cliff.getStatus() == BLOCK_ALL)
	{
		ROS_WARN("%s %d: Remote key %x not valid because of robot lifted up.", __FUNCTION__, __LINE__, remote.get());
		beeper.play_for_command(INVALID);
		speaker.play(VOICE_ERROR_LIFT_UP);
	}
	else if ((!remote.isKeyTrigger(REMOTE_FORWARD) && !remote.isKeyTrigger(REMOTE_LEFT)
			  && !remote.isKeyTrigger(REMOTE_RIGHT) && !remote.isKeyTrigger(REMOTE_HOME))
			  && !battery.isReadyToClean())
	{
		ROS_WARN("%s %d: Battery level low %4dmV(limit in %4dmV)", __FUNCTION__, __LINE__, battery.getVoltage(), (int)BATTERY_READY_TO_CLEAN_VOLTAGE);
		beeper.play_for_command(INVALID);
		speaker.play(VOICE_BATTERY_LOW);
	}
	else
	{
		beeper.play_for_command(VALID);
		switch (remote.get())
		{
			case REMOTE_FORWARD:
			{
				ev.remote_direction_forward = true;
				break;
			}
			case REMOTE_LEFT:
			{
				ev.remote_direction_left = true;
				break;
			}
			case REMOTE_RIGHT:
			{
				ev.remote_direction_right = true;
				break;
			}
			case REMOTE_CLEAN:
			{
				ev.key_clean_pressed = true;
				break;
			}
			case REMOTE_SPOT:
			{
				ev.remote_spot = true;
				break;
			}
			case REMOTE_HOME:
			{
				ev.remote_home = true;
				break;
			}
			case REMOTE_WALL_FOLLOW:
			{
				ev.remote_wallfollow = true;
				break;
			}
			default: // REMOTE_PLAN/REMOTE_MAX is not handled here.
				break;
		}
	}
	remote.reset();
}

void ModeIdle::remoteMax(bool state_now, bool state_last)
{
	beeper.play_for_command(INVALID);
	remote.reset();
}

void ModeIdle::remotePlan(bool state_now, bool state_last)
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
		else if(cliff.getStatus() & (BLOCK_LEFT|BLOCK_FRONT|BLOCK_RIGHT))
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
			//speaker.play(???);
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

void ModeIdle::chargeDetect(bool state_now, bool state_last)
{
	ROS_WARN("%s %d: Detect charge!", __FUNCTION__, __LINE__);
	ev.charge_detect = charger.getChargeStatus();
}

void ModeIdle::keyClean(bool state_now, bool state_last)
{
	ROS_WARN("%s %d: key clean.", __FUNCTION__, __LINE__);

	beeper.play_for_command(VALID);

	// Wait for key released.
	bool long_press = false;
	while (key.getPressStatus())
	{
		if (!long_press && key.getPressTime() > 3)
		{
			ROS_WARN("%s %d: key clean long pressed.", __FUNCTION__, __LINE__);
			beeper.play_for_command(VALID);
			long_press = true;
		}
		usleep(20000);
	}

	if (long_press)
		ev.key_long_pressed = true;
	else
		ev.key_clean_pressed = true;
	ROS_WARN("%s %d: Key clean is released.", __FUNCTION__, __LINE__);

	key.resetTriggerStatus();
}

void ModeIdle::rcon(bool state_now, bool state_last)
{
	auto time_for_now_ = ros::Time::now().toSec();
//	ROS_WARN("%s %d: rcon signal. first: %lf, last: %lf, now: %lf", __FUNCTION__, __LINE__, first_time_seen_charger, last_time_seen_charger, time_for_now);
	if(time_for_now_ - last_time_seen_charger_ > 60)
	{
		/*---more than 1 min haven't seen charger, reset first_time_seen_charger---*/
		first_time_seen_charger_ = time_for_now_;
	}
	else
	{
		/*---received charger signal continuously, check if more than 3 mins---*/
		if(time_for_now_ - first_time_seen_charger_ > 3)
			ev.rcon_triggered = c_rcon.getAll();
	}
	last_time_seen_charger_ = time_for_now_;
}
