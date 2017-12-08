#include <event_manager.h>
#include <pp.h>
#include "arch.hpp"
#include "error.h"

IdleMode::IdleMode()
{
	sp_action_.reset(new ActionIdle);
	register_events();

	key.resetTriggerStatus();
	c_rcon.resetStatus();
	remote.reset();
	robot_timer.resetPlanStatus();
	event_manager_reset_status();

}

IdleMode::~IdleMode()
{
	event_manager_set_enable(false);
}
bool IdleMode::isExit() {
	if(ev.key_clean_pressed)
		cm_set(Clean_Mode_Navigation);
	else if(ev.remote_direction_left || ev.remote_direction_forward || ev.remote_direction_right)
		cm_set(Clean_Mode_Remote);
	else if(ev.remote_spot)
		cm_set(Clean_Mode_Spot);
	else if(ev.remote_home)
		cm_set(Clean_Mode_Go_Charger);
	else if(ev.remote_wallfollow)
		cm_set(Clean_Mode_WallFollow);
	else
		return false;

	return true;
}
IAction* IdleMode::getNextAction() {
	return nullptr;
}
void IdleMode::register_events() {
	event_manager_register_handler(this);
	event_manager_set_enable(true);
}
void IdleMode::remote_cleaning(bool state_now, bool state_last)
{
	ROS_WARN("%s %d: Remote key %x has been pressed.", __FUNCTION__, __LINE__, remote.get());
	//g_robot_stuck = false;

	/* reset charger_signal_start_time when get remote cleaning */
	charger_signal_start_time = time(NULL);

	if (error.get())
	{
		if (remote.isKeyTrigger(REMOTE_CLEAN))
		{
			ROS_WARN("%s %d: Clear the error %x.", __FUNCTION__, __LINE__, error.get());
			if (error.clear(error.get()))
			{
				beeper.play_for_command(VALID);
				led.set_mode(LED_BREATH, LED_GREEN);
				speaker.play(SPEAKER_CLEAR_ERROR);
				error.set(ERROR_CODE_NONE);
				reject_reason = 4;
			}
			else
			{
				beeper.play_for_command(INVALID);
				error.alarm();
				reject_reason = 1;
			}
			key.resetTriggerStatus();
		}
		else
		{
			ROS_WARN("%s %d: Remote key %x not valid because of error %d.", __FUNCTION__, __LINE__, remote.get(), error.get());
			error.alarm();
			beeper.play_for_command(INVALID);
			reject_reason = 1;
		}
	}
	else if (cliff.get_status() == BLOCK_ALL)
	{
		ROS_WARN("%s %d: Remote key %x not valid because of robot lifted up.", __FUNCTION__, __LINE__, remote.get());
		beeper.play_for_command(INVALID);
		speaker.play(SPEAKER_ERROR_LIFT_UP);
		reject_reason = 2;
	}
	else if ((!remote.isKeyTrigger(REMOTE_FORWARD) && !remote.isKeyTrigger(REMOTE_LEFT)
			  && !remote.isKeyTrigger(REMOTE_RIGHT) && !remote.isKeyTrigger(REMOTE_HOME))
			  && !battery.isReadyToClean())
	{
		ROS_WARN("%s %d: Battery level low %4dmV(limit in %4dmV)", __FUNCTION__, __LINE__, battery.getVoltage(), (int)BATTERY_READY_TO_CLEAN_VOLTAGE);
		beeper.play_for_command(INVALID);
		speaker.play(SPEAKER_BATTERY_LOW);
		reject_reason = 3;
	}

	if (!reject_reason)
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
				key.resetTriggerStatus();
				break;
			}
			case REMOTE_SPOT:
			{
				ev.remote_spot = true;
				break;
			}
			case REMOTE_HOME:
			{
//				if (cs_is_paused())
//				{
//					ev.remote_home = true;
//					extern bool g_go_home_by_remote;
//					g_go_home_by_remote = true;
//					temp_mode = Clean_Mode_Navigation;
//				}
//				else
				ev.remote_home = true;
				break;
			}
			case REMOTE_WALL_FOLLOW:
			{
				ev.remote_wallfollow = true;
				break;
			}
		}
	}
	remote.reset();
}
