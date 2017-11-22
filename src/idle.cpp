/**
  ******************************************************************************
  * @file    AI Cleaning Robot
  * @author  ILife Team Dxsong
  * @version V1.0
  * @date    17-Nov-2011
  * @brief   UserInterface Fuction
	           Display Button lights and waiting for user to select cleaning mode
						 Plan setting , set hours and minutes
  ******************************************************************************
  * <h2><center>&copy; COPYRIGHT 2011 ILife CO.LTD</center></h2>
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "config.h"
#include <thread>
#include "dev.h"
#include "idle.h"
#include "robot.hpp"
#include "robotbase.h"
#include "event_manager.h"
#include "core_move.h"
#include "clean_mode.h"
#include "error.h"

uint8_t temp_mode=0;
time_t charger_signal_start_time;
bool charger_signal_received = false;
time_t bat_low_start_time;
uint16_t bat_low_delay = 0;
bool bat_ready_to_clean = true;
bool long_press_to_sleep = false;
uint8_t reject_reason = 0; // 1 for error exist, 2 for robot lifted up, 3 for battery low, 4 for key clean clear the error.
uint8_t plan_status = 0;
extern bool g_charge_turn_connect_fail;
time_t plan_confirm_time = time(NULL);
static Idle_EventHandle eh;
/*------------------------------------------------------------User Interface ----------------------------------*/
void register_events(void)
{
	event_manager_register_handler(&eh);
	event_manager_set_enable(true);
}

void unregister_events(void)
{
	event_manager_set_enable(false);
}

void idle(void)
{
	time_t start_time;

	bool eh_status_now=false, eh_status_last=false;

	// Count for error alarm.
	uint8_t error_alarm_counter = 3;
	charger_signal_received = false;
	bat_low_delay = 0;
	start_time = time(NULL);
	temp_mode=0;
	bat_ready_to_clean = true;

	cs_disable_motors();
	remote.reset();
	timer.set_status(0);
	key.reset();
	c_rcon.reset_status();
	key.reset();
	vacuum.stop();

	ROS_INFO("%s,%d ,BatteryVoltage = \033[32m%dmV\033[0m.",__FUNCTION__,__LINE__, battery.get_voltage());
	// Check the battery to warn the user.
	if(!battery.is_ready_to_clean() && !cs_is_paused())
	{
		ROS_WARN("%s %d: Battery Level Low = \033[31m%4dmV\033[0m(limit = \033[33m%4dmV\033[0m).", __FUNCTION__, __LINE__,
						 battery.get_voltage(),(int)BATTERY_READY_TO_CLEAN_VOLTAGE);
		bat_ready_to_clean = false;
		led.set_mode(LED_BREATH, LED_ORANGE);
		wav.play(WAV_BATTERY_LOW);
	}
	else
		led.set_mode(LED_BREATH, LED_GREEN);

	if(error.get())
		led.set_mode(LED_STEADY, LED_RED);

	event_manager_reset_status();
	register_events();

	if (cs_is_paused())
	{
		saved_pos_x = robot::instance()->getOdomPositionX();
		saved_pos_y = robot::instance()->getOdomPositionY();
	}

	while(ros::ok())
	{
		if (event_manager_check_event(&eh_status_now, &eh_status_last) == 1) {
			continue;
		}

		usleep(10000);

		//bumper.get_lidar_status();
		if (bat_low_delay > 0)
			bat_low_delay--;

		if(bat_ready_to_clean && !battery.is_ready_to_clean() && !cs_is_paused())
		{
			bat_ready_to_clean = false;
			led.set_mode(LED_BREATH, LED_ORANGE);
		}
		if(time(NULL) - start_time > USER_INTERFACE_TIMEOUT)
		{
			ROS_WARN("%s %d: Userinterface mode_ didn't receive any command in 10mins, go to sleep mode_.", __FUNCTION__, __LINE__);
			temp_mode = Clean_Mode_Sleep;
		}

		if (cs_is_paused())
		{
			float distance = sqrtf(powf(saved_pos_x - robot::instance()->getOdomPositionX(), 2) + powf(saved_pos_y - robot::instance()->getOdomPositionY(), 2));
			if (distance > 0.1f){
				cs_paused_setting();
				g_robot_stuck = false;
			}
		}

		if(g_plan_activated)
		{
			temp_mode = Clean_Mode_Navigation;
		}
		// Check for wav playing.
		if (long_press_to_sleep)
		{
			long_press_to_sleep = false;
			if (cs_is_paused())
				cs_paused_setting();
			else
				temp_mode = Clean_Mode_Sleep;
		}
		else if (reject_reason)
		{
			switch (reject_reason)
			{
				case 1:
					error_alarm_counter = 0;
					error.alarm();
					reject_reason = 0;
					break;
				case 2:
					wav.play(WAV_ERROR_LIFT_UP);
					cs_paused_setting();
					reject_reason = 0;
					break;
				case 3:
					wav.play(WAV_BATTERY_LOW);
					reject_reason = 0;
					break;
				case 4:
					led.set_mode(LED_BREATH, LED_GREEN);
					wav.play(WAV_CLEAR_ERROR);
					error_alarm_counter = 0;
					error.set(Error_Code_None);
					reject_reason = 0;
					break;
			}
		}
		else if (plan_status)
		{
			if (plan_status == 2 && (time(NULL) - plan_confirm_time >= 2))
			{
				ROS_WARN("%s %d: Cancel appointment.", __FUNCTION__, __LINE__);
				wav.play(WAV_CANCEL_APPOINTMENT);
				plan_status = 0;
			}
			else if (plan_status == 1 && (time(NULL) - plan_confirm_time >= 2))
			{
				ROS_WARN("%s %d: Confirm appointment.", __FUNCTION__, __LINE__);
				wav.play(WAV_APPOINTMENT_DONE);
				plan_status = 0;
			}
		}
		// Alarm for error.
		else if (error.get()){
			if (error_alarm_counter == 3 || (error_alarm_counter == 2 && (time(NULL) - start_time) >= 10) || (error_alarm_counter == 1 && (time(NULL) - start_time) >= 20))
			{
				error_alarm_counter--;
				error.alarm();
			}
		}
		if(temp_mode != 0)
		{
			cm_set(temp_mode);
			break;
		}
	}

	unregister_events();

	// Make sure alarm will be done.
	if (plan_status == 2)
		wav.play(WAV_CANCEL_APPOINTMENT);
	else if (plan_status == 1)
		wav.play(WAV_APPOINTMENT_DONE);
	plan_status = 0;

	/*--- reset g_charge_turn_connect_fail except Clean_Mode_GoHome and Clean_Mode_Exploration ---*/
	if(temp_mode != Clean_Mode_Go_Charger && temp_mode != Clean_Mode_Exploration)
		g_charge_turn_connect_fail = false;
}

void Idle_EventHandle::cliff_(bool state_now, bool state_last)
{
	ROS_DEBUG("%s %d: Cliff triggered.", __FUNCTION__, __LINE__);

	/*--------------------------------------------------------If manual pause cleaning, check cliff--------------*/
	if (reject_reason != 2 && cs_is_paused())
	{
		ROS_WARN("%s %d: Robot lifted up during manual/stuck pause, reset manual/stuck pause status.", __FUNCTION__, __LINE__);
		reject_reason = 2;
	}
	/*--- reset g_charge_turn_connect_fail when cliff triggered ---*/
	g_charge_turn_connect_fail = false;
}

void Idle_EventHandle::cliff_left(bool state_now, bool state_last)
{
	cliff_(state_now, state_last);
}
void Idle_EventHandle::cliff_left_right(bool state_now, bool state_last)
{
	cliff_(state_now, state_last);
}
void Idle_EventHandle::cliff_right(bool state_now, bool state_last)
{
	cliff_(state_now, state_last);
}
void Idle_EventHandle::cliff_front(bool state_now, bool state_last)
{
	cliff_(state_now, state_last);
}
void Idle_EventHandle::cliff_front_left(bool state_now, bool state_last)
{
	cliff_(state_now, state_last);
}
void Idle_EventHandle::cliff_front_right(bool state_now, bool state_last)
{
	cliff_(state_now, state_last);
}

void Idle_EventHandle::rcon(bool state_now, bool state_last)
{
	if (cs_is_paused())
	{
		c_rcon.reset_status();
		ROS_DEBUG("%s %d: user_interface detects charger signal, but ignore for manual/stuck pause.", __FUNCTION__, __LINE__);
		return;
	}

	if (!charger_signal_received)
	{
		ROS_DEBUG("%s, %d: the first time detects charger signal, set charger_signal_start_time", __FUNCTION__, __LINE__);
		charger_signal_received = true;
		charger_signal_start_time = time(NULL);
	}
	else
	{
		ROS_DEBUG("%s %d: detects charger signal(%8x) for %ds.", __FUNCTION__, __LINE__, c_rcon.get_status(), (int)(time(NULL) - charger_signal_start_time));
		if (time(NULL) - charger_signal_start_time >= 180)// 3 mins//180
		{
			if (error.get())
				ROS_WARN("%s %d: Rcon set go home not valid because of error %d.", __FUNCTION__, __LINE__, error.get());
			else if(cliff.get_status() & (BLOCK_LEFT|BLOCK_FRONT|BLOCK_RIGHT))
				ROS_WARN("%s %d: Rcon set go home not valid because of robot lifted up.", __FUNCTION__, __LINE__);
			else
				temp_mode = Clean_Mode_Go_Charger;
		}
	}
	c_rcon.reset_status();
}

void Idle_EventHandle::battery_low(bool state_now, bool state_last)
{
	if (bat_low_delay == 0)
		bat_low_start_time = time(NULL);
	ROS_DEBUG("%s %d: user_interface detects battery low %dmv for %ds.", __FUNCTION__, __LINE__, battery.get_voltage(), (int)(time(NULL) - bat_low_start_time));
	if (time(NULL) - bat_low_start_time >= 5)// 5 seconds
	{
		temp_mode = Clean_Mode_Sleep;
		return;
	}

	bat_low_delay = 10;
}

void Idle_EventHandle::remote_cleaning(bool state_now, bool state_last)
{
	ROS_WARN("%s %d: Remote key %x has been pressed.", __FUNCTION__, __LINE__, remote.get());
	omni.set_stop(false);
	//g_robot_stuck = false;

	/* reset charger_signal_start_time when get remote cleaning */
	charger_signal_start_time = time(NULL);

	if (error.get())
	{
		if (remote.get() == Remote_Clean)
		{
			ROS_WARN("%s %d: Clear the error %x.", __FUNCTION__, __LINE__, error.get());
			if (error.clear(error.get()))
			{
				beeper.play_for_command(VALID);
				reject_reason = 4;
			}
			else
			{
				beeper.play_for_command(INVALID);
				reject_reason = 1;
			}
			key.reset();
		}
		else
		{
			ROS_WARN("%s %d: Remote key %x not valid because of error %d.", __FUNCTION__, __LINE__, remote.get(), error.get());
			reject_reason = 1;
			beeper.play_for_command(INVALID);
		}
	}
	else if (cliff.get_status() == BLOCK_ALL)
	{
		ROS_WARN("%s %d: Remote key %x not valid because of robot lifted up.", __FUNCTION__, __LINE__, remote.get());
		beeper.play_for_command(INVALID);
		reject_reason = 2;
	}
	else if ((remote.get() != Remote_Forward && remote.get() != Remote_Left && remote.get() != Remote_Right &&
					remote.get() != Remote_Home) && !bat_ready_to_clean)
	{
		ROS_WARN("%s %d: Battery level low %4dmV(limit in %4dmV)", __FUNCTION__, __LINE__, battery.get_voltage(), (int)BATTERY_READY_TO_CLEAN_VOLTAGE);
		beeper.play_for_command(INVALID);
		reject_reason = 3;
	}

	if (!reject_reason)
	{
		beeper.play_for_command(VALID);
		switch (remote.get())
		{
			case Remote_Forward:
			case Remote_Left:
			case Remote_Right:
			{
				temp_mode = Clean_Mode_Remote;
				break;
			}
			case Remote_Clean:
			{
				temp_mode = Clean_Mode_Navigation;
				key.reset();
				break;
			}
			case Remote_Spot:
			{
				temp_mode = Clean_Mode_Spot;
				break;
			}
			case Remote_Home:
			{
				if (cs_is_paused())
				{
					ev.remote_home = true;
					extern bool g_go_home_by_remote;
					g_go_home_by_remote = true;
					temp_mode = Clean_Mode_Navigation;
				}
				else
					temp_mode = Clean_Mode_Exploration;
				break;
			}
			case Remote_Wall_Follow:
			{
				temp_mode = Clean_Mode_WallFollow;
				break;
			}
		}
	}

	remote.reset();
}

void Idle_EventHandle::remote_plan(bool state_now, bool state_last)
{
	/* -----------------------------Check if plan event ----------------------------------*/
	if (timer.get_status())
		plan_confirm_time = time(NULL);

	/* reset charger_signal_start_time when get plan status */
	charger_signal_start_time = time(NULL);

	switch (timer.get_status())
	{
		case 1:
		{
			beeper.play_for_command(VALID);
			plan_status = 1;
			ROS_WARN("%s %d: Plan received, plan status: %d.", __FUNCTION__, __LINE__, plan_status);
			break;
		}
		case 2:
		{
			beeper.play_for_command(VALID);
			plan_status = 2;
			ROS_WARN("%s %d: Plan cancel received, plan status: %d.", __FUNCTION__, __LINE__, plan_status);
			break;
		}
		case 3:
		{
			ROS_WARN("%s %d: Plan activated.", __FUNCTION__, __LINE__);
			if (error.get() != Error_Code_None)
			{
				ROS_INFO("%s %d: Error exists, so cancel the appointment.", __FUNCTION__, __LINE__);
				reject_reason = 1;
				plan_status = 2;
				break;
			}
			else if(cliff.get_status() == BLOCK_ALL)
			{
				ROS_WARN("%s %d: Plan not activated not valid because of robot lifted up.", __FUNCTION__, __LINE__);
				reject_reason = 2;
				plan_status = 2;
				break;
			}
			else if (!battery.is_ready_to_clean())
			{
				ROS_WARN("%s %d: Plan not activated not valid because of battery not ready to clean.", __FUNCTION__, __LINE__);
				reject_reason = 3;
				plan_status = 2;
				break;
			}
			else
			{
				// Sleep for 50ms cause the status 3 will be sent for 3 times.
				usleep(50000);
				if (cs_is_paused())
				{
					cs_paused_setting();
				}
				g_plan_activated = true;
				break;
			}
		}
	}

	timer.set_status(0);
}

void Idle_EventHandle::key_clean(bool state_now, bool state_last)
{
	ROS_WARN("%s %d: Key clean has been pressed.", __FUNCTION__, __LINE__);

	omni.set_stop(false);
	//g_robot_stuck = false;
	time_t key_press_start_time = time(NULL);

	/* reset charger_signal_start_time when get key clean */
	charger_signal_start_time = time(NULL);

	if (error.clear(error.get()))
		beeper.play_for_command(VALID);
	else
		beeper.play_for_command(INVALID);

	while (key.get_press() & KEY_CLEAN)
	{
		if (time(NULL) - key_press_start_time >= 3)
		{
			if (!long_press_to_sleep)
			{
				long_press_to_sleep = true;
				beeper.play_for_command(VALID);
				ROS_WARN("%s %d: Robot is going to sleep.", __FUNCTION__, __LINE__);
			}
		}
		else
			ROS_DEBUG("%s %d: User hasn't release the key.", __FUNCTION__, __LINE__);
		usleep(40000);
	}
	ROS_WARN("%s %d: Key clean is released.", __FUNCTION__, __LINE__);

	if (long_press_to_sleep)
	{
		key.reset();
		key.reset();
		return;
	}

	if (error.get())
	{
		if (error.clear(error.get()))
		{
			reject_reason = 4;
			ROS_WARN("%s %d: Clear the error %x.", __FUNCTION__, __LINE__, remote.get());
		}
		else
			reject_reason = 1;
	}
	else if(cliff.get_status() == BLOCK_ALL)
	{
		ROS_WARN("%s %d: Remote key %x not valid because of robot lifted up.", __FUNCTION__, __LINE__, remote.get());
		reject_reason = 2;
	}
	else if(!bat_ready_to_clean && !cs_is_paused())
	{
		ROS_WARN("%s %d: Battery level low %4dmV(limit in %4dmV)", __FUNCTION__, __LINE__, battery.get_voltage(), (int)BATTERY_READY_TO_CLEAN_VOLTAGE);
		reject_reason = 3;
	}

	if (!reject_reason)
		temp_mode = Clean_Mode_Navigation;

	key.reset();
	key.reset();
}

void Idle_EventHandle::charge_detect(bool state_now, bool state_last)
{
	ROS_WARN("%s %d: Detect charging.", __FUNCTION__, __LINE__);
	temp_mode = Clean_Mode_Charging;
}
