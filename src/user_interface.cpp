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
#include "movement.h"
#include "user_interface.h"
#include <ros/ros.h>
#include "config.h"
#include "wav.h"
#include "robot.hpp"
#include "robotbase.h"
#include "event_manager.h"
#include "core_move.h"

uint8_t temp_mode=0;
time_t charger_signal_start_time;
uint16_t charger_signal_delay = 0;
time_t battery_low_start_time;
uint16_t battery_low_delay = 0;
bool battery_ready_to_clean = true;
bool long_press_to_sleep = false;
uint8_t user_interface_reject_reason = 0; // 1 for error exist, 2 for robot lifted up, 3 for battery low, 4 for key clean clear the error.
uint8_t user_interface_plan_status = 0;
time_t user_interface_plan_confirm_time = time(NULL);
/*------------------------------------------------------------User Interface ----------------------------------*/
void user_interface(void)
{
	time_t start_time;

	bool eh_status_now=false, eh_status_last=false;

	// Count for error alarm.
	uint8_t error_alarm_counter = 3;
	charger_signal_delay = 0;
	battery_low_delay = 0;
	start_time = time(NULL);
	temp_mode=0;
	battery_ready_to_clean = true;

	disable_motors();
	reset_rcon_remote();
	set_plan_status(0);
	reset_stop_event_status();
	reset_rcon_status();
	reset_touch();
	set_vacmode(Vac_Save);

	ROS_INFO("%s,%d ,BatteryVoltage = %dmv.",__FUNCTION__,__LINE__, get_battery_voltage());
	// Check the battery to warn the user.
	if(!check_bat_ready_to_clean() && !robot::instance()->isManualPaused())
	{
		ROS_WARN("%s %d: Battery level low %4dmV(limit in %4dmV).", __FUNCTION__, __LINE__, get_battery_voltage(),(int)BATTERY_READY_TO_CLEAN_VOLTAGE);
		battery_ready_to_clean = false;
		set_led_mode(LED_BREATH, LED_ORANGE);
		wav_play(WAV_BATTERY_LOW);
	}
	else
		set_led_mode(LED_BREATH, LED_GREEN);

	if(get_error_code())
		set_led_mode(LED_STEADY, LED_RED);

	event_manager_reset_status();
	user_interface_register_events();

	if (robot::instance()->isManualPaused())
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

		if (charger_signal_delay > 0)
			charger_signal_delay--;

		if (battery_low_delay > 0)
			battery_low_delay--;

		if(battery_ready_to_clean && !check_bat_ready_to_clean() && !robot::instance()->isManualPaused())
		{
			battery_ready_to_clean = false;
			set_led_mode(LED_BREATH, LED_ORANGE);
		}

		if(time(NULL) - start_time > USER_INTERFACE_TIMEOUT)
		{
			ROS_WARN("%s %d: Userinterface mode didn't receive any command in 10mins, go to sleep mode.", __FUNCTION__, __LINE__);
			set_clean_mode(Clean_Mode_Sleep);
			break;
		}

		if (robot::instance()->isManualPaused())
		{
			float distance = sqrtf(powf(saved_pos_x - robot::instance()->getOdomPositionX(), 2) + powf(saved_pos_y - robot::instance()->getOdomPositionY(), 2));
			if (distance > 0.1f)
				clear_manual_pause();
		}

		if(g_plan_activated)
		{
			temp_mode = Clean_Mode_Navigation;
		}
		// Check for wav playing.
		if (long_press_to_sleep)
		{
			long_press_to_sleep = false;
			if (robot::instance()->isManualPaused())
				clear_manual_pause();
			else
				temp_mode = Clean_Mode_Sleep;
		}
		else if (user_interface_reject_reason)
		{
			switch (user_interface_reject_reason)
			{
				case 1:
					error_alarm_counter = 0;
					alarm_error();
					user_interface_reject_reason = 0;
					break;
				case 2:
					wav_play(WAV_ERROR_LIFT_UP);
					clear_manual_pause();
					user_interface_reject_reason = 0;
					break;
				case 3:
					wav_play(WAV_BATTERY_LOW);
					user_interface_reject_reason = 0;
					break;
				case 4:
					set_led_mode(LED_BREATH, LED_GREEN);
					wav_play(WAV_CLEAR_ERROR);
					error_alarm_counter = 0;
					set_error_code(Error_Code_None);
					user_interface_reject_reason = 0;
					break;
			}
		}
		else if (user_interface_plan_status)
		{
			if (user_interface_plan_status == 2 && (time(NULL) - user_interface_plan_confirm_time >= 2))
			{
				ROS_WARN("%s %d: Cancel appointment.", __FUNCTION__, __LINE__);
				wav_play(WAV_CANCEL_APPOINTMENT);
				user_interface_plan_status = 0;
			}
			else if (user_interface_plan_status == 1 && (time(NULL) - user_interface_plan_confirm_time >= 2))
			{
				ROS_WARN("%s %d: Confirm appointment.", __FUNCTION__, __LINE__);
				wav_play(WAV_APPOINTMENT_DONE);
				user_interface_plan_status = 0;
			}
		}
		// Alarm for error.
		else if (get_error_code()){
			if (error_alarm_counter == 3 || (error_alarm_counter == 2 && (time(NULL) - start_time) >= 10) || (error_alarm_counter == 1 && (time(NULL) - start_time) >= 20))
			{
				error_alarm_counter--;
				alarm_error();
			}
		}
		if(temp_mode != 0)
		{
			set_clean_mode(temp_mode);
			break;
		}
	}

	user_interface_unregister_events();

	// Make sure alarm will be done.
	if (user_interface_plan_status == 2)
		wav_play(WAV_CANCEL_APPOINTMENT);
	else if (user_interface_plan_status == 1)
		wav_play(WAV_APPOINTMENT_DONE);
	user_interface_plan_status = 0;
}

void user_interface_register_events(void)
{
	ROS_INFO("%s %d: Register events", __FUNCTION__, __LINE__);
	event_manager_set_current_mode(EVT_MODE_USER_INTERFACE);

#define event_manager_register_and_enable_x(name, y, enabled) \
	event_manager_register_handler(y, &user_interface_handle_ ##name); \
	event_manager_enable_handler(y, enabled);

	/* Cliff */
	event_manager_register_and_enable_x(cliff, EVT_CLIFF_ALL, true);
	event_manager_register_and_enable_x(cliff, EVT_CLIFF_FRONT_LEFT, true);
	event_manager_register_and_enable_x(cliff, EVT_CLIFF_FRONT_RIGHT, true);
	event_manager_register_and_enable_x(cliff, EVT_CLIFF_LEFT_RIGHT, true);
	event_manager_register_and_enable_x(cliff, EVT_CLIFF_FRONT, true);
	event_manager_register_and_enable_x(cliff, EVT_CLIFF_LEFT, true);
	event_manager_register_and_enable_x(cliff, EVT_CLIFF_RIGHT, true);
	/* Rcon */
	event_manager_register_and_enable_x(rcon, EVT_RCON, true);
	/* Battery */
	event_manager_register_and_enable_x(battery_low, EVT_BATTERY_LOW, true);
	/* Remote */
	event_manager_register_and_enable_x(remote_cleaning, EVT_REMOTE_DIRECTION_FORWARD, true);
	event_manager_register_and_enable_x(remote_cleaning, EVT_REMOTE_DIRECTION_LEFT, true);
	event_manager_register_and_enable_x(remote_cleaning, EVT_REMOTE_DIRECTION_RIGHT, true);
	event_manager_register_and_enable_x(remote_cleaning, EVT_REMOTE_CLEAN, true);
	event_manager_register_and_enable_x(remote_cleaning, EVT_REMOTE_SPOT, true);
	event_manager_register_and_enable_x(remote_cleaning, EVT_REMOTE_HOME, true);
	event_manager_register_and_enable_x(remote_cleaning, EVT_REMOTE_WALL_FOLLOW, true);
	event_manager_enable_handler(EVT_REMOTE_MAX, true);
	event_manager_register_and_enable_x(remote_plan, EVT_REMOTE_PLAN, true);
	/* Key */
	event_manager_register_and_enable_x(key_clean, EVT_KEY_CLEAN, true);
	/* Charge Status */
	event_manager_register_and_enable_x(charge_detect, EVT_CHARGE_DETECT, true);

#undef event_manager_register_and_enable_x

	event_manager_set_enable(true);
}

void user_interface_unregister_events(void)
{
	ROS_WARN("%s %d: Unregister events", __FUNCTION__, __LINE__);
#define event_manager_register_and_disable_x(x) \
	event_manager_register_handler(x, NULL); \
	event_manager_enable_handler(x, false);

	/* Cliff */
	event_manager_register_and_disable_x(EVT_CLIFF_ALL);
	event_manager_register_and_disable_x(EVT_CLIFF_FRONT_LEFT);
	event_manager_register_and_disable_x(EVT_CLIFF_FRONT_RIGHT);
	event_manager_register_and_disable_x(EVT_CLIFF_LEFT_RIGHT);
	event_manager_register_and_disable_x(EVT_CLIFF_FRONT);
	event_manager_register_and_disable_x(EVT_CLIFF_LEFT);
	event_manager_register_and_disable_x(EVT_CLIFF_RIGHT);
	/* Rcon */
	event_manager_register_and_disable_x(EVT_RCON);
	/* Battery */
	event_manager_register_and_disable_x(EVT_BATTERY_LOW);
	/* Remote */
	event_manager_register_and_disable_x(EVT_REMOTE_DIRECTION_FORWARD);
	event_manager_register_and_disable_x(EVT_REMOTE_DIRECTION_LEFT);
	event_manager_register_and_disable_x(EVT_REMOTE_DIRECTION_RIGHT);
	event_manager_register_and_disable_x(EVT_REMOTE_CLEAN);
	event_manager_register_and_disable_x(EVT_REMOTE_SPOT);
	event_manager_register_and_disable_x(EVT_REMOTE_HOME);
	event_manager_register_and_disable_x(EVT_REMOTE_WALL_FOLLOW);
	event_manager_register_and_disable_x(EVT_REMOTE_MAX);
	event_manager_register_and_disable_x(EVT_REMOTE_PLAN);
	/* Key */
	event_manager_register_and_disable_x(EVT_KEY_CLEAN);
	/* Charge Status */
	event_manager_register_and_disable_x(EVT_CHARGE_DETECT);
#undef event_manager_register_and_disable_x

	event_manager_set_enable(false);
}

void user_interface_handle_cliff(bool state_now, bool state_last)
{
	ROS_DEBUG("%s %d: Cliff triggered.", __FUNCTION__, __LINE__);

	/*--------------------------------------------------------If manual pause cleaning, check cliff--------------*/
	if (user_interface_reject_reason != 2 && robot::instance()->isManualPaused())
	{
		ROS_WARN("%s %d: Robot lifted up during manual pause, reset manual pause status.", __FUNCTION__, __LINE__);
		user_interface_reject_reason = 2;
	}
}

void user_interface_handle_rcon(bool state_now, bool state_last)
{
	if (robot::instance()->isManualPaused())
	{
		reset_rcon_status();
		ROS_DEBUG("%s %d: user_interface detects charger signal, but ignore for manual pause.", __FUNCTION__, __LINE__);
		return;
	}

	if (charger_signal_delay == 0)
		charger_signal_start_time = time(NULL);

	ROS_DEBUG("%s %d: user_interface detects charger signal(%8x) for %ds.", __FUNCTION__, __LINE__, get_rcon_status(), (int)(time(NULL) - charger_signal_start_time));
	if (time(NULL) - charger_signal_start_time >= 180)// 3 mins
	{
		if (get_error_code())
			ROS_WARN("%s %d: Rcon set go home not valid because of error %d.", __FUNCTION__, __LINE__, get_error_code());
		else if(get_cliff_status() & (Status_Cliff_Left|Status_Cliff_Front|Status_Cliff_Right))
			ROS_WARN("%s %d: Rcon set go home not valid because of robot lifted up.", __FUNCTION__, __LINE__);
		else
			temp_mode = Clean_Mode_GoHome;
	}

	charger_signal_delay = 250;
	reset_rcon_status();
}

void user_interface_handle_battery_low(bool state_now, bool state_last)
{
	if (battery_low_delay == 0)
		battery_low_start_time = time(NULL);
	ROS_DEBUG("%s %d: user_interface detects battery low %dmv for %ds.", __FUNCTION__, __LINE__, get_battery_voltage(), (int)(time(NULL) - battery_low_start_time));
	if (time(NULL) - battery_low_start_time >= 5)// 5 seconds
	{
		temp_mode = Clean_Mode_Sleep;
		return;
	}

	battery_low_delay = 10;
}

void user_interface_handle_remote_cleaning(bool state_now, bool state_last)
{
	ROS_WARN("%s %d: Remote key %x has been pressed.", __FUNCTION__, __LINE__, get_rcon_remote());
	g_omni_notmove = false;

	/* reset charger_signal_start_time when get remote cleaning */
	charger_signal_start_time = time(NULL);

	if (get_error_code())
	{
		if (get_rcon_remote() == Remote_Clean)
		{
			ROS_WARN("%s %d: Clear the error %x.", __FUNCTION__, __LINE__, get_error_code());
			if (check_error_cleared(get_error_code()))
			{
				beep_for_command(VALID);
				user_interface_reject_reason = 4;
			}
			else
			{
				beep_for_command(INVALID);
				user_interface_reject_reason = 1;
			}
			reset_stop_event_status();
		}
		else
		{
			ROS_WARN("%s %d: Remote key %x not valid because of error %d.", __FUNCTION__, __LINE__, get_rcon_remote(), get_error_code());
			user_interface_reject_reason = 1;
			beep_for_command(INVALID);
		}
	}
	else if (get_cliff_status() == Status_Cliff_All)
	{
		ROS_WARN("%s %d: Remote key %x not valid because of robot lifted up.", __FUNCTION__, __LINE__, get_rcon_remote());
		beep_for_command(INVALID);
		user_interface_reject_reason = 2;
	}
	else if ((get_rcon_remote() != Remote_Forward && get_rcon_remote() != Remote_Left && get_rcon_remote() != Remote_Right && get_rcon_remote() != Remote_Home) && !battery_ready_to_clean)
	{
		ROS_WARN("%s %d: Battery level low %4dmV(limit in %4dmV)", __FUNCTION__, __LINE__, get_battery_voltage(), (int)BATTERY_READY_TO_CLEAN_VOLTAGE);
		beep_for_command(INVALID);
		user_interface_reject_reason = 3;
	}

	if (!user_interface_reject_reason)
	{
		beep_for_command(VALID);
		switch (get_rcon_remote())
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
				reset_stop_event_status();
				break;
			}
			case Remote_Spot:
			{
				temp_mode = Clean_Mode_Spot;
				break;
			}
			case Remote_Home:
			{
				if (robot::instance()->isManualPaused())
				{
					g_remote_home = true;
					extern bool g_go_home_by_remote;
					g_go_home_by_remote = true;
					temp_mode = Clean_Mode_Navigation;
				}
				else
					temp_mode = Clean_Mode_GoHome;
				break;
			}
			case Remote_Wall_Follow:
			{
				temp_mode = Clean_Mode_WallFollow;
				break;
			}
		}
	}

	reset_rcon_remote();
}

void user_interface_handle_remote_plan(bool state_now, bool state_last)
{
	/* -----------------------------Check if plan event ----------------------------------*/
	if (get_plan_status())
		user_interface_plan_confirm_time = time(NULL);

	/* reset charger_signal_start_time when get plan status */
	charger_signal_start_time = time(NULL);

	switch (get_plan_status())
	{
		case 1:
		{
			beep_for_command(VALID);
			user_interface_plan_status = 1;
			ROS_WARN("%s %d: Plan received, plan status: %d.", __FUNCTION__, __LINE__, user_interface_plan_status);
			break;
		}
		case 2:
		{
			beep_for_command(VALID);
			user_interface_plan_status = 2;
			ROS_WARN("%s %d: Plan cancel received, plan status: %d.", __FUNCTION__, __LINE__, user_interface_plan_status);
			break;
		}
		case 3:
		{
			ROS_WARN("%s %d: Plan activated.", __FUNCTION__, __LINE__);
			if (get_error_code() != Error_Code_None)
			{
				ROS_INFO("%s %d: Error exists, so cancel the appointment.", __FUNCTION__, __LINE__);
				user_interface_reject_reason = 1;
				user_interface_plan_status = 2;
				break;
			}
			else if(get_cliff_status() == Status_Cliff_All)
			{
				ROS_WARN("%s %d: Plan not activated not valid because of robot lifted up.", __FUNCTION__, __LINE__);
				user_interface_reject_reason = 2;
				user_interface_plan_status = 2;
				break;
			}
			else if (!check_bat_ready_to_clean())
			{
				ROS_WARN("%s %d: Plan not activated not valid because of battery not ready to clean.", __FUNCTION__, __LINE__);
				user_interface_reject_reason = 3;
				user_interface_plan_status = 2;
				break;
			}
			else
			{
				// Sleep for 50ms cause the status 3 will be sent for 3 times.
				usleep(50000);
				if (robot::instance()->isManualPaused())
				{
					clear_manual_pause();
				}
				g_plan_activated = true;
				break;
			}
		}
	}

	set_plan_status(0);
}

void user_interface_handle_key_clean(bool state_now, bool state_last)
{
	ROS_WARN("%s %d: Key clean has been pressed.", __FUNCTION__, __LINE__);

	g_omni_notmove = false;
	time_t key_press_start_time = time(NULL);

	/* reset charger_signal_start_time when get key clean */
	charger_signal_start_time = time(NULL);

	if (check_error_cleared(get_error_code()))
		beep_for_command(VALID);
	else
		beep_for_command(INVALID);

	while (get_key_press() & KEY_CLEAN)
	{
		if (time(NULL) - key_press_start_time >= 3)
		{
			if (!long_press_to_sleep)
			{
				long_press_to_sleep = true;
				beep_for_command(VALID);
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
		reset_stop_event_status();
		reset_touch();
		return;
	}

	if (get_error_code())
	{
		if (check_error_cleared(get_error_code()))
		{
			user_interface_reject_reason = 4;
			ROS_WARN("%s %d: Clear the error %x.", __FUNCTION__, __LINE__, get_rcon_remote());
		}
		else
			user_interface_reject_reason = 1;
	}
	else if(get_cliff_status() == Status_Cliff_All)
	{
		ROS_WARN("%s %d: Remote key %x not valid because of robot lifted up.", __FUNCTION__, __LINE__, get_rcon_remote());
		user_interface_reject_reason = 2;
	}
	else if(!battery_ready_to_clean && !robot::instance()->isManualPaused())
	{
		ROS_WARN("%s %d: Battery level low %4dmV(limit in %4dmV)", __FUNCTION__, __LINE__, get_battery_voltage(), (int)BATTERY_READY_TO_CLEAN_VOLTAGE);
		user_interface_reject_reason = 3;
	}

	if (!user_interface_reject_reason)
		temp_mode = Clean_Mode_Navigation;

	reset_stop_event_status();
	reset_touch();
}

void user_interface_handle_charge_detect(bool state_now, bool state_last)
{
	ROS_WARN("%s %d: Detect charging.", __FUNCTION__, __LINE__);
	temp_mode = Clean_Mode_Charging;
}
