#include <ros/ros.h>
#include <ros/console.h>
#include <stdio.h>
#include <wav.h>

#include "movement.h"
#include "charger.hpp"
#include "robot.hpp"
#include "gyro.h"
#include "random_runing.h"
#include "core_move.h"
#include "event_manager.h"

#ifdef Turn_Speed
#undef Turn_Speed
#define Turn_Speed 18
#endif


/* Exit charge mode when this counter equals to 0 */
uint8_t g_stop_charge_counter = 0;

uint8_t charge_plan_status = 0;
uint8_t charge_reject_reason = 0;
time_t charge_plan_confirm_time = time(NULL);

extern bool g_resume_cleaning;

/*---------------------------------------------------------------- Charge Function ------------------------*/
void charge_function(void)
{

	volatile uint8_t display_switch=0;

	bool battery_full = false;

	#ifdef ONE_KEY_DISPLAY

	uint8_t one_display_counter=100;

	#endif

	// This counter is for debug message.
	uint8_t show_batv_counter=0;

	// This counter is for checking if battery enough to continue cleaning.
	uint16_t bat_enough_to_continue_cleaning_counter = 0;

	bool eh_status_now=false, eh_status_last=false;
	uint16_t bat_v;

	set_led(100, 100);
	set_start_charge();
	set_plan_status(0);
	charge_register_event();
	event_manager_reset_status();
	wav_play(WAV_BATTERY_CHARGE);
	ROS_INFO("%s %d: Start charger mode.", __FUNCTION__, __LINE__);

	while(ros::ok())
	{
		usleep(10000);
		bat_v = get_battery_voltage();

		if (g_resume_cleaning)
		{
			if (bat_v >= CONTINUE_CLEANING_VOLTAGE)
			{
				bat_enough_to_continue_cleaning_counter++;
				//ROS_INFO("Bat_Enough_To_Continue_Cleaning_Counter = %d.", bat_enough_to_continue_cleaning_counter);
			}
			else
			{
				bat_enough_to_continue_cleaning_counter = 0;
			}

			if (bat_enough_to_continue_cleaning_counter > 500)// About 10 seconds.
			{
				ROS_INFO("%s %d: Robot finish charging, continue cleaning.", __FUNCTION__, __LINE__);
				set_clean_mode(Clean_Mode_Navigation);
				break;
			}
		}
		if(show_batv_counter > 250)
		{
			ROS_INFO("%s %d: In charge mode looping , battery voltage %5.2f V.", __FUNCTION__, __LINE__, bat_v/100.0);
			show_batv_counter = 0;
		}
		else
		{
			show_batv_counter++;
		}

		if(event_manager_check_event(&eh_status_now, &eh_status_last) == 1)
		{
			continue;
		}
		if(g_stop_charge_counter > 0)g_stop_charge_counter--;
		if(g_stop_charge_counter == 0)	//disconnect to charger for 0.5s, exit charge mode
		{
			if (g_resume_cleaning)
			{
				if (robot::instance()->getBatteryVoltage() < LOW_BATTERY_STOP_VOLTAGE)
				{
					ROS_INFO("%s %d: Exit charger mode and but battery too low to continue cleaning.", __FUNCTION__, __LINE__);
					set_clean_mode(Clean_Mode_Userinterface);
					g_resume_cleaning = false;
				}
				else
				{
					ROS_INFO("%s %d: Exit charger mode and continue cleaning.", __FUNCTION__, __LINE__);
					set_clean_mode(Clean_Mode_Navigation);
				}
				break;
			}

			ROS_INFO("%s %d: Exit charger mode and go to userinterface mode.", __FUNCTION__, __LINE__);
			set_clean_mode(Clean_Mode_Userinterface);
			break;
		}
		if (charge_reject_reason)
		{
			switch (charge_reject_reason)
			{
				case 1:
					alarm_error();
					charge_reject_reason = 0;
					break;
				case 2:
					wav_play(WAV_ERROR_LIFT_UP);
					clear_manual_pause();
					charge_reject_reason = 0;
					break;
				case 3:
					wav_play(WAV_BATTERY_LOW);
					charge_reject_reason = 0;
					break;
				case 4:
					wav_play(WAV_CLEAR_ERROR);
					charge_reject_reason = 0;
					set_error_code(Error_Code_None);
					break;
			}
		}
		else if (charge_plan_status)
		{
			if (charge_plan_status == 2 && (time(NULL) - charge_plan_confirm_time >= 2))
			{
				ROS_WARN("%s %d: Cancel appointment.", __FUNCTION__, __LINE__);
				wav_play(WAV_CANCEL_APPOINTMENT);
				charge_plan_status = 0;
			}
			else if (charge_plan_status == 1 && (time(NULL) - charge_plan_confirm_time >= 2))
			{
				ROS_WARN("%s %d: Confirm appointment.", __FUNCTION__, __LINE__);
				wav_play(WAV_APPOINTMENT_DONE);
				charge_plan_status = 0;
			}
		}
		if (get_clean_mode() == Clean_Mode_Navigation)
			break;

		#ifdef ONE_KEY_DISPLAY
		if (check_bat_full() && !battery_full)
		{
			battery_full = true;
			set_led(0, 0);
			wav_play(WAV_BATTERY_CHARGE_DONE);
		}

		if (!battery_full)
		{
			// For displaying breathing LED
			if(display_switch)
			{
				one_display_counter+=2;
				if(one_display_counter>98)display_switch=0;
			}
			else
			{
				one_display_counter-=2;
				if(one_display_counter<2)display_switch=1;
			}

			set_led(one_display_counter, one_display_counter);
		}
		#endif

	}
	charge_unregister_event();
	set_stop_charge();
	// Wait for 20ms to make sure stop charging command has been sent.
	usleep(20000);

	if (charge_plan_status == 2)
		wav_play(WAV_CANCEL_APPOINTMENT);
	else if (charge_plan_status == 1)
		wav_play(WAV_APPOINTMENT_DONE);
	charge_plan_status = 0;
}

void charge_register_event(void)
{
	ROS_WARN("%s, %d: Register events.", __FUNCTION__, __LINE__);
	event_manager_set_current_mode(EVT_MODE_CHARGE);
#define event_manager_register_and_enable_x(name, y, enabled) \
	event_manager_register_handler(y, &charge_handle_ ##name); \
	event_manager_enable_handler(y, enabled)

	/* Charge Status */
	event_manager_register_and_enable_x(charge_detect, EVT_CHARGE_DETECT, true);
	/* Plan */
	event_manager_register_and_enable_x(remote_plan, EVT_REMOTE_PLAN, true);
	/* key */
	event_manager_register_and_enable_x(key_clean, EVT_KEY_CLEAN, true);
	/* Remote */
	event_manager_register_and_enable_x(remote_cleaning, EVT_REMOTE_CLEAN, true);
	event_manager_enable_handler(EVT_REMOTE_HOME, true);
	event_manager_enable_handler(EVT_REMOTE_DIRECTION_FORWARD, true);
	event_manager_enable_handler(EVT_REMOTE_DIRECTION_LEFT, true);
	event_manager_enable_handler(EVT_REMOTE_DIRECTION_RIGHT, true);
	event_manager_enable_handler(EVT_REMOTE_WALL_FOLLOW, true);
	event_manager_enable_handler(EVT_REMOTE_SPOT, true);
	event_manager_enable_handler(EVT_REMOTE_MAX, true);
}

void charge_unregister_event(void)
{
	ROS_WARN("%s, %d: Unregister events.", __FUNCTION__, __LINE__);
#define event_manager_register_and_disable_x(x) \
	event_manager_register_handler(x, NULL); \
	event_manager_enable_handler(x, false)

	/* Charge Status */
	event_manager_register_and_disable_x(EVT_CHARGE_DETECT);
	/* Plan */
	event_manager_register_and_disable_x(EVT_REMOTE_PLAN);
	/* Key */
	event_manager_register_and_disable_x(EVT_KEY_CLEAN);
	/* Remote */
	event_manager_register_and_disable_x(EVT_REMOTE_CLEAN);
	event_manager_enable_handler(EVT_REMOTE_HOME, false);
	event_manager_enable_handler(EVT_REMOTE_DIRECTION_FORWARD, false);
	event_manager_enable_handler(EVT_REMOTE_DIRECTION_LEFT, false);
	event_manager_enable_handler(EVT_REMOTE_DIRECTION_RIGHT, false);
	event_manager_enable_handler(EVT_REMOTE_WALL_FOLLOW, false);
	event_manager_enable_handler(EVT_REMOTE_SPOT, false);
	event_manager_enable_handler(EVT_REMOTE_MAX, false);
}

void charge_handle_charge_detect(bool state_now, bool state_last)
{
	g_stop_charge_counter = 20;
}
void charge_handle_remote_plan(bool state_now, bool state_last)
{
	if (get_plan_status())
		charge_plan_confirm_time = time(NULL);

	switch(get_plan_status())
	{
		case 1:
		{
			beep_for_command(true);
			charge_plan_status = 1;
			ROS_WARN("%s %d: Plan received, plan status: %d.", __FUNCTION__, __LINE__, charge_plan_status);
			break;
		}
		case 2:
		{
			beep_for_command(true);
			charge_plan_status = 2;
			ROS_WARN("%s %d: Plan cancel received, plan status: %d.", __FUNCTION__, __LINE__, charge_plan_status);
			break;
		}
		case 3:
		{
			ROS_WARN("%s %d: Plan activated.", __FUNCTION__, __LINE__);
			if (get_error_code() != Error_Code_None)
			{
				ROS_INFO("%s %d: Error exists, so cancel the appointment.", __FUNCTION__, __LINE__);
				charge_reject_reason = 1;
				charge_plan_status = 2;
				break;
			}
			else if(get_cliff_status() & (Status_Cliff_Left|Status_Cliff_Front|Status_Cliff_Right))
			{
				ROS_WARN("%s %d: Plan not activated not valid because of robot lifted up.", __FUNCTION__, __LINE__);
				charge_reject_reason = 2;
				charge_plan_status = 2;
				break;
			}
			else if (!check_bat_ready_to_clean())
			{
				ROS_WARN("%s %d: Plan not activated not valid because of battery not ready to clean.", __FUNCTION__, __LINE__);
				charge_reject_reason = 3;
				charge_plan_status = 2;
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
				set_clean_mode(Clean_Mode_Navigation);
				break;
			}
		}
	}
	set_plan_status (0);
}
void charge_handle_key_clean(bool state_now, bool state_last)
{
	if (is_direct_charge())
	{
		ROS_WARN("%s %d: Can not go to navigation mode during direct charging.", __FUNCTION__, __LINE__);
		beep_for_command(false);
	}
	else if (get_error_code() != Error_Code_None)
	{
		ROS_INFO("%s %d: Error exists.", __FUNCTION__, __LINE__);
		if (check_error_cleared(get_error_code()))
		{
			beep_for_command(true);
			charge_reject_reason = 4;
		}
		else
		{
			beep_for_command(false);
			charge_reject_reason = 1;
		}
		reset_stop_event_status();
	}
	else if(get_cliff_status() & (Status_Cliff_Left|Status_Cliff_Front|Status_Cliff_Right))
	{
		ROS_WARN("%s %d: Robot lifted up.", __FUNCTION__, __LINE__);
		beep_for_command(false);
		charge_reject_reason = 2;
	}
	else if (!check_bat_ready_to_clean())
	{
		ROS_WARN("%s %d: Battery below BATTERY_READY_TO_CLEAN_VOLTAGE(%d) + 600, can't go to navigation mode.", __FUNCTION__, __LINE__, BATTERY_READY_TO_CLEAN_VOLTAGE);
		beep_for_command(false);
		charge_reject_reason = 3;
	}
	else if (is_on_charger_stub())
	{
		ROS_WARN("%s %d: Exit charger mode and go to navigation mode.", __FUNCTION__, __LINE__);
		beep_for_command(true);
		set_clean_mode(Clean_Mode_Navigation);
	}

	// Key release detection, if user has not release the key, don't do anything.
	while (get_key_press() & KEY_CLEAN)
	{
		ROS_WARN("%s %d: User hasn't release key.", __FUNCTION__, __LINE__);
		usleep(20000);
	}

	reset_touch();
}
void charge_handle_remote_cleaning(bool stat_now, bool state_last)
{
	if (remote_key(Remote_Clean)) {
		reset_rcon_remote();
		if (is_direct_charge())
		{
			ROS_WARN("%s %d: Can not go to navigation mode during direct charging.", __FUNCTION__, __LINE__);
			beep_for_command(false);
		}
		else if (get_error_code() != Error_Code_None)
		{
			ROS_INFO("%s %d: Error exists.", __FUNCTION__, __LINE__);
			if (check_error_cleared(get_error_code()))
			{
				beep_for_command(true);
				charge_reject_reason = 4;
			}
			else
			{
				beep_for_command(false);
				charge_reject_reason = 1;
			}
			reset_stop_event_status();
		}
		else if(get_cliff_status() & (Status_Cliff_Left|Status_Cliff_Front|Status_Cliff_Right))
		{
			ROS_WARN("%s %d: Robot lifted up.", __FUNCTION__, __LINE__);
			beep_for_command(false);
			charge_reject_reason = 2;
		}
		else if (!check_bat_ready_to_clean())
		{
			ROS_WARN("%s %d: Battery below BATTERY_READY_TO_CLEAN_VOLTAGE(%d) + 600, can't go to navigation mode.", __FUNCTION__, __LINE__, BATTERY_READY_TO_CLEAN_VOLTAGE);
			charge_reject_reason = 3;
			beep_for_command(false);
		}
		else if (is_on_charger_stub())
		{
			ROS_WARN("%s %d: Exit charger mode and go to navigation mode.", __FUNCTION__, __LINE__);
			beep_for_command(true);
			set_clean_mode(Clean_Mode_Navigation);
		}
	}
	else{
		beep_for_command(false);
		reset_rcon_remote();
	}
}

