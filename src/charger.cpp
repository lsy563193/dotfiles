#include <ros/ros.h>
#include <ros/console.h>
#include <stdio.h>
#include <wav.h>

#include "go_home.hpp"
#include "movement.h"
#include "charger.hpp"
#include "robot.hpp"
#include "gyro.h"
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
/* value for saving last charge status */
uint8_t last_charge_status = 0;
bool g_charge_turn_connect_fail = false;
/*---------------------------------------------------------------- Charge Function ------------------------*/
void charge_function(void)
{
	bool battery_full = false;
	// This counter is for debug message.
	uint16_t show_batv_counter=0;
	// This counter is for checking if battery enough to continue cleaning.
	uint16_t bat_enough_to_continue_cleaning_counter = 0;
	bool eh_status_now=false, eh_status_last=false;
	uint16_t bat_v;
	set_led_mode(LED_BREATH, LED_ORANGE);
	set_start_charge();
	set_plan_status(0);
	charge_register_event();
	event_manager_reset_status();
	wav_play(WAV_BATTERY_CHARGE);
	ROS_INFO("%s %d: Start charger mode.", __FUNCTION__, __LINE__);

	while(ros::ok())
	{
		if(event_manager_check_event(&eh_status_now, &eh_status_last) == 1)
		{
			continue;
		}

		/* refresh last_charge_status */
		if(robot::instance()->getChargeStatus())
			last_charge_status = robot::instance()->getChargeStatus();

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
				cm_set(Clean_Mode_Navigation);
				break;
			}
		}

		if(++show_batv_counter > 500)//about 10 second
		{
			ROS_INFO("%s %d: In charge mode looping , battery voltage \033[32m%5.2f V\033[0m.", __FUNCTION__, __LINE__, (float)bat_v/100.0);
			show_batv_counter = 0;
		}

		if(g_stop_charge_counter > 0)g_stop_charge_counter--;
		if(g_stop_charge_counter <15)
			ROS_WARN("%s %d: g_stop_charge_counter: %d, charge_status: %d", __FUNCTION__, __LINE__, g_stop_charge_counter, robot::instance()->getChargeStatus());
		if(g_stop_charge_counter == 0)	//disconnect to charger for 0.5s, exit charge mode
		{
			g_charge_detect = 0;
			if(!charge_turn_connect())
			{
				if (g_resume_cleaning)
				{
					if (robot::instance()->getBatteryVoltage() < LOW_BATTERY_STOP_VOLTAGE)
					{
						ROS_INFO("%s %d: Exit charger mode and but battery too low to continue cleaning.", __FUNCTION__, __LINE__);
						cm_set(Clean_Mode_Userinterface);
						g_resume_cleaning = false;
					}
					else
					{
						ROS_INFO("%s %d: Exit charger mode and continue cleaning.", __FUNCTION__, __LINE__);
						cm_set(Clean_Mode_Navigation);
					}
					break;
				}

				ROS_INFO("%s %d: Exit charger mode and go to userinterface mode.", __FUNCTION__, __LINE__);
				cm_set(Clean_Mode_Userinterface);
				break;
			}
			else
			{
				g_stop_charge_counter = 20;
			}
		}
		if(g_cliff_all_triggered)
		{
			disable_motors();
			g_cliff_all_triggered = 0;
			cm_set(Clean_Mode_Userinterface);
			break;
		}
		if(g_plan_activated)
		{
			cm_set(Clean_Mode_Navigation);
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
					reset_clean_paused();
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
		if (cm_is_navigation())
			break;

		if (check_bat_full() && !battery_full)
		{
			battery_full = true;
			set_led_mode(LED_STEADY, LED_OFF);
			wav_play(WAV_BATTERY_CHARGE_DONE);
		}
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
	event_manager_enable_handler(EVT_RCON, true);
	/* Cliff */
	event_manager_register_and_enable_x(cliff_all, EVT_CLIFF_ALL, true);
#undef event_manager_register_and_enable_x

	event_manager_set_enable(true);
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
	/* Cliff */
	event_manager_register_and_disable_x(EVT_CLIFF_ALL);
#undef event_manager_register_and_disable_x

	event_manager_set_enable(false);
}

void charge_handle_charge_detect(bool state_now, bool state_last)
{
	g_charge_detect = 1;
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
			beep_for_command(VALID);
			charge_plan_status = 1;
			ROS_WARN("%s %d: Plan received, plan status: %d.", __FUNCTION__, __LINE__, charge_plan_status);
			break;
		}
		case 2:
		{
			beep_for_command(VALID);
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
			else if(get_cliff_status() & (BLOCK_LEFT|BLOCK_FRONT|BLOCK_RIGHT))
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
				if (is_clean_paused())
				{
					reset_clean_paused();
				}
				g_plan_activated = true;
				break;
			}
		}
	}
	set_plan_status (0);
}
void charge_handle_cliff_all(bool state_now, bool state_last)
{
	g_cliff_all_cnt++;
	if (g_cliff_all_cnt++ > 2)
	{
		g_fatal_quit_event = true;
		g_cliff_all_triggered = true;
	}
}
void charge_handle_key_clean(bool state_now, bool state_last)
{
	if (is_direct_charge())
	{
		ROS_WARN("%s %d: Can not go to navigation mode during direct charging.", __FUNCTION__, __LINE__);
		beep_for_command(INVALID);
	}
	else if (get_error_code() != Error_Code_None)
	{
		ROS_INFO("%s %d: Error exists.", __FUNCTION__, __LINE__);
		if (check_error_cleared(get_error_code()))
		{
			beep_for_command(VALID);
			charge_reject_reason = 4;
		}
		else
		{
			beep_for_command(INVALID);
			charge_reject_reason = 1;
		}
		reset_stop_event_status();
	}
	else if(get_cliff_status() & (BLOCK_LEFT|BLOCK_FRONT|BLOCK_RIGHT))
	{
		ROS_WARN("%s %d: Robot lifted up.", __FUNCTION__, __LINE__);
		beep_for_command(INVALID);
		charge_reject_reason = 2;
	}
	else if (!check_bat_ready_to_clean())
	{
		ROS_WARN("%s %d: Battery below BATTERY_READY_TO_CLEAN_VOLTAGE(%d) + 600, can't go to navigation mode.", __FUNCTION__, __LINE__, BATTERY_READY_TO_CLEAN_VOLTAGE);
		beep_for_command(INVALID);
		charge_reject_reason = 3;
	}
	else if (is_on_charger_stub())
	{
		ROS_WARN("%s %d: Exit charger mode and go to navigation mode.", __FUNCTION__, __LINE__);
		beep_for_command(VALID);
		cm_set(Clean_Mode_Navigation);
	}

	// Key release detection, if user has not release the key, don't do anything.
	while (get_key_press() & KEY_CLEAN)
		usleep(20000);
	ROS_WARN("%s %d: Key clean is released.", __FUNCTION__, __LINE__);

	reset_touch();
}
void charge_handle_remote_cleaning(bool stat_now, bool state_last)
{
	if (remote_key(Remote_Clean)) {
		reset_rcon_remote();
		if (is_direct_charge())
		{
			ROS_WARN("%s %d: Can not go to navigation mode during direct charging.", __FUNCTION__, __LINE__);
			beep_for_command(INVALID);
		}
		else if (get_error_code() != Error_Code_None)
		{
			ROS_INFO("%s %d: Error exists.", __FUNCTION__, __LINE__);
			if (check_error_cleared(get_error_code()))
			{
				beep_for_command(VALID);
				charge_reject_reason = 4;
			}
			else
			{
				beep_for_command(INVALID);
				charge_reject_reason = 1;
			}
			reset_stop_event_status();
		}
		else if(get_cliff_status() & (BLOCK_LEFT|BLOCK_FRONT|BLOCK_RIGHT))
		{
			ROS_WARN("%s %d: Robot lifted up.", __FUNCTION__, __LINE__);
			beep_for_command(INVALID);
			charge_reject_reason = 2;
		}
		else if (!check_bat_ready_to_clean())
		{
			ROS_WARN("%s %d: Battery below BATTERY_READY_TO_CLEAN_VOLTAGE(%d) + 600, can't go to navigation mode.", __FUNCTION__, __LINE__, BATTERY_READY_TO_CLEAN_VOLTAGE);
			charge_reject_reason = 3;
			beep_for_command(INVALID);
		}
		else if (is_on_charger_stub())
		{
			ROS_WARN("%s %d: Exit charger mode and go to navigation mode.", __FUNCTION__, __LINE__);
			beep_for_command(VALID);
			cm_set(Clean_Mode_Navigation);
		}
	}
	else{
		beep_for_command(INVALID);
		reset_rcon_remote();
	}
}

bool charge_turn_connect(void)
{
	/* charging with adapter, no use to execute turn_connect */
	if(last_charge_status == 4)
		return false;
	ROS_INFO("%s %d: Start charge_turn_connect().", __FUNCTION__, __LINE__);
	// This function is for trying turning left and right to adjust the pose of robot, so that it can charge.
	int8_t speed = 5;
	if(g_charge_detect)
	{
		ROS_INFO("Reach charger without turning.");
		return true;
	}
	// Start turning right.
	set_dir_right();
	set_wheel_speed(speed, speed);
	for(int i=0; i<25; i++)
	{
		if (g_charge_detect)
		{
			g_charge_detect = 0;
			disable_motors();
			// Wait for a while to decide if it is really on the charger stub.
			usleep(500000);
			if (g_charge_detect)
			{
				ROS_INFO("Turn left reach charger.");
				return true;
			}
			set_wheel_speed(speed, speed);
		}
		if(g_key_clean_pressed || g_fatal_quit_event)
			return true;
		usleep(50000);
	}
	stop_brifly();
	// Start turning left.
	set_dir_left();
	set_wheel_speed(speed, speed);
	for(int i=0; i<40; i++)
	{
		if (g_charge_detect)
		{
			g_charge_detect = 0;
			disable_motors();
			// Wait for a while to decide if it is really on the charger stub.
			usleep(500000);
			if (g_charge_detect)
			{
				ROS_INFO("Turn left reach charger.");
				return true;
			}
			set_wheel_speed(speed, speed);
		}
		if(g_key_clean_pressed || g_fatal_quit_event)
			return true;
		usleep(50000);
	}
	stop_brifly();
	g_charge_turn_connect_fail = true;
	return false;

}

