#include <ros/ros.h>
#include <ros/console.h>
#include <stdio.h>
#include <wav.h>
#include <key.h>
#include <cliff.h>
#include <led.h>
#include <battery.h>
#include <remote.h>
#include <charger.h>
#include <beep.h>

#include "go_home.hpp"
#include "movement.h"
#include "charger.hpp"
#include "robot.hpp"
#include "gyro.h"
#include "core_move.h"
#include "event_manager.h"
#include "clean_mode.h"
#include "planer.h"

#ifdef Turn_Speed
#undef Turn_Speed
#define Turn_Speed 18
#endif


/* Exit charge mode_ when this counter equals to 0 */
uint8_t g_stop_charge_counter = 0;

uint8_t charge_plan_status = 0;
uint8_t charge_reject_reason = 0;
time_t charge_plan_confirm_time = time(NULL);
/* value for saving last charge status */
uint8_t last_charge_status = 0;
bool g_charge_turn_connect_fail = false;
static Charge_EventHandle eh;
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
	led_set_mode(LED_BREATH, LED_ORANGE);
	charger.set_start();
	planer.set_status(0);
	charge_register_event();
	event_manager_reset_status();
	wav_play(WAV_BATTERY_CHARGE);
	ROS_INFO("%s %d: Start charger mode_.", __FUNCTION__, __LINE__);

	while(ros::ok())
	{
		if(event_manager_check_event(&eh_status_now, &eh_status_last) == 1)
		{
			continue;
		}

		/* refresh last_charge_status */
		if(charger.getChargeStatus())
			last_charge_status = charger.getChargeStatus();

		bat_v = battery.get_voltage();

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
			ROS_INFO("%s %d: In charge mode_ looping , battery voltage \033[32m%5.2f V\033[0m.", __FUNCTION__, __LINE__, (float)bat_v/100.0);
			show_batv_counter = 0;
		}

		if(g_stop_charge_counter > 0)g_stop_charge_counter--;
		if(g_stop_charge_counter <15)
			ROS_WARN("%s %d: g_stop_charge_counter: %d, charge_status: %d", __FUNCTION__, __LINE__, g_stop_charge_counter, charger.getChargeStatus());
		if(g_stop_charge_counter == 0)	//disconnect to charger for 0.5s, exit charge mode_
		{
			ev.charge_detect = 0;
			if(!charge_turn_connect())
			{
				if (g_resume_cleaning)
				{
					if (battery.get_voltage() < LOW_BATTERY_STOP_VOLTAGE)
					{
						ROS_INFO("%s %d: Exit charger mode_ and but battery too low to continue cleaning.", __FUNCTION__, __LINE__);
						cm_set(Clean_Mode_Idle);
						g_resume_cleaning = false;
					}
					else
					{
						ROS_INFO("%s %d: Exit charger mode_ and continue cleaning.", __FUNCTION__, __LINE__);
						cm_set(Clean_Mode_Navigation);
					}
					break;
				}

				ROS_INFO("%s %d: Exit charger mode_ and go to userinterface mode_.", __FUNCTION__, __LINE__);
				cm_set(Clean_Mode_Idle);
				break;
			}
			else
			{
				g_stop_charge_counter = 20;
			}
		}
		if(ev.cliff_all_triggered)
		{
			disable_motors();
			ev.cliff_all_triggered = 0;
			cm_set(Clean_Mode_Idle);
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

		if (battery.is_full() && !battery_full)
		{
			battery_full = true;
			led_set_mode(LED_STEADY, LED_OFF);
			wav_play(WAV_BATTERY_CHARGE_DONE);
		}
	}
	charge_unregister_event();
	charger.set_stop();
	// Wait for 20ms to make sure stop charging command has been sent.
	usleep(30000);

	if (cm_is_navigation())
	{
		// Wait for updated cliff status.
		usleep(30000);
		// Cliff triggered means switch is off, aborting switching to navigation mode_.
		if (cliff.get_status())
		{
			ROS_WARN("%s %d: Switch is not on.", __FUNCTION__, __LINE__);
			cm_set(Clean_Mode_Charging);
			wav_play(WAV_CHECK_SWITCH);
		}
	}
	if (charge_plan_status == 2)
		wav_play(WAV_CANCEL_APPOINTMENT);
	else if (charge_plan_status == 1)
		wav_play(WAV_APPOINTMENT_DONE);
	charge_plan_status = 0;
}

void charge_register_event(void)
{
	event_manager_register_handler(&eh);
	event_manager_set_enable(true);
}

void charge_unregister_event(void)
{
	event_manager_set_enable(false);
}

void Charge_EventHandle::charge_detect(bool state_now, bool state_last)
{
	ev.charge_detect = 1;
	g_stop_charge_counter = 20;
}
void Charge_EventHandle::remote_plan(bool state_now, bool state_last)
{
	if (planer.get_status())
		charge_plan_confirm_time = time(NULL);

	switch(planer.get_status())
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
			else if(cliff.get_status() & (BLOCK_LEFT|BLOCK_FRONT|BLOCK_RIGHT))
			{
				ROS_WARN("%s %d: Plan not activated not valid because of robot lifted up.", __FUNCTION__, __LINE__);
				charge_reject_reason = 2;
				charge_plan_status = 2;
				break;
			}
			else if (!battery.is_ready_to_clean())
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
	planer.set_status(0);
}
void Charge_EventHandle::cliff_all(bool state_now, bool state_last)
{
	g_cliff_all_cnt++;
	if (g_cliff_all_cnt++ > 2)
	{
		ev.fatal_quit = true;
		ev.cliff_all_triggered = true;
	}
}
void Charge_EventHandle::key_clean(bool state_now, bool state_last)
{
	if (charger.is_directed())
	{
		ROS_WARN("%s %d: Can not go to navigation mode_ during direct charging.", __FUNCTION__, __LINE__);
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
	else if(cliff.get_status() & (BLOCK_LEFT|BLOCK_FRONT|BLOCK_RIGHT))
	{
		ROS_WARN("%s %d: Robot lifted up.", __FUNCTION__, __LINE__);
		beep_for_command(INVALID);
		charge_reject_reason = 2;
	}
	else if (!battery.is_ready_to_clean())
	{
		ROS_WARN("%s %d: Battery below BATTERY_READY_TO_CLEAN_VOLTAGE(%d) + 600, can't go to navigation mode_.", __FUNCTION__, __LINE__, BATTERY_READY_TO_CLEAN_VOLTAGE);
		beep_for_command(INVALID);
		charge_reject_reason = 3;
	}
	else if (charger.is_on_stub())
	{
		ROS_WARN("%s %d: Exit charger mode_ and go to navigation mode_.", __FUNCTION__, __LINE__);
		beep_for_command(VALID);
		cm_set(Clean_Mode_Navigation);
	}

	// Key release detection, if user has not release the key, don't do anything.
	while (key.get_press() & KEY_CLEAN)
		usleep(20000);
	ROS_WARN("%s %d: Key clean is released.", __FUNCTION__, __LINE__);

	key.reset();
}
void Charge_EventHandle::remote_clean(bool stat_now, bool state_last)
{
	if (remote.key(Remote_Clean)) {
		remote.reset();
		if (charger.is_directed())
		{
			ROS_WARN("%s %d: Can not go to navigation mode_ during direct charging.", __FUNCTION__, __LINE__);
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
		else if(cliff.get_status() & (BLOCK_LEFT|BLOCK_FRONT|BLOCK_RIGHT))
		{
			ROS_WARN("%s %d: Robot lifted up.", __FUNCTION__, __LINE__);
			beep_for_command(INVALID);
			charge_reject_reason = 2;
		}
		else if (!battery.is_ready_to_clean())
		{
			ROS_WARN("%s %d: Battery below BATTERY_READY_TO_CLEAN_VOLTAGE(%d) + 600, can't go to navigation mode_.", __FUNCTION__, __LINE__, BATTERY_READY_TO_CLEAN_VOLTAGE);
			charge_reject_reason = 3;
			beep_for_command(INVALID);
		}
		else if (charger.is_on_stub())
		{
			ROS_WARN("%s %d: Exit charger mode_ and go to navigation mode_.", __FUNCTION__, __LINE__);
			beep_for_command(VALID);
			cm_set(Clean_Mode_Navigation);
		}
	}
	else{
		beep_for_command(INVALID);
		remote.reset();
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
	if(ev.charge_detect)
	{
		ROS_INFO("Reach charger without turning.");
		return true;
	}
	// Start turning right.
	set_dir_right();
	set_wheel_speed(speed, speed);
	for(int i=0; i<25; i++)
	{
		if (ev.charge_detect)
		{
			ev.charge_detect = 0;
			disable_motors();
			// Wait for a while to decide if it is really on the charger stub.
			usleep(500000);
			if (ev.charge_detect)
			{
				ROS_INFO("Turn left reach charger.");
				return true;
			}
			set_wheel_speed(speed, speed);
		}
		if(ev.key_clean_pressed || ev.fatal_quit)
			return true;
		usleep(50000);
	}
	stop_brifly();
	// Start turning left.
	set_dir_left();
	set_wheel_speed(speed, speed);
	for(int i=0; i<40; i++)
	{
		if (ev.charge_detect)
		{
			ev.charge_detect = 0;
			disable_motors();
			// Wait for a while to decide if it is really on the charger stub.
			usleep(500000);
			if (ev.charge_detect)
			{
				ROS_INFO("Turn left reach charger.");
				return true;
			}
			set_wheel_speed(speed, speed);
		}
		if(ev.key_clean_pressed || ev.fatal_quit)
			return true;
		usleep(50000);
	}
	stop_brifly();
	g_charge_turn_connect_fail = true;
	return false;

}

