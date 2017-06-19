#include <stdint.h>
#include <unistd.h>
#include <ros/ros.h>

#include "sleep.h"
#include "movement.h"
#include "wav.h"
/*----------------------------------------------------------------Sleep mode---------------------------*/
void sleep_mode(void)
{
	uint16_t sleep_time_counter_ = 0;

	reset_stop_event_status();
	reset_rcon_status();
	set_led(0, 0);

	disable_motors();
	set_main_pwr_byte(POWER_DOWN);
	ROS_INFO("%s %d,power status %u ",__FUNCTION__,__LINE__, get_main_pwr_byte());
	while(ros::ok())
	{
		usleep(20000);

		sleep_time_counter_++;
		if (sleep_time_counter_ > 1500)
		{
			// Check the battery for every 30s. If battery below 12.5v, power of core board will be cut off.
			reset_sleep_mode_flag();
			ROS_WARN("Wake up robotbase to check if battery too low(<12.5v).");
			sleep_time_counter_ = 0;
		}
		else if(!get_sleep_mode_flag())
			set_sleep_mode_flag();

		// This is necessary because once the rcon has signal, it means base stm32 board has receive the rcon signal for 3 mins.
		// If not reset here, it may cause directly go home once it sleeps.
		reset_rcon_status();

		if (get_key_press() & KEY_CLEAN)
		{
			ROS_INFO("%s,%d, get key press ",__FUNCTION__,__LINE__);
			set_clean_mode(Clean_Mode_Userinterface);
			set_main_pwr_byte(POWER_ACTIVE);
			reset_sleep_mode_flag();
			set_led(100, 0);
			beep(4, 4, 0, 1);
			usleep(100000);
			beep(3, 4, 0, 1);
			usleep(100000);
			beep(2, 4, 0, 1);
			usleep(100000);
			beep(1, 4, 4, 1);
			// Wait for user to release the key.
			while (get_key_press() & KEY_CLEAN)
			{
				ROS_WARN("User still holds the key.");
				usleep(20000);
			}
			reset_stop_event_status();
			break;
		}

		// Check if plan activated.
		if (get_plan_status() == 3)
		{
			if (get_error_code() == Error_Code_None)
			{
				set_main_pwr_byte(Clean_Mode_Navigation);
				reset_sleep_mode_flag();
				beep(4, 4, 0, 1);
				usleep(100000);
				beep(3, 4, 0, 1);
				usleep(100000);
				beep(2, 4, 0, 1);
				usleep(100000);
				beep(1, 4, 4, 1);
				reset_stop_event_status();
				set_clean_mode(Clean_Mode_Navigation);
				break;
			}
			else
			{
				ROS_INFO("%s %d: Error exists, so cancel the appointment.", __FUNCTION__, __LINE__);
				alarm_error();
				wav_play(WAV_CANCEL_APPOINTMENT);
				set_plan_status(0);
			}
		}

		if(remote_key(Remote_Clean))
		{
			set_clean_mode(Clean_Mode_Userinterface);
			set_main_pwr_byte(POWER_ACTIVE);
			reset_sleep_mode_flag();
			reset_rcon_remote();
			beep(4, 4, 0, 1);
			usleep(100000);
			beep(3, 4, 0, 1);
			usleep(100000);
			beep(2, 4, 0, 1);
			usleep(100000);
			beep(1, 4, 4, 1);
			break;
		}
		reset_rcon_remote();
		if(is_on_charger_stub() || is_direct_charge())//on base but miss charging , adjust position to charge
		{
			set_main_pwr_byte(POWER_ACTIVE);
			reset_sleep_mode_flag();
			reset_rcon_remote();
			beep(4, 4, 0, 1);
			usleep(100000);
			beep(3, 4, 0, 1);
			usleep(100000);
			beep(2, 4, 0, 1);
			usleep(100000);
			beep(1, 4, 4, 1);
			if (is_direct_charge() || turn_connect())
			{
				set_clean_mode(Clean_Mode_Charging);
				break;
			}
		}

		/*-----------------Check if near the charging base-----------------------------*/
		if(is_station() && (!get_error_code()))
		{
			ROS_INFO("%s,%d,Rcon_status = %x",__FUNCTION__,__LINE__, get_rcon_status());
			reset_rcon_status();
			set_clean_mode(Clean_Mode_GoHome);
			set_home_remote();
			set_main_pwr_byte(POWER_ACTIVE);
			reset_sleep_mode_flag();
			break;
		}
		if(is_charge_on())
		{
			set_clean_mode(Clean_Mode_Charging);
			set_main_pwr_byte(POWER_ACTIVE);
			reset_sleep_mode_flag();
			break;
		}
	}
	// Alarm for error.
	if (get_clean_mode() == Clean_Mode_Userinterface && get_error_code())
	{
		set_led(0, 100);
		alarm_error();
	}

	// Wait 1.5s to avoid gyro can't open if switch to navigation mode too soon after waking up.
	usleep(1500000);
}
