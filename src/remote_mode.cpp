
 /**
  ******************************************************************************
  * @file	 AI Cleaning Robot
  * @author  ILife Team Dxsong
  * @version V1.0
  * @date	 17-Nov-2011
  * @brief	 this mode the robot follows the command of the remote ,
			   Upkey : move forward untill stop command or obstacle event
  ******************************************************************************
  * <h2><center>&copy; COPYRIGHT 2011 ILife CO.LTD</center></h2>
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/

#include "movement.h"
#include "gyro.h"
#include "remote_mode.h"
#include <ros/ros.h>
#include "wav.h"
#include "robot.hpp"
#include "robotbase.h"
#include "event_manager.h"

extern volatile uint32_t Left_Wheel_Step,Right_Wheel_Step;

RemoteModeMoveType move_flag = REMOTE_MODE_STAY;
boost::mutex move_flag_mutex;
float pos_x, pos_y;

void Remote_Mode(void)
{
	uint8_t Moving_Speed=0;
	uint8_t Dec_Counter=0;
	uint32_t OBS_Stop=0;
	bool eh_status_now=false, eh_status_last=false;
	g_battery_low_cnt = 0;
  //Display_Clean_Status(Display_Remote);

	set_led(100, 0);
	reset_wheel_step();
	reset_stop_event_status();
	work_motor_configure();
	reset_rcon_remote();
	set_move_flag_(REMOTE_MODE_STAY);
//    set_vacmode(Vac_Normal);

	event_manager_reset_status();
	remote_mode_register_events();

	while(ros::ok())
	{
		if (event_manager_check_event(&eh_status_now, &eh_status_last) == 1) {
			continue;
		}
#ifdef OBS_DYNAMIC_MOVETOTARGET
		/* Dyanmic adjust obs trigger val . */
		robotbase_obs_adjust_count(20);
#endif

		if (get_clean_mode() != Clean_Mode_Remote)
			break;

		switch (get_move_flag_())
		{
			case REMOTE_MODE_FORWARD:
			{
				if(get_obs_status())
				{
					Dec_Counter++;
					if(Moving_Speed>10)Moving_Speed--;
					move_forward(Moving_Speed, Moving_Speed);
				}
				else
				{
					Moving_Speed++;
					if(Moving_Speed<25)Moving_Speed=25;
					if(Moving_Speed>42)Moving_Speed=42;
					move_forward(Moving_Speed, Moving_Speed);
					//work_motor_configure();
					OBS_Stop=0;
				}
				break;
			}
			case REMOTE_MODE_BACKWARD:
			{
				set_dir_backward();
				set_wheel_speed(20, 20);
				if (sqrtf(powf(pos_x - robot::instance()->getOdomPositionX(), 2) + powf(pos_y - robot::instance()->getOdomPositionY(), 2)) > 0.01f)
				{
					ROS_DEBUG("%s %d: Move back finished.", __FUNCTION__, __LINE__);
					set_move_flag_(REMOTE_MODE_STAY);
				}
				break;
			}
			case REMOTE_MODE_STAY:
			{
				set_wheel_speed(0, 0);
				Moving_Speed = 0;
				break;
			}
			case REMOTE_MODE_LEFT:
			{
				turn_left(Turn_Speed, 300);
				set_move_flag_(REMOTE_MODE_STAY);
				break;
			}
			case REMOTE_MODE_RIGHT:
			{
				turn_right(Turn_Speed, 300);
				set_move_flag_(REMOTE_MODE_STAY);
				break;
			}
		}

		/*------------------------------------------------check motor over current event ---------*/
		uint8_t octype =0;
		octype = check_motor_current();
		if(octype){
			if(self_check(octype)){
				set_clean_mode(Clean_Mode_Userinterface);
				break;
			}
		}
	}
	disable_motors();
	remote_mode_unregister_events();
}

void set_move_flag_(RemoteModeMoveType flag)
{
	move_flag_mutex.lock();
	move_flag = flag;
	move_flag_mutex.unlock();
}

RemoteModeMoveType get_move_flag_(void)
{
	RemoteModeMoveType flag;
	move_flag_mutex.lock();
	flag = move_flag;
	move_flag_mutex.unlock();
	return flag;
}

void remote_mode_register_events(void)
{
	ROS_WARN("%s %d: Register events", __FUNCTION__, __LINE__);
	event_manager_set_current_mode(EVT_MODE_REMOTE);

#define event_manager_register_and_enable_x(name, y, enabled) \
	event_manager_register_handler(y, &remote_mode_handle_ ##name); \
	event_manager_enable_handler(y, enabled);

	/* Bumper */
	event_manager_register_and_enable_x(bumper, EVT_BUMPER_ALL, true);
	event_manager_register_and_enable_x(bumper, EVT_BUMPER_LEFT, true);
	event_manager_register_and_enable_x(bumper, EVT_BUMPER_RIGHT, true);
	/* Cliff */
	event_manager_register_and_enable_x(cliff_all, EVT_CLIFF_ALL, true);
	event_manager_register_and_enable_x(cliff, EVT_CLIFF_FRONT_LEFT, true);
	event_manager_register_and_enable_x(cliff, EVT_CLIFF_FRONT_RIGHT, true);
	event_manager_register_and_enable_x(cliff, EVT_CLIFF_LEFT_RIGHT, true);
	event_manager_register_and_enable_x(cliff, EVT_CLIFF_FRONT, true);
	event_manager_register_and_enable_x(cliff, EVT_CLIFF_LEFT, true);
	event_manager_register_and_enable_x(cliff, EVT_CLIFF_RIGHT, true);
	/* OBS */
	event_manager_register_and_enable_x(obs, EVT_OBS_FRONT, true);
	event_manager_register_and_enable_x(obs, EVT_OBS_LEFT, true);
	event_manager_register_and_enable_x(obs, EVT_OBS_RIGHT, true);
	///* Rcon */
	//event_manager_register_and_enable_x(rcon, EVT_RCON, true);
	/* Battery */
	event_manager_register_and_enable_x(battery_low, EVT_BATTERY_LOW, true);
	/* Key */
	event_manager_register_and_enable_x(key_clean, EVT_KEY_CLEAN, true);
	/* Remote */
	event_manager_register_and_enable_x(remote_direction_forward, EVT_REMOTE_DIRECTION_FORWARD, true);
	event_manager_register_and_enable_x(remote_direction_left, EVT_REMOTE_DIRECTION_LEFT, true);
	event_manager_register_and_enable_x(remote_direction_right, EVT_REMOTE_DIRECTION_RIGHT, true);
	event_manager_register_and_enable_x(remote_max, EVT_REMOTE_MAX, true);
	event_manager_register_and_enable_x(remote_exit, EVT_REMOTE_CLEAN, true);
	event_manager_register_and_enable_x(remote_exit, EVT_REMOTE_SPOT, true);
	event_manager_register_and_enable_x(remote_exit, EVT_REMOTE_WALL_FOLLOW, true);
	event_manager_register_and_enable_x(remote_exit, EVT_REMOTE_HOME, true);
	event_manager_enable_handler(EVT_REMOTE_PLAN, true);
	/* Charge Status */
	event_manager_register_and_enable_x(charge_detect, EVT_CHARGE_DETECT, true);
}

void remote_mode_unregister_events(void)
{
	ROS_WARN("%s %d: Unregister events", __FUNCTION__, __LINE__);
#define event_manager_register_and_disable_x(x) \
	event_manager_register_handler(x, NULL); \
	event_manager_enable_handler(x, false);

	/* Bumper */
	event_manager_register_and_disable_x(EVT_BUMPER_ALL);
	event_manager_register_and_disable_x(EVT_BUMPER_LEFT);
	event_manager_register_and_disable_x(EVT_BUMPER_RIGHT);
	/* Cliff */
	event_manager_register_and_disable_x(EVT_CLIFF_ALL);
	event_manager_register_and_disable_x(EVT_CLIFF_FRONT_LEFT);
	event_manager_register_and_disable_x(EVT_CLIFF_FRONT_RIGHT);
	event_manager_register_and_disable_x(EVT_CLIFF_LEFT_RIGHT);
	event_manager_register_and_disable_x(EVT_CLIFF_FRONT);
	event_manager_register_and_disable_x(EVT_CLIFF_LEFT);
	event_manager_register_and_disable_x(EVT_CLIFF_RIGHT);
	/* OBS */
	event_manager_register_and_disable_x(EVT_OBS_FRONT);
	event_manager_register_and_disable_x(EVT_OBS_LEFT);
	event_manager_register_and_disable_x(EVT_OBS_RIGHT);
	///* Rcon */
	//event_manager_register_and_disable_x(EVT_RCON);
	/* Battery */
	event_manager_register_and_disable_x(EVT_BATTERY_LOW);
	/* Key */
	event_manager_register_and_disable_x(EVT_KEY_CLEAN);
	/* Remote */
	event_manager_register_and_disable_x(EVT_REMOTE_DIRECTION_FORWARD);
	event_manager_register_and_disable_x(EVT_REMOTE_DIRECTION_LEFT);
	event_manager_register_and_disable_x(EVT_REMOTE_DIRECTION_RIGHT);
	event_manager_register_and_disable_x(EVT_REMOTE_MAX);
	event_manager_register_and_disable_x(EVT_REMOTE_CLEAN);
	event_manager_register_and_disable_x(EVT_REMOTE_SPOT);
	event_manager_register_and_disable_x(EVT_REMOTE_WALL_FOLLOW);
	event_manager_register_and_disable_x(EVT_REMOTE_HOME);
	event_manager_register_and_disable_x(EVT_REMOTE_PLAN);
	/* Charge Status */
	event_manager_register_and_disable_x(EVT_CHARGE_DETECT);
}

void remote_mode_handle_bumper(bool state_now, bool state_last)
{
	pos_x = robot::instance()->getOdomPositionX();
	pos_y = robot::instance()->getOdomPositionY();
	if (state_last == false)
	{
		ROS_WARN("%s %d: Bumper triggered. Mark current pos(%f, %f).", __FUNCTION__, __LINE__, pos_x, pos_y);
	}
	set_move_flag_(REMOTE_MODE_BACKWARD);
}

void remote_mode_handle_cliff_all(bool state_now, bool state_last)
{
	ROS_WARN("%s %d: Robot lifted up.", __FUNCTION__, __LINE__);
	disable_motors();
	wav_play(WAV_ERROR_LIFT_UP);
	set_clean_mode(Clean_Mode_Userinterface);
}

void remote_mode_handle_cliff(bool state_now, bool state_last)
{
	pos_x = robot::instance()->getOdomPositionX();
	pos_y = robot::instance()->getOdomPositionY();
	if (state_last == false)
	{
		ROS_WARN("%s %d: Cliff triggered. Mark current pos(%f, %f).", __FUNCTION__, __LINE__, pos_x, pos_y);
	}
	set_move_flag_(REMOTE_MODE_BACKWARD);
}

void remote_mode_handle_obs(bool state_now, bool state_last)
{
	if (get_move_flag_() == REMOTE_MODE_FORWARD)
	{
		ROS_WARN("%s %d: OBS triggered, stop the robot.", __FUNCTION__, __LINE__);
		set_move_flag_(REMOTE_MODE_STAY);
	}
}

void remote_mode_handle_remote_direction_forward(bool state_now, bool state_last)
{
	ROS_WARN("%s %d: Remote forward is pressed.", __FUNCTION__, __LINE__);
	if (get_move_flag_() == REMOTE_MODE_BACKWARD)
		beep_for_command(false);
	else if (get_move_flag_() == REMOTE_MODE_STAY)
	{
		set_move_flag_(REMOTE_MODE_FORWARD);
		beep_for_command(true);
	}
	else
	{
		set_move_flag_(REMOTE_MODE_STAY);
		beep_for_command(true);
	}
	reset_rcon_remote();
}

void remote_mode_handle_remote_direction_left(bool state_now, bool state_last)
{
	ROS_WARN("%s %d: Remote left is pressed.", __FUNCTION__, __LINE__);
	if (get_move_flag_() == REMOTE_MODE_BACKWARD)
		beep_for_command(false);
	else if (get_move_flag_() == REMOTE_MODE_STAY)
	{
		beep_for_command(true);
		set_move_flag_(REMOTE_MODE_LEFT);
	}
	else
	{
		beep_for_command(true);
		set_move_flag_(REMOTE_MODE_STAY);
	}
	reset_rcon_remote();
}

void remote_mode_handle_remote_direction_right(bool state_now, bool state_last)
{
	ROS_WARN("%s %d: Remote right is pressed.", __FUNCTION__, __LINE__);
	if (get_move_flag_() == REMOTE_MODE_BACKWARD)
		beep_for_command(false);
	else if (get_move_flag_() == REMOTE_MODE_STAY)
	{
		beep_for_command(true);
		set_move_flag_(REMOTE_MODE_RIGHT);
	}
	else
	{
		beep_for_command(true);
		set_move_flag_(REMOTE_MODE_STAY);
	}
	reset_rcon_remote();
}

void remote_mode_handle_remote_max(bool state_now, bool state_last)
{
	ROS_WARN("%s %d: Remote max is pressed.", __FUNCTION__, __LINE__);
	beep_for_command(true);
	switch_vac_mode(true);
	reset_rcon_remote();
}

void remote_mode_handle_remote_exit(bool state_now, bool state_last)
{
	ROS_WARN("%s %d: Remote %x is pressed.", __FUNCTION__, __LINE__, get_rcon_remote());
	beep_for_command(true);
	disable_motors();
	if (get_rcon_remote() == Remote_Home)
		set_clean_mode(Clean_Mode_GoHome);
	else
		set_clean_mode(Clean_Mode_Userinterface);
	reset_rcon_remote();
}

void remote_mode_handle_key_clean(bool state_now, bool state_last)
{
	ROS_WARN("%s %d: Key clean is pressed.", __FUNCTION__, __LINE__);
	beep_for_command(true);
	while (get_key_press() == KEY_CLEAN)
	{
		ROS_WARN("%s %d: User hasn't release the key.", __FUNCTION__, __LINE__);
		usleep(40000);
	}
	set_clean_mode(Clean_Mode_Userinterface);
	reset_touch();
}

void remote_mode_handle_charge_detect(bool state_now, bool state_last)
{
	ROS_WARN("%s %d: Detect charging.", __FUNCTION__, __LINE__);
	if (robot::instance()->getChargeStatus() == 3)
	{
		set_clean_mode(Clean_Mode_Charging);
		disable_motors();
	}
}

void remote_mode_handle_battery_low(bool state_now, bool state_last)
{
	ROS_DEBUG("%s %d: Detects battery low.", __FUNCTION__, __LINE__);
	if (g_battery_low_cnt++ > 50)
	{
		ROS_WARN("%s %d: Battery too low: %dmV.", __FUNCTION__, __LINE__, get_battery_voltage());
		disable_motors();
		wav_play(WAV_BATTERY_LOW);
		set_clean_mode(Clean_Mode_Userinterface);
	}
}
