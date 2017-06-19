
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
#include "robotbase.h"
#include "event_manager.h"

extern volatile uint32_t Left_Wheel_Step,Right_Wheel_Step;

uint8_t forward_flag = 0;
// turn_flag: 0 for not turning, -1 for turning left, 1 for turning right.
int8_t turn_flag = 0;

void Remote_Mode(void)
{
	uint32_t Moving_Speed=0;
	uint8_t Dec_Counter=0;
	uint32_t OBS_Stop=0;
	bool eh_status_now=false, eh_status_last=false;
	forward_flag = 0;
	turn_flag = 0;
  //Display_Clean_Status(Display_Remote);

	set_led(100, 0);
	reset_wheel_step();
	reset_stop_event_status();
	work_motor_configure();
//    set_vacmode(Vac_Normal);

	remote_mode_register_events();

	while(ros::ok())
	{
		usleep(20000);

		if (event_manager_check_event(&eh_status_now, &eh_status_last) == 1) {
			continue;
		}
#ifdef OBS_DYNAMIC_MOVETOTARGET
		/* Dyanmic adjust obs trigger val . */
		robotbase_obs_adjust_count(20);
#endif

		if (get_clean_mode() != Clean_Mode_Remote)
			break;

		if(forward_flag)
		{
			if(get_obs_status())
			{
				Dec_Counter++;
				if(Moving_Speed>10)Moving_Speed--;
				move_forward(Moving_Speed, Moving_Speed);
				OBS_Stop++;
				if(OBS_Stop>8)forward_flag=0;
			}
			else
			{
				Moving_Speed=(get_right_wheel_step()/80)+25;
				if(Moving_Speed<25)Moving_Speed=25;
				if(Moving_Speed>42)Moving_Speed=42;
				move_forward(Moving_Speed, Moving_Speed);
				//work_motor_configure();
				OBS_Stop=0;
			}
		}
		else
		{
			stop_brifly();
			//work_motor_configure();
		}

		if(turn_flag == -1)
		{
			turn_left(Turn_Speed, 300);
			forward_flag=0;
			turn_flag = 0;
		}
		if(turn_flag == 1)
		{
			Turn_Right(Turn_Speed,300);
			forward_flag=0;
			turn_flag = 0;
		}

	  /*------------------------------------------------------stop event-----------------------*/
		if(stop_event())
		{
			// Key release detection, if user has not release the key, don't do anything.
			while (get_key_press() & KEY_CLEAN)
			{
				ROS_INFO("%s %d: User hasn't release key or still cliff detected.", __FUNCTION__, __LINE__);
				usleep(20000);
			}
			// Key relaesed, then the touch status and stop event status should be cleared.
			if (stop_event() == 3) {
				disable_motors();
				wav_play(WAV_ERROR_LIFT_UP);
			}
			reset_stop_event_status();
			set_clean_mode(Clean_Mode_Userinterface);
			break;
		}

		/*------------------------------------------------------Check Battery-----------------------*/
		if(check_bat_stop())
		{
			set_clean_mode(Clean_Mode_Userinterface);
			break;
		}
		/*-------------------------------------------Bumper  and cliff Event-----------------------*/
		if(get_cliff_trig())
		{
			move_back();
			if(get_cliff_trig()){
				move_back();
			}
			stop_brifly();
			disable_motors();
			wav_play(WAV_ERROR_LIFT_UP);
			set_clean_mode(Clean_Mode_Userinterface);
			break;
		}
		if(get_bumper_status())
		{
			random_back();
			is_bumper_jamed();
			break;
		}
		if(get_cliff_trig() == (Status_Cliff_All)){
			quick_back(20,20);
			stop_brifly();
			if(get_cliff_trig() == (Status_Cliff_All)){
				quick_back(20,20);
				stop_brifly();
			}
			if(get_cliff_trig() == Status_Cliff_All){
				quick_back(20,20);
				stop_brifly();
				disable_motors();
				ROS_INFO("Cliff trigger three times stop robot ");
				wav_play(WAV_ERROR_LIFT_UP);
				set_clean_mode(Clean_Mode_Userinterface);
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
		/* check plan set */
		if(get_plan_status() == 1)
		{
			set_plan_status(0);
			beep_for_command(false);
		}
	}
	disable_motors();
	remote_mode_unregister_events();
}

void remote_mode_register_events(void)
{
	ROS_WARN("%s %d: Register events", __FUNCTION__, __LINE__);
	event_manager_set_current_mode(EVT_MODE_REMOTE);

#define event_manager_register_and_enable_x(name, y, enabled) \
	event_manager_register_handler(y, &remote_mode_handle_ ##name); \
	event_manager_enable_handler(y, enabled);

	///* Cliff */
	//event_manager_register_and_enable_x(cliff_all, EVT_CLIFF_ALL, true);
	///* Rcon */
	//event_manager_register_and_enable_x(rcon, EVT_RCON, true);
	///* Battery */
	//event_manager_register_and_enable_x(battery_low, EVT_BATTERY_LOW, true);
	/* Remote */
	event_manager_register_and_enable_x(remote_direction_forward, EVT_REMOTE_DIRECTION_FORWARD, true);
	event_manager_register_and_enable_x(remote_direction_left, EVT_REMOTE_DIRECTION_LEFT, true);
	event_manager_register_and_enable_x(remote_direction_right, EVT_REMOTE_DIRECTION_RIGHT, true);
	event_manager_register_and_enable_x(remote_max, EVT_REMOTE_MAX, true);
	event_manager_register_and_enable_x(remote_exit, EVT_REMOTE_CLEAN, true);
	event_manager_register_and_enable_x(remote_exit, EVT_REMOTE_SPOT, true);
	event_manager_register_and_enable_x(remote_exit, EVT_REMOTE_WALL_FOLLOW, true);
	event_manager_register_and_enable_x(remote_exit, EVT_REMOTE_HOME, true);
	//event_manager_register_and_enable_x(remote_plan, EVT_REMOTE_PLAN, true);
	///* Key */
	//event_manager_register_and_enable_x(key_clean, EVT_KEY_CLEAN, true);
	///* Charge Status */
	//event_manager_register_and_enable_x(charge_detect, EVT_CHARGE_DETECT, true);
}

void remote_mode_unregister_events(void)
{
	ROS_WARN("%s %d: Unregister events", __FUNCTION__, __LINE__);
#define event_manager_register_and_disable_x(x) \
	event_manager_register_handler(x, NULL); \
	event_manager_enable_handler(x, false);

	///* Cliff */
	//event_manager_register_and_disable_x(EVT_CLIFF_ALL);
	///* Rcon */
	//event_manager_register_and_disable_x(EVT_RCON);
	///* Battery */
	//event_manager_register_and_disable_x(EVT_BATTERY_LOW);
	/* Remote */
	event_manager_register_and_disable_x(EVT_REMOTE_DIRECTION_FORWARD);
	event_manager_register_and_disable_x(EVT_REMOTE_DIRECTION_LEFT);
	event_manager_register_and_disable_x(EVT_REMOTE_DIRECTION_RIGHT);
	event_manager_register_and_disable_x(EVT_REMOTE_MAX);
	event_manager_register_and_disable_x(EVT_REMOTE_CLEAN);
	event_manager_register_and_disable_x(EVT_REMOTE_SPOT);
	event_manager_register_and_disable_x(EVT_REMOTE_WALL_FOLLOW);
	event_manager_register_and_disable_x(EVT_REMOTE_HOME);
	//event_manager_register_and_disable_x(EVT_REMOTE_PLAN);
	///* Key */
	//event_manager_register_and_disable_x(EVT_KEY_CLEAN);
	///* Charge Status */
	//event_manager_register_and_disable_x(EVT_CHARGE_DETECT);
}

void remote_mode_handle_remote_direction_forward(bool state_now, bool state_last)
{
	ROS_WARN("%s %d: Remote forward is pressed.", __FUNCTION__, __LINE__);
	beep_for_command(true);
	forward_flag=1-forward_flag;
	reset_rcon_remote();
}

void remote_mode_handle_remote_direction_left(bool state_now, bool state_last)
{
	ROS_WARN("%s %d: Remote left is pressed.", __FUNCTION__, __LINE__);
	if (turn_flag == 1)
	{
		beep_for_command(false);
		reset_rcon_remote();
		return;
	}
	beep_for_command(true);
	turn_flag = -1;
	reset_rcon_remote();
}

void remote_mode_handle_remote_direction_right(bool state_now, bool state_last)
{
	ROS_WARN("%s %d: Remote right is pressed.", __FUNCTION__, __LINE__);
	if (turn_flag == -1)
	{
		beep_for_command(false);
		reset_rcon_remote();
		return;
	}
	beep_for_command(true);
	turn_flag = 1;
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
