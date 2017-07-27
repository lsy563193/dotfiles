
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
#include "core_move.h"

extern volatile uint32_t Left_Wheel_Step,Right_Wheel_Step;

RemoteModeMoveType move_flag = REMOTE_MODE_STAY;
boost::mutex move_flag_mutex;
int16_t remote_target_angle;
bool remote_exit;
time_t remote_cmd_time;
uint8_t remote_rcon_cnt = 0;

void remote_mode(void)
{
	if (!is_gyro_on())
	{
		// Restart the gyro.
		set_gyro_off();
		// Wait for 30ms to make sure the off command has been effectived.
		usleep(30000);
		// Set gyro on before wav_play can save the time for opening the gyro.
		set_gyro_on();
		wav_play(WAV_SYSTEM_INITIALIZING);
		if (!wait_for_gyro_on())
		{
			set_clean_mode(Clean_Mode_Userinterface);
			return;
		}
	}
	remote_exit = false;
	g_battery_low_cnt = 0;

	set_led_mode(LED_STEADY, LED_GREEN);
	reset_rcon_remote();
	robot::instance()->setBaselinkFrameType(Odom_Position_Odom_Angle);

	event_manager_reset_status();
	remote_mode_register_events();

#ifdef OBS_DYNAMIC_MOVETOTARGET
	/* Dyanmic adjust obs trigger val . */
	robotbase_obs_adjust_count(20);
#endif

	while (ros::ok())
	{
		if (g_fatal_quit_event)
		{
			set_clean_mode(Clean_Mode_Userinterface);
			break;
		}

		if (g_key_clean_pressed || remote_exit)
			break;

		remote_move();

		if (cm_should_self_check())
			cm_self_check();

	}
	disable_motors();
	remote_mode_unregister_events();

	if (g_battery_low)
		wav_play(WAV_BATTERY_LOW);

	if (g_cliff_all_triggered)
		wav_play(WAV_ERROR_LIFT_UP);
}

void remote_move(void)
{
	uint8_t moving_speed=0;
	uint8_t tick_ = 0;
	bool eh_status_now=false, eh_status_last=false;
	// Set timeout time as 10s.
	uint8_t remote_timeout = 10;

	remote_rcon_cnt = 0;
	remote_cmd_time = time(NULL);

	set_move_flag_(REMOTE_MODE_STAY);
	work_motor_configure();

	while(ros::ok())
	{
		if (event_manager_check_event(&eh_status_now, &eh_status_last) == 1) {
			continue;
		}

		if (g_fatal_quit_event || cm_should_self_check() || get_clean_mode() != Clean_Mode_Remote)
			break;

		if (time(NULL) - remote_cmd_time >= remote_timeout)
		{
			set_clean_mode(Clean_Mode_Userinterface);
			remote_exit = true;
			break;
		}

		switch (get_move_flag_())
		{
			case REMOTE_MODE_FORWARD:
			{
				if(get_obs_status())
				{
					if(moving_speed>10)moving_speed--;
					move_forward(moving_speed, moving_speed);
				}
				else
				{
					moving_speed++;
					if(moving_speed<25)moving_speed=25;
					if(moving_speed>42)moving_speed=42;
					move_forward(moving_speed, moving_speed);
				}
				break;
			}
			case REMOTE_MODE_BACKWARD:
			{
				g_move_back_finished = false;
				set_dir_backward();
				set_wheel_speed(20, 20);

				if (sqrtf(powf(saved_pos_x - robot::instance()->getOdomPositionX(), 2) + powf(saved_pos_y - robot::instance()->getOdomPositionY(), 2)) < 0.02f)
					break;

				if (g_bumper_triggered)
				{
					// Check if still bumper triggered.
					if(!get_bumper_status())
					{
						ROS_INFO("%s %d: Move back for bumper finished.", __FUNCTION__, __LINE__);
						g_move_back_finished = true;
						g_bumper_triggered = false;
						g_bumper_cnt = 0;
					}
					else if (++g_bumper_cnt >= 2)
					{
						// Should switch to cm_self_check() function to resume.
						g_bumper_jam = true;
						break;
					}
					else
					{
						// Move back for one more time.
						ROS_WARN("%s %d: Move back for one more time.", __FUNCTION__, __LINE__);
						saved_pos_x = robot::instance()->getOdomPositionX();
						saved_pos_y = robot::instance()->getOdomPositionY();
						continue;
					}
				}
				else
				{
					if (!get_cliff_status())
					{
						ROS_INFO("%s %d: Move back for cliff finished.", __FUNCTION__, __LINE__);
						g_move_back_finished = true;
						g_cliff_triggered = 0;
						g_cliff_cnt = 0;
						g_cliff_all_cnt = 0;
					}
					else if (++g_cliff_cnt >= 2)
					{
						// Should switch to cm_self_check() function to resume.
						g_cliff_jam = true;
						break;
					}
					else
					{
						// Move back for one more time.
						ROS_WARN("%s %d: Move back for one more time.", __FUNCTION__, __LINE__);
						saved_pos_x = robot::instance()->getOdomPositionX();
						saved_pos_y = robot::instance()->getOdomPositionY();
						continue;
					}
				}
				ROS_DEBUG("%s %d: Move back finished.", __FUNCTION__, __LINE__);
				set_move_flag_(REMOTE_MODE_STAY);
				break;
			}
			case REMOTE_MODE_STAY:
			{
				set_wheel_speed(0, 0);
				moving_speed = 0;
				remote_rcon_cnt = 0;
				break;
			}
			case REMOTE_MODE_LEFT:
			case REMOTE_MODE_RIGHT:
			{
				auto diff = ranged_angle(remote_target_angle - gyro_get_angle());

				if (std::abs(diff) < 10) {
					set_wheel_speed(0, 0);
					ROS_INFO("%s %d: remote_target_angle: %d\tGyro: %d\tDiff: %d", __FUNCTION__, __LINE__, remote_target_angle,
									 gyro_get_angle(), diff);
					set_move_flag_(REMOTE_MODE_STAY);
					tick_ = 0;
				}

				//tick_++;
				////ROS_WARN("%s %d: tick_: %d, diff: %d. moving speed: %d.", __FUNCTION__, __LINE__, tick_,  diff, moving_speed);
				//if (tick_ > 1)
				//{
				//	tick_ = 0;
				if (std::abs(diff) > 80){
					moving_speed = std::min(++moving_speed, ROTATE_TOP_SPEED);
					//ROS_WARN("%s %d: tick_: %d, diff: %d. moving speed: %d.", __FUNCTION__, __LINE__, tick_,  diff, moving_speed);
				}
				else{
					--moving_speed;
					moving_speed = std::max(--moving_speed, ROTATE_LOW_SPEED);
					//ROS_WARN("%s %d: tick_: %d, diff: %d. moving speed: %d.", __FUNCTION__, __LINE__, tick_,  diff, moving_speed);
				}
				//}

				if (get_move_flag_() == REMOTE_MODE_LEFT)
					set_dir_left();
				else
					set_dir_right();
				set_wheel_speed(moving_speed, moving_speed);
				break;
			}
		}
	}
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
	/* Over Current */
	event_manager_enable_handler(EVT_OVER_CURRENT_BRUSH_LEFT, true);
	//event_manager_register_and_enable_x(over_current_brush_main, EVT_OVER_CURRENT_BRUSH_MAIN, true);
	event_manager_enable_handler(EVT_OVER_CURRENT_BRUSH_RIGHT, true);
	event_manager_register_and_enable_x(over_current_wheel_left, EVT_OVER_CURRENT_WHEEL_LEFT, true);
	event_manager_register_and_enable_x(over_current_wheel_right, EVT_OVER_CURRENT_WHEEL_RIGHT, true);
	event_manager_register_and_enable_x(over_current_suction, EVT_OVER_CURRENT_SUCTION, true);
	/* Rcon */
	event_manager_register_and_enable_x(rcon, EVT_RCON, true);
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
	/* Over Current */
	event_manager_register_and_disable_x(EVT_OVER_CURRENT_BRUSH_LEFT);
	//event_manager_register_and_disable_x(EVT_OVER_CURRENT_BRUSH_MAIN);
	event_manager_register_and_disable_x(EVT_OVER_CURRENT_BRUSH_RIGHT);
	event_manager_register_and_disable_x(EVT_OVER_CURRENT_WHEEL_LEFT);
	event_manager_register_and_disable_x(EVT_OVER_CURRENT_WHEEL_RIGHT);
	event_manager_register_and_disable_x(EVT_OVER_CURRENT_SUCTION);
	/* Rcon */
	event_manager_register_and_disable_x(EVT_RCON);
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
	g_bumper_triggered = true;

	if (!state_last && g_move_back_finished)
	{
		saved_pos_x = robot::instance()->getOdomPositionX();
		saved_pos_y = robot::instance()->getOdomPositionY();
		ROS_WARN("%s %d: Bumper triggered. Mark current pos(%f, %f).", __FUNCTION__, __LINE__, saved_pos_x, saved_pos_y);
	}
	set_move_flag_(REMOTE_MODE_BACKWARD);
}

void remote_mode_handle_cliff_all(bool state_now, bool state_last)
{
	g_cliff_all_cnt++;
	if (g_cliff_all_cnt++ > 2)
	{
		g_cliff_all_triggered = true;
		g_fatal_quit_event = true;
	}
	g_cliff_triggered = Status_Cliff_All;
	if (g_move_back_finished && !g_cliff_jam && !state_last)
		ROS_WARN("%s %d: is called, state now: %s\tstate last: %s", __FUNCTION__, __LINE__, state_now ? "true" : "false", state_last ? "true" : "false");
}

void remote_mode_handle_cliff(bool state_now, bool state_last)
{
	g_cliff_triggered = Status_Cliff_All;

	if (!state_last && g_move_back_finished)
	{
		saved_pos_x = robot::instance()->getOdomPositionX();
		saved_pos_y = robot::instance()->getOdomPositionY();
		ROS_WARN("%s %d: Cliff triggered. Mark current pos(%f, %f).", __FUNCTION__, __LINE__, saved_pos_x, saved_pos_y);
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
	remote_cmd_time = time(NULL);
	if (get_move_flag_() == REMOTE_MODE_BACKWARD || g_bumper_jam || g_cliff_jam)
		beep_for_command(INVALID);
	else if (get_move_flag_() == REMOTE_MODE_STAY)
	{
		set_move_flag_(REMOTE_MODE_FORWARD);
		beep_for_command(VALID);
	}
	else
	{
		set_move_flag_(REMOTE_MODE_STAY);
		beep_for_command(VALID);
	}
	reset_rcon_remote();
}

void remote_mode_handle_remote_direction_left(bool state_now, bool state_last)
{
	ROS_WARN("%s %d: Remote left is pressed.", __FUNCTION__, __LINE__);
	remote_cmd_time = time(NULL);
	if (get_move_flag_() == REMOTE_MODE_BACKWARD || g_bumper_jam || g_cliff_jam)
		beep_for_command(INVALID);
	else if (get_move_flag_() == REMOTE_MODE_STAY)
	{
		beep_for_command(VALID);
		remote_target_angle = gyro_get_angle() + 300;
		if (remote_target_angle >= 3600)
			remote_target_angle -= 3600;
		ROS_INFO("%s %d: angle: 300(%d)\tcurrent: %d", __FUNCTION__, __LINE__, remote_target_angle, gyro_get_angle());
		set_move_flag_(REMOTE_MODE_LEFT);
	}
	else
	{
		beep_for_command(VALID);
		set_move_flag_(REMOTE_MODE_STAY);
	}
	reset_rcon_remote();
}

void remote_mode_handle_remote_direction_right(bool state_now, bool state_last)
{
	ROS_WARN("%s %d: Remote right is pressed.", __FUNCTION__, __LINE__);
	remote_cmd_time = time(NULL);
	if (get_move_flag_() == REMOTE_MODE_BACKWARD || g_bumper_jam || g_cliff_jam)
		beep_for_command(INVALID);
	else if (get_move_flag_() == REMOTE_MODE_STAY)
	{
		beep_for_command(VALID);
		remote_target_angle = ranged_angle(gyro_get_angle() - 300);
		ROS_INFO("%s %d: angle: 300(%d)\tcurrent: %d", __FUNCTION__, __LINE__, remote_target_angle, gyro_get_angle());
		set_move_flag_(REMOTE_MODE_RIGHT);
	}
	else
	{
		beep_for_command(VALID);
		set_move_flag_(REMOTE_MODE_STAY);
	}
	reset_rcon_remote();
}

void remote_mode_handle_remote_max(bool state_now, bool state_last)
{
	ROS_WARN("%s %d: Remote max is pressed.", __FUNCTION__, __LINE__);
	remote_cmd_time = time(NULL);
	if (!g_bumper_jam && !g_cliff_jam)
	{
		beep_for_command(VALID);
		switch_vac_mode(true);
	}
	else
		beep_for_command(INVALID);
	reset_rcon_remote();
}

void remote_mode_handle_remote_exit(bool state_now, bool state_last)
{
	ROS_WARN("%s %d: Remote %x is pressed.", __FUNCTION__, __LINE__, get_rcon_remote());
	remote_cmd_time = time(NULL);
	if (get_rcon_remote() == Remote_Clean)
	{
		beep_for_command(VALID);
		g_key_clean_pressed = true;
		set_clean_mode(Clean_Mode_Userinterface);
		disable_motors();
	}
	else if (!g_bumper_jam && !g_cliff_jam)
	{
		beep_for_command(VALID);
		disable_motors();
		remote_exit = true;
		if (get_rcon_remote() == Remote_Home)
			set_clean_mode(Clean_Mode_GoHome);
		else
			set_clean_mode(Clean_Mode_Userinterface);
	}
	else
		beep_for_command(INVALID);
	reset_rcon_remote();
}

void remote_mode_handle_rcon(bool state_now, bool state_last)
{
	if (get_move_flag_() == REMOTE_MODE_FORWARD && (get_rcon_status() & (RconFL_HomeT | RconFR_HomeT | RconFL2_HomeT | RconFR2_HomeT)))
	{
		ROS_DEBUG("%s %d: Rcon HomeT signal received.", __FUNCTION__, __LINE__);
		if (remote_rcon_cnt++ >= 2)
		{
			ROS_WARN("%s %d: Stop for rcon signal.", __FUNCTION__, __LINE__);
			set_move_flag_(REMOTE_MODE_STAY);
		}
	}
	reset_rcon_status();
}

void remote_mode_handle_key_clean(bool state_now, bool state_last)
{
	ROS_WARN("%s %d: Key clean is pressed.", __FUNCTION__, __LINE__);
	remote_cmd_time = time(NULL);
	beep_for_command(VALID);
	disable_motors();
	while (get_key_press() & KEY_CLEAN)
		usleep(40000);
	ROS_WARN("%s %d: Key clean is released.", __FUNCTION__, __LINE__);
	set_clean_mode(Clean_Mode_Userinterface);
	g_key_clean_pressed = true;
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

void remote_mode_handle_over_current_wheel_left(bool state_now, bool state_last)
{
	ROS_DEBUG("%s %d: is called.", __FUNCTION__, __LINE__);

	if ((uint32_t) robot::instance()->getLwheelCurrent() < Wheel_Stall_Limit) {
		g_oc_wheel_left_cnt = 0;
		return;
	}

	if (g_oc_wheel_left_cnt++ > 40){
		g_oc_wheel_left_cnt = 0;
		ROS_WARN("%s %d: left wheel over current, %u mA", __FUNCTION__, __LINE__, (uint32_t) robot::instance()->getLwheelCurrent());
		g_oc_wheel_left = true;
	}
}

void remote_mode_handle_over_current_wheel_right(bool state_now, bool state_last)
{
	ROS_DEBUG("%s %d: is called.", __FUNCTION__, __LINE__);

	if ((uint32_t) robot::instance()->getRwheelCurrent() < Wheel_Stall_Limit) {
		g_oc_wheel_right_cnt = 0;
		return;
	}

	if (g_oc_wheel_right_cnt++ > 40){
		g_oc_wheel_right_cnt = 0;
		ROS_WARN("%s %d: right wheel over current, %u mA", __FUNCTION__, __LINE__, (uint32_t) robot::instance()->getRwheelCurrent());

		g_oc_wheel_right = true;
	}
}

void remote_mode_handle_over_current_suction(bool state_now, bool state_last)
{
	ROS_DEBUG("%s %d: is called.", __FUNCTION__, __LINE__);

	if (!robot::instance()->getVacuumOc()) {
		g_oc_suction_cnt = 0;
		return;
	}

	if (g_oc_suction_cnt++ > 40) {
		g_oc_suction_cnt = 0;
		ROS_WARN("%s %d: vacuum over current", __FUNCTION__, __LINE__);

		g_oc_suction = true;
	}
}

void remote_mode_handle_battery_low(bool state_now, bool state_last)
{
	ROS_DEBUG("%s %d: Detects battery low.", __FUNCTION__, __LINE__);
	if (g_battery_low_cnt++ > 50)
	{
		ROS_WARN("%s %d: Battery too low: %dmV.", __FUNCTION__, __LINE__, get_battery_voltage());
		disable_motors();
		g_battery_low = true;
		g_fatal_quit_event = true;
	}
}
