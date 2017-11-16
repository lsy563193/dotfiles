
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
#include "clean_mode.h"

extern volatile uint32_t Left_Wheel_Step,Right_Wheel_Step;

RemoteModeMoveType move_flag = REMOTE_MODE_STAY;
boost::mutex move_flag_mutex;
int16_t remote_target_angle;
bool g_remote_exit;
time_t remote_cmd_time;
uint8_t remote_rcon_cnt = 0;
bool remote_rcon_triggered = false;
static RM_EventHandle eh;

void remote_mode(void)
{
	ROS_INFO("\n-------Remote mode------\n");
	set_main_pwr_byte(Clean_Mode_Remote);
	g_is_low_bat_pause = false;
	reset_clean_paused();


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
			cm_set(Clean_Mode_Userinterface);
			return;
		}
	}
	g_remote_exit = false;
	g_battery_low_cnt = 0;
	remote_rcon_triggered = false;

	set_led_mode(LED_STEADY, LED_GREEN);
	reset_rcon_remote();
	robot::instance()->setBaselinkFrameType(Odom_Position_Odom_Angle);

	event_manager_reset_status();
	remote_mode_register_events();

#ifdef OBS_DYNAMIC_MOVETOTARGET
	/* Dyanmic adjust obs trigger val . */
	robot::instance()->obsAdjustCount(20);
#endif

	while (ros::ok())
	{
		if (ev.fatal_quit)
		{
			cm_set(Clean_Mode_Userinterface);
			break;
		}

		if (ev.key_clean_pressed || g_remote_exit)
			break;

		remote_move();

		if (cm_should_self_check())
			cm_self_check();

	}
	disable_motors();
	remote_mode_unregister_events();

	if (ev.battery_low)
		wav_play(WAV_BATTERY_LOW);

	if (ev.cliff_all_triggered)
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

		if (ev.fatal_quit || cm_should_self_check() || cm_get() != Clean_Mode_Remote)
			break;

		if (time(NULL) - remote_cmd_time >= remote_timeout)
		{
			cm_set(Clean_Mode_Userinterface);
			g_remote_exit = true;
			break;
		}

		switch (get_move_flag_())
		{
			case REMOTE_MODE_FORWARD:
			{
				if (get_obs_status())
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

				float distance = sqrtf(powf(saved_pos_x - robot::instance()->getOdomPositionX(), 2) + powf(saved_pos_y - robot::instance()->getOdomPositionY(), 2));
				ROS_DEBUG("%s %d: current pos(%f, %f), distance:%f.", __FUNCTION__, __LINE__, robot::instance()->getOdomPositionX(), robot::instance()->getOdomPositionY(), distance);
				if (distance < (remote_rcon_triggered ? 0.06f : 0.02f))
					break;

				if (ev.bumper_triggered)
				{
					// Check if still bumper triggered.
					if(!get_bumper_status())
					{
						ROS_INFO("%s %d: Move back for bumper finished.", __FUNCTION__, __LINE__);
						g_move_back_finished = true;
						ev.bumper_triggered = false;
						g_bumper_cnt = 0;
					}
					else if (++g_bumper_cnt >= 2)
					{
						// Should switch to cm_self_check() function to resume.
						ev.bumper_jam = true;
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
				else if (ev.cliff_triggered)
				{
					if (!get_cliff_status())
					{
						ROS_INFO("%s %d: Move back for cliff finished.", __FUNCTION__, __LINE__);
						g_move_back_finished = true;
						ev.cliff_triggered = 0;
						g_cliff_cnt = 0;
						g_cliff_all_cnt = 0;
					}
					else if (++g_cliff_cnt >= 2)
					{
						// Should switch to cm_self_check() function to resume.
						ev.cliff_jam = true;
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
				else if (remote_rcon_triggered)
					remote_rcon_triggered = false;

				ROS_DEBUG("%s %d: Move back finished.", __FUNCTION__, __LINE__);
				set_move_flag_(REMOTE_MODE_STAY);
				break;
			}
			case REMOTE_MODE_STAY:
			{
				if (ev.bumper_triggered || ev.cliff_triggered || remote_rcon_triggered)
				{
					if (robot::instance()->getLinearX() <= 0 && robot::instance()->getLinearY() <= 0)
					{
						saved_pos_x = robot::instance()->getOdomPositionX();
						saved_pos_y = robot::instance()->getOdomPositionY();
						ROS_WARN("%s %d: Move back. Mark current pos(%f, %f).", __FUNCTION__, __LINE__, saved_pos_x, saved_pos_y);
						set_move_flag_(REMOTE_MODE_BACKWARD);
					}
					set_dir_backward();
					set_wheel_speed(20, 20);
				}
				else
				{
					set_wheel_speed(0, 0);
					moving_speed = 0;
					remote_rcon_cnt = 0;
				}
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

				if (std::abs(diff) > 80){
					//moving_speed = std::min(++moving_speed, ROTATE_TOP_SPEED);
					moving_speed = ROTATE_TOP_SPEED;
				}
				else{
					--moving_speed;
					moving_speed = std::max(--moving_speed, ROTATE_LOW_SPEED);
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
	event_manager_register_handler(&eh);
	event_manager_set_enable(true);
}

void remote_mode_unregister_events(void)
{
	event_manager_set_enable(false);
}

void RM_EventHandle::bumper(bool state_now, bool state_last)
{
	if (!ev.bumper_triggered)
	{
		ROS_WARN("%s %d: Bumper triggered.", __FUNCTION__, __LINE__);
		ev.bumper_triggered = true;
		set_move_flag_(REMOTE_MODE_STAY);
	}
}

void RM_EventHandle::bumper_all(bool state_now, bool state_last)
{
	bumper(state_now, state_last);
}
void RM_EventHandle::bumper_right(bool state_now, bool state_last)
{
	bumper(state_now, state_last);
}
void RM_EventHandle::bumper_left(bool state_now, bool state_last)
{
	bumper(state_now, state_last);
}
void RM_EventHandle::cliff_all(bool state_now, bool state_last)
{
	g_cliff_all_cnt++;
	if (g_cliff_all_cnt++ > 2)
	{
		ev.cliff_all_triggered = true;
		ev.fatal_quit = true;
	}
	ev.cliff_triggered = BLOCK_ALL;
	if (g_move_back_finished && !ev.cliff_jam && !state_last)
		ROS_WARN("%s %d: is called, state now: %s\tstate last: %s", __FUNCTION__, __LINE__, state_now ? "true" : "false", state_last ? "true" : "false");
}

void RM_EventHandle::cliff(bool state_now, bool state_last)
{
	if (!ev.cliff_triggered)
	{
		ROS_WARN("%s %d: Cliff triggered.", __FUNCTION__, __LINE__);
		ev.cliff_triggered = BLOCK_ALL;
		set_move_flag_(REMOTE_MODE_STAY);
	}
}

void RM_EventHandle::cliff_right(bool state_now, bool state_last)
{
	cliff(state_now, state_last);
}

void RM_EventHandle::cliff_left(bool state_now, bool state_last)
{
	cliff(state_now, state_last);
}

void RM_EventHandle::cliff_left_right(bool state_now, bool state_last)
{
	cliff(state_now, state_last);
}

void RM_EventHandle::cliff_front(bool state_now, bool state_last)
{
	cliff(state_now, state_last);
}

void RM_EventHandle::cliff_front_left(bool state_now, bool state_last)
{
	cliff(state_now, state_last);
}

void RM_EventHandle::cliff_front_right(bool state_now, bool state_last)
{
	cliff(state_now, state_last);
}

void RM_EventHandle::obs(bool state_now, bool state_last)
{
	if (get_move_flag_() == REMOTE_MODE_FORWARD)
	{
		ROS_WARN("%s %d: OBS triggered, stop the robot.", __FUNCTION__, __LINE__);
		set_move_flag_(REMOTE_MODE_STAY);
	}
}

void RM_EventHandle::obs_front(bool state_now, bool state_last)
{
	obs(state_now, state_last);
}
void RM_EventHandle::obs_left(bool state_now, bool state_last)
{
	obs(state_now, state_last);
}
void RM_EventHandle::obs_right(bool state_now, bool state_last)
{
	obs(state_now, state_last);
}

void RM_EventHandle::remote_direction_forward(bool state_now, bool state_last)
{
	ROS_WARN("%s %d: Remote forward is pressed.", __FUNCTION__, __LINE__);
	remote_cmd_time = time(NULL);
	if (get_move_flag_() == REMOTE_MODE_BACKWARD || ev.bumper_jam || ev.cliff_jam)
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

void RM_EventHandle::remote_direction_left(bool state_now, bool state_last)
{
	ROS_WARN("%s %d: Remote left is pressed.", __FUNCTION__, __LINE__);
	remote_cmd_time = time(NULL);
	if (get_move_flag_() == REMOTE_MODE_BACKWARD || ev.bumper_jam || ev.cliff_jam)
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

void RM_EventHandle::remote_direction_right(bool state_now, bool state_last)
{
	ROS_WARN("%s %d: Remote right is pressed.", __FUNCTION__, __LINE__);
	remote_cmd_time = time(NULL);
	if (get_move_flag_() == REMOTE_MODE_BACKWARD || ev.bumper_jam || ev.cliff_jam)
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

void RM_EventHandle::remote_max(bool state_now, bool state_last)
{
	ROS_WARN("%s %d: Remote max is pressed.", __FUNCTION__, __LINE__);
	remote_cmd_time = time(NULL);
	if (!ev.bumper_jam && !ev.cliff_jam)
	{
		beep_for_command(VALID);
		switch_vac_mode(true);
	}
	else
		beep_for_command(INVALID);
	reset_rcon_remote();
}

void RM_EventHandle::remote_exit(bool state_now, bool state_last)
{
	ROS_WARN("%s %d: Remote %x is pressed.", __FUNCTION__, __LINE__, get_rcon_remote());
	remote_cmd_time = time(NULL);
	if (get_rcon_remote() == Remote_Clean)
	{
		beep_for_command(VALID);
		ev.key_clean_pressed = true;
		cm_set(Clean_Mode_Userinterface);
		disable_motors();
	}
	else if (!ev.bumper_jam && !ev.cliff_jam)
	{
		beep_for_command(VALID);
		disable_motors();
		g_remote_exit = true;
		if (get_rcon_remote() == Remote_Home)
			//cm_set(Clean_Mode_Gohome);
			cm_set(Clean_Mode_Exploration);
		else
			cm_set(Clean_Mode_Userinterface);
	}
	else
		beep_for_command(INVALID);
	reset_rcon_remote();
}
void RM_EventHandle::remote_spot(bool state_now, bool state_last)
{
 remote_exit(state_now, state_last);
}
void RM_EventHandle::remote_clean(bool state_now, bool state_last)
{
 remote_exit(state_now, state_last);
}
void RM_EventHandle::remote_home(bool state_now, bool state_last)
{
 remote_exit(state_now, state_last);
}
void RM_EventHandle::remote_wall_follow(bool state_now, bool state_last)
{
 remote_exit(state_now, state_last);
}

void RM_EventHandle::rcon(bool state_now, bool state_last)
{
	if (get_move_flag_() == REMOTE_MODE_FORWARD && !remote_rcon_triggered
		&& get_rcon_status() & (RconFL_HomeT | RconFR_HomeT | RconFL2_HomeT | RconFR2_HomeT)
		&& ++remote_rcon_cnt >= 1)
	{
		ROS_WARN("%s %d: Move back for Rcon.", __FUNCTION__, __LINE__);
		remote_rcon_triggered = true;
		set_move_flag_(REMOTE_MODE_STAY);
	}
	reset_rcon_status();
}

void RM_EventHandle::key_clean(bool state_now, bool state_last)
{
	ROS_WARN("%s %d: Key clean is pressed.", __FUNCTION__, __LINE__);
	remote_cmd_time = time(NULL);
	beep_for_command(VALID);
	disable_motors();
	while (get_key_press() & KEY_CLEAN)
		usleep(40000);
	ROS_WARN("%s %d: Key clean is released.", __FUNCTION__, __LINE__);
	cm_set(Clean_Mode_Userinterface);
	ev.key_clean_pressed = true;
	reset_touch();
}

void RM_EventHandle::charge_detect(bool state_now, bool state_last)
{
	ROS_WARN("%s %d: Detect charging.", __FUNCTION__, __LINE__);
	if (robot::instance()->getChargeStatus() == 3)
	{
		cm_set(Clean_Mode_Charging);
		disable_motors();
	}
}

void RM_EventHandle::over_current_wheel_left(bool state_now, bool state_last)
{
	ROS_DEBUG("%s %d: is called.", __FUNCTION__, __LINE__);

	if ((uint32_t) robot::instance()->getLwheelCurrent() < Wheel_Stall_Limit) {
		g_oc_wheel_left_cnt = 0;
		return;
	}

	if (g_oc_wheel_left_cnt++ > 40){
		g_oc_wheel_left_cnt = 0;
		ROS_WARN("%s %d: left wheel over current, %u mA", __FUNCTION__, __LINE__, (uint32_t) robot::instance()->getLwheelCurrent());
		ev.oc_wheel_left = true;
	}
}

void RM_EventHandle::over_current_wheel_right(bool state_now, bool state_last)
{
	ROS_DEBUG("%s %d: is called.", __FUNCTION__, __LINE__);

	if ((uint32_t) robot::instance()->getRwheelCurrent() < Wheel_Stall_Limit) {
		g_oc_wheel_right_cnt = 0;
		return;
	}

	if (g_oc_wheel_right_cnt++ > 40){
		g_oc_wheel_right_cnt = 0;
		ROS_WARN("%s %d: right wheel over current, %u mA", __FUNCTION__, __LINE__, (uint32_t) robot::instance()->getRwheelCurrent());

		ev.oc_wheel_right = true;
	}
}

void RM_EventHandle::over_current_suction(bool state_now, bool state_last)
{
	ROS_DEBUG("%s %d: is called.", __FUNCTION__, __LINE__);

	if (!robot::instance()->getVacuumOc()) {
		g_oc_suction_cnt = 0;
		return;
	}

	if (g_oc_suction_cnt++ > 40) {
		g_oc_suction_cnt = 0;
		ROS_WARN("%s %d: vacuum over current", __FUNCTION__, __LINE__);

		ev.oc_suction = true;
	}
}

void RM_EventHandle::battery_low(bool state_now, bool state_last)
{
	ROS_DEBUG("%s %d: Detects battery low.", __FUNCTION__, __LINE__);
	if (g_battery_low_cnt++ > 50)
	{
		ROS_WARN("%s %d: Battery too low: %dmV.", __FUNCTION__, __LINE__, get_battery_voltage());
		disable_motors();
		ev.battery_low = true;
		ev.fatal_quit = true;
	}
}
