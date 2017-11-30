
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
#include "key.h"
#include "remote_mode.h"
#include <ros/ros.h>
#include <vacuum.h>
#include <cliff.h>
#include <battery.h>
#include <bumper.h>
#include <pp.h>
#include <remote.h>
#include <obs.h>
#include <beep.h>
#include <charger.h>
#include <wheel.hpp>
#include <odom.h>
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
	ROS_INFO("\n-------Remote mode_------\n");
	serial.setCleanMode(Clean_Mode_Remote);
	g_is_low_bat_pause = false;
	cs_paused_setting();


	if (!gyro.isOn())
	{
		// Restart the gyro.
		gyro.setOff();
		// Wait for 30ms to make sure the off command has been effectived.
		usleep(30000);
		// Set gyro on before wav.play can save the time for opening the gyro.
		gyro.setOn();
		wav.play(WAV_SYSTEM_INITIALIZING);
		if (!gyro.waitForOn())
		{
			cm_set(Clean_Mode_Idle);
			return;
		}
	}
	g_remote_exit = false;
	g_battery_low_cnt = 0;
	remote_rcon_triggered = false;

	led.set_mode(LED_STEADY, LED_GREEN);
	remote.reset();
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
			cm_set(Clean_Mode_Idle);
			break;
		}

		if (ev.key_clean_pressed || g_remote_exit)
			break;

		remote_move();

		if (cm_should_self_check())
			cm_self_check();

	}
	cs_disable_motors();
	remote_mode_unregister_events();

	if (ev.battery_low)
		wav.play(WAV_BATTERY_LOW);

	if (ev.cliff_all_triggered)
		wav.play(WAV_ERROR_LIFT_UP);
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
	cs_work_motor();

	while(ros::ok())
	{
		if (event_manager_check_event(&eh_status_now, &eh_status_last) == 1) {
			continue;
		}

		if (ev.fatal_quit || cm_should_self_check() || cm_get() != Clean_Mode_Remote)
			break;

		if (time(NULL) - remote_cmd_time >= remote_timeout)
		{
			cm_set(Clean_Mode_Idle);
			g_remote_exit = true;
			break;
		}

		switch (get_move_flag_())
		{
			case REMOTE_MODE_FORWARD:
			{
				if (obs.getStatus())
				{
					if(moving_speed>10)moving_speed--;
					wheel.moveForward(moving_speed, moving_speed);
				}
				else
				{
					moving_speed++;
					if(moving_speed<25)moving_speed=25;
					if(moving_speed>42)moving_speed=42;
					wheel.moveForward(moving_speed, moving_speed);
				}
				break;
			}
			case REMOTE_MODE_BACKWARD:
			{
				g_move_back_finished = false;
				wheel.setDirBackward();
				wheel.setPidTargetSpeed(20, 20);

				float distance = sqrtf(powf(saved_pos_x - odom.getX(), 2) + powf(saved_pos_y - odom.getY(), 2));
				ROS_DEBUG("%s %d: current pos(%f, %f), distance:%f.", __FUNCTION__, __LINE__, odom.getX(), odom.getY(), distance);
				if (distance < (remote_rcon_triggered ? 0.06f : 0.02f))
					break;

				if (ev.bumper_triggered)
				{
					// Check if still bumper triggered.
					if(!bumper.get_status())
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
						saved_pos_x = odom.getX();
						saved_pos_y = odom.getY();
						continue;
					}
				}
				else if (ev.cliff_triggered)
				{
					if (!cliff.get_status())
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
						saved_pos_x = odom.getX();
						saved_pos_y = odom.getY();
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
					if (odom.getMovingSpeed() <= 0)
					{
						saved_pos_x = odom.getX();
						saved_pos_y = odom.getY();
						ROS_WARN("%s %d: Move back. Mark current pos(%f, %f).", __FUNCTION__, __LINE__, saved_pos_x, saved_pos_y);
						set_move_flag_(REMOTE_MODE_BACKWARD);
					}
					wheel.setDirBackward();
					wheel.setPidTargetSpeed(20, 20);
				}
				else
				{
					wheel.stop();
					moving_speed = 0;
					remote_rcon_cnt = 0;
				}
				break;
			}
			case REMOTE_MODE_LEFT:
			case REMOTE_MODE_RIGHT:
			{
				auto diff = ranged_angle(remote_target_angle - robot::instance()->getPoseAngle());

				if (std::abs(diff) < 10) {
					wheel.stop();
					ROS_INFO("%s %d: remote_target_angle: %d\tGyro: %d\tDiff: %d", __FUNCTION__, __LINE__, remote_target_angle,
									 robot::instance()->getPoseAngle(), diff);
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
					wheel.setDirectionLeft();
				else
					wheel.setDirectionRight();
				wheel.setPidTargetSpeed(moving_speed, moving_speed);
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
		beeper.play_for_command(INVALID);
	else if (get_move_flag_() == REMOTE_MODE_STAY)
	{
		set_move_flag_(REMOTE_MODE_FORWARD);
		beeper.play_for_command(VALID);
	}
	else
	{
		set_move_flag_(REMOTE_MODE_STAY);
		beeper.play_for_command(VALID);
	}
	remote.reset();
}

void RM_EventHandle::remote_direction_left(bool state_now, bool state_last)
{
	ROS_WARN("%s %d: Remote left is pressed.", __FUNCTION__, __LINE__);
	remote_cmd_time = time(NULL);
	if (get_move_flag_() == REMOTE_MODE_BACKWARD || ev.bumper_jam || ev.cliff_jam)
		beeper.play_for_command(INVALID);
	else if (get_move_flag_() == REMOTE_MODE_STAY)
	{
		beeper.play_for_command(VALID);
		remote_target_angle = robot::instance()->getPoseAngle() + 300;
		if (remote_target_angle >= 3600)
			remote_target_angle -= 3600;
		ROS_INFO("%s %d: angle: 300(%d)\tcurrent: %d", __FUNCTION__, __LINE__, remote_target_angle, robot::instance()->getPoseAngle());
		set_move_flag_(REMOTE_MODE_LEFT);
	}
	else
	{
		beeper.play_for_command(VALID);
		set_move_flag_(REMOTE_MODE_STAY);
	}
	remote.reset();
}

void RM_EventHandle::remote_direction_right(bool state_now, bool state_last)
{
	ROS_WARN("%s %d: Remote right is pressed.", __FUNCTION__, __LINE__);
	remote_cmd_time = time(NULL);
	if (get_move_flag_() == REMOTE_MODE_BACKWARD || ev.bumper_jam || ev.cliff_jam)
		beeper.play_for_command(INVALID);
	else if (get_move_flag_() == REMOTE_MODE_STAY)
	{
		beeper.play_for_command(VALID);
		remote_target_angle = ranged_angle(robot::instance()->getPoseAngle() - 300);
		ROS_INFO("%s %d: angle: 300(%d)\tcurrent: %d", __FUNCTION__, __LINE__, remote_target_angle, robot::instance()->getPoseAngle());
		set_move_flag_(REMOTE_MODE_RIGHT);
	}
	else
	{
		beeper.play_for_command(VALID);
		set_move_flag_(REMOTE_MODE_STAY);
	}
	remote.reset();
}

void RM_EventHandle::remote_max(bool state_now, bool state_last)
{
	ROS_WARN("%s %d: Remote max is pressed.", __FUNCTION__, __LINE__);
	remote_cmd_time = time(NULL);
	if (!ev.bumper_jam && !ev.cliff_jam)
	{
		beeper.play_for_command(VALID);
		vacuum.switchToNext(true);
	}
	else
		beeper.play_for_command(INVALID);
	remote.reset();
}

void RM_EventHandle::remote_exit(bool state_now, bool state_last)
{
	ROS_WARN("%s %d: Remote %x is pressed.", __FUNCTION__, __LINE__, remote.get());
	remote_cmd_time = time(NULL);
	if (remote.get() == Remote_Clean)
	{
		beeper.play_for_command(VALID);
		ev.key_clean_pressed = true;
		cm_set(Clean_Mode_Idle);
		cs_disable_motors();
	}
	else if (!ev.bumper_jam && !ev.cliff_jam)
	{
		beeper.play_for_command(VALID);
		cs_disable_motors();
		g_remote_exit = true;
		if (remote.get() == Remote_Home)
			//cm_set(Clean_Mode_Gohome);
			cm_set(Clean_Mode_Exploration);
		else
			cm_set(Clean_Mode_Idle);
	}
	else
		beeper.play_for_command(INVALID);
	remote.reset();
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
		&& c_rcon.getStatus() & (RconFL_HomeT | RconFR_HomeT | RconFL2_HomeT | RconFR2_HomeT)
		&& ++remote_rcon_cnt >= 1)
	{
		ROS_WARN("%s %d: Move back for Rcon.", __FUNCTION__, __LINE__);
		remote_rcon_triggered = true;
		set_move_flag_(REMOTE_MODE_STAY);
	}
	c_rcon.resetStatus();
}

void RM_EventHandle::key_clean(bool state_now, bool state_last)
{
	ROS_WARN("%s %d: Key clean is pressed.", __FUNCTION__, __LINE__);
	remote_cmd_time = time(NULL);
	beeper.play_for_command(VALID);
	cs_disable_motors();
	while (key.getPressStatus())
		usleep(40000);
	ROS_WARN("%s %d: Key clean is released.", __FUNCTION__, __LINE__);
	cm_set(Clean_Mode_Idle);
	ev.key_clean_pressed = true;
	key.resetTriggerStatus();
}

void RM_EventHandle::charge_detect(bool state_now, bool state_last)
{
	ROS_WARN("%s %d: Detect charging.", __FUNCTION__, __LINE__);
	if (charger.getChargeStatus() == 3)
	{
		cm_set(Clean_Mode_Charging);
		cs_disable_motors();
	}
}

void RM_EventHandle::over_current_wheel_left(bool state_now, bool state_last)
{
	ROS_DEBUG("%s %d: is called.", __FUNCTION__, __LINE__);

	if ((uint32_t) wheel.getLeftWheelCurrent() < Wheel_Stall_Limit) {
		g_oc_wheel_left_cnt = 0;
		return;
	}

	if (g_oc_wheel_left_cnt++ > 40){
		g_oc_wheel_left_cnt = 0;
		ROS_WARN("%s %d: left wheel over current, %u mA", __FUNCTION__, __LINE__, (uint32_t) wheel.getLeftWheelCurrent());
		ev.oc_wheel_left = true;
	}
}

void RM_EventHandle::over_current_wheel_right(bool state_now, bool state_last)
{
	ROS_DEBUG("%s %d: is called.", __FUNCTION__, __LINE__);

	if ((uint32_t) wheel.getRightWheelCurrent() < Wheel_Stall_Limit) {
		g_oc_wheel_right_cnt = 0;
		return;
	}

	if (g_oc_wheel_right_cnt++ > 40){
		g_oc_wheel_right_cnt = 0;
		ROS_WARN("%s %d: right wheel over current, %u mA", __FUNCTION__, __LINE__, (uint32_t) wheel.getRightWheelCurrent());

		ev.oc_wheel_right = true;
	}
}

void RM_EventHandle::over_current_suction(bool state_now, bool state_last)
{
	ROS_DEBUG("%s %d: is called.", __FUNCTION__, __LINE__);

	if (!vacuum.getOc()) {
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
		ROS_WARN("%s %d: Battery too low: %dmV.", __FUNCTION__, __LINE__, battery.getVoltage());
		cs_disable_motors();
		ev.battery_low = true;
		ev.fatal_quit = true;
	}
}
