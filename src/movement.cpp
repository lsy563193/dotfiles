#include <stdint.h>
#include <unistd.h>
#include <math.h>
#include <time.h>
#include <ros/ros.h>
#include <motion_manage.h>
#include <vacuum.h>
#include <brush.h>
#include <bumper.h>
#include <tilt.h>
#include <wheel.h>

#include "robot.hpp"
#include "core_move.h"
#include "wav.h"
#include "clean_mode.h"
#include "error.h"

uint8_t cs_self_check(uint8_t Check_Code)
{
	static time_t mboctime;
	static time_t vacoctime;
	static uint8_t mbrushchecking = 0;
	uint8_t Time_Out = 0;
	int32_t Wheel_Current_Summary = 0;
	uint8_t Left_Wheel_Slow = 0;
	uint8_t Right_Wheel_Slow = 0;

/*
	if(cm_is_navigation())
		cm_move_back_(COR_BACK_20MM);
	else
		quick_back(30,20);
*/
	cs_disable_motors();
	usleep(10000);
	/*------------------------------Self Check right wheel -------------------*/
	if (Check_Code == Check_Right_Wheel)
	{
		Right_Wheel_Slow = 0;
		if (wheel.get_direction_flag() == Direction_Flag_Left)
		{
			wheel.set_dir_right();
		} else
		{
			wheel.set_dir_left();
		}
		wheel.set_speed(30, 30);
		usleep(50000);
		Time_Out = 50;
		Wheel_Current_Summary = 0;
		while (Time_Out--)
		{
			Wheel_Current_Summary += (uint32_t) robot::instance()->getRwheelCurrent();
			usleep(20000);
		}
		Wheel_Current_Summary /= 50;
		if (Wheel_Current_Summary > Wheel_Stall_Limit)
		{
			cs_disable_motors();
			ROS_WARN("%s,%d right wheel stall maybe, please check!!\n", __FUNCTION__, __LINE__);
			error.set(Error_Code_RightWheel);
			error.alarm();
			return 1;

		}
		/*
		if(Right_Wheel_Slow>100)
		{
			cs_disable_motors();
			error.set(Error_Code_RightWheel);
			return 1;
		}
		*/
		wheel.stop();
		//turn_right(Turn_Speed,1800);
	}
		/*---------------------------Self Check left wheel -------------------*/
	else if (Check_Code == Check_Left_Wheel)
	{
		Left_Wheel_Slow = 0;
		if (wheel.get_direction_flag() == Direction_Flag_Right)
		{
			wheel.set_dir_left();
		} else
		{
			wheel.set_dir_right();
		}
		wheel.set_speed(30, 30);
		usleep(50000);
		Time_Out = 50;
		Wheel_Current_Summary = 0;
		while (Time_Out--)
		{
			Wheel_Current_Summary += (uint32_t) robot::instance()->getLwheelCurrent();
			usleep(20000);
		}
		Wheel_Current_Summary /= 50;
		if (Wheel_Current_Summary > Wheel_Stall_Limit)
		{
			cs_disable_motors();
			ROS_WARN("%s %d,left wheel stall maybe, please check!!", __FUNCTION__, __LINE__);
			error.set(Error_Code_LeftWheel);
			error.alarm();
			return 1;
		}
		/*
		if(Left_Wheel_Slow>100)
		{
			cs_disable_motors();
			error.set(Error_Code_RightWheel);
			return 1;
		}
		*/
		wheel.stop();
		//turn_left(Turn_Speed,1800);
	} else if (Check_Code == Check_Main_Brush)
	{
		if (!mbrushchecking)
		{
			brush.set_main_pwm(0);
			mbrushchecking = 1;
			mboctime = time(NULL);
		} else if ((uint32_t) difftime(time(NULL), mboctime) >= 3)
		{
			mbrushchecking = 0;
			error.set(Error_Code_MainBrush);
			cs_disable_motors();
			error.alarm();
			return 1;
		}
		return 0;
	} else if (Check_Code == Check_Vacuum)
	{
#ifndef BLDC_INSTALL
		ROS_INFO("%s, %d: Vacuum Over Current!!", __FUNCTION__, __LINE__);
		ROS_INFO("%d", vacuum.get_self_check_status());
		while (vacuum.get_self_check_status() != 0x10)
		{
			/*-----wait until self check begin-----*/
			vacuum.start_self_check();
		}
		ROS_INFO("%s, %d: Vacuum Self checking", __FUNCTION__, __LINE__);
		/*-----reset command for start self check-----*/
		vacuum.reset_self_check();
		/*-----wait for the end of self check-----*/
		while (vacuum.get_self_check_status() == 0x10);
		ROS_INFO("%s, %d: end of Self checking", __FUNCTION__, __LINE__);
		if (vacuum.get_self_check_status() == 0x20)
		{
			ROS_INFO("%s, %d: Vacuum error", __FUNCTION__, __LINE__);
			/*-----vacuum error-----*/
			error.set(Error_Code_Fan_H);
			cs_disable_motors();
			error.alarm();
			vacuum.reset_self_check();
			return 1;
		}
		vacuum.reset_self_check();
#else
		Disable_Motors();
		//wheel.stop();
		Set_Vac_Speed();
		usleep(100000);
		vacoctime = time(NULL);
		uint16_t tmpnoc_n = 0;
		while((uint32_t)difftime(time(NULL),vacoctime)<=3){
			if(!robot::instance()->robot_get_vacuum_oc()){
				tmpnoc_n++;
				if(tmpnoc_n>20){
					Work_Motor_Configure();
					tmpnoc_n = 0;
					return 0;
				}
			}
			usleep(50000);
		}
		error.set(Error_Code_Fan_H);
		cs_disable_motors();
		Alarm_Error();
		return 1;
#endif
	} else if (Check_Code == Check_Left_Brush)
	{
		error.set(Error_Code_LeftBrush);
		cs_disable_motors();
		error.alarm();
		return 1;
	} else if (Check_Code == Check_Right_Brush)
	{
		error.set(Error_Code_RightBrush);
		cs_disable_motors();
		error.alarm();
		return 1;
	}
	wheel.stop();
	Left_Wheel_Slow = 0;
	Right_Wheel_Slow = 0;
	cs_work_motor();
	//wheel.move_forward(5,5);
	return 0;
}

void cs_disable_motors(void)
{
	wheel.stop();
	brush.stop();
	vacuum.stop();
}

void cs_work_motor(void)
{
	if (cs_is_going_home())
	{
		// Set the vacuum to a normal mode_
		vacuum.mode(Vac_Normal, false);
	} else {
		vacuum.mode(Vac_Save);
	}

	// Turn on the main brush and side brush
	brush.set_side_pwm(50, 50);
	brush.set_main_pwm(30);
}

bool cs_is_paused()
{
	return (g_is_manual_pause || g_robot_stuck);
}

void cs_paused_setting(void)
{
	if (g_is_manual_pause || g_robot_stuck)
	{
		g_robot_stuck = false;
		// These are all the action that ~MotionManage() won't do if isManualPaused() returns true.
		ROS_WARN("Reset manual/stuck pause status.");
		wav_play(WAV_CLEANING_STOP);
		g_is_manual_pause = false;
		robot::instance()->savedOffsetAngle(0);
		if (MotionManage::s_slam != nullptr)
		{
			delete MotionManage::s_slam;
			MotionManage::s_slam = nullptr;
		}
		cm_reset_go_home();
		g_resume_cleaning = false;
	}
}

void quick_back(uint8_t speed, uint16_t distance)
{
	// The distance is for mm.
	float saved_x, saved_y;
	saved_x = robot::instance()->getOdomPositionX();
	saved_y = robot::instance()->getOdomPositionY();
	// Quickly move back for a distance.
	wheel.set_dir_backward();
	wheel.reset_step();
	wheel.set_speed(speed, speed);
	while (sqrtf(powf(saved_x - robot::instance()->getOdomPositionX(), 2) +
							 powf(saved_y - robot::instance()->getOdomPositionY(), 2)) < (float) distance / 1000)
	{
		ROS_DEBUG("%s %d: saved_x: %f, saved_y: %f current x: %f, current y: %f.", __FUNCTION__, __LINE__, saved_x, saved_y,
							robot::instance()->getOdomPositionX(), robot::instance()->getOdomPositionY());
		if (ev.fatal_quit || ev.key_clean_pressed || ev.charge_detect || ev.cliff_all_triggered)
			break;
		usleep(20000);
	}
	ROS_INFO("quick_back finished.");
}

bool check_pub_scan()
{
	//ROS_INFO("%s %d: get_left_wheel.speed() = %d, get_right_wheel.speed() = %d.", __FUNCTION__, __LINE__, wheel.get_left_speed(), wheel.get_right_speed());
	if (g_motion_init_succeeded &&
		((fabs(robot::instance()->getLeftWheelSpeed() - robot::instance()->getRightWheelSpeed()) > 0.1)
		|| (robot::instance()->getLeftWheelSpeed() * robot::instance()->getRightWheelSpeed() < 0)
		|| bumper.get_status() || tilt.get_status()
		|| abs(wheel.get_left_speed() - wheel.get_right_speed()) > 100
		|| wheel.get_left_speed() * wheel.get_right_speed() < 0))
		return false;
	else
		return true;
}

