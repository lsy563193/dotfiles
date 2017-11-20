#include <stdint.h>
#include <unistd.h>
#include <math.h>
#include <time.h>
#include <ros/ros.h>
#include <time.h>
#include <fcntl.h>
#include <motion_manage.h>
#include <move_type.h>
#include <ctime>
#include <clean_state.h>
#include <vacuum.h>
#include <cliff.h>
#include <brush.h>
#include <bumper.h>
#include <controller.h>
#include <obs.h>
#include <accelerator.h>
#include <tilt.h>
#include <wheel.h>

#include "gyro.h"
#include "key.h"
#include "robot.hpp"
#include "movement.h"
#include "crc8.h"
#include "serial.h"
#include "robotbase.h"
#include "config.h"
#include "core_move.h"
#include "wav.h"
#include "slam.h"
#include "event_manager.h"
#include "laser.hpp"
#include "clean_mode.h"


volatile uint8_t g_error_code = 0;

/*----------------------- Set error functions--------------------------*/
void set_error_code(uint8_t code)
{
	g_error_code = code;
}

uint8_t get_error_code()
{
	return g_error_code;
}

void alarm_error(void)
{
	switch (get_error_code())
	{
		case Error_Code_LeftWheel:
		{
			wav_play(WAV_ERROR_LEFT_WHEEL);
			break;
		}
		case Error_Code_RightWheel:
		{
			wav_play(WAV_ERROR_RIGHT_WHEEL);
			break;
		}
		case Error_Code_LeftBrush:
		{
			wav_play(WAV_ERROR_LEFT_BRUSH);
			break;
		}
		case Error_Code_RightBrush:
		{
			wav_play(WAV_ERROR_RIGHT_BRUSH);
			break;
		}
		case Error_Code_MainBrush:
		{
			wav_play(WAV_ERROR_MAIN_BRUSH);
			break;
		}
		case Error_Code_Fan_H:
		{
			wav_play(WAV_ERROR_SUCTION_FAN);
			break;
		}
		case Error_Code_Cliff:
		{
			wav_play(WAV_ERROR_CLIFF);
			break;
		}
		case Error_Code_Bumper:
		{
			wav_play(WAV_ERROR_BUMPER);
			break;
		}
		case Error_Code_Omni:
		{
			wav_play(WAV_ERROR_MOBILITY_WHEEL);
			break;
		}
		case Error_Code_Laser:
		{
			wav_play(WAV_TEST_LIDAR);
			break;
		}
		case Error_Code_Stuck:
		{
			wav_play(WAV_ROBOT_STUCK);
			break;
		}
		default:
		{
			break;
		}
	}

}

bool check_error_cleared(uint8_t error_code)
{
	bool error_cleared = true;
	switch (error_code)
	{
		case Error_Code_LeftWheel:
		case Error_Code_RightWheel:
		case Error_Code_LeftBrush:
		case Error_Code_RightBrush:
		case Error_Code_MainBrush:
		case Error_Code_Fan_H:
			break;
		case Error_Code_Cliff:
		{
			if (cliff.get_status())
			{
				ROS_WARN("%s %d: Cliff still triggered.", __FUNCTION__, __LINE__);
				error_cleared = false;
			}
			break;
		}
		case Error_Code_Bumper:
		{
			if (bumper.get_status())
			{
				ROS_WARN("%s %d: Bumper still triggered.", __FUNCTION__, __LINE__);
				error_cleared = false;
			}
			break;
		}
		case Error_Code_Omni:
		{
			if(g_omni_notmove)
			{
				ROS_WARN("%s %d: Omni still triggered.", __FUNCTION__, __LINE__);
				error_cleared = false;
			}
			break;
		}
		default:
			break;
	}

	return error_cleared;
}

/*-----------------------------------------------------------Self Check-------------------*/
uint8_t self_check(uint8_t Check_Code)
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
	disable_motors();
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
			disable_motors();
			ROS_WARN("%s,%d right wheel stall maybe, please check!!\n", __FUNCTION__, __LINE__);
			set_error_code(Error_Code_RightWheel);
			alarm_error();
			return 1;

		}
		/*
		if(Right_Wheel_Slow>100)
		{
			disable_motors();
			set_error_code(Error_Code_RightWheel);
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
			disable_motors();
			ROS_WARN("%s %d,left wheel stall maybe, please check!!", __FUNCTION__, __LINE__);
			set_error_code(Error_Code_LeftWheel);
			alarm_error();
			return 1;
		}
		/*
		if(Left_Wheel_Slow>100)
		{
			disable_motors();
			set_error_code(Error_Code_RightWheel);
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
			set_error_code(Error_Code_MainBrush);
			disable_motors();
			alarm_error();
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
			set_error_code(Error_Code_Fan_H);
			disable_motors();
			alarm_error();
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
		set_error_code(Error_Code_Fan_H);
		disable_motors();
		Alarm_Error();
		return 1;
#endif
	} else if (Check_Code == Check_Left_Brush)
	{
		set_error_code(Error_Code_LeftBrush);
		disable_motors();
		alarm_error();
		return 1;
	} else if (Check_Code == Check_Right_Brush)
	{
		set_error_code(Error_Code_RightBrush);
		disable_motors();
		alarm_error();
		return 1;
	}
	wheel.stop();
	Left_Wheel_Slow = 0;
	Right_Wheel_Slow = 0;
	work_motor_configure();
	//wheel.move_forward(5,5);
	return 0;
}

//------------------------------------------------------------------------------------------------
void disable_motors(void)
{
	wheel.stop();
	brush.stop();
	vacuum.stop();
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

uint8_t is_robot_slip()
{
	uint8_t ret = 0;
	if(s_laser != nullptr && s_laser->isScan2Ready() && s_laser->isRobotSlip()){
		ROS_INFO("\033[35m""%s,%d,robot slip!!""\033[0m",__FUNCTION__,__LINE__);
		ret = 1;
	}
	return ret;
}

bool is_clean_paused()
{
	bool ret = false;
	if(g_is_manual_pause || g_robot_stuck)
	{
		ret= true;
	}
	return ret;
}

void reset_clean_paused(void)
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

void work_motor_configure(void)
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

//void cm_set(uint8_t mode_)
//{
//	g_cleaning_mode = mode_;
//}

