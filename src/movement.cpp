#include <stdint.h>
#include <unistd.h>
#include <math.h>
#include <time.h>
#include <ros/ros.h>
#include <motion_manage.h>
#include <vacuum.h>
#include <brush.h>
#include <bumper.h>
#include <wheel.hpp>
#include <odom.h>

#include "robot.hpp"
#include "core_move.h"
#include "wav.h"
#include "clean_mode.h"
#include "error.h"
#include "slam.h"
#include "gyro.h"

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
		if (wheel.getDirection() == DIRECTION_LEFT)
		{
			wheel.setDirectionRight();
		} else
		{
			wheel.setDirectionLeft();
		}
		wheel.setPidTargetSpeed(30, 30);
		usleep(50000);
		Time_Out = 50;
		Wheel_Current_Summary = 0;
		while (Time_Out--)
		{
			Wheel_Current_Summary += (uint32_t) wheel.getRightWheelCurrent();
			usleep(20000);
		}
		Wheel_Current_Summary /= 50;
		if (Wheel_Current_Summary > Wheel_Stall_Limit)
		{
			cs_disable_motors();
			ROS_WARN("%s,%d right wheel stall maybe, please check!!\n", __FUNCTION__, __LINE__);
			error.set(ERROR_CODE_RIGHTWHEEL);
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
		if (wheel.getDirection() == DIRECTION_RIGHT)
		{
			wheel.setDirectionLeft();
		} else
		{
			wheel.setDirectionRight();
		}
		wheel.setPidTargetSpeed(30, 30);
		usleep(50000);
		Time_Out = 50;
		Wheel_Current_Summary = 0;
		while (Time_Out--)
		{
			Wheel_Current_Summary += (uint32_t) wheel.getLeftWheelCurrent();
			usleep(20000);
		}
		Wheel_Current_Summary /= 50;
		if (Wheel_Current_Summary > Wheel_Stall_Limit)
		{
			cs_disable_motors();
			ROS_WARN("%s %d,left wheel stall maybe, please check!!", __FUNCTION__, __LINE__);
			error.set(ERROR_CODE_LEFTWHEEL);
			error.alarm();
			return 1;
		}
		/*
		if(Left_Wheel_Slow>100)
		{
			cs_disable_motors();
			error.set(ERROR_CODE_RIGHTWHEEL);
			return 1;
		}
		*/
		wheel.stop();
		//turn_left(Turn_Speed,1800);
	} else if (Check_Code == Check_Main_Brush)
	{
		if (!mbrushchecking)
		{
			brush.setMainPwm(0);
			mbrushchecking = 1;
			mboctime = time(NULL);
		} else if ((uint32_t) difftime(time(NULL), mboctime) >= 3)
		{
			mbrushchecking = 0;
			error.set(ERROR_CODE_MAINBRUSH);
			cs_disable_motors();
			error.alarm();
			return 1;
		}
		return 0;
	} else if (Check_Code == Check_Vacuum)
	{
#ifndef BLDC_INSTALL
		ROS_INFO("%s, %d: Vacuum Over Current!!", __FUNCTION__, __LINE__);
		ROS_INFO("%d", vacuum.getSelfCheckStatus());
		while (vacuum.getSelfCheckStatus() != 0x10)
		{
			/*-----wait until self check begin-----*/
			vacuum.startSelfCheck();
		}
		ROS_INFO("%s, %d: Vacuum Self checking", __FUNCTION__, __LINE__);
		/*-----reset command for start self check-----*/
		vacuum.resetSelfCheck();
		/*-----wait for the end of self check-----*/
		while (vacuum.getSelfCheckStatus() == 0x10);
		ROS_INFO("%s, %d: end of Self checking", __FUNCTION__, __LINE__);
		if (vacuum.getSelfCheckStatus() == 0x20)
		{
			ROS_INFO("%s, %d: Vacuum error", __FUNCTION__, __LINE__);
			/*-----vacuum error-----*/
			error.set(ERROR_CODE_FAN_H);
			cs_disable_motors();
			error.alarm();
			vacuum.resetSelfCheck();
			return 1;
		}
		vacuum.resetSelfCheck();
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
		error.set(ERROR_CODE_LEFTBRUSH);
		cs_disable_motors();
		error.alarm();
		return 1;
	} else if (Check_Code == Check_Right_Brush)
	{
		error.set(ERROR_CODE_RIGHTBRUSH);
		cs_disable_motors();
		error.alarm();
		return 1;
	}
	wheel.stop();
	Left_Wheel_Slow = 0;
	Right_Wheel_Slow = 0;
	cs_work_motor();
	//wheel.moveForward(5,5);
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
	if (cs.is_going_home() || cs.is_back_from_charger())
	{
		// Set the vacuum to a normal mode
		vacuum.setMode(Vac_Normal, false);
		// Turn on the main brush and side brush
		brush.setSidePwm(30, 30);
	} else {
		vacuum.setMode(Vac_Save);
		// Turn on the main brush and side brush
		brush.setSidePwm(50, 50);
	}

	brush.setMainPwm(30);
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
		wav.play(WAV_CLEANING_STOP);
		g_is_manual_pause = false;
		robot::instance()->savedOffsetAngle(0);
		if (slam.isMapReady())
			slam.stop();
		cm_reset_go_home();
		g_resume_cleaning = false;
	}
}

void quick_back(uint8_t speed, uint16_t distance)
{
	// The distance is for mm.
	float saved_x, saved_y;
	saved_x = odom.getX();
	saved_y = odom.getY();
	// Quickly move back for a distance.
	wheel.setDirBackward();
	wheel.resetStep();
	wheel.setPidTargetSpeed(speed, speed);
	while (sqrtf(powf(saved_x - odom.getX(), 2) + powf(saved_y - odom.getY(), 2)) < (float) distance / 1000)
	{
		ROS_DEBUG("%s %d: saved_x: %f, saved_y: %f current x: %f, current y: %f."
				,__FUNCTION__, __LINE__, saved_x, saved_y, odom.getX(), odom.getY());
		if (ev.fatal_quit || ev.key_clean_pressed || ev.charge_detect || ev.cliff_all_triggered)
			break;
		usleep(20000);
	}
	ROS_INFO("quick_back finished.");
}

bool check_pub_scan()
{
	//ROS_INFO("%s %d: get_left_wheel.speed() = %d, get_right_wheel.speed() = %d.", __FUNCTION__, __LINE__, wheel.getLeftSpeedAfterPid(), wheel.getRightSpeedAfterPid());
	if (g_motion_init_succeeded &&
		((fabs(wheel.getLeftWheelActualSpeed() - wheel.getRightWheelActualSpeed()) > 0.1)
		|| (wheel.getLeftWheelActualSpeed() * wheel.getRightWheelActualSpeed() < 0)
		|| bumper.get_status() || gyro.getTiltCheckingStatus()
		|| abs(wheel.getLeftSpeedAfterPid() - wheel.getRightSpeedAfterPid()) > 100
		|| wheel.getLeftSpeedAfterPid() * wheel.getRightSpeedAfterPid() < 0))
		return false;
	else
		return true;
}

