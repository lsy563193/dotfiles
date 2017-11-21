#include <limits.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <controller.h>

#include "path_planning.h"
#include "gyro.h"
#include "movement.h"
#include "wav.h"
#include "mathematics.h"
#include "robot.hpp"
#include "robotbase.h"
#include "event_manager.h"
#include "charger.h"
#include "error.h"

int16_t gyro_angle;   //[3:3]   = Angle
int16_t gyro_xacc;        //[7:8]   = X Acceleration
int16_t gyro_yacc;        //[9:10]  = Y Acceleration
int16_t gyro_zacc;        //[11:12] = Z Acceleration

uint8_t gyro_calibration = 255;

static uint8_t Gyro_Status = 0;

int16_t gyro_get_x_acc(void) {
	return gyro_xacc;
}

int16_t gyro_get_y_acc(void) {
	return gyro_yacc;
}

int16_t gyro_get_z_acc(void) {
	return gyro_zacc;
}

uint8_t gyro_get_calibration(void) {
	return gyro_calibration;
}

int16_t gyro_get_angle(void)
{
	return gyro_angle;
}

void gyro_set_angle(int16_t angle)
{
	gyro_angle = angle;
}

void set_gyro_status(void)
{
	Gyro_Status = 1;
}

void reset_gyro_status(void)
{
	Gyro_Status = 0;
}

uint8_t is_gyro_on(void)
{
	return Gyro_Status;
}

void set_gyro_on(void)
{
	if (is_gyro_on()){
		ROS_INFO("gyro on already");
	}
	else
	{
		//ROS_INFO("Set gyro on");
		controller.set(CTL_GYRO, 0x02);
		ROS_DEBUG("Set gyro on");
	}
}

bool wait_for_gyro_on(void)
{
	// Count for cliff triggered during opening gyro.
	uint8_t error_count = 0;
	// Count for detecting angle_v_ jump, it means that gyro has been successly turned on.
	int success_count = 0;
	// Count for 20ms that should skip checking to avoid robot still moving before re-open gyro again..
	uint8_t skip_count = 0;
	bool open_gyro_success = false;
	bool	eh_status_now=false, eh_status_last=false;

	ROS_INFO("waiting for gyro start");
	while (error_count < 10)
	{
		if (event_manager_check_event(&eh_status_now, &eh_status_last) == 1)
			continue;

		usleep(10000);

		// This count is for how many count of looping should it skip after robot lifted up and put down during gyro opening.
		if (skip_count != 0)
		{
			skip_count--;
			if (skip_count == 0)
			{
				ROS_WARN("re-open gyro");
			}
		}
		else
		{
			set_gyro_on();
		}

		if (ev.key_clean_pressed || ev.cliff_all_triggered || ev.fatal_quit || charger.is_directed())
			break;

		if (skip_count == 0 && robot::instance()->getAngleV() != 0){
			success_count++;
		}
		ROS_DEBUG("Opening%d, angle_v_ = %f.angle = %f.", success_count, robot::instance()->getAngleV(),
							robot::instance()->getAngle());
		if (success_count == 5)
		{
			if (check_gyro_stable())
			{
				open_gyro_success = true;
				break;
			}
			else
			{
				skip_count = 25;
				error_count++;
				success_count = 0;
			}
		}
		//ROS_WARN("gyro start ready(%d),angle_v_(%f)", count, robot::instance()->getAngleV());
	}
	if(open_gyro_success)
	{
		ROS_INFO("gyro start ok");
		set_gyro_status();
		return true;
	}
	ROS_INFO("gyro start fail");
	set_gyro_off();
	return false;
}

bool check_gyro_stable()
{
	// Average angle value for checking whether gyro is stable.
	float average_angle = 0;
	// Current angle is the current robot angle value from gyro.
	float current_angle = 0;
	// Count for 20ms that the angle is stable after turning the gyro on.
	uint8_t check_stable_count = 0;
	bool	eh_status_now=false, eh_status_last=false;

	ROS_DEBUG("Gyro open success, check stablization.");
	// Wait for 1s to see if the angle change too much.
	while (check_stable_count < 50)
	{
		if (event_manager_check_event(&eh_status_now, &eh_status_last) == 1)
			continue;
		usleep(10000);
		if (ev.key_clean_pressed || ev.cliff_all_triggered || ev.fatal_quit || charger.is_directed())
			break;
		current_angle = robot::instance()->getAngle();
		ROS_DEBUG("Checking%d, angle_v_ = %f.angle = %f, average_angle = %f.", check_stable_count,
							robot::instance()->getAngleV(), current_angle, average_angle);
		if (current_angle > 0.02 || current_angle < -0.02)
		{
			set_gyro_off();
			wav.play(WAV_SYSTEM_INITIALIZING);
			break;
		}
		check_stable_count++;
		average_angle = (average_angle + current_angle) / 2;
	}

	if (check_stable_count == 50)
	{
		if (average_angle > 0.002)
		{
			ROS_WARN("%s %d: Robot is moved when opening gyro, re-open gyro, average_angle = %f.", __FUNCTION__, __LINE__, average_angle);
			set_gyro_off();
			average_angle = 0;
			wav.play(WAV_SYSTEM_INITIALIZING);
		}
		else
		{
			ROS_DEBUG("%s %d: Robot succeeded opening gyro, average_angle = %f.", __FUNCTION__, __LINE__, average_angle);
			// Gyro stable now, break the waiting loop.
			return true;
		}
	}
	else
	{
		// If check_stable_count < 50 means the process is broken by events or current angle too big. Events can be handle by the main while loop.
		//ROS_WARN("Reset counting.");
		check_stable_count = 0;
	}
	return false;
}

void set_gyro_off()
{
	controller.set(CTL_GYRO, 0x00);
	if (!is_gyro_on()){
		ROS_INFO("gyro stop already");
		return;
	}
	uint8_t count = 0;
	uint8_t sum = 0;

	ROS_INFO("waiting for gyro stop");
	auto angle_v = robot::instance()->getAngleV();

	while(count <= 10)
	{
		controller.set(CTL_GYRO, 0x00);
		usleep(20000);
		count++;
		if (robot::instance()->getAngleV() != angle_v){
			count=0;
			sum++;
			ROS_DEBUG("Current angle_v_ = %f, angle_v_ = %f, sum = %d.", robot::instance()->getAngleV(), angle_v, sum);
			angle_v = robot::instance()->getAngleV();
			if (sum > 10) {
				error.set(Error_Code_Gyro);
				ROS_WARN("%s,%d, gyro off failed!",__FUNCTION__,__LINE__);
				return;
			}
		}
//		ROS_INFO("gyro stop ready(%d),angle_v_(%f)", count, robot::instance()->getAngleV());
	}
	reset_gyro_status();
	robot::instance()->offsetAngle(0);
	ROS_INFO("gyro stop ok");
}

#if GYRO_DYNAMIC_ADJUSTMENT
void set_gyro_dynamic_on(void)
{
	if (is_gyro_on())
	{
		uint8_t gyro_byte = controller.get(CTL_GYRO);
		controller.set(CTL_GYRO, gyro_byte | 0x01);
	}
	//else
	//{
	//	controller.set(CTL_GYRO, 0x01);
	//}
}

void set_gyro_dynamic_off(void)
{
	if (is_gyro_on())
	{
		uint8_t gyro_byte = controller.get(CTL_GYRO);
		controller.set(CTL_GYRO, gyro_byte | 0x00);
	}
	//else
	//{
	//	controller.set(CTL_GYRO, 0x00);
	//}
}
#endif
