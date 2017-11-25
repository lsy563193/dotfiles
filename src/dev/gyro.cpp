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

Gyro gyro;

int16_t Gyro::getXAcc(void)
{
	return x_acc_;
}

void Gyro::setXAcc(int16_t x_acc)
{
	x_acc_ = x_acc;
}

int16_t Gyro::getYAcc(void)
{
	return y_acc_;
}

void Gyro::setYAcc(int16_t y_acc)
{
	y_acc_ = y_acc;
}

int16_t Gyro::getZAcc(void)
{
	return z_acc_;
}

void Gyro::setZAcc(int16_t z_acc)
{
	z_acc_ = z_acc;
}

uint8_t Gyro::getCalibration(void)
{
	return calibration_status_;
}

float Gyro::getAngle(void)
{
	return angle_;
}

void Gyro::setAngle(float angle)
{
	angle_ = angle;
}

float Gyro::getAngleV()
{
	return angle_v_;
}

void Gyro::setAngleV(float angle_v)
{
	angle_v_ = angle_v;
}

int16_t Gyro::getInitXAcc() const
{
	return init_x_acc_;
}

int16_t Gyro::getInitYAcc() const
{
	return init_y_acc_;
}

int16_t Gyro::getInitZAcc() const
{
	return init_z_acc_;
}

void Gyro::setInitXAcc(int16_t val)
{
	init_x_acc_ = val;
}

void Gyro::setInitYAcc(int16_t val)
{
	init_y_acc_ = val;
}

void Gyro::setInitZAcc(int16_t val)
{
	init_z_acc_ = val;
}

void Gyro::setStatus(void)
{
	status_ = true;
}

void Gyro::resetStatus(void)
{
	status_ = false;
}

bool Gyro::isOn(void)
{
	return status_;
}

void Gyro::setOn(void)
{
	if (isOn()){
		ROS_INFO("gyro on already");
	}
	else
	{
		//ROS_INFO("Set gyro on");
		controller.set(CTL_GYRO, 0x02);
		ROS_DEBUG("Set gyro on");
	}
}

bool Gyro::waitForOn(void)
{
	// Count for cliff triggered during opening gyro.
	uint8_t error_count = 0;
	// Count for detecting angle_v_ jump, it means that gyro has been successly turned on.
	int success_count = 0;
	// Count for 20ms that should skip checking to avoid robot still moving before re-open gyro again..
	uint8_t skip_count = 0;
	bool open_success = false;
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
			setOn();
		}

		if (ev.key_clean_pressed || ev.cliff_all_triggered || ev.fatal_quit || charger.is_directed())
			break;

		if (skip_count == 0 && getAngleV() != 0){
			success_count++;
		}
		ROS_DEBUG("Opening%d, angle_v_ = %f.angle = %f.", success_count, getAngleV(), getAngle());
		if (success_count == 5)
		{
			if (isStable())
			{
				open_success = true;
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
	if(open_success)
	{
		ROS_INFO("gyro start ok");
		setStatus();
		return true;
	}
	ROS_INFO("gyro start fail");
	setOff();
	return false;
}

bool Gyro::isStable()
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
		current_angle = getAngle();
		ROS_DEBUG("Checking%d, angle_v_ = %f.angle = %f, average_angle = %f.", check_stable_count,
							getAngleV(), current_angle, average_angle);
		if (current_angle > 0.02 || current_angle < -0.02)
		{
			Gyro::setOff();
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
			Gyro::setOff();
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

void Gyro::setOff()
{
	controller.set(CTL_GYRO, 0x00);
	if (!Gyro::isOn()){
		ROS_INFO("gyro stop already");
		return;
	}
	uint8_t count = 0;
	uint8_t sum = 0;

	ROS_INFO("waiting for gyro stop");
	auto angle_v = getAngleV();

	while(count <= 10)
	{
		controller.set(CTL_GYRO, 0x00);
		usleep(20000);
		count++;
		if (getAngleV() != angle_v){
			count=0;
			sum++;
			ROS_DEBUG("Current angle_v_ = %f, angle_v_ = %f, sum = %d.", getAngleV(), angle_v, sum);
			angle_v = getAngleV();
			if (sum > 10) {
				error.set(Error_Code_Gyro);
				ROS_WARN("%s,%d, gyro off failed!",__FUNCTION__,__LINE__);
				return;
			}
		}
//		ROS_INFO("gyro stop ready(%d),angle_v_(%f)", count, robot::instance()->getAngleV());
	}
	resetStatus();
	robot::instance()->offsetAngle(0);
	ROS_INFO("gyro stop ok");
}

#if GYRO_DYNAMIC_ADJUSTMENT
void Gyro::setDynamicOn(void)
{
	if (Gyro::isOn())
	{
		uint8_t byte = controller.get(CTL_GYRO);
		controller.set(CTL_GYRO, byte | 0x01);
	}
	//else
	//{
	//	controller.set(CTL_GYRO, 0x01);
	//}
}

void Gyro::setDynamicOff(void)
{
	if (isOn())
	{
		uint8_t byte = controller.get(CTL_GYRO);
		controller.set(CTL_GYRO, byte | 0x00);
	}
	//else
	//{
	//	controller.set(CTL_GYRO, 0x00);
	//}
}
#endif
