#include <limits.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>

#include "path_planning.h"
#include "gyro.h"
#include "movement.h"
#include "wav.h"
#include "mathematics.h"
#include "robot.hpp"

uint16_t gyro_angle;   //[3:3]   = Angle
int16_t gyro_xacc;        //[7:8]   = X Acceleration
int16_t gyro_yacc;        //[9:10]  = Y Acceleration
int16_t gyro_zacc;        //[11:12] = Z Acceleration

uint8_t gyro_calibration = 255;

static uint8_t Gyro_Status = 0;

int16_t Gyro_GetXAcc(void) {
	return gyro_xacc;
}

int16_t Gyro_GetYAcc(void) {
	return gyro_yacc;
}

int16_t Gyro_GetZAcc(void) {
	return gyro_zacc;
}

uint8_t Gyro_GetCalibration(void) {
	return gyro_calibration;
}

int16_t Gyro_GetAngle(void)
{
	return gyro_angle;
}

void Gyro_SetAngle(int16_t angle)
{
	if(angle <0)
		angle += 3600;

	gyro_angle = angle;
}

void Set_Gyro_Status(void)
{
	Gyro_Status = 1;
}

void Reset_Gyro_Status(void)
{
	Gyro_Status = 0;
}

uint8_t Is_Gyro_On(void)
{
	return Gyro_Status;
}

void Set_Gyro_On(void)
{
	if (Is_Gyro_On()){
		ROS_INFO("gyro on already");
	}
	else
	{
		//ROS_INFO("Set gyro on");
		control_set(CTL_GYRO, 0x02);
		ROS_DEBUG("Set gyro on");
	}
}

bool Wait_For_Gyro_On(void)
{
	bool stop_waiting = false;
	// Count for cliff triggered during opening gyro.
	uint8_t error_count = 0;
	// Count for detecting angle_v_ jump, it means that gyro has been successly turned on.
	int success_count = 0;
	// Count for 20ms that should skip checking to avoid robot still moving before re-open gyro again..
	uint8_t skip_count = 0;
	// Count for 20ms that the angle is stable after turning the gyro on.
	uint8_t check_stable_count = 0;
	// Average angle value for checking whether gyro is stable.
	float average_angle = 0;
	// Current angle is the current robot angle value from gyro.
	float current_angle = 0;
	ROS_INFO("waiting for gyro start");
	while (!stop_waiting && error_count < 10)
	{
		usleep(20000);

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
			Set_Gyro_On();
		}

		switch (Stop_Event())
		{
			case 1:
			{
				// Key release detection, if user has not release the key, don't do anything.
				while (Get_Key_Press() & KEY_CLEAN)
				{
					ROS_INFO("%s %d: User hasn't release key or still cliff detected.", __FUNCTION__, __LINE__);
					usleep(20000);
				}
				stop_waiting = true;
				success_count = 0;
				break;
			}
			case 2:
			{
				stop_waiting = true;
				success_count = 0;
				break;
			}
			case 3:
			{
				Reset_Stop_Event_Status();
				Set_Gyro_Off();
				wav_play(WAV_ERROR_LIFT_UP);
				skip_count = 25;
				error_count++;
				success_count = 0;
				break;
			}
			case 4:
			{
				stop_waiting = true;
				success_count = 0;
				break;
			}
			case 0:
			{
				// Detect for robot lifted up.
				if (Get_Cliff_Trig() & (Status_Cliff_Left | Status_Cliff_Front | Status_Cliff_Right))
				{
					Set_Gyro_Off();
					wav_play(WAV_ERROR_LIFT_UP);
					skip_count = 25;
					error_count++;
					success_count = 0;
				}
				break;
			}
		}

		if (skip_count == 0 && robot::instance()->getAngleV() != 0){
			success_count++;
		}
		ROS_DEBUG("Opening%d, angle_v_ = %f.angle = %f.", success_count, robot::instance()->getAngleV(),
							robot::instance()->getAngle());

		if (success_count == 5)
		{
			ROS_DEBUG("Gyro open success, check stablization.");
			// Wait for 1s to see if the angle change too much.
			while (!Stop_Event() && check_stable_count < 50)
			{
				usleep(20000);
				current_angle = robot::instance()->getAngle();
				ROS_DEBUG("Checking%d, angle_v_ = %f.angle = %f, average_angle = %f.", check_stable_count,
									robot::instance()->getAngleV(), current_angle, average_angle);
				if (current_angle > 0.02 || current_angle < -0.02)
				{
					Set_Gyro_Off();
					skip_count = 25;
					wav_play(WAV_SYSTEM_INITIALIZING);
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
					Set_Gyro_Off();
					skip_count = 25;
					success_count = 0;
					average_angle = 0;
					wav_play(WAV_SYSTEM_INITIALIZING);
				}
				else
				{
					ROS_DEBUG("%s %d: Robot succeeded opening gyro, average_angle = %f.", __FUNCTION__, __LINE__, average_angle);
					// Gyro stable now, break the waiting loop.
					break;
				}
			}
			else
			{
				// If check_stable_count < 50 means Stop_Event() triggered or current angle too big. Stop_Event() triggered can be handle by the main while loop.
				//ROS_WARN("Reset counting.");
				check_stable_count = 0;
				success_count = 0;
			}
		}
		//ROS_WARN("gyro start ready(%d),angle_v_(%f)", count, robot::instance()->getAngleV());
	}
	if(check_stable_count == 50)
	{
		ROS_INFO("gyro start ok");
		Set_Gyro_Status();
		return true;
	}
	ROS_INFO("gyro start fail");
	Reset_Gyro_Status();
	if (Stop_Event() != 4)
	{
		Set_Gyro_Off();
	}
	return false;
}

void Set_Gyro_Off()
{
	control_set(CTL_GYRO, 0x00);
	if (!Is_Gyro_On()){
		ROS_INFO("gyro stop already");
		return;
	}
	uint8_t count = 0;
	uint8_t sum = 0;

	ROS_INFO("waiting for gyro stop");
	auto angle_v = robot::instance()->getAngleV();

	while(count <= 10)
	{
		control_set(CTL_GYRO, 0x00);
		usleep(20000);
		count++;
		if (robot::instance()->getAngleV() != angle_v){
			count=0;
			sum++;
			angle_v = robot::instance()->getAngleV();
			ROS_DEBUG("Current angle_v_ = %f, angle_v_ = %f, sum = %d.", robot::instance()->getAngleV(), angle_v, sum);
			if (sum > 10) {
				Set_Error_Code(Error_Code_Gyro);
				ROS_WARN("%s,%d, gyro off failed!",__FUNCTION__,__LINE__);
				return;
			}
		}
//		ROS_INFO("gyro stop ready(%d),angle_v_(%f)", count, robot::instance()->getAngleV());
	}
	Reset_Gyro_Status();
	ROS_INFO("gyro stop ok");
}

#if GYRO_DYNAMIC_ADJUSTMENT
void Set_Gyro_Dynamic_On(void)
{
	if (Is_Gyro_On())
	{
		control_set(CTL_GYRO, 0x03);
	}
	else
	{
		control_set(CTL_GYRO, 0x01);
	}
}

void Set_Gyro_Dynamic_Off(void)
{
	if (Is_Gyro_On())
	{
		control_set(CTL_GYRO, 0x02);
	}
	else
	{
		control_set(CTL_GYRO, 0x00);
	}
}
#endif
