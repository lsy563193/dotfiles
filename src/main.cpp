#include <pthread.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <ros/ros.h>
#include "charger.hpp"
#include "core_move.h"
#include "gyro.h"
#include "movement.h"
#include "laser.hpp"
#include "robot.hpp"
#include "main.h"
#include "serial.h"
#include "crc8.h"
#include "robotbase.h"
#include "spot.h"
#include "user_interface.h"
#include "remote_mode.h"
#include "random_runing.h"
#include "sleep.h"
#include "wall_follow_multi.h"
#include "wav.h"

#if VERIFY_CPU_ID || VERIFY_KEY
#include "verify.h"
#endif

void protect_function()
{
	//Bumper protect
	if (Get_Bumper_Status()){
		ROS_INFO("Bumper protect  check");
		Turn_Left_At_Init(Max_Speed, 1800);//save itself
	}
	if (Get_Bumper_Status())
		wav_play(WAV_ERROR_BUMPER);//can't save itself, stop and give an alarm by beep
}

void *core_move_thread(void *)
{
	pthread_detach(pthread_self());
	ROS_INFO("Waiting for robot sensor ready.");
	while (!robot::instance()->robot_is_all_ready()) {
		usleep(1000);
	}
	ROS_INFO("Robot sensor ready.");
	//Set_Clean_Mode(Clean_Mode_Navigation);
	//Set_Clean_Mode(Clean_Mode_GoHome);

	// Restart the gyro.
	Set_Gyro_Off();
	// Wait for 30ms to make sure the off command has been effectived.
	usleep(30000);
	// Set gyro on before wav_play can save the time for opening the gyro.
	Set_Gyro_On();
	// Wait for 0.5s to make sure gyro should be on after the wav_play().
	usleep(500000);
	wav_play(WAV_WELCOME_ILIFE);
	while (!Wait_For_Gyro_On())
	{
		usleep(100000);
		Reset_Rcon_Remote();
		Reset_Touch();
	}

	protect_function();

//	wav_play(WAV_BATTERY_CHARGE_DONE);
	while(ros::ok()){
		usleep(20000);
		switch(Get_Clean_Mode()){
			case Clean_Mode_Userinterface:
				ROS_INFO("\n-------User_Interface mode------\n");
				set_main_pwr(Clean_Mode_Userinterface);
//				wav_play(WAV_TEST_MODE);
				User_Interface();
				break;
			case Clean_Mode_WallFollow:
				ROS_INFO("\n-------wall follow mode------\n");
				set_main_pwr(Clean_Mode_WallFollow);
#if MANUAL_PAUSE_CLEANING
				Clear_Manual_Pause();
#endif
				Wall_Follow(Map_Wall_Follow_Escape_Trapped);
				break;
			case Clean_Mode_RandomMode:
				ROS_INFO("\n-------Random_Running mode------\n");
#if MANUAL_PAUSE_CLEANING
				Clear_Manual_Pause();
#endif
				Random_Running_Mode();
				break;
			case Clean_Mode_Navigation:
				/*
					Turn_Right(10, 850);
					Turn_Right(10, 1750);
					Turn_Right(10, 1000);
					Turn_Left(10, 850);
					Turn_Left(10, 1750);
					Turn_Left(10, 1000);
				*/
				ROS_INFO("\n-------Navigation mode------\n");
				set_main_pwr(Clean_Mode_Navigation);
				CM_Touring();
				//Set_Clean_Mode(Clean_Mode_GoHome);
				break;
			case Clean_Mode_Charging:
				ROS_INFO("\n-------Charge mode------\n");
				set_main_pwr(Clean_Mode_Charging);
				Charge_Function();

				break;
			case Clean_Mode_GoHome:
				//goto_charger();
				ROS_INFO("\n-------GoHome mode------\n");
				set_main_pwr(Clean_Mode_GoHome);
#if MANUAL_PAUSE_CLEANING
				Clear_Manual_Pause();
#endif
				if (!Is_Gyro_On())
				{
					// Restart the gyro.
					Set_Gyro_Off();
					// Wait for 30ms to make sure the off command has been effectived.
					usleep(30000);
					// Set gyro on before wav_play can save the time for opening the gyro.
					Set_Gyro_On();
					wav_play(WAV_BACK_TO_CHARGER);

					if (!Wait_For_Gyro_On())
					{
						Set_Clean_Mode(Clean_Mode_Userinterface);
						break;
					}
				}
				else
				{
					wav_play(WAV_BACK_TO_CHARGER);
				}

				while (Get_Clean_Mode()==Clean_Mode_GoHome)
				{
					// If GoHome() set clean mode as Clean_Mode_GoHome, it means it still needs to go to charger stub.
#if Random_Find_Charger_Stub
					HomeStraight_Mode();
#else
					GoHome();
#endif
				}
				//Set_Clean_Mode(Clean_Mode_Charging);
				break;
			case Clean_Mode_Test:

				break;
			case Clean_Mode_Remote:
				ROS_INFO("\n-------Remote mode------\n");
				set_main_pwr(Clean_Mode_Remote);
#if MANUAL_PAUSE_CLEANING
				Clear_Manual_Pause();
#endif
				Remote_Mode();
				break;
			case Clean_Mode_Spot:
				ROS_INFO("\n-------Spot mode------\n");
				set_main_pwr(Clean_Mode_Spot);
#if MANUAL_PAUSE_CLEANING
				Clear_Manual_Pause();
#endif
				Spot_Mode(NormalSpot);
				Disable_Motors();
				usleep(200000);
//				Beep(1,25,25,2);
//				Beep(2,25,25,2);
				break;
			case Clean_Mode_Mobility:

				break;
			case Clean_Mode_Sleep:
				ROS_INFO("\n-------Sleep mode------\n");
				set_main_pwr(Clean_Mode_Sleep);
#if MANUAL_PAUSE_CLEANING
				Clear_Manual_Pause();
#endif
				Disable_Motors();
				Sleep_Mode();
				break;
			default:
				Set_Clean_Mode(Clean_Mode_Userinterface);
				break;

		}
	}
	
	return NULL;
	//pthread_exit(NULL);	
}


int main(int argc, char **argv)
{
	int			baudrate, ret1, core_move_thread_state, slam_type;
	bool		line_align_active, verify_ok = true;
	pthread_t	core_move_thread_id;
	std::string	serial_port;

	ros::init(argc, argv, "pp");
	ros::NodeHandle	nh_private("~");

	laser	laser_obj;
	robot	robot_obj;

	nh_private.param<std::string>("serial_port", serial_port, "/dev/ttyS3");
	nh_private.param<int>("baudrate", baudrate, 57600);
	nh_private.param<bool>("line_align", line_align_active, false);
	nh_private.param<int>("slam_type", slam_type, 0);

	serial_init(serial_port.c_str(), baudrate);
	robot::instance()->align_active(line_align_active);
	robot::instance()->slam_type(slam_type);


#if VERIFY_CPU_ID
	if (verify_cpu_id() < 0) {
		verify_ok = false;
	}
#endif

#if VERIFY_KEY
	if (verify_ok == true && verify_key() == 0) {
		verify_ok = false;
	}
#endif

	robotbase_reset_send_stream();
	robotbase_init();

	if (verify_ok == true) {
		ret1 = pthread_create(&core_move_thread_id, 0, core_move_thread, NULL);
		if (ret1 != 0) {
			core_move_thread_state = 0;
		} else {
			ROS_INFO("%s %d: core_move thread is up!", __FUNCTION__, __LINE__);
			core_move_thread_state = 1;
		}
		ros::spin();
	} else {
		printf("turn on led\n");
		Set_LED(100, 100);
		sleep(10);
	}

	robotbase_deinit();
	return 0;
}
