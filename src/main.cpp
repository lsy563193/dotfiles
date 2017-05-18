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

void laser_pm_gpio(char val);

void protect_function()
{
	//Bumper protect
	if (Get_Bumper_Status()){
		Random_Back();
		Is_Bumper_Jamed();
	}
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

	wav_play(WAV_WELCOME_ILIFE);

	protect_function();

//	wav_play(WAV_BATTERY_CHARGE_DONE);
	while(ros::ok()){
		usleep(20000);
		switch(Get_Clean_Mode()){
			case Clean_Mode_Userinterface:
				ROS_INFO("\n-------User_Interface mode------\n");
				Set_Main_PwrByte(Clean_Mode_Userinterface);
//				wav_play(WAV_TEST_MODE);
				User_Interface();
				break;
			case Clean_Mode_WallFollow:
				ROS_INFO("\n-------wall follow mode------\n");
				Set_Main_PwrByte(Clean_Mode_WallFollow);
				CM_reset_cleaning_low_bat_pause();
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
				ROS_INFO("\n-------Navigation mode------\n");
				Set_Main_PwrByte(Clean_Mode_Navigation);
				CM_Touring();
				break;
			case Clean_Mode_Charging:
				ROS_INFO("\n-------Charge mode------\n");
				Set_Main_PwrByte(Clean_Mode_Charging);
				Charge_Function();
				break;
			case Clean_Mode_GoHome:
				//goto_charger();
				ROS_INFO("\n-------GoHome mode------\n");
				Set_Main_PwrByte(Clean_Mode_GoHome);
				CM_reset_cleaning_low_bat_pause();
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
				break;
			case Clean_Mode_Test:

				break;
			case Clean_Mode_Remote:
				ROS_INFO("\n-------Remote mode------\n");
				Set_Main_PwrByte(Clean_Mode_Remote);
				CM_reset_cleaning_low_bat_pause();
#if MANUAL_PAUSE_CLEANING
				Clear_Manual_Pause();
#endif
				Remote_Mode();
				break;
			case Clean_Mode_Spot:
				ROS_INFO("\n-------Spot mode------\n");
				Set_Main_PwrByte(Clean_Mode_Spot);
				CM_reset_cleaning_low_bat_pause();
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
				//Set_Main_PwrByte(Clean_Mode_Sleep);
				CM_reset_cleaning_low_bat_pause();
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
	int			baudrate, ret1, core_move_thread_state;
	bool		line_align_active, verify_ok = true;
	pthread_t	core_move_thread_id;
	std::string	serial_port;


	laser_pm_gpio('1');
	ros::init(argc, argv, "pp");
	ros::NodeHandle	nh_private("~");

	robot	robot_obj;

	nh_private.param<std::string>("serial_port", serial_port, "/dev/ttyS3");
	nh_private.param<int>("baudrate", baudrate, 57600);

	serial_init(serial_port.c_str(), baudrate);

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
	laser_pm_gpio('0');
	return 0;
}
