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
#include "wall_follow_slam.h"
#include "wall_follow_trapped.h"
#include "wav.h"
#include "event_manager.h"

#if VERIFY_CPU_ID || VERIFY_KEY
#include "verify.h"
#endif

void laser_pm_gpio(char val);

bool selfCheckAtLaunch()
{
	// Skip self check if using direct charge or at charger stub, because robot can not move during direct charge.
	if (is_direct_charge() || is_on_charger_stub())
		return true;

	//Bumper protect
	if (get_bumper_status())
	{
		set_led(0, 100);
		wav_play(WAV_ERROR_BUMPER);
		if (!Is_Gyro_On())
		{
			Set_Gyro_On();
			while (!Wait_For_Gyro_On())
				wav_play(WAV_SYSTEM_INITIALIZING);
		}
		if (is_bumper_jamed())
			return false;
		else
		{
			stop_brifly();
			wav_play(WAV_CLEAR_ERROR);
			usleep(500000);
		}
	}
	set_error_code(Error_Code_None);
	set_led(100, 0);
	return true;
}

void *core_move_thread(void *)
{
	pthread_detach(pthread_self());
	ROS_INFO("Waiting for robot sensor ready.");
	while (!robot::instance()->isSensorReady()) {
		usleep(1000);
	}
	ROS_INFO("Robot sensor ready.");
	wav_play(WAV_WELCOME_ILIFE);
	usleep(200000);
	if (selfCheckAtLaunch())
	{
		if (is_direct_charge() || is_on_charger_stub())
			set_clean_mode(Clean_Mode_Charging);
		else if (check_bat_ready_to_clean())
			wav_play(WAV_PLEASE_START_CLEANING);
	}

	while(ros::ok()){
		usleep(20000);
		switch(get_clean_mode()){
			case Clean_Mode_Userinterface:
				ROS_INFO("\n-------User_Interface mode------\n");
				set_main_pwr_byte(Clean_Mode_Userinterface);
//				wav_play(WAV_TEST_MODE);
				User_Interface();
				break;
			case Clean_Mode_WallFollow:
				ROS_INFO("\n-------wall follow mode------\n");
				set_main_pwr_byte(Clean_Mode_WallFollow);
				robot::instance()->resetLowBatPause();

				clear_manual_pause();

				wall_follow(Map_Wall_Follow_Escape_Trapped);
//				cm_touring();
				break;
			case Clean_Mode_RandomMode:
				ROS_INFO("\n-------Random_Running mode------\n");

				clear_manual_pause();

				Random_Running_Mode();
				break;
			case Clean_Mode_Navigation:
				ROS_INFO("\n-------Navigation mode------\n");
				set_main_pwr_byte(Clean_Mode_Navigation);
				cm_touring();
				break;
			case Clean_Mode_Charging:
				ROS_INFO("\n-------Charge mode------\n");
				set_main_pwr_byte(Clean_Mode_Charging);
				Charge_Function();
				break;
			case Clean_Mode_GoHome:
				//goto_charger();
				ROS_INFO("\n-------GoHome mode------\n");
				set_main_pwr_byte(Clean_Mode_GoHome);
				robot::instance()->resetLowBatPause();

				clear_manual_pause();

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
						set_clean_mode(Clean_Mode_Userinterface);
						break;
					}
				}
				else
				{
					wav_play(WAV_BACK_TO_CHARGER);
				}

				while (get_clean_mode()==Clean_Mode_GoHome)
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
				set_main_pwr_byte(Clean_Mode_Remote);
				robot::instance()->resetLowBatPause();

				clear_manual_pause();

				if (!Is_Gyro_On())
				{
					// Restart the gyro.
					Set_Gyro_Off();
					// Wait for 30ms to make sure the off command has been effectived.
					usleep(30000);
					// Set gyro on before wav_play can save the time for opening the gyro.
					Set_Gyro_On();
					wav_play(WAV_SYSTEM_INITIALIZING);
					if (!Wait_For_Gyro_On())
					{
						set_clean_mode(Clean_Mode_Userinterface);
						break;
					}
				}

				Remote_Mode();
				break;
			case Clean_Mode_Spot:
				ROS_INFO("\n-------Spot mode------\n");
				set_main_pwr_byte(Clean_Mode_Spot);
				robot::instance()->resetLowBatPause();

				clear_manual_pause();

				//Spot_Mode(NormalSpot);
				Spot_WithCell(NormalSpot,1.0);
				disable_motors();
				usleep(200000);
//				beep(1,25,25,2);
//				beep(2,25,25,2);
				break;
			case Clean_Mode_Mobility:

				break;
			case Clean_Mode_Sleep:
				ROS_INFO("\n-------Sleep mode------\n");
				//set_main_pwr_byte(Clean_Mode_Sleep);
				robot::instance()->resetLowBatPause();

				clear_manual_pause();

				disable_motors();
				sleep_mode();
				break;
			default:
				set_clean_mode(Clean_Mode_Userinterface);
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
	pthread_t	core_move_thread_id, event_manager_thread_id, event_handler_thread_id;
	std::string	serial_port;


	laser_pm_gpio('1');
	ros::init(argc, argv, "pp");
	ros::NodeHandle	nh_private("~");

	robot	robot_obj;

	event_manager_init();

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
#if 1
		ret1 = pthread_create(&event_manager_thread_id, 0, event_manager_thread, NULL);
		if (ret1 != 0) {
			ROS_ERROR("%s %d: event manager thread fails to run!", __FUNCTION__, __LINE__);
		} else {
			ROS_INFO("%s %d: event manager thread is up!", __FUNCTION__, __LINE__);
		}
#endif


#if 1
		ret1 = pthread_create(&event_handler_thread_id, 0, event_handler_thread, NULL);
		if (ret1 != 0) {
			ROS_ERROR("%s %d: event manager thread fails to run!", __FUNCTION__, __LINE__);
		} else {
			ROS_INFO("%s %d: event manager thread is up!", __FUNCTION__, __LINE__);
		}
#endif

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
		set_led(100, 100);
		sleep(10);
	}

	robotbase_deinit();
	laser_pm_gpio('0');
	return 0;
}
