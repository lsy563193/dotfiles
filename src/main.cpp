#include <pthread.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <ros/ros.h>
#include <battery.h>
#include "charger.hpp"
#include "core_move.h"
#include "gyro.h"
#include "bumper.h"
#include "movement.h"
#include "laser.hpp"
#include "robot.hpp"
#include "main.h"
#include "serial.h"
#include "crc8.h"
#include "robotbase.h"
#include "spot.h"
#include "idle.h"
#include "remote_mode.h"
#include "sleep.h"
#include "wall_follow_trapped.h"
#include "event_manager.h"
#include "go_home.hpp"
#include "wav.h"
#include "clean_mode.h"

#if VERIFY_CPU_ID || VERIFY_KEY
#include "verify.h"
#endif

void *core_move_thread(void *)
{
	pthread_detach(pthread_self());
	//ROS_INFO("Waiting for robot sensor ready.");
	while (!robot::instance()->isSensorReady()) {
		usleep(1000);
	}
	//ROS_INFO("Robot sensor ready.");
	//wav_play(WAV_WELCOME_ILIFE);
	usleep(200000);

	if (is_direct_charge() || is_on_charger_stub())
		cm_set(Clean_Mode_Charging);
	else if (battery.is_ready_to_clean())
		wav_play(WAV_PLEASE_START_CLEANING);

	while(ros::ok()){
		usleep(20000);
		switch(cm_get()){
			case Clean_Mode_Idle:
				ROS_INFO("\n-------idle mode_------\n");
				set_main_pwr_byte(Clean_Mode_Idle);
//				wav_play(WAV_TEST_MODE);
				idle();
				break;
			case Clean_Mode_WallFollow:
				ROS_INFO("\n-------wall follow mode_------\n");
				set_main_pwr_byte(Clean_Mode_WallFollow);
				g_is_low_bat_pause = false;

				reset_clean_paused();

//				wall_follow(Map_Wall_Follow_Escape_Trapped);
				cm_cleaning();
				break;
			case Clean_Mode_Navigation:
				ROS_INFO("\n-------Navigation mode_------\n");
				set_main_pwr_byte(Clean_Mode_Navigation);
				cm_cleaning();
				break;
			case Clean_Mode_Charging:
				ROS_INFO("\n-------Charge mode_------\n");
				set_main_pwr_byte(Clean_Mode_Charging);
				charge_function();
				break;
			case Clean_Mode_Go_Charger:
				//goto_charger();
				ROS_INFO("\n-------GoHome mode_------\n");
				set_main_pwr_byte(Clean_Mode_Go_Charger);
				g_is_low_bat_pause = false;
				reset_clean_paused();
#if GO_HOME_REGULATOR
				cm_cleaning();
#else
				go_home();
#endif
				break;

			case Clean_Mode_Exploration:
				//goto_charger();
				ROS_INFO("\n-------Exploration mode_------\n");
				set_main_pwr_byte(Clean_Mode_Exploration);
				g_is_low_bat_pause = false;
				reset_clean_paused();
				cm_cleaning();
				break;

			case Clean_Mode_Test:
				break;

			case Clean_Mode_Remote:
				remote_mode();
				break;

			case Clean_Mode_Spot:
				ROS_INFO("\n-------Spot mode_------\n");
				set_main_pwr_byte(Clean_Mode_Spot);
				g_is_low_bat_pause = false;
				reset_clean_paused();
				reset_rcon_remote();
				SpotMovement::instance()->setSpotType(NORMAL_SPOT);
				cm_cleaning();
				disable_motors();
				usleep(200000);
				break;

			case Clean_Mode_Sleep:
				ROS_INFO("\n-------Sleep mode_------\n");
				//set_main_pwr_byte(Clean_Mode_Sleep);
				g_is_low_bat_pause = false;
				reset_clean_paused();
				disable_motors();
				sleep_mode();
				break;
			default:
				cm_set(Clean_Mode_Idle);
				break;

		}
	}
	
	return NULL;
	//pthread_exit(NULL);	
}


int main(int argc, char **argv)
{
	int		baudrate, ret1, core_move_thread_state;
	bool	verify_ok = true;
	pthread_t	core_move_thread_id, event_manager_thread_id, event_handler_thread_id;
	std::string	serial_port;
	std::string lidar_bumper;

	ros::init(argc, argv, "pp");
	ros::NodeHandle	nh_private("~");

	robot	robot_obj;

	SpotMovement spot_obj(1.0);

	event_manager_init();

	nh_private.param<std::string>("serial_port", serial_port, "/dev/ttyS3");
	nh_private.param<int>("baudrate", baudrate, 57600);
	nh_private.param<std::string>("lidar_bumper_file", lidar_bumper, "/dev/input/event0");
	
	serial_init(serial_port.c_str(), baudrate);
	if(bumper_lidar_init(lidar_bumper.c_str()) == -1){
		ROS_ERROR(" lidar bumper open fail!");
	}
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
			ROS_ERROR("%s %d: event_manager_thread fails to run!", __FUNCTION__, __LINE__);
		} else {
			ROS_INFO("%s %d: \033[32mevent_manager_thread\033[0m is up!", __FUNCTION__, __LINE__);
		}
#endif


#if 1
		ret1 = pthread_create(&event_handler_thread_id, 0, event_handler_thread, NULL);
		if (ret1 != 0) {
			ROS_ERROR("%s %d: event_handler_thread fails to run!", __FUNCTION__, __LINE__);
		} else {
			ROS_INFO("%s %d: \033[32mevent_handler_thread\033[0m is up!", __FUNCTION__, __LINE__);
		}
#endif

		ret1 = pthread_create(&core_move_thread_id, 0, core_move_thread, NULL);
		if (ret1 != 0) {
			core_move_thread_state = 0;
		} else {
			ROS_INFO("%s %d: \033[32mcore_move_thread\033[0m is up!", __FUNCTION__, __LINE__);
			core_move_thread_state = 1;
		}
		ros::spin();
	} else {
		printf("turn on led\n");
		set_led_mode(LED_STEADY, LED_ORANGE);
		sleep(10);
	}
	bumper_lidar_deinit();
	robotbase_deinit();
	return 0;
}
