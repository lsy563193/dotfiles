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

void protect_function()
{
	//Bumper protect
	if (Get_Bumper_Status()){
		ROS_INFO("Bumper protect  check");
		Set_Gyro_On();
		Turn_Left_At_Init(Max_Speed, 1800);//save itself
	}
	if (Get_Bumper_Status())
		Beep(5, 20, 0, 1); //can't save itself, stop and give an alarm by beep
}

void *core_move_thread(void *)
{
	pthread_detach(pthread_self());
	while (!robot::instance()->robot_is_all_ready()) {
		usleep(1000);
	}
	//Set_Clean_Mode(Clean_Mode_Navigation);
	//Set_Clean_Mode(Clean_Mode_GoHome);
	protect_function();

	while(ros::ok()){
		usleep(20000);
		switch(Get_Clean_Mode()){
			case Clean_Mode_Userinterface:
				ROS_INFO("\n-------User_Interface mode------\n");
				User_Interface();
				break;
			case Clean_Mode_WallFollow:
				ROS_INFO("\n-------wall follow mode------\n");
				Wall_Follow(Map_Wall_Follow_Escape_Trapped);
				break;
			case Clean_Mode_RandomMode:
				ROS_INFO("\n-------Random_Running mode------\n");
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
				CM_Touring();
				//Set_Clean_Mode(Clean_Mode_GoHome);
				break;
			case Clean_Mode_Charging:
				ROS_INFO("\n-------Charge mode------\n");
				Charge_Function();
				break;
			case Clean_Mode_GoHome:
				//goto_charger();
				ROS_INFO("\n-------GoHome mode------\n");
#if Random_Find_Charger_Stub
				HomeStraight_Mode();
#else
				GoHome();
#endif
				//Set_Clean_Mode(Clean_Mode_Charging);
				break;
			case Clean_Mode_Test:

				break;
			case Clean_Mode_Remote:
				ROS_INFO("\n-------Remote mode------\n");
				Remote_Mode();
				break;
			case Clean_Mode_Spot:
				ROS_INFO("\n-------Spot mode------\n");
				Spot_Mode();
				Disable_Motors();
				usleep(200000);
				Beep(1,25,25,2);
				Beep(2,25,25,2);
				break;
			case Clean_Mode_Mobility:

				break;
			case Clean_Mode_Sleep:
				ROS_INFO("\n-------Sleep mode------\n");
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
	pthread_t	core_move_thread_id;
	std::string	serial_port;
	int slam_type;
	bool line_align_active;
	ros::init(argc, argv, "pp");
	ros::NodeHandle	nh_private("~");

	laser	laser_obj;
	robot	robot_obj;

	nh_private.param<std::string>("serial_port", serial_port, "/dev/ttyS3");
	nh_private.param<int>("baudrate", baudrate, 57600);
	nh_private.param<bool>("line_align", line_align_active, false);
//	nh_private.param<robot::Slam_type>("slam_type", slam_type, robot::Slam_type::GMAPPING);
	nh_private.param<int>("slam_type", slam_type, 0);

	wav_play(WAV_WELCOME_ILIFE);

	serial_init(serial_port.c_str(), baudrate);
	robot::instance()->align_active(line_align_active);
	robot::instance()->slam_type(slam_type);
	robotbase_init();

#if 1
	ret1 = pthread_create(&core_move_thread_id, 0, core_move_thread, NULL);
	if (ret1 != 0) {
		core_move_thread_state = 0;
	} else {
		ROS_INFO("%s %d: core_move thread is up!", __FUNCTION__, __LINE__);
		core_move_thread_state = 1;
	}
#endif
	//ros::MultiThreadedSpinner spiner(4);
	//spiner.spin();
	ros::spin();

	if (core_move_thread_state == 1) {
		//pthread_join(core_move_thread_id, NULL);
	}

	robotbase_deinit();
//	serial_close();

	//pthread_exit(NULL);
	return 0;
}
