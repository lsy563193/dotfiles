#include <pthread.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <ros/ros.h>
#include "gotocharger.hpp"
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

// This is the start time of cleanning
time_t Work_Timer_Start = 0;

void *core_move_thread(void *)
{
	pthread_detach(pthread_self());
	while (!robot::instance()->robot_is_all_ready()) {
		usleep(1000);
	}
	Set_Clean_Mode(Clean_Mode_Navigation);

	while(ros::ok()){
		usleep(20000);
		switch(Get_Clean_Mode()){
			case Clean_Mode_Userinterface:
				//ROS_INFO("\n-------user interface  mode------\n");
				User_Interface();
				break;
			case Clean_Mode_WallFollow:
				ROS_INFO("\n-------wall follow mode------\n");

				break;
			case Clean_Mode_RandomMode:
				ROS_INFO("\n-------random mode------\n");

				break;
			case Clean_Mode_Navigation:
				ROS_INFO("\n---------navigation mode-----------\n");

				/*
					Turn_Right(10, 850);
					Turn_Right(10, 1750);
					Turn_Right(10, 1000);
					Turn_Left(10, 850);
					Turn_Left(10, 1750);
					Turn_Left(10, 1000);
				*/
				CM_Touring();
				Set_Clean_Mode(Clean_Mode_Charging);
				break;
			case Clean_Mode_Charging:
				ROS_INFO("\n-------charging mode------\n");
				goto_charger();
				break;
			case Clean_Mode_GoHome:
				ROS_INFO("\n-----------go home mode--------\n");
				break;
			case Clean_Mode_Test:
				ROS_INFO("\n-----------test mode--------\n");

				break;
			case Clean_Mode_Remote:
				ROS_INFO("\n-----------remote mode--------\n");
				Remote_Mode();
				break;
			case Clean_Mode_Spot:
				ROS_INFO("\n-----------spot mode--------\n");
				Spot_Mode();
				Disable_Motors();
				usleep(200000);
				Beep(1,25,25,2);
				Beep(2,25,25,2);
				break;
			case Clean_Mode_Mobility:
				ROS_INFO("\n-----------mobility mode--------\n");

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

	ros::init(argc, argv, "pp");
	ros::NodeHandle	nh_private("~");

	laser	laser_obj;
	robot	robot_obj;

	nh_private.param<std::string>("serial_port", serial_port, "/dev/ttyS3");
	nh_private.param<int>("baudrate", baudrate, 115200);

	serial_init(serial_port.c_str(), baudrate);
	robotbase_init();

#if 1
	ret1 = pthread_create(&core_move_thread_id, 0, core_move_thread, NULL);
	if (ret1 != 0) {
		core_move_thread_state = 0;
	} else {
		printf("%s %d: core_move thread is up!\n", __FUNCTION__, __LINE__);
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
