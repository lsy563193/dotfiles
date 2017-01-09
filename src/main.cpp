#include <pthread.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <ros/ros.h>
#include "core_move.h"
#include "gyro.h"
#include "movement.h"
#include "laser.hpp"
#include "robot.hpp"
#include "main.h"

#include "serial.h"
#include "crc8.h"
#include "robotbase.h"

void *core_move_thread(void *)
{
	while (!robot::instance()->robot_is_all_ready()) {
		usleep(1000);
	}
	/*
	Turn_Right(10, 850);
	Turn_Right(10, 1750);
	Turn_Right(10, 1000);
	Turn_Left(10, 850);
	Turn_Left(10, 1750);
	Turn_Left(10, 1000);
	*/
	CM_Touring();
	
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
	serial_close();

	pthread_exit(NULL);
	return 0;
}
