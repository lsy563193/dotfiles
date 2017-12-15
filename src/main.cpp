#include <pthread.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <ros/ros.h>
#include <battery.h>
#include <remote.hpp>
#include <charger.h>
#include <clean_mode.hpp>
#include "charge.hpp"
#include "core_move.h"
#include "gyro.h"
#include "bumper.h"
#include "led.h"
#include "movement.h"
#include "lidar.hpp"
#include "robot.hpp"
#include "main.h"
#include "serial.h"
#include "robotbase.h"
#include "spot.h"
#include "idle.h"
#include "remote_mode.h"
#include "sleep.h"
#include "wall_follow_trapped.h"
#include "event_manager.h"
#include "go_home.hpp"
#include "speaker.h"
#include "clean_mode.h"
#include "arch.hpp"


#if VERIFY_CPU_ID || VERIFY_KEY
#include "verify.h"
#endif

robot* robot_instance;

int main(int argc, char **argv)
{
	int		baudrate;

	ros::init(argc, argv, "pp");
	ros::NodeHandle	nh_private("~");

	std::string	serial_port;
	nh_private.param<std::string>("serial_port", serial_port, "/dev/ttyS2");
	nh_private.param<int>("baudrate", baudrate, 115200);

	std::string lidar_bumper_dev;
	nh_private.param<std::string>("lidar_bumper_file", lidar_bumper_dev, "/dev/input/event0");

	robot_instance = new robot(serial_port, baudrate, lidar_bumper_dev);

//	SpotMovement spot_obj(1.0);

	ros::spin();
	delete robot_instance;

	return 0;
}
