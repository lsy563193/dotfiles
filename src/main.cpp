#include "pp.h"
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
