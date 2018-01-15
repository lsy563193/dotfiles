#include "robot.hpp"


#if VERIFY_CPU_ID || VERIFY_KEY
#include "verify.h"
#endif

robot* robot_instance = nullptr;

int main(int argc, char **argv)
{

	ros::init(argc, argv, "pp");
	ros::NodeHandle	nh_dev("~");

	std::string	serial_port;
	nh_dev.param<std::string>("serial_port", serial_port, "/dev/ttyS2");

	int		baud_rate;
	nh_dev.param<int>("baud_rate", baud_rate, 115200);

	std::string lidar_bumper_dev;
	nh_dev.param<std::string>("lidar_bumper_file", lidar_bumper_dev, "/dev/input/event0");

	robot_instance = new robot(serial_port, baud_rate, lidar_bumper_dev);

	ros::spin();
	delete robot_instance;

	return 0;
}
