#include <serial.h>
#include <robotbase.h>
#include <bumper.h>
#include <signal.h>
#include "robot.hpp"
#include "speaker.h"
#include "beep.h"
#if VERIFY_CPU_ID || VERIFY_KEY
#include "verify.h"
#endif

robot* robot_instance = nullptr;

void Ooops(int sig)
{
	switch(sig){
		case SIGSEGV:
		{
			ROS_ERROR("Oops!!! pp receive SIGSEGV signal,which means out of memery!");
			if(robot_instance != nullptr){
				speaker.play(VOICE_CLEANING_STOP,false);
				delete robot_instance;
			}
			break;
		}
		case SIGINT:
		{
			ROS_ERROR("Oops!!! pp receive SIGINT signal,ctrl+c press");
			beeper.play(3,50,50,5);
			if(robot_instance != nullptr){
				speaker.play(VOICE_CLEANING_STOP,false);
				delete robot_instance;
			}
			break;
		}
		case SIGTERM:
		{	
			ROS_ERROR("Ouch!!! pp receive SIGTERM signal,being kill!");
			if(robot_instance != nullptr){ 
				speaker.play(VOICE_CLEANING_STOP,false);
				delete robot_instance;
			}
			break;
		}
		default:
			ROS_ERROR("Oops!! pp receive %d signal",sig);
	}
}

int main(int argc, char **argv)
{

	ros::init(argc, argv, "pp");
	ros::NodeHandle	nh_dev("~");

	struct sigaction act;
	act.sa_handler = Ooops;
	sigemptyset(&act.sa_mask);
	act.sa_flags = SA_RESETHAND;
	sigaction(SIGTERM,&act,NULL);
	sigaction(SIGSEGV,&act,NULL);
	sigaction(SIGINT,&act,NULL);
	ROS_INFO("set signal action done!");

	std::string	serial_port;
	nh_dev.param<std::string>("serial_port", serial_port, "/dev/ttyS2");

	int		baud_rate;
	nh_dev.param<int>("baud_rate", baud_rate, 115200);

	// Init for serial.
	if (!serial.init(serial_port.c_str(), baud_rate))
	{
		ROS_ERROR("%s %d: Serial init failed!!", __FUNCTION__, __LINE__);
	}

	std::string lidar_bumper_dev;
	nh_dev.param<std::string>("lidar_bumper_file", lidar_bumper_dev, "/dev/input/event0");

	if (bumper.lidarBumperInit(lidar_bumper_dev.c_str()) == -1)
		ROS_ERROR(" lidar bumper open fail!");

	robot_instance = new robot();

	ros::spin();
	delete robot_instance;

	return 0;
}
