#include <serial.h>
#include <bumper.h>
#include <signal.h>
#include "robot.hpp"
#include "speaker.h"

#if R16_BOARD_TEST
#include "r16_board_test.hpp"
#endif

#if VERIFY_CPU_ID || VERIFY_KEY
#include "verify.h"
#endif

robot* robot_instance = nullptr;

void signal_catch(int sig)
{
	switch(sig){
		case SIGSEGV:
		{
			ROS_ERROR("Oops!!! pp receive SIGSEGV signal,segment fault!");
			if(robot_instance != nullptr){
				speaker.play(VOICE_CLEANING_STOP,false);
				delete robot_instance;
			}
			break;
		}
		case SIGINT:
		{
			ROS_ERROR("Oops!!! pp receive SIGINT signal,ctrl+c press");
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
	robot_instance = nullptr;
	ros::shutdown();
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "pp");
	ros::NodeHandle	nh_dev("~");

	struct sigaction act;
	act.sa_handler = signal_catch;
	sigemptyset(&act.sa_mask);
	act.sa_flags = SA_RESETHAND;
	sigaction(SIGTERM,&act,NULL);
	sigaction(SIGSEGV,&act,NULL);
	sigaction(SIGINT,&act,NULL);
	ROS_INFO("set signal action done!");

	robot_instance = new robot();

	ros::spin();
	return 0;
}
