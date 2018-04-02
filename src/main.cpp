#include <serial.h>
#include <bumper.h>
#include <signal.h>
#include <path_algorithm.h>
#include "robot.hpp"
#include "speaker.h"
#include "execinfo.h"

#if VERIFY_CPU_ID || VERIFY_KEY
#include "verify.h"
#endif

robot* robot_instance = nullptr;

void server_backtrace(int sig)
{
	void *pTrace[256];
    char **ppszMsg = NULL;
    size_t uTraceSize = 0;

    do {

        if (0 == (uTraceSize = backtrace(pTrace, sizeof(pTrace) / sizeof(void *)))) {
            break;
        }
        if (NULL == (ppszMsg = backtrace_symbols(pTrace, uTraceSize))) {
            break;
        }

        printf("%d. call stack:\n", sig);
        for (size_t i = 0; i < uTraceSize; ++i) {
              printf("%s\n", ppszMsg[i]);
        }
    } while (0);

    if (NULL != ppszMsg) {
        free(ppszMsg);
        ppszMsg = NULL;
    }
}

void handle_crash_exit(int sig)
{
	ROS_ERROR("Oops!!! pp receive (%d) signal",sig);
	server_backtrace(sig);
	exit(-1);
}

void handle_normal_exit(int sig)
{
	ROS_ERROR("Oops!!! pp receive SIGINT signal,ctrl+c press");
	if(robot_instance != nullptr){
//		speaker.play(VOICE_PROCESS_ERROR,false);
//		speaker.play(VOICE_NULL);
		speaker.stop();
		delete robot_instance;
		ros::shutdown();
	}
	exit(0);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "pp");
	ros::NodeHandle	nh_dev("~");

#if ENABLE_DEBUG
	signal(SIGABRT,handle_crash_exit);
	signal(SIGSEGV,handle_crash_exit);
	signal(SIGPIPE,handle_crash_exit);
	signal(SIGFPE,handle_crash_exit);
#endif

	signal(SIGINT,handle_normal_exit);
	ROS_INFO("set signal action done!");

	robot_instance = new robot();

	//test code by lsy563193
	/*GridMap map;
//test
/*	ROS_INFO("set signal action done!");
	GridMap map;
	case_2(map);
	map.print(CLEAN_MAP, 0, 0);

	boost::shared_ptr<APathAlgorithm> clean_path_algorithm_{};
	Dir_t old_dir_=MAP_POS_X;
	Points remain_path_{};
	ROS_INFO("clean_path_algorithm_!");
	clean_path_algorithm_.reset(new NavCleanPathAlgorithm);
	if (clean_path_algorithm_->generatePath(map, getPosition(), old_dir_, remain_path_)) {

	}
	ROS_INFO("~~~~~~~~~~~~~~~~~~~!");*/
	ros::spin();
	return 0;
}
// Test code by lsy563193
void case_2(GridMap &map) {

//	map.setBlockWithBound({-5,-1}, {5,1}, CLEANED, 0);
//	map.setBlockWithBound({-7,0}, {-7,0}, CLEANED, 1);
//	map.setBlockWithBound({7,0}, {7,0}, CLEANED, 1);
	map.setBlockWithBound({-15,-15}, {15,15}, CLEANED, 0);
	map.setBlockWithBound({-5,-5},{5,5},CLEANED,1);
	map.setBlockWithBound({-10,-5},{-7,5},UNCLEAN,1);
	map.setBlockWithBound({-11,-6},{6,-6},CLEANED,0);
	map.setBlockWithBound({-5,-6},{6,-6},CLEANED,0);

//	map_set_block({-2,0},{3,0},);
//	map_set_block({-2,0},{3,0},UNCLEAN);
}
