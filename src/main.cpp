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
    //open file
    time_t tSetTime;
    time(&tSetTime);
    struct tm* ptm = localtime(&tSetTime);
    char fname[256] = {0};
    sprintf(fname, "core.%d-%d-%d_%d_%d_%d",
            ptm->tm_year+1900, ptm->tm_mon+1, ptm->tm_mday,
            ptm->tm_hour, ptm->tm_min, ptm->tm_sec);
    FILE* f = fopen(fname, "a");
    if (f == NULL){
        return;
    }
    int fd = fileno(f);

    //lock file
    struct flock fl;
    fl.l_type = F_WRLCK;
    fl.l_start = 0;
    fl.l_whence = SEEK_SET;
    fl.l_len = 0;
    fl.l_pid = getpid();
    fcntl(fd, F_SETLKW, &fl);

    //output path
    char buffer[4096];
    memset(buffer, 0, sizeof(buffer));
    int count = readlink("/proc/self/exe", buffer, sizeof(buffer));
    if(count > 0){
        buffer[count] = '\n';
        buffer[count + 1] = 0;
        fwrite(buffer, 1, count+1, f);
    }

    //output time 
    memset(buffer, 0, sizeof(buffer));
    sprintf(buffer, "Dump Time: %d-%d-%d %d:%d:%d\n",
            ptm->tm_year+1900, ptm->tm_mon+1, ptm->tm_mday,
            ptm->tm_hour, ptm->tm_min, ptm->tm_sec);
    fwrite(buffer, 1, strlen(buffer), f);

    //signal
    sprintf(buffer, "Curr thread: %u, Catch signal:%d\n",
            (int)pthread_self(), sig);
    fwrite(buffer, 1, strlen(buffer), f);

    //stack
    void* DumpArray[256];
    int    nSize =    backtrace(DumpArray, 256);
    sprintf(buffer, "backtrace rank = %d\n", nSize);
    fwrite(buffer, 1, strlen(buffer), f);
    if (nSize > 0){
        char** symbols = backtrace_symbols(DumpArray, nSize);
        if (symbols != NULL){
            for (int i=0; i<nSize; i++){
                fwrite(symbols[i], 1, strlen(symbols[i]), f);
                fwrite("\n", 1, 1, f);
            }
            free(symbols);
        }
    }

    //file unlock and close
    fl.l_type = F_UNLCK;
    fcntl(fd, F_SETLK, &fl);
    fclose(f);
}

void handle_crash_exit(int sig)
{
	ROS_ERROR("Oops!!! pp receive (%d) signal,segment fault!",sig);	
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

	signal(SIGTERM,handle_crash_exit);
	signal(SIGABRT,handle_crash_exit);
	signal(SIGSEGV,handle_crash_exit);
	signal(SIGPIPE,handle_crash_exit);
	signal(SIGFPE,handle_crash_exit);
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
