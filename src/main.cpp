#include <serial.h>
#include <bumper.h>
#include <signal.h>
#include <path_algorithm.h>
#include <main.h>
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

void handle_exit(int sig) 
{
ROS_ERROR("Oops!!! pp receive signal %d",sig);
	if(sig == SIGINT)
	{
		if(robot_instance != nullptr){
			speaker.stop();
			delete robot_instance;
			ros::shutdown();
		}
		exit(0);
	}
	else if(sig == SIGSEGV)
	{	
		server_backtrace(sig);
		exit(-1);
	}
	else if(sig == SIGFPE) {	
		server_backtrace(sig);
		exit(-1);
	}

	else if(sig == SIGFPE)
	{	
		server_backtrace(sig);
		exit(-1);
	}

	else if(sig == SIGABRT)
	{	
		server_backtrace(sig);
		exit(-1);
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "pp");
	ros::NodeHandle	nh_dev("~");

#if ENABLE_DEBUG

	struct sigaction act;

	act.sa_handler = handle_exit;
	act.sa_flags = SA_RESETHAND;
	sigemptyset(&act.sa_mask);
	sigaction(SIGINT,&act,NULL);
	sigaction(SIGSEGV,&act,NULL);
	sigaction(SIGPIPE,&act,NULL);
	sigaction(SIGFPE,&act,NULL);
	sigaction(SIGABRT,&act,NULL);
	ROS_INFO("set signal action done!");

#endif
	robot_instance = new robot();
// Test code for path algorithm by Austin.
//	test_map();
#if 1
	//test code by lsy563193
	sleep(1);
	GridMap map;
	//test
	Cell_t curr{};
	Dir_t old_dir_=MAP_POS_X;

	Points remain_path_{};

	NavCleanPathAlgorithm clean_path_algorithm_;
	map.loadMap(true,curr, old_dir_, clean_path_algorithm_.trend_pos);
	clean_path_algorithm_.curr_filter_ = &clean_path_algorithm_.filter_short_path;
	setPosition(cellToCount(curr.x),cellToCount(curr.y));

	Cells cells{};
//	auto is_found = map.gen(curr, cells,[&](const Cell_t& c_it){return c_it == Cell_t{-2,0};},true);
	if (clean_path_algorithm_.generatePath(map, Point_t{cellToCount(curr.x),cellToCount(curr.y)}, old_dir_, remain_path_)) {
	}
	ROS_INFO("end~~~~~~~~~");
#endif
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

// Test code by Austin.
int test_time()
{
	time_t current_time = time(NULL);
	time_t *p_current_time = &current_time;
	printf("time(): %d.\n", *p_current_time);
	printf("ctime(): %s", ctime(p_current_time));
	struct tm *gm_p_current_time = gmtime(p_current_time);
	printf("gmtime(): %s", asctime(gm_p_current_time));
	struct tm *local_p_current_time = localtime(p_current_time);
	printf("localtime(): %s", asctime(local_p_current_time));

	printf("gmtime->time(): %d.\n", mktime(gmtime(p_current_time)));
	printf("localtime->time(): %d.\n", mktime(localtime(p_current_time)));
	time_t gm = mktime(gmtime(p_current_time));
	time_t *p_gm = &gm;
	printf("ctime(gmtime->time()): %s", ctime(p_gm));
	time_t local = mktime(localtime(p_current_time));
	time_t *p_local = &local;
	printf("ctime(localtime->time()): %s", ctime(p_local));
}

// Test code for path algorithm by Austin.
void test_map()
{
	auto path_algo = new NavCleanPathAlgorithm();
	GridMap map;
	map.loadMap(-10, 9, -2, 16);
	Cell_t curr_cell{8, 14};

	Point_t curr_point = {curr_cell.x * CELL_SIZE, curr_cell.y * CELL_SIZE};
	Dir_t dir = MAP_NEG_X;
	Points points;
	path_algo->generatePath(map, curr_point, dir, points);
}
