#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <string.h>
#include <pthread.h>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

#include "robot.hpp"

#include "main.h"
#include "core_move.h"
#include "path_planning.h"
#include "shortest_path.h"

#include "control.h"
#include "crc8.h"
#include "log.h"
#include "serial.h"


#define	TAG	"Main. (%d):\t"

static int move_speed = 50;

static int32_t log_level = LOG_ERROR;
static int32_t log_enabled = 0;

extern int32_t rowMin, rowMax, columnMin, columnMax;

void usage(void)
{
	printf("\npp:\n"
		"  -l: Log level\n"
		"  -h: This help manual.\n"
		"\n"
		);
}

void process_args(int argc, char **argv)
{
	int c;

	while ((c = getopt(argc, argv, "l:")) != -1) {
		switch (c) {
			case 'l':
				log_enabled = 1;
				log_level = atoi(optarg);
				break;
			case 'h':
			default:
				usage();
				return;
		}
	}
}

void intialize(void)
{

	init_crc8();

	if (log_enabled == 1) {
		if (log_init() == -1)
			log_enabled = 0;
		else
			log_set_level(log_level);
	}

	serial_init();

}


void *core_move_thread(void *)
{
	while (!robot::instance()->robot_is_ready()) {
		usleep(1000);
	}
	CM_Touring();
}

int main(int argc, char **argv)
{
	int			ret, core_move_thread_state;
	pthread_t	core_move_thread_id;

	process_args(argc, argv);

	intialize();

	ros::init(argc, argv, "pp");

	robot	robot_obj;

#if 1
	printf("%s, %d\n", __FUNCTION__, __LINE__);
	ret = pthread_create(&core_move_thread_id, 0, core_move_thread, NULL);
	if (ret != 0) {
		printf("%s, %d\n", __FUNCTION__, __LINE__);
		core_move_thread_state = 0;
		log_msg(LOG_FATAL, TAG "fails to core move thread\n");
	} else {
		printf("%s, %d\n", __FUNCTION__, __LINE__);
		core_move_thread_state = 1;
	}
#endif
	ros::spin();

	if (core_move_thread_state == 1) {
		pthread_join(core_move_thread_id, NULL);
	}

	if (log_enabled == 1) {
		log_deinit();
	}

	serial_close();

	return 0;
}

