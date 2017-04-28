//
// Created by lsy563193 on 4/25/17.
//

#include <movement.h>
#include <robot.hpp>
#include <wav.h>
#include <config.h>

#include "obstacle_detector.cpp"

#include "motion_controler.h"

int8_t enable_slam_offset = 0;

bool start_slam(void)
{
	robot::instance()->start_slam();
	if(except_event()){
		robot::instance()->stop_slam();
		return false;
	}

	enable_slam_offset = 1;

	auto count_n_10ms = 1000;
	while(robot::instance()->map_ready() == false && --count_n_10ms != 0){
//		ROS_WARN("%s %d: Map is still not ready after 10s, timeout and return.", __FUNCTION__, __LINE__);
		usleep(10000);
	}
	if(count_n_10ms == 0){
		ROS_INFO("%s %d: Map is still not ready after 10s, timeout and return.", __FUNCTION__, __LINE__);
		return false;
	}
	return true;

}

/*

void show_time(std::function<void(void)> task){
	auto gyro_start = std::chrono::system_clock::now();
	task();
	auto diff = std::chrono::system_clock::now() - gyro_start;
	auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(diff);
	std::cout <<"this task runs:" << ms.count() << " ms" << std::endl;
}
*/

Motion_controller::Motion_controller()
{
#if CONTINUE_CLEANING_AFTER_CHARGE
	if (robot::instance()->Is_Cleaning_Low_Bat_Paused())
	{
		Work_Motor_Configure();
		robot::instance()->start_lidar();
		enable_slam_offset = 1;
	} else
#endif
#if MANUAL_PAUSE_CLEANING
	{
		if (robot::instance()->Is_Cleaning_Manual_Paused())
		{
			Work_Motor_Configure();
			robot::instance()->start_lidar();
			enable_slam_offset = 1;
		}
		else
#endif
		{
			Work_Motor_Configure();
			if (!robot::instance()->start_lidar())
				return;
			if (Get_Clean_Mode() == Clean_Mode_Navigation && robot::instance()->align_active())
			{
				ObstacleDetector od;
//				if (!start_obstacle_detector()) return;
				if (!robot::instance()->align()) return;
			}
			start_slam();
		}
#if MANUAL_PAUSE_CLEANING
	}
#endif
}

Motion_controller::~Motion_controller()
{
#if CONTINUE_CLEANING_AFTER_CHARGE
	if (robot::instance()->Is_Cleaning_Low_Bat_Paused())
	{
		Disable_Motors();
		robot::instance()->stop_lidar();
		enable_slam_offset = 0;
	} else
#endif
#if MANUAL_PAUSE_CLEANING
	{
		if (robot::instance()->Is_Cleaning_Manual_Paused())
		{
			Disable_Motors();
			robot::instance()->stop_lidar();
			enable_slam_offset = 0;
			wav_play(WAV_TEST_FAIL);
		}
		else
#endif
		{
			Disable_Motors();

//			if(start_bit[lidar])
			//try 3times;make sure to stop
			if(Get_Cliff_Trig())
				wav_play(WAV_ERROR_LIFT_UP);

			wav_play(WAV_CLEANING_FINISHED);

			robot::instance()->stop_lidar();
//			if(start_bit[align])
			robot::instance()->align_exit();

//			if(start_bit[slam])
			robot::instance()->stop_slam();

		}
#if MANUAL_PAUSE_CLEANING
	}
#endif
};

