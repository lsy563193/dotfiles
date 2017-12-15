//
// Created by lsy563193 on 5/10/17.
//
#include "pp.h"
Slam slam;

Slam::Slam():is_map_ready_(false)
{
	//todo: slam should add a status for setting mapReady in case slam still get a map after shutting down.
};

Slam::~Slam(){
	stop();
//	align_cli_.shutdown();
//	map_sub_.shutdown();
//	nh_local_.shutdown();
//	nh_.shutdown();
};

void Slam::stop(void)
{
	robot::instance()->setBaselinkFrameType(Odom_Position_Odom_Angle);

	robot::instance()->slamStop();
	robot::instance()->resetCorrection();
	isMapReady(false);
	ROS_INFO("%s %d: End slam request finished.", __FUNCTION__, __LINE__);
}

void Slam::start(void)
{
	isMapReady(false);
	robot::instance()->slamStart();
	robot::instance()->resetCorrection();
	open_command_time_stamp_ = time(NULL);
}

bool Slam::openTimeOut(void)
{
	auto time_diff = time(NULL) - open_command_time_stamp_;
	//ROS_INFO("%s %d: Time diff:%d", __FUNCTION__, __LINE__, time_diff);
	if (time_diff > 10)
	{
		ROS_ERROR("%s %d: Slam Open time out.", __FUNCTION__, __LINE__);
		ev.fatal_quit = true;
		error.set(ERROR_CODE_SLAM);
		return true;
	}

	return false;
}

void Slam::mapCb(const nav_msgs::OccupancyGrid::ConstPtr &map)
{
	slam_map.setWidth(map->info.width);
	slam_map.setHeight(map->info.height);
	slam_map.setResolution(map->info.resolution);
	slam_map.setOriginX(map->info.origin.position.x);
	slam_map.setOriginY(map->info.origin.position.y);
	slam_map.setData(map->data);

	slam_grid_map.convertFromSlamMap(0.2);
	//slam_grid_map.print(CLEAN_MAP, 0, 0);

	isMapReady(true);

	ROS_INFO("%s %d:finished map callback,nav_map.size(\033[33m%d,%d\033[0m),resolution(\033[33m%f\033[0m),nav_map.origin(\033[33m%f,%f\033[0m)",
			 __FUNCTION__, __LINE__, slam_map.getWidth(), slam_map.getHeight(), slam_map.getResolution(), slam_map.getOriginX(), slam_map.getOriginY());
}

