//
// Created by lsy563193 on 5/10/17.
//
#include <robot.hpp>
#include <error.h>
#include <beeper.h>
#include "event_manager.h"
#include "slam.h"
#include "protocol/wifi_map_protocol.h"
#include "wifi/wifi.h"

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
	robot::instance()->setBaselinkFrameType(ODOM_POSITION_ODOM_ANGLE);

	robot::instance()->slamStop();
	robot::instance()->resetCorrection();
	isMapReady(false);
	ROS_INFO("%s %d: End slam request finished.", __FUNCTION__, __LINE__);
}

void Slam::start(void)
{
	isMapReady(false);
	while (!robot::instance()->slamStart())
		ROS_WARN("%s %d: Start slam service not received, retry.", __FUNCTION__, __LINE__);
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

	auto size = static_cast<int16_t >MAP_SIZE;
	BoundingBox2 bound = {{static_cast<int16_t >(-size), static_cast<int16_t >(-size)},
						  {size, size}};
	slam_grid_map.convertFromSlamMap(CELL_SIZE, 0.2, bound);

//	slam_grid_map.print(getPosition().toCell(), CLEAN_MAP, {{0,0}});

	GridMap android_grid_map;
	Cell_t cell{static_cast<int16_t>(getPosition().x * WIFI_MAP_RESOLUTION / map->info.resolution),
				static_cast<int16_t>(getPosition().y * WIFI_MAP_RESOLUTION / map->info.resolution)};

	size = WIFI_MAP_WIDTH_HALF;

	bound = {{static_cast<int16_t >(-size), static_cast<int16_t >(-size)},
			 {size,                         size}};
	auto point = getPosition();
	cell = {static_cast<int16_t>(round(point.x / 0.05)), static_cast<int16_t>(round(point.y / 0.05))};
	// -- left top corner (max)
	bound.SetMaximum(cell + Cell_t{WIFI_MAP_WIDTH_HALF, WIFI_MAP_WIDTH_HALF});
	// -- right down corner (min)
	bound.SetMinimum(cell - Cell_t{WIFI_MAP_WIDTH_HALF, WIFI_MAP_WIDTH_HALF});

	ROS_INFO("%s %d: size(%d), curr((%d,%d),(%d,%d)),bound({%d,%d}{%d,%d})", __FUNCTION__, __LINE__, size,
			 cell.x, cell.y, getPosition().toCell().x, getPosition().toCell().y, bound.min.x, bound.min.y,
			 bound.max.x, bound.max.y);
	android_grid_map.convertFromSlamMap(WIFI_MAP_RESOLUTION, 0.25, bound);

	//static int count = 0;
	//count++;
//	if(count % 20 == 0)##
	{
		//count = 0;
		wifiMapManage.serialize(android_grid_map, bound);
//		android_grid_map.printInRange(cell,CLEAN_MAP, Cells{cell},true,bound);
//		beeper.debugBeep();
	}
	isMapReady(true);

	s_wifi.taskPushBack(S_Wifi::ACT::ACT_UPLOAD_MAP);

	ROS_INFO(
			"%s %d:finished map callback,nav_map.size(\033[33m%d,%d\033[0m),resolution(\033[33m%f\033[0m),nav_map.origin(\033[33m%f,%f\033[0m)",
			__FUNCTION__, __LINE__, slam_map.getWidth(), slam_map.getHeight(), slam_map.getResolution(),slam_map.getOriginX(), slam_map.getOriginY());
}
