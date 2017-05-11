//
// Created by lsy563193 on 5/10/17.
//
#include "ros/ros.h"
#include <cstdlib>
#include <movement.h>
#include <nav_msgs/OccupancyGrid.h>
#include "slam.h"
#include "std_srvs/Empty.h"

Slam::Slam():nh_("~"),is_map_ready_(false)
{
	start();
};

Slam::~Slam(){
	stop();
	align_cli_.shutdown();
	map_sub_.shutdown();
	nh_.shutdown();
};

void Slam::robot_map_cb(const nav_msgs::OccupancyGrid::ConstPtr& map)
{
	is_map_ready_=true;
	ROS_INFO("%s %d:finished map callback", __FUNCTION__, __LINE__);
}

void Slam::start(void)
{

	nh_.param<int>("slam_type",slam_type_,0);
	if (slam_type_ == 0)
		system("roslaunch pp gmapping.launch 2>/dev/null &");
	else if (slam_type_ == 1)
		system("roslaunch slam_karto karto_slam_w_params.launch 2>/dev/null &");
	else if (slam_type_ == 2)
		system("roslaunch pp cartographer_slam.launch 2>/dev/null &");
/*
	if(Stop_Event()){
		stop();
		return false;
	}
	return true;*/
}

void Slam::stop(void)
{
	extern int8_t g_enable_slam_offset;
	g_enable_slam_offset = 0;
	is_map_ready_ = false;
	if (slam_type_ == 0)
		system("rosnode kill /slam_gmapping 2>/dev/null &");
	else if (slam_type_ == 1)
		system("rosnode kill /slam_karto 2>/dev/null &");
	else if (slam_type_ == 2)
		system("rosnode kill /cartographer_node 2>/dev/null &");
}

void Slam::enable_map_update()
{
	align_cli_ = nh_.serviceClient<std_srvs::Empty>("align");
	std_srvs::Empty empty;
	align_cli_.call(empty);
//	ROS_INFO("subscribe");
	map_sub_ = nh_.subscribe("/map", 1, &Slam::robot_map_cb, this);
//	ROS_INFO("subscribe ok");
}

bool Slam::is_map_ready()
{
	return is_map_ready_;
}
