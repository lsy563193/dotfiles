//
// Created by lsy563193 on 5/10/17.
//
#include "ros/ros.h"
#include <cstdlib>
#include <movement.h>
#include <nav_msgs/OccupancyGrid.h>
#include "slam.h"
#include "robot.hpp"
#include "std_srvs/Empty.h"

Slam::Slam():nh_local_("~"),nh_("/"),is_map_ready_(false)
{
	start();
};

Slam::~Slam(){
	stop();
//	align_cli_.shutdown();
//	map_sub_.shutdown();
//	nh_local_.shutdown();
//	nh_.shutdown();
};

void Slam::start(void)
{

#if SLAM_METHOD_2
	return;
#else
	nh_local_.param<int>("slam_type",slam_type_,0);
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
#endif
}

void Slam::stop(void)
{
	robot::instance()->setBaselinkFrameType(Odom_Position_Odom_Angle);
	is_map_ready_ = false;

#if SLAM_METHOD_2
	end_slam_cli_ = nh_.serviceClient<std_srvs::Empty>("End_Slam");
	std_srvs::Empty empty;
	end_slam_cli_.call(empty);
	ROS_INFO("%s %d: End slam request finished.", __FUNCTION__, __LINE__);
#else
	if (slam_type_ == 0)
		system("rosnode kill /slam_gmapping 2>/dev/null &");
	else if (slam_type_ == 1)
		system("rosnode kill /slam_karto 2>/dev/null &");
	else if (slam_type_ == 2)
		system("rosnode kill /cartographer_node 2>/dev/null &");
#endif
}

void Slam::enableMapUpdate()
{
	std_srvs::Empty empty;
#if SLAM_METHOD_2
	start_slam_cli_ = nh_.serviceClient<std_srvs::Empty>("Start_Slam");
	start_slam_cli_.call(empty);
#else
	align_cli_ = nh_.serviceClient<std_srvs::Empty>("align");
//	align_cli_.waitForExistence(ros::Duration(10));
	align_cli_.call(empty);
#endif

}

