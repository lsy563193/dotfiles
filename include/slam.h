//
// Created by lsy563193 on 5/10/17.
//

#include "ros/ros.h"
#include "boost/thread.hpp"
#include "pose.h"

#ifndef PP_SLAM_H
#define PP_SLAM_H
class Slam
{
public:
	Slam();
	~Slam();
	//todo
	void enableMapUpdate();
	bool isMapReady(void)
	{
		boost::mutex::scoped_lock(is_map_ready_mutex_);
		return is_map_ready_;
	}

	void isMapReady(bool val)
	{
		boost::mutex::scoped_lock(is_map_ready_mutex_);
		is_map_ready_ = val;
	}

	void start(void);
	void stop(void);

	Pose pose;
private:

//	void mapCb(const nav_msgs::OccupancyGrid::ConstPtr &map);
	ros::ServiceClient align_cli_;
#if SLAM_METHOD_2
	ros::ServiceClient start_slam_cli_;
	ros::ServiceClient end_slam_cli_;
#endif
	bool	is_map_ready_;
	boost::mutex is_map_ready_mutex_;
//	ros::Subscriber map_sub_;
	ros::NodeHandle nh_;
	ros::NodeHandle nh_local_;

};

#endif //PP_SLAM_H
