//
// Created by lsy563193 on 5/10/17.
//

#include "ros/ros.h"
#include "boost/thread.hpp"
#include "pose.h"
#include <nav_msgs/OccupancyGrid.h>

#ifndef PP_SLAM_H
#define PP_SLAM_H
class Slam
{
public:
	Slam();
	~Slam();

	bool isMapReady(void)
	{
		return is_map_ready_;
	}

	void isMapReady(bool val)
	{
		is_map_ready_ = val;
	}

	void start(void);
	bool openTimeOut(void);
	void stop(void);

	void mapCb(const nav_msgs::OccupancyGrid::ConstPtr &map);

	Pose pose;
private:

	bool	is_map_ready_;


	// For opening slam.
	time_t open_command_time_stamp_;
};

extern Slam slam;

#endif //PP_SLAM_H
