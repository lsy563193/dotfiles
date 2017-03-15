#ifndef __LASER_H__
#define __LASER_H__

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

class laser
{
public:
	laser();
	~laser();

	static laser *instance();

	void init();

	bool laser_obstcal_detected(double distance, int angle, double range);

	void is_ready(bool val);

	bool is_ready();

private:
	bool is_laser_ready;

	ros::NodeHandle	laser_node_handler;
	ros::Subscriber	scan_sub;

	sensor_msgs::LaserScan		laser_scan_data;

	void laser_scan_cb(const sensor_msgs::LaserScan::ConstPtr& msg);
};

#endif
