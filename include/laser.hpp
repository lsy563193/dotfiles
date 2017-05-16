#ifndef __LASER_H__
#define __LASER_H__

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

class Laser
{
public:
	Laser();
	~Laser();

	bool laser_obstcal_detected(double distance, int angle, double range);

	double get_laser_distance(int begin, int end, double range);

	void is_ready(bool val);

	bool is_ready();

private:
	void stop(void);
	void start(void);
	bool is_ready_;

	ros::NodeHandle	nh_;
	ros::Subscriber	scan_sub_;

	sensor_msgs::LaserScan laser_scan_data;

	void laser_scan_cb(const sensor_msgs::LaserScan::ConstPtr& msg);

	ros::ServiceClient start_mator_cli_;
	ros::ServiceClient stop_mator_cli_;
};

#endif
