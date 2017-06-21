#ifndef __LASER_H__
#define __LASER_H__

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

class Laser
{
public:
	Laser();
	~Laser();

	bool laserObstcalDetected(double distance, int angle, double range);

	double getLaserDistance(int begin, int end, double range);

	void isReady(uint8_t val);
	int8_t isReady();
	bool isNewDataReady();
	double getLaserDistance(uint16_t angle);

private:
	void stop(void);
	void start(void);
	void scanCb(const sensor_msgs::LaserScan::ConstPtr &msg);
	int8_t is_ready_;
	bool is_scanDataReady_;
	ros::NodeHandle	nh_;
	ros::Subscriber	scan_sub_;

	sensor_msgs::LaserScan laser_scan_data_;

	ros::ServiceClient start_mator_cli_;
	ros::ServiceClient stop_mator_cli_;
};

#endif
