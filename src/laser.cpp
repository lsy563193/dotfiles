#include <math.h>
#include <stdio.h>
#include <time.h>

#include <angles/angles.h>

#include "mathematics.h"

#include "laser.hpp"

static	laser *laser_obj = NULL;

laser::laser()
{
	this->init();

	this->scan_sub = this->laser_node_handler.subscribe("scan", 1, &laser::laser_scan_cb, this);

	this->is_laser_ready = false;

	printf("%s %d: laser init done!\n", __FUNCTION__, __LINE__);
}

laser::~laser()
{
	laser_obj = NULL;
}

laser *laser::instance()
{
	return laser_obj;
}

void laser::init()
{
	if (laser_obj == NULL) {
		laser_obj = this;
	}
}

bool laser::is_ready()
{
	return (this->is_laser_ready) ? true : false;
}

void laser::laser_scan_cb(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	int i, count = 0;

	this->laser_scan_data = *msg;
	count = (int)((msg->angle_max - msg->angle_min) / msg->angle_increment);
//	printf("%s %d: seq: %d\tangle_min: %f\tangle_max: %f\tcount: %d\tdist: %f\n", __FUNCTION__, __LINE__, msg->header.seq, msg->angle_min, msg->angle_max, count, msg->ranges[180]);

//	ROS_INFO("LASER_SCAN_CB!!!!!!!!");
	this->is_laser_ready = true;
}

bool laser::laser_obstcal_detected(double distance, int angle, double range)
{
	int		i, count;
	bool	found = false;
	double	angle_min, angle_max, tmp, range_tmp;

	if (range < 0.0) {
		range_tmp = 0.155;
	} else {
		range_tmp = range;
	}
	angle_min = deg2rad((double) (angle % 360), 1) - atan(range_tmp / (distance + 0.155));
	angle_max = deg2rad((double) (angle % 360), 1) + atan(range_tmp / (distance + 0.155));

	count = (int)((laser_scan_data.angle_max - laser_scan_data.angle_min) / laser_scan_data.angle_increment);
	//printf("%s %d %f %f %f %f\n", __FUNCTION__, __LINE__, range_tmp, distance + 0.155, range_tmp / (distance + 0.155), atan(range_tmp / (distance + 0.155)));
	//printf("%s %d: angle min: %f max: %f\tcount: %d\tdtor: %f\ttan: %f\n", __FUNCTION__, __LINE__, angle_min, angle_max, count, deg2rad((double) (angle % 360), 1),  atan(range_tmp / (distance + 0.155)));
	for (i = 0; found == false && i < count; i++) {
		tmp = this->laser_scan_data.angle_min + i * this->laser_scan_data.angle_increment;
		if (tmp > angle_min && tmp < angle_max && laser_scan_data.ranges[i] < distance + 0.155) {
			//printf("%s %d: i: %d\ttmp: %f(%f, %f)\tdist: %f(%f)\n", __FUNCTION__, __LINE__, i, tmp, angle_min, angle_max, laser_scan_data.ranges[i], distance + 0.155);
			found = true;
		}
	}

	return found;
}

void laser::is_ready(bool val)
{
	is_laser_ready = val;
}
