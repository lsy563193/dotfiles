#include <math.h>
#include <stdio.h>
#include <time.h>

#include <angles/angles.h>
#include <std_srvs/Empty.h>
#include <movement.h>
#include <fcntl.h>

#include "mathematics.h"

#include "laser.hpp"
//#include <obstacle_detector.h>
//#include <figures/point.h>

//using namespace obstacle_detector;

void laser_pm_gpio(char val)
{
	int fd = open("/proc/driver/wifi-pm/power", O_WRONLY);
	char buf[] = {val};
	write(fd,buf,1);
	close(fd);
}

Laser::Laser():nh_(),is_ready_(false)
{
	scan_sub_ = nh_.subscribe("scan", 1, &Laser::laser_scan_cb, this);
	start_mator_cli_ = nh_.serviceClient<std_srvs::Empty>("start_motor");
	stop_mator_cli_ = nh_.serviceClient<std_srvs::Empty>("stop_motor");
	laser_pm_gpio('1');
	ROS_INFO("Laser init done!");

	start();
}

Laser::~Laser()
{
	stop();
//	scan_sub_ = nh_.subscribe("scan", 1, &Laser::laser_scan_cb, this);
	scan_sub_.shutdown();
	start_mator_cli_.shutdown();
	stop_mator_cli_.shutdown();
	nh_.shutdown();
	ROS_INFO("Laser stop");
}

void Laser::laser_scan_cb(const sensor_msgs::LaserScan::ConstPtr& scan)
{
	int i, count = 0;

	laser_scan_data = *scan;
	count = (int)((scan->angle_max - scan->angle_min) / scan->angle_increment);
//	ROS_INFO("%s %d: seq: %d\tangle_min: %f\tangle_max: %f\tcount: %d\tdist: %f", __FUNCTION__, __LINE__, msg->header.seq, msg->angle_min, msg->angle_max, count, msg->ranges[180]);

	is_ready_ = true;

}

bool Laser::laser_obstcal_detected(double distance, int angle, double range)
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
	//ROS_INFO("%s %d %f %f %f %f", __FUNCTION__, __LINE__, range_tmp, distance + 0.155, range_tmp / (distance + 0.155), atan(range_tmp / (distance + 0.155)));
	//ROS_INFO("%s %d: angle min: %f max: %f\tcount: %d\tdtor: %f\ttan: %f", __FUNCTION__, __LINE__, angle_min, angle_max, count, deg2rad((double) (angle % 360), 1),  atan(range_tmp / (distance + 0.155)));
	for (i = 0; found == false && i < count; i++) {
		tmp = laser_scan_data.angle_min + i * laser_scan_data.angle_increment;
		if (tmp > angle_min && tmp < angle_max && laser_scan_data.ranges[i] < distance + 0.155) {
			//ROS_INFO("%s %d: i: %d\ttmp: %f(%f, %f)\tdist: %f(%f)", __FUNCTION__, __LINE__, i, tmp, angle_min, angle_max, laser_scan_data.ranges[i], distance + 0.155);
			found = true;
		}
	}

	return found;
}

bool Laser::is_ready()
{
	return is_ready_;
}

void Laser::is_ready(bool val)
{
	is_ready_ = val;
}

void Laser::start(void)
{
	std_srvs::Empty empty;
	auto count_3s = 0;
	auto try_times = 3;
	bool  first_start = true;
	do
	{
		try_times--;
		if (! first_start)
		{
			ROS_INFO("lidar start false, power off and try again!!!");
			stop();
			sleep(1);
		}
		first_start = false;
		laser_pm_gpio('1');
		usleep(2000);
		ROS_INFO("start_lidar");
		start_mator_cli_.call(empty);
		count_3s = 300;//set reboot lidar time to 3 seconds
		is_ready_ = false;
		while (is_ready_ == false && --count_3s > 0 && !Stop_Event())
		{
			if (count_3s % 100 == 0)
				ROS_INFO("laser start not success yet, will try to restart after %d s", count_3s / 100);
			usleep(10000);
		}
	}while ((count_3s == 0 && try_times != 0) && !Stop_Event());

	ROS_INFO("start_motor: %d", is_ready_);
}

void Laser::stop(void){
	std_srvs::Empty empty;
	is_ready_ = false;
	ROS_INFO("stop_laser");
	stop_mator_cli_.call(empty);
	laser_pm_gpio('0');
}

