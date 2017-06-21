#include <math.h>
#include <stdio.h>
#include <time.h>

#include <angles/angles.h>
#include <std_srvs/Empty.h>
#include <movement.h>
#include <fcntl.h>

#include "mathematics.h"
#include "event_manager.h"
#include "laser.hpp"
//#include <obstacle_detector.h>
//#include <figures/point.h>

//using namespace obstacle_detector;

int read_line(int fd, char* buf){
	char temp;
		int i = 0;
		do
		{
			if (read(fd, &temp, 1) == 0)break;
			if (temp == '\n') break;
			buf[i++] = temp;
			printf("%c", temp);
		}while(1);
	printf("\n");
	return 1;
}

void laser_pm_gpio(char val)
{

	if(val != '1' && val != '0'){
		ROS_ERROR("ERROR, laser_pm_gpio has wrong param, default set 1");
		val = '1';
	}

	char w_buf[] = {val};
	char r_buf[28] = {0};

	int fd = open("/proc/driver/wifi-pm/power", O_RDWR);

/*		ap6212 : wl power state on
 *		ap6212 : wl power state off
 *		check	25th bit diff in " o 'n' " or "o 'f' f"
 */
	char r_val = (val == '1') ? 'n' : 'f';
	do
	{
		write(fd, w_buf, 1);
		usleep(200000);
		read_line(fd, r_buf);
//		ROS_INFO("laser gpio val %s", r_buf);
	}while(r_buf[25] != r_val);

	close(fd);
}

Laser::Laser():nh_(),is_ready_(0),is_scanDataReady_(false)
{
	scan_sub_ = nh_.subscribe("scan", 1, &Laser::scanCb, this);
	start_mator_cli_ = nh_.serviceClient<std_srvs::Empty>("start_motor");
	stop_mator_cli_ = nh_.serviceClient<std_srvs::Empty>("stop_motor");
	ROS_INFO("Laser init done!");

	start();
}

Laser::~Laser()
{
	stop();
//	scan_sub_ = nh_.subscribe("scan", 1, &Laser::scanCb, this);
//	scan_sub_.shutdown();
//	start_mator_cli_.shutdown();
//	stop_mator_cli_.shutdown();
//	nh_.shutdown();
	ROS_INFO("Laser stop");
}

void Laser::scanCb(const sensor_msgs::LaserScan::ConstPtr &scan)
{
	int i, count = 0;
	is_scanDataReady_ = false;
	laser_scan_data_ = *scan;
	count = (int)((scan->angle_max - scan->angle_min) / scan->angle_increment);
//	ROS_INFO("%s %d: seq: %d\tangle_min: %f\tangle_max: %f\tcount: %d\tdist: %f", __FUNCTION__, __LINE__, msg->header.seq, msg->angle_min, msg->angle_max, count, msg->ranges[180]);
	is_scanDataReady_ = true;
	isReady(1);
}

/*
 * @author mengshige1988@qq.com
 * @brief 
 * @param None
 * @return bool
 * */
bool Laser::isNewDataReady()
{
	return this->is_scanDataReady_;
}

bool Laser::laserObstcalDetected(double distance, int angle, double range)
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

	count = (int)((laser_scan_data_.angle_max - laser_scan_data_.angle_min) / laser_scan_data_.angle_increment);
	//ROS_INFO("%s %d %f %f %f %f", __FUNCTION__, __LINE__, range_tmp, distance + 0.155, range_tmp / (distance + 0.155), atan(range_tmp / (distance + 0.155)));
	//ROS_INFO("%s %d: angle min: %f max: %f\tcount: %d\tdtor: %f\ttan: %f", __FUNCTION__, __LINE__, angle_min, angle_max, count, deg2rad((double) (angle % 360), 1),  atan(range_tmp / (distance + 0.155)));
	for (i = 0; found == false && i < count; i++) {
		tmp = laser_scan_data_.angle_min + i * laser_scan_data_.angle_increment;
		if (tmp > angle_min && tmp < angle_max && laser_scan_data_.ranges[i] < distance + 0.155) {
			//ROS_INFO("%s %d: i: %d\ttmp: %f(%f, %f)\tdist: %f(%f)", __FUNCTION__, __LINE__, i, tmp, angle_min, angle_max, laser_scan_data_.ranges[i], distance + 0.155);
			found = true;
		}
	}

	return found;
}

/*
 * @author mengshige1988@qq.com
 * @brief according giveing angle return laser range data
 * @angle angle from 0 to 359
 * @return distance value (meter)
 * */
double Laser::getLaserDistance(uint16_t angle){
	if(angle >359 || angle < 0){
		ROS_WARN("%s,%d,angle should be in range 0 to 359,input angle = %u",__FUNCTION__,__LINE__,angle);
		return 0;
	}
	else{
		return this->laser_scan_data_.ranges[angle];
	}
}

double Laser::getLaserDistance(int begin, int end, double range)
{
	int		i, count;
	bool	found = false;
	double	angle_min, angle_max, tmp, range_tmp;
	double	laser_distance = 0;
	int		sum = 0;

	for (i = begin; i < end; i++) {//default:begin = 260, end =270
		if (laser_scan_data_.ranges[i] < range) {
			laser_distance = laser_scan_data_.ranges[i] + laser_distance;
			sum++;
		}
		//ROS_INFO("wall_distance = %lf, i = %d", laser_distance, i);
	}
	laser_distance = laser_distance / sum;
	ROS_INFO("laser_distance_averange = %lf, sum = %d",laser_distance, sum);
	//ROS_WARN("scan end");

	return laser_distance;
}
int8_t Laser::isReady()
{
	return is_ready_;
}

void Laser::isReady(uint8_t val)
{
	is_ready_ = val;
}

void Laser::start(void)
{
	std_srvs::Empty empty;
	time_t start_time = time(NULL);
	time_t stop_time = time(NULL);
	uint8_t try_times = 1;
	bool stopped = false;
	bool eh_status_now=false, eh_status_last=false;

	laser_pm_gpio('1');
	usleep(20000);
	ROS_WARN("%s %d: Start laser for the %d time.", __FUNCTION__, __LINE__, try_times);
	start_mator_cli_.call(empty);

	while (ros::ok() && try_times <= 3)
	{
		if (event_manager_check_event(&eh_status_now, &eh_status_last) == 1) {
			continue;
		}

		if (g_fatal_quit_event || g_key_clean_pressed)
		{
			isReady(0);
			ROS_WARN("%s %d: Laser starting interrupted, status: %d", __FUNCTION__, __LINE__, isReady());
			return;
		}

		if (stopped)
		{
			if (time(NULL) - stop_time >= 1)
			{
				laser_pm_gpio('1');
				usleep(20000);
				ROS_WARN("%s %d: Start laser for %d time.", __FUNCTION__, __LINE__, try_times);
				start_mator_cli_.call(empty);
				start_time = time(NULL);
				stopped = false;
			}
		}
		else
		{
			if (isReady() == 1)
			{
				ROS_WARN("%s %d: Laser start successed, status:%d.", __FUNCTION__, __LINE__, isReady());
				return;
			}

			if (time(NULL) - start_time >= 3)
			{
				try_times++;
				ROS_WARN("%s %d: Laser starting failed, power off before retry.", __FUNCTION__, __LINE__);
				stop();
				stop_time = time(NULL);
				stopped = true;
			}
		}
	}

	isReady(-1);
	ROS_ERROR("%s %d: Laser start timeout, status:%d.", __FUNCTION__, __LINE__, isReady());
}

void Laser::stop(void){
	std_srvs::Empty empty;
	isReady(0);
	is_scanDataReady_ = false;
	ROS_INFO("%s %d: Stop laser.", __FUNCTION__, __LINE__);
	stop_mator_cli_.call(empty);
	laser_pm_gpio('0');
}

