#include <math.h>
#include <stdio.h>
#include <time.h>
#include <cstring>
#include <angles/angles.h>
#include <std_srvs/Empty.h>
#include <movement.h>
#include <fcntl.h>
#include <vector>
#include <visualization_msgs/Marker.h>
#include <ros/ros.h>
#include <std_srvs/SetBool.h>
#include <pp/SetLidar.h>
#include <move_type.h>

#include "mathematics.h"
#include "event_manager.h"
#include "laser.hpp"
#include "core_move.h"
#include "robot.hpp"
#include "gyro.h"
boost::mutex scan_mutex_;
boost::mutex scan2_mutex_;
//float* Laser::last_ranges_ = NULL;
uint8_t Laser::is_ready_ = 0;
uint8_t Laser::is_scan2_ready_ = 0;
uint32_t new_laser_seq;


Laser::Laser():nh_()
{
	scan_sub_ = nh_.subscribe("scan", 1, &Laser::scanCb, this);
	scan_sub2_ = nh_.subscribe("scan2",1,&Laser::scanCb2, this);
	lidar_motor_cli_ = nh_.serviceClient<pp::SetLidar>("lidar_motor_ctrl");
	lidar_shield_detect_ = nh_.serviceClient<std_srvs::SetBool>("lidar_shield_ctrl");
	setScanReady(0);
	setScan2Ready(0);
	//last_ranges_ = new float[360];
	//memset(last_ranges_,0.0,360*sizeof(float));
	lidarMotorCtrl(ON);
}

Laser::~Laser()
{
	lidarShieldDetect(OFF);
	lidarMotorCtrl(OFF);
	setScanReady(0);
	setScan2Ready(0);
	scan_sub_.shutdown();
	scan_sub2_.shutdown();
	lidar_motor_cli_.shutdown();
	lidar_shield_detect_.shutdown();
	nh_.shutdown();
	//delete []last_ranges_;
	ROS_INFO("\033[35m" "%s %d: Laser stopped." "\033[0m", __FUNCTION__, __LINE__);
}

void Laser::scanCb(const sensor_msgs::LaserScan::ConstPtr &scan)
{
	int count = 0;
	boost::mutex::scoped_lock(scan_mutex_);
	laserScanData_ = *scan;
	count = (int)((scan->angle_max - scan->angle_min) / scan->angle_increment);

	//ROS_INFO("%s %d: seq: %d\tangle_min: %f\tangle_max: %f\tcount: %d\tdist: %f", __FUNCTION__, __LINE__, scan->header.seq, scan->angle_min, scan->angle_max, count, scan->ranges[180]);
	setScanReady(1);
}

void Laser::scanCb2(const sensor_msgs::LaserScan::ConstPtr &scan)
{
	int count = 0;
	boost::mutex::scoped_lock(scan2_mutex_);
	laserScanData_2_ = *scan;
	count = (int)((scan->angle_max - scan->angle_min) / scan->angle_increment);
	
	//ROS_INFO("%s %d: seq: %d\tangle_min: %f\tangle_max: %f\tcount: %d\tdist: %f", __FUNCTION__, __LINE__, scan->header.seq, scan->angle_min, scan->angle_max, count, scan->ranges[180]);
	setScan2Ready(1);
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
	angle_min = deg_to_rad((double) (angle % 360), 1) - atan(range_tmp / (distance + 0.155));
	angle_max = deg_to_rad((double) (angle % 360), 1) + atan(range_tmp / (distance + 0.155));

	scan_mutex_.lock();
	auto tmp_scan_data = laserScanData_;
	scan_mutex_.unlock();
	count = (int)((tmp_scan_data.angle_max - tmp_scan_data.angle_min) / tmp_scan_data.angle_increment);
	//ROS_INFO("%s %d %f %f %f %f", __FUNCTION__, __LINE__, range_tmp, distance + 0.155, range_tmp / (distance + 0.155), atan(range_tmp / (distance + 0.155)));
	//ROS_INFO("%s %d: angle min: %f max: %f\tcount: %d\tdtor: %f\ttan: %f", __FUNCTION__, __LINE__, angle_min, angle_max, count, deg_to_rad((double) (angle % 360), 1),  atan(range_tmp / (distance + 0.155)));
	for (i = 0; found == false && i < count; i++) {
		tmp = tmp_scan_data.angle_min + i * tmp_scan_data.angle_increment;
		if (tmp > angle_min && tmp < angle_max && tmp_scan_data.ranges[i] < distance + 0.155) {
			//ROS_INFO("%s %d: i: %d\ttmp: %f(%f, %f)\tdist: %f(%f)", __FUNCTION__, __LINE__, i, tmp, angle_min, angle_max, tmp_scan_data.ranges[i], distance + 0.155);
			found = true;
		}
	}

	return found;
	return found;
}

bool Laser::isNewDataReady()
{
	scan_mutex_.lock();
	if(new_laser_seq == 0) {
		new_laser_seq = laserScanData_.header.seq;
		scan_mutex_.unlock();
		return false;
	}
	if(laserScanData_.header.seq == new_laser_seq) {
		scan_mutex_.unlock();
		return false;
	}
	new_laser_seq = laserScanData_.header.seq;
	scan_mutex_.unlock();
	return true;
}

int8_t Laser::isScan2Ready()
{
	return is_scan2_ready_;
}
int8_t Laser::isScanReady()
{
	return is_ready_;
}

void Laser::setScanReady(uint8_t val)
{
	is_ready_ = val;
}

void Laser::setScan2Ready(uint8_t val)
{
	is_scan2_ready_ = val;
}
void Laser::lidarMotorCtrl(bool switch_)
{
	pp::SetLidar trigger;
	time_t start_time = time(NULL);
	uint8_t try_times = 0;
	bool eh_status_now = false, eh_status_last = false;
	bool request_sent = false;
	bool temp_switch_ = switch_;
	if(switch_){
		trigger.request.x_acc_init= robot::instance()->getInitXAcc();
		trigger.request.y_acc_init= robot::instance()->getInitYAcc();
		trigger.request.z_acc_init= robot::instance()->getInitZAcc();
	}
	while(ros::ok())
	{
		if (event_manager_check_event(&eh_status_now, &eh_status_last) == 1) {
			continue;
		}

		if (switch_ && (g_fatal_quit_event || g_key_clean_pressed || g_cliff_all_triggered)) // Interrupt only happens during starting laser.
		{
			if (!g_fatal_quit_event || g_cliff_all_triggered)
			{
				setScanReady(0);
				ROS_WARN("\033[34m" "%s %d: Laser starting interrupted, status: %d" "\033[0m", __FUNCTION__, __LINE__, isScanReady());
			}
			else
				setScanReady(-1);//open lidar fail ,stop process
			break;
		}

		if (!request_sent)
		{
			trigger.request.data = temp_switch_;
			request_sent = true;
			start_time = time(NULL);
			ROS_INFO("\033[35m" "%s %d: Send command %s!" "\033[0m", __FUNCTION__, __LINE__, temp_switch_?"ON":"OFF");
			if (lidar_motor_cli_.call(trigger)){
				ROS_INFO("\033[35m" "%s %d: Service response: %s" "\033[0m", __FUNCTION__, __LINE__,trigger.response.message.c_str());
				if (!temp_switch_){
					setScanReady(0);
					if (!switch_)
						// For stop command.
						break;
				}
			}
			else{
				ROS_ERROR("\033[35m" "%s %d: Service not received!" "\033[0m",__FUNCTION__,__LINE__);
				setScanReady(0);
				break;
			}
		}

		if (switch_ && isScanReady())
		{
			ROS_INFO("\033[32m" "%s %d: Scan topic received, start laser successed." "\033[0m", __FUNCTION__, __LINE__);
			break;
		}

		if (temp_switch_ && (time(NULL) - start_time > 4)){ // Time out
			try_times++;
			if(try_times > 2){
				//ROS_ERROR("laser.cpp, %s,%d,start lidar motor timeout",__FUNCTION__,__LINE__);
				g_fatal_quit_event = true;
				continue;
			}
			ROS_WARN("\033[34m" "%s %d: Start lidar motor timeout, retry for the %d times." "\033[0m", __FUNCTION__, __LINE__, try_times);
			temp_switch_ = false;
			request_sent = false;
		}
		else if (!temp_switch_ && (time(NULL) - start_time >= 1)){ // Wait for turning off.
			temp_switch_ = true;
			request_sent = false;
		}
	}
}

void Laser::lidarShieldDetect(bool switch_)
{
	std_srvs::SetBool trig;

	if(switch_)
		trig.request.data = true;
	else
		trig.request.data = false;

	lidar_shield_detect_.call(trig);
	ROS_INFO("\033[35m" "%s %d: Turn %s lidar shield detect %s." "\033[0m", __FUNCTION__, __LINE__, switch_?"on":"off", trig.response.success?"succeeded":"failed");
}

bool Laser::laserGetFitLine(int begin, int end, double range, double dis_lim, double *line_angle, double *distance)
{
	int		i, count;
	bool	found = false;
	double	angle_min, angle_max, tmp, range_tmp;
	double	laser_range_offset;
	double	laser_distance = 0;
	int		sum = 0;
	double	th;
	double 	a, b, c;
	//double	line_angle;
	Double_Point	New_Laser_Point;
	Laser_Point.clear();
	ROS_WARN("laserGetFitLine");
	scan_mutex_.lock();
	auto tmp_scan_data = laserScanData_;
	scan_mutex_.unlock();
	laser_range_offset = atan2(LIDAR_OFFSET_X, ROBOT_RADIUS) * 180 / PI;
	if (begin != 180)
		begin = begin - int(laser_range_offset);
	if (end != 180)
		end = end + int(laser_range_offset);
	ROS_INFO("laser_range_offset = %d, begin = %d, end = %d", int(laser_range_offset), begin, end);
	if (begin < 0 || end > 359) {
		ROS_ERROR("%s %d: laser_range_offset error! Return!", __FUNCTION__, __LINE__);
		return false;
	}
	for (int j = begin; j < end; j++) {//default:begin = 260, end =270
		i = j - int(LIDAR_THETA / 10);
		if (i > 359) {
			i = i - 360;
		} else if (i < 0) {
			i = i + 360;
		}
		//ROS_INFO("i = %d", i);
		if (tmp_scan_data.ranges[i] < 4) {
			th = i * 1.0;
			th = th + 180.0;
			New_Laser_Point.x = cos(th * PI / 180.0) * tmp_scan_data.ranges[i];
			New_Laser_Point.y = sin(th * PI / 180.0) * tmp_scan_data.ranges[i];
			coordinate_transform(&New_Laser_Point.x, &New_Laser_Point.y, LIDAR_THETA, LIDAR_OFFSET_X, LIDAR_OFFSET_Y);
			Laser_Point.push_back(New_Laser_Point);
		}
		//laser_distance = tmp_scan_data.ranges[i];
		//ROS_INFO("wall_distance = %lf, i = %d", laser_distance, i);
		//ROS_INFO("Laser_Point_x = %lf, Laser_Point_y = %lf, th = %lf, distance = %lf", New_Laser_Point.x, New_Laser_Point.y, th, tmp_scan_data.ranges[i]);
	}
	splitLine(Laser_Point, 0.01, 10);
	splitLine2nd(&Laser_Group, 0.01,10);
	mergeLine(&Laser_Group, 0.01);
	fitLineGroup(&Laser_Group, 0.1, dis_lim);
	//*line_angle = atan2(-a, b) * 180 / PI;
	Laser_Group.clear();
	if (!fit_line.empty()) {
		if (mt_is_left()) {
			*line_angle = atan2(0 - fit_line.begin()->A, fit_line.begin()->B) * 180 / PI;
			*distance = fabs(fit_line.begin()->C / (sqrt(fit_line.begin()->A * fit_line.begin()->A + fit_line.begin()->B * fit_line.begin()->B)));
		} else {
			*line_angle = atan2(0 - fit_line.back().A, fit_line.back().B) * 180 / PI;
			*distance = fabs(fit_line.back().C / (sqrt(fit_line.back().A * fit_line.back().A + fit_line.back().B * fit_line.back().B)));
		}
		//ROS_INFO("a = %lf, b = %lf, c = %lf", a, b, c);
		ROS_INFO("line_angle = %lf", *line_angle);
		return true;
	} else {
		ROS_INFO("no line to fit!");
		return false;
	}
}

bool Laser::lineFit(const std::vector<Double_Point> &points, double &a, double &b, double &c)
{
	int size = points.size();
	if(size < 2) {
		a = 0;
		b = 0;
		c = 0;
		return false;
	}
	double x_mean = 0;
	double y_mean = 0;

	for(int i = 0; i < size; i++) {
		x_mean += points[i].x;
		y_mean += points[i].y;
	}
	x_mean /= size;
	y_mean /= size;
	double Dxx = 0, Dxy = 0, Dyy = 0;

	for(int i = 0; i < size; i++) {
		Dxx += (points[i].x - x_mean) * (points[i].x - x_mean);
		Dxy += (points[i].x - x_mean) * (points[i].y - y_mean);
		Dyy += (points[i].y - y_mean) * (points[i].y - y_mean);
	}

	double lambda = ( (Dxx + Dyy) - sqrt( (Dxx - Dyy) * (Dxx - Dyy) + 4 * Dxy * Dxy) ) / 2.0;
	double den = sqrt( Dxy * Dxy + (lambda - Dxx) * (lambda - Dxx) );

	//ROS_INFO("cells.size = %d, den = %lf", size, den);
	if(fabs(den) < 1e-5) {
		if( fabs(Dxx / Dyy - 1) < 1e-5) {
			ROS_INFO("line fit failed!");
			return false;
		}
		else {
			a = 1;
			b = 0;
			c = - x_mean;
		}
	}
	else {
		a = Dxy / den;
		b = (lambda - Dxx) / den;
		c = - a * x_mean - b * y_mean;
	}
	return true;
}

bool Laser::splitLine(const std::vector<Double_Point> &points, double consec_lim, int points_count_lim) {
	int points_size = points.size();
	double distance;
	std::vector<Double_Point> new_line;
	for(int i = 0; i < (points_size - 1); i++) {
		//ROS_INFO("i = %d", i);
		distance = sqrt((points[i].x - points[i+1].x) * (points[i].x - points[i+1].x) + (points[i].y - points[i+1].y) * (points[i].y - points[i+1].y));
		//ROS_INFO("distance = %lf", distance);
		if (distance <= consec_lim) {
			new_line.push_back(points[i]);
			if (i == (points_size - 2)) {
				//ROS_INFO("%s %d: loop end, push back the line", __FUNCTION__, __LINE__);
				Laser_Group.push_back(new_line);
				new_line.clear();
			}
		} else {
			//ROS_INFO("%s %d: distance = %lf", __FUNCTION__, __LINE__, distance);
			//ROS_INFO("split!");
			Laser_Group.push_back(new_line);
			new_line.clear();
			new_line.push_back(points[i]);
		}
	}
	for (std::vector<std::vector<Double_Point> >::iterator iter = Laser_Group.begin(); iter != Laser_Group.end();) {
		if (iter->size() < points_count_lim) {
			iter = Laser_Group.erase(iter);
		} else {
			++iter;
		}
	}
	ROS_DEBUG("splitLine : Laser_Group.size = %lu", Laser_Group.size());
	return true;
}

bool Laser::splitLine2nd(std::vector<std::vector<Double_Point> > *groups, double t_lim, int points_count_lim) {
	int groups_size = (*groups).size();
	int points_size;
	int erased_size;
	double a, b, c;
	double x1, y1, x2, y2;
	double t, t_max;
	int	points_index_max;
	bool end_iterate_flag = true;
	std::vector<int> groups_erased_index;
	std::vector<Double_Point> new_line;
	std::vector<std::vector<Double_Point> > new_group;

	ROS_DEBUG("Laser::splitLine2nd");
	groups_erased_index.clear();
	new_line.clear();
	new_group.clear();

	/*loop for lines in groups*/
	for (std::vector<std::vector<Double_Point> >::iterator iter = (*groups).begin(); iter != (*groups).end(); ++iter) {
		x1 = iter->begin()->x;
		y1 = iter->begin()->y;
		x2 = (iter->end() - 1)->x;
		y2 = (iter->end() - 1)->y;
		points_size = iter->size();
		//ROS_INFO("points_size = %d", points_size);
		//ROS_INFO("next line");
		end_iterate_flag = true;
		if (x1 != x2 && y1 != y2) {
			a = y2 - y1;
			b = x1 - x2;
			c= x2 * y1 - x1 * y2;
			t = 0;
			t_max = 0;
			//ROS_INFO("x1 = %lf, y1 = %lf, x2 = %lf, y2 = %lf, a = %lf, b = %lf, c = %lf", x1, y1, x2, y2, a, b, c);

			/*loop for finding the t_max*/
			//ROS_INFO("loop for finding the t_max");
			for (int j = 0; j < points_size; j++) {
				t = fabs((a * (iter->begin() + j)->x + b * (iter->begin() +j)->y + c) / sqrt(a * a + b * b));
				//ROS_INFO("t = %lf", t);
				//ROS_INFO("points_index = %d, x = %lf, y = %lf",j ,(iter->begin() + j)->x, (iter->begin() + j)->y);
				if (t >= t_max) {
					t_max = t;
					points_index_max = j;
					//ROS_INFO("t >= t_max,points_index_max = %d", points_index_max);
				}
			}
			//ROS_INFO("t_max = %lf", t_max);

			/*if t_max > t_lim, then split into 2 lines*/
			//ROS_INFO("loop for split");
			if (t_max > t_lim) {
				end_iterate_flag &= false;
				//ROS_INFO("t_tmax > t_lim,end_iterate_flag = %d", end_iterate_flag);
				int index = std::distance((*groups).begin(), iter);
				groups_erased_index.push_back(index);
				//ROS_INFO("push_back erased index = %d",index);
				//ROS_INFO("points_size = %d", points_size);
				for (int j = 0; j < points_size; j++) {
					new_line.push_back(*(iter->begin() + j));
					//ROS_INFO("j = %d", j);
					if (j == points_index_max) {
						if (j == (points_size - 1)) {
							//ROS_INFO("split the rearest point");
							new_line.pop_back();
							//ROS_INFO("new_line.size() = %d",new_line.size());
							new_group.push_back(new_line);
							new_line.clear();
						} else {
							//ROS_INFO("split front");
							//ROS_INFO("new_line.size() = %d",new_line.size());
							new_group.push_back(new_line);
							new_line.clear();
						}
					} else if ((j > points_index_max) && (j == (points_size - 1))) {
						//ROS_INFO("split rear");
						//ROS_INFO("new_line.size() = %d",new_line.size());
						new_group.push_back(new_line);
						new_line.clear();
					}
					//ROS_INFO("x1 = %lf, y1 = %lf, x2 = %lf, y2 = %lf, a = %lf, b = %lf, c = %lf", x1, y1, x2, y2, a, b, c);
				}
			} else {//else push_back the line into new_group
				end_iterate_flag &= true;
				//ROS_INFO("t_max < t_lim,end_iterate_flag = %d", end_iterate_flag);
				/*for (int j = 0; j < points_size; j++) {
					new_line.push_back(*(iter->begin() + j));
					if (j == (points_size - 1)) {
						new_group.push_back(new_line);
						new_line.clear();
					}
				}*/
			}
		}
	}

	ROS_DEBUG("1Laser_Group.size = %lu", (*groups).size());

	/*loop for erasing the old lines which are unsplit and push_back lines in new_group into groups*/
	ROS_DEBUG("loop for erasing the old lines which are unsplit");
	erased_size = groups_erased_index.size();
	if (!groups_erased_index.empty()) {
		for (int i = 0; i < erased_size; i++) {
			ROS_DEBUG("erase the unsplit line! index = %d", i);
			std::vector<std::vector<Double_Point> >::iterator iter = (*groups).begin() + (groups_erased_index[i] + i);
			(*groups).erase(iter);
			ROS_DEBUG("2Laser_Group.size = %lu", (*groups).size());

			for (int j = 0; j < 2; j++) {
				ROS_DEBUG("push_back lines in new_group");
				ROS_DEBUG("new_group.begin->size() = %lu", (new_group.begin()->size()));
				ROS_DEBUG("new_group.size() = %lu", (new_group.size()));
				(*groups).insert((*groups).begin() + (groups_erased_index[i] + j), *(new_group.begin() + j));
				//(*groups).push_back(new_group.begin());
				//new_group.pop_back();
				ROS_DEBUG("3Laser_Group.size = %lu", (*groups).size());
			}
		}
	}

	ROS_DEBUG("new_group.size = %lu", new_group.size());
	groups_erased_index.clear();
	new_group.clear();

	/*loop for erasing the line which size is less than points_count_lim*/
	ROS_DEBUG("loop for erasing the line which size is less than points_count_lim");
	for (std::vector<std::vector<Double_Point> >::iterator iter = (*groups).begin(); iter != (*groups).end();) {
		if (iter->size() < points_count_lim) {
			iter = (*groups).erase(iter);
			//ROS_INFO("iter->size() = %d", iter->size());
		} else {
			++iter;
		}
	}

	ROS_DEBUG("4Laser_Group.size = %lu", (*groups).size());

	/*if the lines still can be splited, iterate to split it*/
	if (end_iterate_flag == false) {
		//ROS_INFO("iterate!");
		splitLine2nd(&Laser_Group, 0.01,10);
	}


	//Laser_Group.clear();
	//Laser_Group_2nd.clear();

	return true;
}

bool Laser::mergeLine(std::vector<std::vector<Double_Point> > *groups, double t_lim) {
	double a, b, c;
	double x1, y1, x2, y2;
	int points_size, points_size_2nd;
	double t, t_max = 0;
	int erased_size;
	std::vector<int> merge_index;
	std::vector<Double_Point> new_line;
	std::vector<std::vector<Double_Point> > new_group;
	new_line.clear();
	new_group.clear();
	if (!(*groups).empty()) {
		for (std::vector<std::vector<Double_Point> >::iterator iter = (*groups).begin(); iter != (*groups).end() - 1; ++iter) {
			x1 = iter->begin()->x;
			y1 = iter->begin()->y;
			x2 = ((iter + 1)->end() - 1)->x;
			y2 = ((iter + 1)->end() - 1)->y;
			if (x1 != x2 && y1 != y2) {
				a = y2 - y1;
				b = x1 - x2;
				c= x2 * y1 - x1 * y2;
				//ROS_INFO("x1 = %lf, y1 = %lf, x2 = %lf, y2 = %lf, a = %lf, b = %lf, c = %lf", x1, y1, x2, y2, a, b, c);
				//ROS_INFO("%s %d: x1 = %lf, y1 = %lf, x2 = %lf, y2 = %lf, a = %lf, b = %lf, c = %lf", __FUNCTION__, __LINE__, x1, y1, x2, y2, a, b, c);
				points_size = iter->size();
				points_size_2nd = (iter + 1)->size();
				/*loop for checking the first line*/
				for (int j = 0; j < points_size; j++) {
					t = fabs((a * (iter->begin() + j)->x + b * (iter->begin() +j)->y + c) / sqrt(a * a + b * b));
					//ROS_INFO("t = %lf", t);
					//ROS_INFO("points_index = %d, x = %lf, y = %lf",j ,(iter->begin() + j)->x, (iter->begin() + j)->y);
					if (t >= t_max) {
						t_max = t;
						//ROS_INFO("new t_max = %lf", t_max);
						//points_index_max = j;
						//ROS_INFO("t >= t_max,points_index_max = %d", points_index_max);
					}
				}
				/*loop for checking the second line*/
				for (int j = 0; j < points_size_2nd; j++) {
					t = fabs((a * ((iter + 1)->begin() + j)->x + b * ((iter + 1)->begin() +j)->y + c) / sqrt(a * a + b * b));
					//ROS_INFO("t = %lf", t);
					//ROS_INFO("points_index = %d, x = %lf, y = %lf",j ,((iter + 1)->begin() + j)->x, ((iter + 1)->begin() + j)->y);
					if (t >= t_max) {
						t_max = t;
						//ROS_INFO("new t_max = %lf", t_max);
						//points_index_max = j;
						//ROS_INFO("t >= t_max,points_index_max = %d", points_index_max);
					}
				}
				//ROS_INFO("merge: t_max = %lf", t_max);
				if (t_max < t_lim) {
					int index = std::distance((*groups).begin(), iter);
					merge_index.push_back(index);
					ROS_DEBUG("5Laser_Group.size = %lu", (*groups).size());
					ROS_DEBUG("merge! index = %d and %d", index, (index + 1));
				}
			}
		}
	}

	/*do merge*/
	if (!merge_index.empty()) {
		int loop_count = 0;
		for (std::vector<int>::iterator m_iter = merge_index.begin(); m_iter != merge_index.end(); ++m_iter) {
			std::vector<std::vector<Double_Point> >::iterator iter = (*groups).begin() + (*m_iter);
			points_size = iter->size();
			points_size_2nd = (iter + 1)->size();
			for (int j = 0; j < points_size; j++) {
				new_line.push_back(*(iter->begin() + j));
			}
			for (int j = 0; j < points_size_2nd; j++) {
				new_line.push_back(*((iter + 1)->begin() + j));
			}
			(*groups).erase(iter - loop_count);
			ROS_DEBUG("merge : erase front");
			ROS_DEBUG("Laser_Group.size = %lu", (*groups).size());
			(*groups).erase(iter + 1 - loop_count);
			ROS_DEBUG("merge : erase rear");
			ROS_DEBUG("5Laser_Group.size = %lu", (*groups).size());
			(*groups).insert((*groups).begin() + (*m_iter) - loop_count, new_line);
			ROS_DEBUG("merge : insert");
			ROS_DEBUG("5Laser_Group.size = %lu", (*groups).size());
			loop_count++;
		}
	}
	ROS_INFO("pub line marker");
	pubLineMarker(&Laser_Group);
	return true;
}

void Laser::pubLineMarker(std::vector<std::vector<Double_Point> > *groups) {
	int points_size;
	visualization_msgs::Marker line_marker;
	line_marker.ns = "line_marker";
	line_marker.id = 0;
	line_marker.type = visualization_msgs::Marker::SPHERE_LIST;
	line_marker.action= 0;//add
	line_marker.lifetime=ros::Duration(0);
	line_marker.scale.x = 0.03;
	line_marker.scale.y = 0.03;
	line_marker.scale.z = 0.03;
	line_marker.color.r = 0.0;
	line_marker.color.g = 1.0;
	line_marker.color.b = 0.0;
	line_marker.color.a = 1.0;
	line_marker.header.frame_id = "/base_link";
	line_marker.header.stamp = ros::Time::now();
	laser_points_.x = 0.0;
	laser_points_.y = 0.0;
	laser_points_.z = 0.0;

	/*line_marker.pose.position.x = 0.0;
	line_marker.pose.position.y = 0.0;
	line_marker.pose.position.z = 0.0;
	line_marker.pose.orientation.x = 0.0;
	line_marker.pose.orientation.y = 0.0;
	line_marker.pose.orientation.z = 0.0;
	line_marker.pose.orientation.w = 1.0;*/
	if (!(*groups).empty()) {
		for (std::vector<std::vector<Double_Point> >::iterator iter = (*groups).begin(); iter != (*groups).end(); ++iter) {
			/*x1 = iter->begin()->x;
			y1 = iter->begin()->y;
			x2 = (iter->end() - 1)->x;
			y2 = (iter->end() - 1)->y;*/
			points_size = iter->size();
			for (int j = 0; j < points_size; j++) {
				//line_marker.pose.position.x = (iter->begin() + j)->x;
				//line_marker.pose.position.y = (iter->begin() + j)->y;
				laser_points_.x = (iter->begin() + j)->x;
				laser_points_.y = (iter->begin() + j)->y;
				//ROS_INFO("laser_points_.x = %lf laser_points_.y = %lf",laser_points_.x, laser_points_.y);
				line_marker.points.push_back(laser_points_);
			}
		}
		line_marker_pub.publish(line_marker);
		line_marker.points.clear();
	} else {
		line_marker.points.clear();
		line_marker_pub.publish(line_marker);
	}
}


bool Laser::fitLineGroup(std::vector<std::vector<Double_Point> > *groups, double t_lim, double dis_lim) {
	double 	a, b, c;
	double	x_0, y_0;
	double line_angle;
	const double L =0.2727716;
	int	loop_count = 0;
	LineABC	new_fit_line;
	fit_line.clear();
	if (!(*groups).empty()) {
		for (std::vector<std::vector<Double_Point> >::iterator iter = (*groups).begin(); iter != (*groups).end(); ++iter) {
			lineFit((*iter), a, b, c);
			//line_angle = atan2(-a, b) * 180 / PI;
			new_fit_line.A = a;
			new_fit_line.B = b;
			new_fit_line.C = c;
			x_0 = 0 - c / a;
			y_0 = 0 - c / b;
			line_angle = atan2(y_0, 0 - x_0) * 180 / PI;
			ROS_INFO("a = %lf, b = %lf, c = %lf", a, b, c);
			ROS_INFO("x_0 = %lf, y_0 = %lf", x_0, y_0);
			/*erase the lines which are far away from the robot*/
			//double x_l = fabs(c / a);
			//double y_l = fabs(c / b);
			double dis = fabs(c / (sqrt(a * a + b * b)));
			//if ((x_l > L) && (y_l > L)) {
			if (dis > dis_lim || dis < 0.167 || x_0 < 0) {
				//ROS_INFO("the line is too far away from robot. x_l = %lf, y_l = %lf", x_l, y_l);
				ROS_INFO("the line is too far away from robot. dis = %lf", dis);
				continue;
			}
			fit_line.push_back(new_fit_line);
			//pubFitLineMarker(a, b, c, -0.5, 0.5);
			pubFitLineMarker(a, b, c, iter->begin()->y, (iter->end() - 1)->y);
			//ROS_INFO("iter->begin()->y = %lf, (iter->end() - 1)->y= %lf",iter->begin()->y, (iter->end() - 1)->y);
			ROS_INFO("%s %d: line_angle%d = %lf", __FUNCTION__, __LINE__, loop_count, line_angle);
			loop_count++;
		}
	} else {
		fit_line_marker.points.clear();
		fit_line_marker_pub.publish(fit_line_marker);
	}
	fit_line_marker.points.clear();
	return true;
}

void Laser::pubFitLineMarker(double a, double b, double c, double y1, double y2) {
	double x1, x2;
	//visualization_msgs::Marker fit_line_marker;
	fit_line_marker.ns = "fit_line_marker";
	fit_line_marker.id = 0;
	//fit_line_marker.type = visualization_msgs::Marker::SPHERE_LIST;
	fit_line_marker.type = 5;
	fit_line_marker.action= 0;//add
	fit_line_marker.lifetime=ros::Duration(0);
	fit_line_marker.scale.x = 0.03;
	fit_line_marker.scale.y = 0.03;
	fit_line_marker.scale.z = 0.03;
	fit_line_marker.color.r = 0.0;
	fit_line_marker.color.g = 1.0;
	fit_line_marker.color.b = 0.0;
	fit_line_marker.color.a = 1.0;
	fit_line_marker.header.frame_id = "/base_link";
	fit_line_marker.header.stamp = ros::Time::now();
	laser_points_.x = 0.0;
	laser_points_.y = 0.0;
	laser_points_.z = 0.0;

	x1 = (0 - b * y1 - c) / a;
	x2 = (0 - b * y2 - c) / a;

	laser_points_.x = x1;
	laser_points_.y = y1;
	//ROS_INFO("Fit line laser_points_.x = %lf laser_points_.y = %lf",laser_points_.x, laser_points_.y);
	fit_line_marker.points.push_back(laser_points_);
	laser_points_.x = x2;
	laser_points_.y = y2;
	//ROS_INFO("Fit line laser_points_.x = %lf laser_points_.y = %lf",laser_points_.x, laser_points_.y);
	fit_line_marker.points.push_back(laser_points_);
	fit_line_marker_pub.publish(fit_line_marker);
	//fit_line_marker.cells.clear();
}

/*
 * @author mengshige1988@qq.com
 * @brief according giveing angle return laser range data
 * @angle angle from 0 to 359
 * @return distance value (meter)
 * */

double Laser::getLaserDistance(uint16_t angle){
	ROS_INFO("%s,%d,input angle = %u",__FUNCTION__,__LINE__,angle);
	if(angle >359 || angle < 0){
		ROS_WARN("%s,%d,angle should be in range 0 to 359,input angle = %u",__FUNCTION__,__LINE__,angle);
		return 0;
	}
	else{
		scan_mutex_.lock();
		auto tmp_scan_data = laserScanData_;
		scan_mutex_.unlock();
		return tmp_scan_data.ranges[angle];
	}
}

/*----set laser marker according to direction-----*/
static uint8_t setLaserMarkerAcr2Dir(double X_MIN,double X_MAX,int angle_from,int angle_to,int dx,int dy,const sensor_msgs::LaserScan *scan_range,uint8_t *laser_status,uint8_t obs_status)
{
	double x,y,th;
	const	double Y_MIN = 0.140;//0.167
	const	double Y_MAX = 0.237;//0.279
	int count = 0;
	uint8_t ret = 0;
	int i =angle_from;
	for (int j = angle_from; j < angle_to; j++) {
		i = j>359? j-360:j;
		if (scan_range->ranges[i] < 4) {
			th = i*1.0 + 180.0;
			if(j >= 314 && j < 404){//314~44
				x = -cos(th * PI / 180.0) * scan_range->ranges[i];
				//y = -sin(th * PI / 180.0) * scan_range->ranges[i];
				if (x > X_MIN && x < X_MAX ) {
					count++;
				}
			}
			else if( j >= 44 && j < 134){
				//x = cos(th * PI / 180.0) * scan_range->ranges[i];
				y = -sin(th * PI / 180.0) * scan_range->ranges[i];
				if (y > Y_MIN && y < Y_MAX ) {
					count++;
				}
			}
			else if( j >= 134 && j < 224){
				x = cos(th * PI / 180.0) * scan_range->ranges[i];
				//y = sin(th * PI / 180.0) * scan_range->ranges[i];
				if (x > X_MIN && x < X_MAX ) {
					count++;
				}
			}
			else if( j >= 224 && j < 314){
				//x = cos(th * PI / 180.0) * scan_range->ranges[i];
				y = sin(th * PI / 180.0) * scan_range->ranges[i];
				if (y > Y_MIN && y < Y_MAX ) {
					count++;
				}
			}
		}
	}
	if (count > 10) {
		int32_t x_tmp,y_tmp;
		cm_world_to_point(gyro_get_angle(), CELL_SIZE * dy, CELL_SIZE * dx, &x_tmp, &y_tmp);
		if (map_get_cell(MAP, count_to_cell(x_tmp), count_to_cell(y_tmp)) != BLOCKED_BUMPER)
		{
			ROS_INFO("\033[36mlaser marker : (%d,%d)\033[0m",count_to_cell(x_tmp),count_to_cell(y_tmp));
			map_set_cell(MAP, x_tmp, y_tmp, BLOCKED_LASER);
		}
		ret = 1;
		*laser_status |= obs_status;
	}
	return ret;

}

static uint8_t checkCellTrigger(double X_MIN, double X_MAX, const sensor_msgs::LaserScan *scan_range, uint8_t *laser_status, bool is_wall_follow)
{
//	ROS_INFO("  %s,%d" __FUNCTION__,__LINE__);
	double x, y, th;
	int dx, dy;
	const	double Y_MIN = 0.140;//0.167
	const	double Y_MAX = 0.237;//0.279
	const	double TRIGGER_RANGE = X_MAX;//0.278
	int count = 0;
	uint8_t ret = 0;
	int	count_array[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

	for (int i = 0; i < 360; i++) {
		if (scan_range->ranges[i] < 4) {
			th = i*1.0 + 180.0;
			x = cos(th * PI / 180.0) * scan_range->ranges[i];
			y = sin(th * PI / 180.0) * scan_range->ranges[i];
			coordinate_transform(&x, &y, LIDAR_THETA, LIDAR_OFFSET_X, LIDAR_OFFSET_Y);
			//front
			if (x > ROBOT_RADIUS && x <  TRIGGER_RANGE) {
				//middle
				if (y > -0.056 && y < 0.056) {
					count_array[0]++;
				}
				//left
				if (y > 0.056 && y < 0.168) {
					count_array[1]++;
				}
				//right
				if (y > -0.168 && y < -0.056) {
					count_array[2]++;
				}
			}
			//back
			if (x < (0 - ROBOT_RADIUS) && x > (0 - TRIGGER_RANGE)) {
				//middle
				if (y > -0.056 && y < 0.056) {
					count_array[3]++;
				}
				//left
				if (y > 0.056 && y < 0.168) {
					count_array[4]++;
				}
				//right
				if (y > -0.168 && y < -0.056) {
					count_array[5]++;
				}
			}
			//left
			if (y > ROBOT_RADIUS && y <  TRIGGER_RANGE) {
				//middle
				if (x > -0.056 && x < 0.056) {
					count_array[6]++;
				}
				//front
				if (x > 0.056 && x < 0.168) {
					count_array[7]++;
				}
				//back
				/*
				if (x > -0.168 && x < -0.056) {
					count_array[8]++;
				}*/
			}
			//right
			if (y < (0 - ROBOT_RADIUS) && y >  (0 - TRIGGER_RANGE)) {
				//middle
				if (x > -0.056 && x < 0.056) {
					count_array[9]++;
				}
				//front
				if (x > 0.056 && x < 0.168) {
					count_array[10]++;
				}
				//back
				/*
				if (x > -0.168 && x < -0.056) {
					count_array[11]++;
				}*/
			}
		}
	}

	std::string msg = "";
	std::string direction_msg = "";
	for (int i = 0; i < 12; i++) {
		if (count_array[i] > 10) {
			int32_t x_tmp,y_tmp;
			switch(i) {
				case 0 : {
					dx = 2;
					dy = 0;
					*laser_status |= Status_Front_OBS;
					direction_msg = "front middle";
					break;
				}
				case 1 : {
					dx = 2;
					dy = 1;
					*laser_status |= Status_Left_OBS;
					direction_msg = "front left";
					break;
				}
				case 2 : {
					dx = 2;
					dy = -1;
					*laser_status |= Status_Right_OBS;
					direction_msg = "front right";
					break;
				}
				case 3 : {
					dx = -2;
					dy = 0;
					direction_msg = "back middle";
					break;
				}
				case 4 : {
					dx = -2;
					dy = 1;
					direction_msg = "back left";
					break;
				}
				case 5 : {
					dx = -2;
					dy = -1;
					direction_msg = "back right";
					break;
				}
				case 6 : {
					dx = 0;
					dy = 2;
					direction_msg = "left middle";
					break;
				}
				case 7 : {
					dx = 1;
					dy = 2;
					direction_msg = "left front";
					break;
				}
				case 8 : {
					dx = -1;
					dy = 2;
					direction_msg = "left back";
					break;
				}
				case 9 : {
					dx = 0;
					dy = -2;
					direction_msg = "right middle";
					break;
				}
				case 10 : {
					dx = 1;
					dy = -2;
					direction_msg = "right front";
					break;
				}
				case 11 : {
					dx = -1;
					dy = -2;
					direction_msg = "right back";
					break;
				}
			}
			cm_world_to_point(gyro_get_angle(), CELL_SIZE * dy, CELL_SIZE * dx, &x_tmp, &y_tmp);
			auto cell_status = map_get_cell(MAP, count_to_cell(x_tmp), count_to_cell(y_tmp));
			if (cell_status != BLOCKED_BUMPER && cell_status != BLOCKED_OBS)
			{
				//ROS_INFO("    \033[36mlaser marker : (%d,%d), i = %d, dx = %d, dy = %d.\033[0m",count_to_cell(x_tmp),count_to_cell(y_tmp), i, dx, dy);
				msg += direction_msg + "(" + std::to_string(count_to_cell(x_tmp)) + ", " + std::to_string(count_to_cell(y_tmp)) + ")";
				map_set_cell(MAP, x_tmp, y_tmp, BLOCKED_LASER); //BLOCKED_OBS);
			}
			if (is_wall_follow) {
				if (i == 0)
					ret = 1;
			} else {
				if (i <= 2) {
					ret = 1;
				}
			}
		}
	}
	if (!msg.empty())
		ROS_INFO("%s %d: \033[36mlaser marker: %s.\033[0m", __FUNCTION__, __LINE__, msg.c_str());
	return ret;

}

uint8_t Laser::laserMarker(bool is_mark,double X_MIN,double X_MAX)
{
	//double	angle_min, angle_max, tmp, range_tmp;
	//double	laser_distance = 0;
	scan_mutex_.lock();
	auto tmp_scan_data = laserScanData_;
	scan_mutex_.unlock();
	static  uint32_t seq = tmp_scan_data.header.seq;
	bool	is_triggered = 0;
	uint8_t laser_status;
	//ROS_ERROR("is_skip = %d", is_skip);
	//ROS_INFO("laserMarker");
	//ROS_INFO("seq = %d", tmp_scan_data.header.seq);
	if (tmp_scan_data.header.seq == seq) {
		//ROS_ERROR("laser seq still same, quit!seq = %d", tmp_scan_data.header.seq);
		return 0;
	}
	seq = tmp_scan_data.header.seq;
	if (!is_mark)
		return 0;
	//ROS_ERROR("2 : is_skip = %d", is_skip);
	/*if (is_skip) {
		//is_skip = 0;
		//ROS_INFO("skip!");
		return false;
	}*/
	//ROS_INFO("new laser! seq = %d", tmp_scan_data.header.seq);
	is_triggered |= checkCellTrigger(X_MIN, X_MAX, &tmp_scan_data, &laser_status, mt_is_follow_wall());

	if (is_triggered) {
		return laser_status;
	} else {
		return 0;
	}
}

uint8_t Laser::isRobotSlip()
{
	static int16_t slip_count= 0;
	static uint16_t seq_count = 0;
	static uint32_t seq=0;
	static uint8_t last_ranges_init = 0;
	static std::vector<float> last_ranges ;
	const float PERCENT = 0.85;//85%

	const float acur1 = 0.07;//accuracy 1 ,in meters
	const float acur2 = 0.05;//accuracy 2 ,in meters
	const float acur3 = 0.03;//accuracy 3 ,in meters
	const float acur4 = 0.01;//accuracy 4 ,in meters

	const float dist1 = 3.5;//range distance 1 ,in meters
	const float dist2 = 2.5;//range distance 2
	const float dist3 = 1.5;//range distance 3
	const float dist4 = 0.5;//range distance 4

	const int COUNT = 6;//count number before trigger stuck
	const float WSL = 0.04; //wheel speed limit ,in m/s
	uint16_t same_count = 0;
	uint8_t ret = 0;
	uint16_t tol_count = 0;
	scan2_mutex_.lock();
	auto tmp_scan_data = laserScanData_2_;
	scan2_mutex_.unlock();
	if(g_robot_slip_enable && seq != tmp_scan_data.header.seq && isScan2Ready() && ( absolute(robot::instance()->getLeftWheelSpeed()) >= WSL || absolute(robot::instance()->getRightWheelSpeed()) >= WSL ) )
	{
		seq = tmp_scan_data.header.seq;
		if(last_ranges_init == 0){//for the first time
			last_ranges_init = 1;
			last_ranges = tmp_scan_data.ranges;
			return ret;
		}
		for(int i =0;i<=359;i=i+2){
			if(tmp_scan_data.ranges[i] < dist1){
				tol_count++;
				if(laserScanData_2_.ranges[i] >dist2 && laserScanData_2_.ranges[i] < dist1){//
					if(absolute( laserScanData_2_.ranges[i] - last_ranges[i] ) <= acur1 ){
						same_count++;
					}
				} 
				else if(laserScanData_2_.ranges[i] >dist3 && laserScanData_2_.ranges[i] < dist2){//
					if(absolute( laserScanData_2_.ranges[i] - last_ranges[i] ) <= acur2 ){
						same_count++;
					}
				}
				else if(laserScanData_2_.ranges[i] >dist4 && laserScanData_2_.ranges[i] < dist3){//
					if(absolute( laserScanData_2_.ranges[i] - last_ranges[i] ) <= acur3 ){
						same_count++;
					}
				}
				else if(laserScanData_2_.ranges[i] <= dist4){
					if(absolute( laserScanData_2_.ranges[i] - last_ranges[i] ) <= acur4 ){
						same_count++;
					}
				}
			}
		}
		if(++seq_count >2){//store last ranges after 2 sequance
			seq_count=0;
			last_ranges = tmp_scan_data.ranges;
		}
		//ROS_INFO("\033[1;45;37msame_count %d,tol_count %d,\033[0m",same_count, tol_count );
		if((same_count*1.0)/(tol_count*1.0) >= PERCENT){
			if(++slip_count>= COUNT){
				slip_count = 0;
				ret = 1;
			}
		}
		else{
			//slip_count= (slip_count>0)? slip_count - 1: 0;
			slip_count = 0;
		}
	}
	else if(!g_robot_slip_enable)
	{
		seq = 0;
		seq_count = 0;
		slip_count= 0;
		last_ranges_init = 0;
	}
	return ret;
}

/*
 * @author Alvin Xie
 * @brief make use of lidar to judge x+ or x- is more closer to the wall
 * @return the closer direction of x to the wall
 * if x+ is closer, return 1, else return 0
 * */
int Laser::compLaneDistance()
{
	int ret = 0;
	static  uint32_t seq = 0;
	double x,y,th,x1,y1;
	int angle_from, angle_to;
	double x_front_min = 4;
	double x_back_min = 4;

	scan_mutex_.lock();
	auto tmp_scan_data = laserScanData_;
	scan_mutex_.unlock();

	if (tmp_scan_data.header.seq == seq) {
		//ROS_WARN("laser seq still same, quit!seq = %d", tmp_scan_data.header.seq);
		return 0;
	}
	ROS_INFO("compLaneDistance");
	seq = tmp_scan_data.header.seq;
	int cur_angle = gyro_get_angle() / 10;
#if 0
	angle_from = 149 - cur_angle;
	angle_to = 210 - cur_angle;
	ROS_INFO("cur_angle = %d", cur_angle);
	for (int j = angle_from; j < angle_to; j++) {
		int i = j>359? j-360:j;
		if (tmp_scan_data.ranges[i] < 4) {
			th = i*1.0 + 180.0;
			x = cos(th * PI / 180.0) * tmp_scan_data.ranges[i];
			y = sin(th * PI / 180.0) * tmp_scan_data.ranges[i];
			x1 = x * cos(0 - cur_angle * PI / 180.0) + y * sin(0 - cur_angle * PI / 180.0);
			//ROS_INFO("x = %lf, y = %lf, x1 = %lf, y1 = %lf", x, y, x1, y1);
			if (fabs(y1) < 0.167) {
				if (fabs(x1) <= x_front_min) {
					x_front_min = fabs(x1);
					//ROS_WARN("x_front_min = %lf", x_front_min);
				}
			}
		}
	}

	angle_from = 329 - cur_angle;
	angle_to = 400 - cur_angle;
	for (int j = angle_from; j < angle_to; j++) {
		int i = j>359? j-360:j;
		if (tmp_scan_data.ranges[i] < 4) {
			th = i*1.0 + 180.0;
			x = cos(th * PI / 180.0) * tmp_scan_data.ranges[i];
			y = sin(th * PI / 180.0) * tmp_scan_data.ranges[i];
			x1 = x * cos(0 - cur_angle * PI / 180.0) + y * sin(0 - cur_angle * PI / 180.0);
			y1 = y * cos(0 - cur_angle * PI / 180.0) - x * sin(0 - cur_angle * PI / 180.0);
			//ROS_INFO("x = %lf, y = %lf, x1 = %lf, y1 = %lf", x, y, x1, y1);
			if (fabs(y1) < 0.167) {
				if (fabs(x1) <= x_back_min) {
					x_back_min = fabs(x1);
					//ROS_WARN("x_back_min = %lf", x_back_min);
				}
			}
		}
	}
	ROS_INFO("x_front_min = %lf, x_back_min = %lf", x_front_min, x_back_min);
#endif
//	ret = (x_front_min < x_back_min) ? 1 : 0;
	for (int i = 0; i < 360; i++) {
		if (tmp_scan_data.ranges[i] < 4) {
			th = i*1.0 + 180.0;
			x = cos(th * PI / 180.0) * tmp_scan_data.ranges[i];
			y = sin(th * PI / 180.0) * tmp_scan_data.ranges[i];
			coordinate_transform(&x, &y, LIDAR_THETA, LIDAR_OFFSET_X, LIDAR_OFFSET_Y);
			coordinate_transform(&x, &y, cur_angle * 10, 0, 0);
			//ROS_INFO("x = %lf, y = %lf", x, y);
			if (fabs(y) < 0.167) {
				if (x >= 0){
					if (fabs(x) <= x_front_min) {
						x_front_min = fabs(x);
						ROS_WARN("x_front_min = %lf",x_front_min);
					}
				} else {
					if (fabs(x) <= x_back_min) {
						x_back_min = fabs(x);
						ROS_ERROR("x_back_min = %lf", x_back_min);
					}
				}
			}
		}
	}
	if(x_front_min < x_back_min)
		ret = 1;
	if(x_front_min > x_back_min)
		ret = -1;
	if(x_front_min == x_back_min)
		ret = 0;
	return ret;
}
void Laser::laserDataFilter(sensor_msgs::LaserScan& laserScanData,double delta){
	for(int i = 0;i < 360; i++) {
		if (i == 0) {
			if (fabs(laserScanData.ranges[359] - laserScanData.ranges[0]) > delta ||
					fabs(laserScanData.ranges[1] - laserScanData.ranges[0]) > delta) {
//				ROS_WARN("laserDataNoise[359]:%f,laserDataNoise[0]:%f,laserDataNoise[1]:%f",laserScanData.ranges[359],laserScanData.ranges[0],laserScanData.ranges[1]);
				isNoiseNow = true;
			} else{
				isNoiseNow = false;
			}
		} else if (i == 359) {
			if (fabs(laserScanData.ranges[359] - laserScanData.ranges[358]) > delta ||
					fabs(laserScanData.ranges[359] - laserScanData.ranges[0]) > delta) {
//				ROS_WARN("laserDataNoise[358]:%f,laserDataNoise[359]:%f,laserDataNoise[0]:%f",laserScanData.ranges[358],laserScanData.ranges[359],laserScanData.ranges[0]);
				isNoiseNow = true;
			} else{
				isNoiseNow = false;
			}
		}else {
			if(fabs(laserScanData.ranges[i+1] - laserScanData.ranges[i]) > delta ||
				 fabs(laserScanData.ranges[i-1] - laserScanData.ranges[i]) > delta) {
//				ROS_WARN("laserDataNoise[%d]:%f,laserDataNoise[%d]:%f,laserDataNoise[%d]:%f",i-1,laserScanData.ranges[i-1],i,laserScanData.ranges[i],i+1,laserScanData.ranges[i+1]);
				isNoiseNow = true;
			} else{
				isNoiseNow = false;
			}
		}
		if(isNoiseLast == true)
			laserScanData.ranges[i-1] = DBL_MAX;
		isNoiseLast = isNoiseNow;
	}
	isNoiseLast = isNoiseNow = false;
}

/*
 * @author Alvin Xie
 * @brief make use of lidar to get the obstacle distance
 * @param dir:0-front 1-back 2-left 3-right
 *        range: dectect range
 * @return the distance to the obstacle
 * */
bool Laser::getObstacleDistance(uint8_t dir, double range, uint32_t &seq, laserDistance& laser_distance)
{
	double x,y,th;
	double x_to_robot,y_to_robot;
	scan_mutex_.lock();
	auto tmp_scan_data = laserScanData_;
	scan_mutex_.unlock();
	if(range < 0.056)
	{
		ROS_ERROR("range should be higher than 0.056");
		return 0;
	}
	if (tmp_scan_data.header.seq == seq) {
		//ROS_WARN("laser seq still same, quit!seq = %d", tmp_scan_data.header.seq);
		return 0;
	}
	//ROS_INFO("compLaneDistance");
	seq = tmp_scan_data.header.seq;
	laser_distance.reset();
	this->laserDataFilter(tmp_scan_data,0.02);
//	laser_filter_pub.publish(tmp_scan_data);
	for (int i = 0; i < 360; i++) {
		if (tmp_scan_data.ranges[i] < 4) {
			th = i*1.0 + 180.0;
			x = cos(th * PI / 180.0) * tmp_scan_data.ranges[i];
			y = sin(th * PI / 180.0) * tmp_scan_data.ranges[i];
			coordinate_transform(&x, &y, LIDAR_THETA, LIDAR_OFFSET_X, LIDAR_OFFSET_Y);
			x_to_robot = fabs(x) - ROBOT_RADIUS * sin(acos(fabs(y) / ROBOT_RADIUS));
			y_to_robot = fabs(y) - ROBOT_RADIUS * sin(acos(fabs(x) / ROBOT_RADIUS));
			//ROS_INFO("x = %lf, y = %lf", x, y);
			if (dir == 0) {
				if (y > 0.056 && y <= range) {
					if (x >= 0){
						if (x_to_robot < laser_distance.front[0]) {
							laser_distance.front[0] = x_to_robot;
						}
					}
				}
				if (y >= -0.056 && y < 0.056) {
					if (x >= 0){
						if (x_to_robot <= laser_distance.front[1]) {
							laser_distance.front[1] = x_to_robot;
						}
					}
				}
				if (y >= -range && y <= -0.056) {
					if (x >= 0){
						if (x_to_robot <= laser_distance.front[2]) {
							laser_distance.front[2] = x_to_robot;
						}
					}
				}
			} else if (dir == 1) {
				if (fabs(y) < range) {
					if (x < 0){
						if (x_to_robot < laser_distance.back) {
							laser_distance.back = x_to_robot;
							//ROS_WARN("back = %lf", back);
						}
					}
				}
			} else if (dir == 2) {
				if (fabs(x) < range) {
					if (y >= 0 && x > 0){
						if (y_to_robot < laser_distance.left) {
							laser_distance.left = y_to_robot;
							//ROS_WARN("left = %lf",left);
						}
					}
				}
			} else if (dir == 3) {
				if (fabs(x) < range) {
					if (y < 0 && x > 0){
						if (y_to_robot < laser_distance.right) {
							laser_distance.right = y_to_robot;
							//ROS_WARN("right = %lf",right);
						}
					}
				}
			}
		}
	}
	return true;
}
void Laser::pubPointMarker(std::vector<Double_Point> *point)
{
	int points_size;
	visualization_msgs::Marker point_marker;
	point_marker.ns = "point_marker";
	point_marker.id = 0;
	point_marker.type = visualization_msgs::Marker::SPHERE_LIST;
	point_marker.action= 0;//add
	point_marker.lifetime=ros::Duration(0);
	point_marker.scale.x = 0.01;
	point_marker.scale.y = 0.01;
	point_marker.scale.z = 0.01;
	point_marker.color.r = 0.0;
	point_marker.color.g = 1.0;
	point_marker.color.b = 0.0;
	point_marker.color.a = 1.0;
	point_marker.header.frame_id = "/base_link";
	point_marker.header.stamp = ros::Time::now();
	laser_points_.x = 0.0;
	laser_points_.y = 0.0;
	laser_points_.z = 0.0;

	if (!(*point).empty()) {
		for (std::vector<Double_Point>::iterator iter = (*point).begin(); iter != (*point).end(); ++iter) {
			laser_points_.x = iter->x;
			laser_points_.y = iter->y;
			point_marker.points.push_back(laser_points_);
		}
		point_marker_pub.publish(point_marker);
		point_marker.points.clear();
	} else {
		point_marker.points.clear();
		point_marker_pub.publish(point_marker);
	}
}
