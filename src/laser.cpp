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
#include <vector>

#include <visualization_msgs/Marker.h>
#include <ros/ros.h>
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
	start_motor_cli_ = nh_.serviceClient<std_srvs::Empty>("start_motor");
	stop_motor_cli_ = nh_.serviceClient<std_srvs::Empty>("stop_motor");
	start_laser_shield_cli_ = nh_.serviceClient<std_srvs::Empty>("start_laser_shield");
	stop_laser_shield_cli_ = nh_.serviceClient<std_srvs::Empty>("stop_laser_shield");
	ROS_INFO("Laser init done!");

	start();
}

Laser::~Laser()
{
	stop();
//	scan_sub_ = nh_.subscribe("scan", 1, &Laser::scanCb, this);
//	scan_sub_.shutdown();
//	start_motor_cli_.shutdown();
//	stop_motor_cli_.shutdown();
//	nh_.shutdown();
	ROS_INFO("Laser stop");
}

void Laser::scanCb(const sensor_msgs::LaserScan::ConstPtr &scan)
{
	int count = 0;
	is_scanDataReady_ = false;
	laser_scan_data_ = *scan;
	count = (int)((scan->angle_max - scan->angle_min) / scan->angle_increment);
	//ROS_INFO("%s %d: seq: %d\tangle_min: %f\tangle_max: %f\tcount: %d\tdist: %f", __FUNCTION__, __LINE__, scan->header.seq, scan->angle_min, scan->angle_max, count, scan->ranges[180]);
	is_scanDataReady_ = true;
	isReady(1);
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
	start_motor_cli_.call(empty);

	while (ros::ok() && try_times <= 2)
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
			if (time(NULL) - stop_time > 0)
			{
				laser_pm_gpio('1');
				usleep(20000);
				ROS_WARN("%s %d: Start laser for %d time.", __FUNCTION__, __LINE__, try_times);
				start_motor_cli_.call(empty);
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

			if (time(NULL) - start_time > 5)
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

void Laser::stop(void)
{
	stopShield();
	std_srvs::Empty empty;
	isReady(0);
	is_scanDataReady_ = false;
	ROS_INFO("%s %d: Stop laser.", __FUNCTION__, __LINE__);
	stop_motor_cli_.call(empty);
	laser_pm_gpio('0');
}

void Laser::startShield(void)
{
	std_srvs::Empty empty;
	start_laser_shield_cli_.call(empty);
	ROS_INFO("%s %d: Start laser shield.", __FUNCTION__, __LINE__);
}

void Laser::stopShield(void)
{
	std_srvs::Empty empty;
	stop_laser_shield_cli_.call(empty);
	ROS_INFO("%s %d: Stop laser shield.", __FUNCTION__, __LINE__);
}

bool Laser::getLaserDistance(int begin, int end, double range, double dis_lim, double *line_angle)
{
	int		i, count;
	bool	found = false;
	double	angle_min, angle_max, tmp, range_tmp;
	double	laser_distance = 0;
	int		sum = 0;
	double	th;
	double 	a, b, c;
	//double	line_angle;
	Double_Point	New_Laser_Point;
	Laser_Point.clear();
	ROS_ERROR("getLaserDistance");
	for (i = begin; i < end; i++) {//default:begin = 260, end =270
		if (laser_scan_data_.ranges[i] < 4) {
			th = i * 1.0;
			th = th + 180.0;
			New_Laser_Point.x = cos(th * PI / 180.0) * laser_scan_data_.ranges[i];
			New_Laser_Point.y = sin(th * PI / 180.0) * laser_scan_data_.ranges[i];
			Laser_Point.push_back(New_Laser_Point);
		}
		//laser_distance = laser_scan_data_.ranges[i];
		//ROS_INFO("wall_distance = %lf, i = %d", laser_distance, i);
		//ROS_INFO("Laser_Point_x = %lf, Laser_Point_y = %lf, th = %lf, distance = %lf", New_Laser_Point.x, New_Laser_Point.y, th, laser_scan_data_.ranges[i]);
	}
	lineFit(Laser_Point, a, b, c);
	splitLine(Laser_Point, 0.01, 10);
	splitLine2nd(&Laser_Group, 0.01,10);
	mergeLine(&Laser_Group, 0.01);
	fitLineGroup(&Laser_Group, 0.1, dis_lim);
	//*line_angle = atan2(-a, b) * 180 / PI;
	Laser_Group.clear();
	if (!fit_line.empty()) {
		*line_angle = atan2(0 - fit_line.begin()->A, fit_line.begin()->B) * 180 / PI;
		//ROS_INFO("a = %lf, b = %lf, c = %lf", a, b, c);
		ROS_WARN("line_angle = %lf", *line_angle);
		return true;
	} else {
		ROS_WARN("no line to fit!");
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

	//ROS_WARN("points.size = %d, den = %lf", size, den);
	if(fabs(den) < 1e-5) {
		if( fabs(Dxx / Dyy - 1) < 1e-5) {
			ROS_WARN("line fit failed!");
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
				//ROS_WARN("%s %d: loop end, push back the line", __FUNCTION__, __LINE__);
				Laser_Group.push_back(new_line);
				new_line.clear();
			}
		} else {
			//ROS_WARN("%s %d: distance = %lf", __FUNCTION__, __LINE__, distance);
			//ROS_WARN("split!");
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
	ROS_INFO("splitLine : Laser_Group.size = %d", Laser_Group.size());
	return true;
}
#if 0
bool Laser::splitLine2nd(const std::vector<std::vector<Double_Point> > &groups, double t_lim, int points_count_lim) {
	int groups_size = groups.size();
	int points_size;
	double a, b, c;
	double x1, y1, x2, y2;
	double t, t_max;
	int	points_index_max;
	std::vector<Double_Point> new_line;
	ROS_INFO("split 2nd");
	for (int i = 0; i < groups_size; i++) {
		x1 = groups[i].begin()->x;
		y1 = groups[i].begin()->y;
		x2 = (groups[i].end() - 1)->x;
		y2 = (groups[i].end() - 1)->y;
		points_size = groups[i].size();
		ROS_INFO("next line");
		if (x1 != x2 && y1 != y2) {
			a = y2 - y1;
			b = x2 - x1;
			c= x2 * y1 - x1 * y2;
			t = 0;
			t_max = 0;

			for (int j = 0; j < points_size; j++) {
				t = fabs((a * groups[i][j].x + b * groups[i][j].y + c) / sqrt(a * a + b * b));
				if (t >= t_max) {
					t_max = t;
					points_index_max = j;
					ROS_INFO("t = %lf", t);
				}
			}
			ROS_INFO("t_max = %lf", t_max);
			if (t_max > t_lim) {
				for (int j = 0; j < points_size; j++) {
					if (j < points_index_max) {
						new_line.push_back(groups[i][j]);
						if (j == (points_size - 1)) {
							Laser_Group_2nd.push_back(new_line);
							new_line.clear();
						}
					} else {
						ROS_WARN("split 2nd!");
						Laser_Group_2nd.push_back(new_line);
						new_line.clear();
						new_line.push_back(groups[i][j]);
					}
					//ROS_INFO("x1 = %lf, y1 = %lf, x2 = %lf, y2 = %lf, a = %lf, b = %lf, c = %lf", x1, y1, x2, y2, a, b, c);
				}	
			} else {
				for (int j = 0; j < points_size; j++) {
					new_line.push_back(groups[i][j]);
					if (j == (points_size - 1)) {
						Laser_Group_2nd.push_back(new_line);
						new_line.clear();
					}
				}
			}
			/*for (int j = 0; j < points_size; j++) {
				if (t < t_lim) {
					new_line.push_back(groups[i][j]);
					if (j == (points_size - 1)) {
						Laser_Group_2nd.push_back(new_line);
						new_line.clear();
					}
				} else {
					ROS_WARN("split 2nd!");
					Laser_Group_2nd.push_back(new_line);
					new_line.clear();
					new_line.push_back(groups[i][j]);
				}
				//ROS_INFO("x1 = %lf, y1 = %lf, x2 = %lf, y2 = %lf, a = %lf, b = %lf, c = %lf", x1, y1, x2, y2, a, b, c);
				ROS_INFO("t = %lf", t);
			}*/	
		}
	}

	for (std::vector<std::vector<Double_Point> >::iterator iter = Laser_Group_2nd.begin(); iter != Laser_Group_2nd.end();) {
		if (iter->size() < points_count_lim) {
			iter = Laser_Group_2nd.erase(iter);
		} else {
			++iter;
		}
	}
	ROS_INFO("Laser_Group_2nd.size = %d", Laser_Group_2nd.size());
	Laser_Group.clear();
	Laser_Group_2nd.clear();

	return true;
}
#endif

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

	ROS_INFO("Laser::splitLine2nd");
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
		//ROS_WARN("points_size = %d", points_size);
		//ROS_WARN("next line");
		end_iterate_flag = true;
		if (x1 != x2 && y1 != y2) {
			a = y2 - y1;
			b = x1 - x2;
			c= x2 * y1 - x1 * y2;
			t = 0;
			t_max = 0;
			//ROS_WARN("x1 = %lf, y1 = %lf, x2 = %lf, y2 = %lf, a = %lf, b = %lf, c = %lf", x1, y1, x2, y2, a, b, c);

			/*loop for finding the t_max*/
			//ROS_WARN("loop for finding the t_max");
			for (int j = 0; j < points_size; j++) {
				t = fabs((a * (iter->begin() + j)->x + b * (iter->begin() +j)->y + c) / sqrt(a * a + b * b));
				//ROS_INFO("t = %lf", t);
				//ROS_INFO("points_index = %d, x = %lf, y = %lf",j ,(iter->begin() + j)->x, (iter->begin() + j)->y);
				if (t >= t_max) {
					t_max = t;
					points_index_max = j;
					//ROS_WARN("t >= t_max,points_index_max = %d", points_index_max);
				}
			}
			//ROS_WARN("t_max = %lf", t_max);

			/*if t_max > t_lim, then split into 2 lines*/
			//ROS_WARN("loop for split");
			if (t_max > t_lim) {
				end_iterate_flag &= false;
				//ROS_WARN("t_tmax > t_lim,end_iterate_flag = %d", end_iterate_flag);
				int index = std::distance((*groups).begin(), iter);
				groups_erased_index.push_back(index);
				//ROS_WARN("push_back erased index = %d",index);
				//ROS_WARN("points_size = %d", points_size);
				for (int j = 0; j < points_size; j++) {
					new_line.push_back(*(iter->begin() + j));
					//ROS_INFO("j = %d", j);
					if (j == points_index_max) {
						if (j == (points_size - 1)) {
							//ROS_WARN("split the rearest point");
							new_line.pop_back();
							//ROS_WARN("new_line.size() = %d",new_line.size());
							new_group.push_back(new_line);
							new_line.clear();
						} else {
							//ROS_WARN("split front");
							//ROS_WARN("new_line.size() = %d",new_line.size());
							new_group.push_back(new_line);
							new_line.clear();
						}
					} else if ((j > points_index_max) && (j == (points_size - 1))) {
						//ROS_WARN("split rear");
						//ROS_WARN("new_line.size() = %d",new_line.size());
						new_group.push_back(new_line);
						new_line.clear();
					}
					//ROS_INFO("x1 = %lf, y1 = %lf, x2 = %lf, y2 = %lf, a = %lf, b = %lf, c = %lf", x1, y1, x2, y2, a, b, c);
				}
			} else {//else push_back the line into new_group
				end_iterate_flag &= true;
				//ROS_WARN("t_max < t_lim,end_iterate_flag = %d", end_iterate_flag);
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

	ROS_WARN("1Laser_Group.size = %d", (*groups).size());
#if 0
	/*loop for erasing the old lines which are unsplit*/
	ROS_WARN("loop for erasing the old lines which are unsplit");
	if (!groups_erased_index.empty()) {
		for (std::vector<int>::iterator er_id_iter = groups_erased_index.begin(); er_id_iter != groups_erased_index.end(); ++er_id_iter) {
			for (std::vector<std::vector<Double_Point> >::iterator iter = (*groups).begin(); iter != (*groups).end();) {
				int index = std::distance((*groups).begin(), iter);
				if (index == (*er_id_iter)) {
					ROS_WARN("erase the unsplit line! index = %d", index);
					iter = (*groups).erase(iter);
					ROS_WARN("2Laser_Group.size = %d", (*groups).size());
				} else {
					++iter;
				}
			}
		}
	}
#endif

#if 0
	/*loop for erasing the old lines which are unsplit*/
	ROS_WARN("loop for erasing the old lines which are unsplit");
	if (!groups_erased_index.empty()) {
		for (std::vector<int>::iterator er_id_iter = groups_erased_index.begin(); er_id_iter != groups_erased_index.end(); ++er_id_iter) {
					ROS_WARN("erase the unsplit line! index = %d", (*er_id_iter));
					std::vector<std::vector<Double_Point> >::iterator iter = (*groups).begin() + (*er_id_iter);
					(*groups).erase(iter);
					ROS_WARN("2Laser_Group.size = %d", (*groups).size());
		}
	}
	groups_erased_index.clear();

	/*loop for push_back lines in new_group into groups*/
	for (std::vector<std::vector<Double_Point> >::iterator iter = new_group.begin(); iter != new_group.end(); ++iter) {
		(*groups).push_back(*iter);
		ROS_WARN("push_back lines in new_group");
		ROS_WARN("3Laser_Group.size = %d", (*groups).size());
	}
	new_group.clear();
#endif

	/*loop for erasing the old lines which are unsplit and push_back lines in new_group into groups*/
	ROS_WARN("loop for erasing the old lines which are unsplit");
	erased_size = groups_erased_index.size();
	if (!groups_erased_index.empty()) {
		for (int i = 0; i < erased_size; i++) {
			ROS_WARN("erase the unsplit line! index = %d", i);
			std::vector<std::vector<Double_Point> >::iterator iter = (*groups).begin() + (groups_erased_index[i] + i);
			(*groups).erase(iter);
			ROS_WARN("2Laser_Group.size = %d", (*groups).size());

			for (int j = 0; j < 2; j++) {
				ROS_WARN("push_back lines in new_group");
				ROS_WARN("new_group.begin->size() = %d", (new_group.begin()->size()));
				ROS_WARN("new_group.size() = %d", (new_group.size()));
				(*groups).insert((*groups).begin() + (groups_erased_index[i] + j), *(new_group.begin() + j));
				//(*groups).push_back(new_group.begin());
				//new_group.pop_back();
				ROS_WARN("3Laser_Group.size = %d", (*groups).size());
			}
		}
	}

	ROS_WARN("new_group.size = %d", new_group.size());
	groups_erased_index.clear();
	new_group.clear();

	/*loop for erasing the line which size is less than points_count_lim*/
	ROS_WARN("loop for erasing the line which size is less than points_count_lim");
	for (std::vector<std::vector<Double_Point> >::iterator iter = (*groups).begin(); iter != (*groups).end();) {
		if (iter->size() < points_count_lim) {
			iter = (*groups).erase(iter);
			//ROS_WARN("iter->size() = %d", iter->size());
		} else {
			++iter;
		}
	}

	ROS_WARN("4Laser_Group.size = %d", (*groups).size());

	/*if the lines still can be splited, iterate to split it*/
	if (end_iterate_flag == false) {
		//ROS_WARN("iterate!");
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
				//ROS_WARN("x1 = %lf, y1 = %lf, x2 = %lf, y2 = %lf, a = %lf, b = %lf, c = %lf", x1, y1, x2, y2, a, b, c);
				//ROS_WARN("%s %d: x1 = %lf, y1 = %lf, x2 = %lf, y2 = %lf, a = %lf, b = %lf, c = %lf", __FUNCTION__, __LINE__, x1, y1, x2, y2, a, b, c);
				points_size = iter->size();
				points_size_2nd = (iter + 1)->size();
				/*loop for checking the first line*/
				for (int j = 0; j < points_size; j++) {
					t = fabs((a * (iter->begin() + j)->x + b * (iter->begin() +j)->y + c) / sqrt(a * a + b * b));
					//ROS_INFO("t = %lf", t);
					//ROS_INFO("points_index = %d, x = %lf, y = %lf",j ,(iter->begin() + j)->x, (iter->begin() + j)->y);
					if (t >= t_max) {
						t_max = t;
						//ROS_WARN("new t_max = %lf", t_max);
						//points_index_max = j;
						//ROS_WARN("t >= t_max,points_index_max = %d", points_index_max);
					}
				}
				/*loop for checking the second line*/
				for (int j = 0; j < points_size_2nd; j++) {
					t = fabs((a * ((iter + 1)->begin() + j)->x + b * ((iter + 1)->begin() +j)->y + c) / sqrt(a * a + b * b));
					//ROS_INFO("t = %lf", t);
					//ROS_INFO("points_index = %d, x = %lf, y = %lf",j ,((iter + 1)->begin() + j)->x, ((iter + 1)->begin() + j)->y);
					if (t >= t_max) {
						t_max = t;
						//ROS_WARN("new t_max = %lf", t_max);
						//points_index_max = j;
						//ROS_WARN("t >= t_max,points_index_max = %d", points_index_max);
					}
				}
				//ROS_WARN("merge: t_max = %lf", t_max);
				if (t_max < t_lim) {
					int index = std::distance((*groups).begin(), iter);
					merge_index.push_back(index);
					ROS_WARN("5Laser_Group.size = %d", (*groups).size());
					ROS_WARN("merge! index = %d and %d", index, (index + 1));
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
			ROS_WARN("merge : erase front");
			ROS_WARN("Laser_Group.size = %d", (*groups).size());
			(*groups).erase(iter + 1 - loop_count);
			ROS_WARN("merge : erase rear");
			ROS_WARN("5Laser_Group.size = %d", (*groups).size());
			(*groups).insert((*groups).begin() + (*m_iter) - loop_count, new_line);
			ROS_WARN("merge : insert");
			ROS_WARN("5Laser_Group.size = %d", (*groups).size());
			loop_count++;
		}
	}
	ROS_WARN("pub line marker");
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
				//ROS_WARN("laser_points_.x = %lf laser_points_.y = %lf",laser_points_.x, laser_points_.y);
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
	double line_angle;
	const double L =0.2727716;
	int	loop_count = 0;
	LineABC	new_fit_line;
	fit_line.clear();
	if (!(*groups).empty()) {
		for (std::vector<std::vector<Double_Point> >::iterator iter = (*groups).begin(); iter != (*groups).end(); ++iter) {
			lineFit((*iter), a, b, c);
			line_angle = atan2(-a, b) * 180 / PI;
			new_fit_line.A = a;
			new_fit_line.B = b;
			new_fit_line.C = c;
			/*erase the lines which are far away from the robot*/
			//double x_l = fabs(c / a);
			//double y_l = fabs(c / b);
			double dis = fabs(c / (sqrt(a * a + b * b)));
			//if ((x_l > L) && (y_l > L)) {
			if (dis > dis_lim || dis < 0.167) {
				//ROS_WARN("the line is too far away from robot. x_l = %lf, y_l = %lf", x_l, y_l);
				ROS_WARN("the line is too far away from robot. dis = %lf", dis);
				continue;
			}
			fit_line.push_back(new_fit_line);
			//pubFitLineMarker(a, b, c, -0.5, 0.5);
			pubFitLineMarker(a, b, c, iter->begin()->y, (iter->end() - 1)->y);
			//ROS_WARN("iter->begin()->y = %lf, (iter->end() - 1)->y= %lf",iter->begin()->y, (iter->end() - 1)->y);
			ROS_WARN("%s %d: line_angle%d = %lf", __FUNCTION__, __LINE__, loop_count, line_angle);
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
	//ROS_WARN("Fit line laser_points_.x = %lf laser_points_.y = %lf",laser_points_.x, laser_points_.y);
	fit_line_marker.points.push_back(laser_points_);
	laser_points_.x = x2;
	laser_points_.y = y2;
	//ROS_WARN("Fit line laser_points_.x = %lf laser_points_.y = %lf",laser_points_.x, laser_points_.y);
	fit_line_marker.points.push_back(laser_points_);
	fit_line_marker_pub.publish(fit_line_marker);
	//fit_line_marker.points.clear();
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
