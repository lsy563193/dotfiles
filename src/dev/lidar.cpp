#include "lidar.hpp"
#include "robot.hpp"
#include <std_srvs/Empty.h>
#include <std_srvs/SetBool.h>
//#include <pp/SetLidar.h>
#include <wheel.hpp>
#include <mode.hpp>
#include <mathematics.h>
#include <beeper.h>
#include <gyro.h>

boost::mutex scanLinear_mutex_;
boost::mutex scanOriginal_mutex_;
boost::mutex scanCompensate_mutex_;
boost::mutex lidarXYPoint_mutex_;
//float* Lidar::last_ranges_ = NULL;

sensor_msgs::LaserScan Lidar::lidarScanData_original_;
double Lidar::wheel_cliff_trigger_time_ = 0;
double Lidar::gyro_tilt_trigger_time_ = 0;

Lidar lidar;

Lidar::Lidar():angle_n_(0)
{
	setScanLinearReady(0);
	setScanOriginalReady(0);
	setScanCompensateReady(0);
//	setScanLinearReady(0);
//	setScanOriginalReady(0);
//	scanLinear_update_time_ = ros::Time::now().toSec();
//	scanOriginal_update_time_ = ros::Time::now().toSec();
	//last_ranges_ = new float[360];
	//memset(last_ranges_,0.0,360*sizeof(float));
}

Lidar::~Lidar()
{
//	motorCtrl(OFF);
//	setScanLinearReady(0);
//	setScanOriginalReady(0);
//	//delete []last_ranges_;
//	ROS_INFO("\033[35m" "%s %d: Lidar stopped." "\033[0m", __FUNCTION__, __LINE__);
}

void Lidar::scanLinearCb(const sensor_msgs::LaserScan::ConstPtr &scan)
{
	if(switch_){
		scanLinear_mutex_.lock();
		lidarScanData_linear_ = *scan;
		scanLinear_mutex_.unlock();
		angle_n_ = (int)((scan->angle_max - scan->angle_min) / scan->angle_increment);
		setScanLinearReady(1);
		scanLinear_update_time_ = ros::Time::now().toSec();
	}
}

void Lidar::scanOriginalCb(const sensor_msgs::LaserScan::ConstPtr &scan)
{
	if (switch_)
	{
		uint8_t scan2_valid_cnt = 0;
		for (uint16_t i = 0; i < 360; i++)
		{
			if (scan->ranges[i] != std::numeric_limits<float>::infinity())
			{
				if (++scan2_valid_cnt > 30)
					break;
			}
		}
		if (scan2_valid_cnt > 30) {
			// lidar not been covered.
			setLidarScanDataOriginal(scan);
			scanOriginal_update_time_ = ros::Time::now().toSec();
			setScanOriginalReady(1);
		}
	}
}

void Lidar::scanCompensateCb(const sensor_msgs::LaserScan::ConstPtr &scan)
{
	if (switch_)
	{
		scanCompensate_mutex_.lock();
		lidarScanData_compensate_ = *scan;
		scanCompensate_mutex_.unlock();
		setScanCompensateReady(1);
		scanCompensate_update_time_ = ros::Time::now().toSec();
	}
}

void Lidar::scantestCb(const sensor_msgs::LaserScan::ConstPtr &scan)
{
	if (switch_)
	{
		setLidarScanDataOriginal(scan);
		scanOriginal_update_time_ = ros::Time::now().toSec();
		setScanOriginalReady(1);

		// Print the range value.
		int print_limit_in_line = 18;
		int print_cnt_in_line = 0;
		ROS_INFO("Scan %d ranges:", scan->header.seq);
		for (auto i : scan->ranges)
		{
			if (print_cnt_in_line++ < print_limit_in_line - 1)
				printf("%1.3f\t", i);
			else
			{
				print_cnt_in_line = 0;
				printf("%1.3f\n", i);
			}
		}
	}
}

void Lidar::lidarXYPointCb(const visualization_msgs::Marker &point_marker) {
	if (isScanCompensateReady())
	{
		lidarXYPoint_mutex_.lock();
		lidarXY_points = point_marker.points;
		lidarXYPoint_mutex_.unlock();
		scanXYPoint_update_time_ = ros::Time::now().toSec();
	}
}

int8_t Lidar::isScanOriginalReady()
{
	return is_scanOriginal_ready_;
}

int8_t Lidar::isScanLinearReady()
{
	return is_scanLinear_ready_;
}

int8_t Lidar::isScanCompensateReady()
{
	return is_scanCompensate_ready_;
}

void Lidar::setScanLinearReady(uint8_t val)
{
//	ROS_ERROR("setScanLinearReady(%d)", val);
	is_scanLinear_ready_ = val;
}

void Lidar::setScanOriginalReady(uint8_t val)
{
//	ROS_ERROR("setScanOriginalReady(%d)", val);
	is_scanOriginal_ready_ = val;
}

void Lidar::setScanCompensateReady(uint8_t val)
{
//	ROS_ERROR("setScanCompensateReady(%d)", val);
	is_scanCompensate_ready_ = val;
}

bool Lidar::motorCtrl(bool new_switch_)
{
	switch_ = new_switch_;
	if(switch_){
		scanLinear_update_time_ = ros::Time::now().toSec();
		scanOriginal_update_time_ = ros::Time::now().toSec();
		ROS_INFO("\033[35m" "%s %d: Open lidar." "\033[0m", __FUNCTION__, __LINE__);
	}
	else
	{
		setScanLinearReady(0);
		setScanOriginalReady(0);
		setScanCompensateReady(0);
		slip_frame_cnt_ = 0;
		slip_status_ = false;
		//delete []last_ranges_;
		ROS_INFO("\033[35m" "%s %d: Stop lidar." "\033[0m", __FUNCTION__, __LINE__);
	}

	if (!robot::instance()->lidarMotorCtrl(switch_))
	{
		ROS_ERROR("%s %d: Lidar service not received!",__FUNCTION__,__LINE__);
		return false;
	}

	return true;

}

/*
 * @author mengshige1988@qq.com
 * @brief for combine each lines in lines
 * @param1 lines before
 * @param2 lines after
 * @return false if no more lines for combine ,else return true.
 * */
static bool lineCombine(std::vector<LineABC> *lines_before,std::vector<LineABC> *lines_after)
{
	bool ret = false;
	const float MIN_COMBINE_ANGLE=10.0;//in degrees
	const float MIN_COMBINE_DIST=0.6;//in meters
	LineABC line_tmp;
	std::vector<LineABC>::iterator it;
	for(it = lines_before->begin(); it != lines_before->end(); it++){
		if( (it+1)!=lines_before->end() ){
			if(std::abs(it->K - (it+1)->K) <= MIN_COMBINE_ANGLE){
				if(two_points_distance_double(it->x2,it->y2,(it+1)->x1,(it+1)->y1) < MIN_COMBINE_DIST){
					line_tmp.x1 = it->x1;
					line_tmp.y1 = it->y1;
					line_tmp.x2 = (it+1)->x2;
					line_tmp.y2 = (it+1)->y2;
					line_tmp.A = line_tmp.y2 - line_tmp.y1;
					line_tmp.B = line_tmp.x1 - line_tmp.x2;
					line_tmp.C = line_tmp.x2 * line_tmp.y1 - line_tmp.x1 * line_tmp.y2;
					line_tmp.len = two_points_distance_double(line_tmp.x1,line_tmp.y1,line_tmp.x2,line_tmp.y2);
					line_tmp.K = radian_to_degree(atan(-1*(line_tmp.A / line_tmp.B)));
					lines_after->push_back(line_tmp);
					it++;
					ret = true;
				}
			}
			else
				lines_after->push_back(*it);
		}
	}
	return ret;
}

/*
 * @author mengshige1988@qq.com
 * @brief find lines from scan data
 * @param1 lines vector
 * @return ture if found lines, alse return false
 * */
bool Lidar::findLines(std::vector<LineABC> *lines,bool combine)
{
	if(!isScanLinearReady())
		return false;
	const float MAX_LIDAR_DIST = 4.0;
	const float ACCR_PERSET = 0.05;//%5

	sensor_msgs::LaserScan scan_data;
	if (lidarCheckFresh(0.210,1))
	{
		scanLinear_mutex_.lock();
		scan_data = lidarScanData_linear_;
		scanLinear_mutex_.unlock();
	}
	else
		return false;

	/*---------translate lidar distance to point set----------*/
	int n_angle = angle_n_;
	int i=0;
	Vector2<double> lidar_point_pos;
	std::vector<Vector2<double>> point_set;
	for(i=0;i<n_angle;i++){
		if(scan_data.ranges[i] <= MAX_LIDAR_DIST){
			auto cur_radian = robot::instance()->getWorldPoseRadian();
			auto cur_p_x = robot::instance()->getWorldPoseX();
			auto cur_p_y = robot::instance()->getWorldPoseY();

			double ranges = scan_data.ranges[i];
			lidar_point_pos.x = cos(cur_radian + degree_to_radian(i) + PI) *ranges + cur_p_x;//in meters
			lidar_point_pos.y = sin(cur_radian + degree_to_radian(i) + PI) *ranges + cur_p_y;//in meters
			coordinate_transform(&lidar_point_pos.x, &lidar_point_pos.y, LIDAR_THETA, LIDAR_OFFSET_X, LIDAR_OFFSET_Y);
			point_set.push_back(lidar_point_pos);
			lidar_point_pos.x = 0.0;
			lidar_point_pos.y = 0.0;
		}
	}
	//pubPointMarkers(&point_set);

	if(point_set.size() <2)
		return false;// not enough point to make lines , at least got 2 points

	/*--------ready to find lines in point set-----------*/
	LineABC line;
	lines->clear();
	const float LINE_n_P_DIST_MAX = 0.05;//max line and point distance
	const float POINT_n_P_DIST_MAX = 0.3;//max point and point distance
	int line_count = 0;
	int point_count = 0;
	bool find_next_line = true;
	i = 0;
	while(i < (point_set.size() -1)){
		if(find_next_line){
			double x1 = point_set.at(i).x;
			double y1 = point_set.at(i).y;
			i++;
			double x2 = point_set.at(i).x;
			double y2 = point_set.at(i).y;
			double dist = two_points_distance_double(x1,y1,x2,y2);
			if(dist > POINT_n_P_DIST_MAX)//find next line
				continue;
			find_next_line = false;
			line.len = dist;
			line.A = y2-y1;
			line.B = x1-x2;
			line.C = x2*y1-x1*y2;
			line.K = radian_to_degree(atan((-1)*line.A/line.B));
			line.x1 = x1;
			line.y1 = y1;
			line.x2 = x2;
			line.y2 = y2;
			lines->push_back(line);
			line_count++;
			point_count+=2;
		}
		else{
			double xn = point_set.at(i).x;
			double yn = point_set.at(i).y;
			double dist_p2l = std::abs(line.A*xn+line.B*yn+line.C)/sqrt(line.A*line.A+line.B*line.B);//point to line distance
			double dist_p2p = two_points_distance_double(point_set.at(i-1).x ,point_set.at(i-1).y ,xn ,yn);//current point to last point distance
			point_count++;
			if(dist_p2l> LINE_n_P_DIST_MAX || dist_p2p > POINT_n_P_DIST_MAX){ //find next line
				find_next_line = true;
				point_count = 0;
				continue;
			}
			if(point_count>3){
				double xn_2 = (point_set.at(i-1).x - point_set.at(i-2).x) /2.0 + point_set.at(i-2).x;
				double yn_2 = (point_set.at(i-1).y - point_set.at(i-2).y) /2.0 + point_set.at(i-2).y;
				/*---init A,B,C,x2,y2,K---*/
				LineABC *last_line = &lines->back();
				last_line->A = yn_2 - line.y1;
				last_line->B = line.x1 - xn_2;
				last_line->C = xn_2*line.y1-line.x1*yn_2;
				last_line->K= radian_to_degree(atan(-1*(last_line->A / last_line->B)));
				last_line->x2 = xn_2;
				last_line->y2 = yn_2;
				last_line->len = two_points_distance_double(line.x1,line.y1,xn_2,yn_2);
			}
		}
		i++;
	}

	/*------combine short lines to long lines------*/
	if(combine){
		std::vector<LineABC> lines_after;
		while(lineCombine(lines,&lines_after)){
			lines->clear();
			*lines = lines_after;
			lines_after.clear();
		}
	}
	/*------print line data------*/
	/*
	std::string msg("");
	std::vector<LineABC>::iterator it;
	for(it = lines->begin();it!= lines->end();it++){
		msg+= "\n[A:"+ std::to_string(it->A) +",B:" + std::to_string(it->B) +",C:" +
			std::to_string(it->C)+",len = "+std::to_string(it->size_of_path)+",K = " +
			std::to_string(it->K)+",("+std::to_string(it->x1)+","+
			std::to_string(it->y1)+"),("+std::to_string(it->x2)+","+std::to_string(it->y2)+")]";
	}
	ROS_INFO("%s,%d,pub line markers,lines numbers \033[35m%u\033[0m, lines:\033[32m %s \033[0m",__FUNCTION__,__LINE__,lines->size(),msg.c_str());
	*/
//	robot::instance()->pubLineMarker(lines);
	return true;
}

void Lidar::startAlign()
{
	align_finish_ = false;
}
void Lidar::setAlignFinish()
{
	align_finish_ = true;
}

bool Lidar::isAlignFinish()
{
	return align_finish_;
}

/*
 * @auther mengshige1988@qq.com
 * @breif get ac_align angle
 * @param1 liens vector
 * @param2 ac_align angle
 * @retrun true if found ac_align angle ,alse return false
 * */
//bool Lidar::getAlignAngle(const std::vector<LineABC> *lines ,float *align_angle)
//{
//	if(lines->empty())
//		return false;
//	const float DIF_ANG_RANGE = 10.0;//different angle ranges
//	const float LONG_ENOUGTH = 1.0;//in meters
//	std::vector<float> same_angle_count;
//	std::vector<LineABC>::const_iterator it1,it2;
//	float size_of_path = 0.0;
//	int i=0;
//	int pos = 0;
//	/*----find the longest one in lines---*/
//	for(it1 = lines->cbegin(); it1!= lines->cend(); it1++){
//		if(it1->len >= size_of_path){
//			len = it1->size_of_path;
//			pos = i;
//		}
//		i++;
//	}
//	if(lines->at(pos).size_of_path >= LONG_ENOUGTH){
//		*align_angle = lines->at(pos).K;
//		ROS_INFO("%s,%d: find long line %f ,align_angle %f",__FUNCTION__,__LINE__,lines->at(pos).size_of_path,*align_angle);
//		align_finish_ = true;
//		return true;
//	}
//
//	/*---find those most populars in lines ---*/
//	float sum = 0.0;
//	float avg = 0.0;
//	i=0;
//	if(lines->size() >1){
//		for(it2 = lines->cbegin();it2!=lines->cend();it2++){
//			if(i > lines->size()/2)
//				return false;
//			i++;
//			sum = 0.0;
//			same_angle_count.clear();
//			for(it1 = it2;it1!= lines->cend(); it1++){
//				if((it1+1) != lines->cend()){
//					if(std::abs((it1+1)->K - it1->K) < DIF_ANG_RANGE){
//						sum += it1->K;
//						same_angle_count.push_back(it1->K);
//						//printf("similar angle %f\n",it1->K);
//						if(same_angle_count.size() > lines->size()/2 ){//if number count more than 1/2 of total ,return
//							avg = sum / (1.0*same_angle_count.size());
//							*align_angle = avg;
//							align_finish_ = true;
//							ROS_INFO("%s,%d,get most popular line angle %f",__FUNCTION__,__LINE__,avg);
//							return true;
//						}
//					}
//				}
//			}
//		}
//	}
//	return false;
//}

bool Lidar::getFitLine(double r_begin, double r_end, double range, double dis_lim, double *line_radian,
											 double *distance, bool is_left, bool is_align)
{
//	ROS_WARN("angle_range_raw(%lf, %lf)", radian_to_degree(r_begin), radian_to_degree(r_end));
	if(isScanOriginalReady() == 0){
//		INFO_BLUE("ScanOriginal NOT Ready! Break!");
		return false;
	}
	bool isReverse = false;
	float consecutive_lim = 0.10;
	int points_count_lim = 10;
	float t_lim_split = 0.10;
	float t_lim_merge = 0.10;
	double th;
	Vector2<double>	New_Lidar_Point;
	Lidar_Point.clear();

	if(is_align){
		consecutive_lim = 0.06;
		points_count_lim = 10;
		t_lim_split = 0.06;
		t_lim_merge = 0.06;
	}
	ROS_WARN("getFitLine");
	scanOriginal_mutex_.lock();
//	auto tmp_scan_data = lidarScanData_compensate_;
	auto tmp_scan_data = lidarScanData_original_;
	scanOriginal_mutex_.unlock();
/*	r_begin = radian_to_degree(atan2(ROBOT_RADIUS * sin(r_begin), LIDAR_OFFSET_X + ROBOT_RADIUS * cos(r_begin)));
	r_end = radian_to_degree(atan2(ROBOT_RADIUS * sin(r_end), LIDAR_OFFSET_X + ROBOT_RADIUS * cos(r_end)));
	r_begin -= radian_to_degree(LIDAR_THETA);
	r_end -= radian_to_degree(LIDAR_THETA);*/

	auto d_begin = radian_to_degree(r_begin);
	auto d_end = radian_to_degree(r_end);
	if (!is_align) {
		auto laser_range_offset = atan2(LIDAR_OFFSET_X, ROBOT_RADIUS);
		if (int(d_begin)!= 180) {
			d_begin = d_begin - radian_to_degree(laser_range_offset);
//			ROS_INFO("(d_begin) != 180");
		} else {
//			ROS_INFO("(d_begin) == 180");
		}
		if (int(d_end) != 180) {
			d_end = d_end + radian_to_degree(laser_range_offset);
//			ROS_INFO("radian_to_degree(d_begin) != 180");
		}
//		ROS_INFO("laser_range_offset = %d, d_begin = %lf, d_end = %lf", int(radian_to_degree(laser_range_offset)), d_begin, d_end);
	}
	d_begin -= radian_to_degree(LIDAR_THETA);
	d_end -= radian_to_degree(LIDAR_THETA);
	if(d_begin < 0)
		d_begin += 360;
	if(d_end < 0)
		d_end += 360;
	if(d_begin >= d_end)
		isReverse = true;
//	ROS_WARN("angle_range_after(%lf, %lf)", d_begin, d_end);
	for (auto i = static_cast<int>(d_begin); (i < static_cast<int>(d_end) || isReverse);) {
		if(i > 359)
			i = i - 360;
//		ROS_INFO("i = %d, range = %f", i, tmp_scan_data.ranges[i]);
		if (tmp_scan_data.ranges[i] < 4) {
			th = i * 1.0;
			th = th + 180.0;
			New_Lidar_Point.x = cos(degree_to_radian(th)) * tmp_scan_data.ranges[i];
			New_Lidar_Point.y = sin(degree_to_radian(th)) * tmp_scan_data.ranges[i];
			coordinate_transform(&New_Lidar_Point.x, &New_Lidar_Point.y, LIDAR_THETA, LIDAR_OFFSET_X, LIDAR_OFFSET_Y);
			Lidar_Point.push_back(New_Lidar_Point);
		}
		i++;
		if(i == static_cast<int>(d_end))
			isReverse = false;
	}

	if (Lidar_Point.empty())
	{
		ROS_ERROR("%s %d: No lidar point available!", __FUNCTION__, __LINE__);
		return false;
	}
	splitLine(Lidar_Point, consecutive_lim,points_count_lim);
	splitLine2nd(&Lidar_Group, t_lim_split,points_count_lim);
	mergeLine(&Lidar_Group, t_lim_merge, is_align);
	fitLineGroup(&Lidar_Group,dis_lim , is_align);
	//*line_radian = atan2(-a, b);
	Lidar_Group.clear();
	if (!fit_line.empty()) {
		if (is_left) {
			*line_radian = atan2(0 - fit_line.begin()->A, fit_line.begin()->B);
			*distance = std::abs(fit_line.begin()->C / (sqrt(fit_line.begin()->A * fit_line.begin()->A + fit_line.begin()->B * fit_line.begin()->B)));
		} else {
			*line_radian = atan2(0 - fit_line.back().A, fit_line.back().B);
			*distance = std::abs(fit_line.back().C / (sqrt(fit_line.back().A * fit_line.back().A + fit_line.back().B * fit_line.back().B)));
		}
		//ROS_INFO("a = %lf, b = %lf, c = %lf", a, b, c);
		ROS_ERROR("fit line succeed! line_angle = %lf", radian_to_degree(*line_radian));
		return true;
	} else {
		*distance = 0;
		ROS_ERROR("no line to fit!");
		return false;
	}
}

bool Lidar::lineFit(const std::deque<Vector2<double>> &points, double &a, double &b, double &c)
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
//		ROS_INFO("points(%lf, %lf)", points[i].x, points[i].y);
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
	if(std::abs(den) < 1e-5) {
		if( std::abs(Dxx / Dyy - 1) < 1e-5) {
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

bool Lidar::splitLine(const std::vector<Vector2<double>> &points, double consecutive_lim, int points_count_lim)
{
	int points_size = points.size();
	double distance;
	std::deque<Vector2<double>> new_line;
	if (points.empty())
		return false;
	new_line.push_back(points[0]);
	for(int i = 1; i < (points_size - 1); i++) {
		distance = sqrt((points[i].x - points[i-1].x) * (points[i].x - points[i-1].x) + (points[i].y - points[i-1].y) * (points[i].y - points[i-1].y));
//		ROS_WARN("distance = %lf", distance);
		if (distance <= consecutive_lim) {
			new_line.push_back(points[i]);
//			ROS_INFO("i1(%d, %lf, %lf)", i, points[i].x, points[i].y);
			if (i == (points_size - 2)) {
				Lidar_Group.push_back(new_line);
//				ROS_ERROR("split1!");
				new_line.clear();
			}
		} else {
			Lidar_Group.push_back(new_line);
//			ROS_ERROR("split2!");
			new_line.clear();
			new_line.push_back(points[i]);
//			ROS_INFO("i2(%d)", i);
		}
	}
	for (std::vector<std::deque<Vector2<double>> >::iterator iter = Lidar_Group.begin(); iter != Lidar_Group.end();){
		if (iter->size() < points_count_lim) {
			iter = Lidar_Group.erase(iter);
//			ROS_ERROR("erase!");
		} else {
			++iter;
		}
	}
//	ROS_INFO("splitLine : Lidar_Group.size = %lu", Lidar_Group.size());
//	robot::instance()->pubLineMarker(&Lidar_Group,"splitLine");
	return true;
}

bool Lidar::splitLine2nd(std::vector<std::deque<Vector2<double>> > *groups, double t_lim, int points_count_lim)
{
	int points_size;
	int erased_size;
	double a, b, c;
	double x1, y1, x2, y2;
	double t, t_max;
	int	points_index_max;
	bool end_iterate_flag = true;
	std::vector<int> groups_erased_index;
	std::deque<Vector2<double>> new_line;
	std::vector<std::deque<Vector2<double>> > new_group;

//	INFO_BLUE("Lidar::splitLine2nd");
//	ROS_INFO("(*groups).size(%d)", (*groups).size());
	groups_erased_index.clear();
	new_line.clear();
	new_group.clear();

	/*loop for lines in groups*/
	for (std::vector<std::deque<Vector2<double>> >::iterator iter = (*groups).begin(); iter != (*groups).end(); ++iter) {
		x1 = iter->begin()->x;
		y1 = iter->begin()->y;
		x2 = (iter->end() - 1)->x;
		y2 = (iter->end() - 1)->y;
		points_size = static_cast<int>(iter->size());
		end_iterate_flag = true;
		if (x1 != x2 && y1 != y2) {
			a = y2 - y1;
			b = x1 - x2;
			c= x2 * y1 - x1 * y2;
			t = 0;
			t_max = 0;
			for (int j = 0; j < points_size; j++) {
				t = std::abs((a * (iter->begin() + j)->x + b * (iter->begin() +j)->y + c) / sqrt(a * a + b * b));
				if (t >= t_max) {
					t_max = t;
					points_index_max = j;
				}
			}
			if (t_max > t_lim) {
				end_iterate_flag &= false;
				int index = static_cast<int>(std::distance((*groups).begin(), iter));
				groups_erased_index.push_back(index);
				for (int j = 0; j < points_size; j++) {
					new_line.push_back(*(iter->begin() + j));
					if (j == points_index_max) {
						if (j == (points_size - 1)) {
							new_line.pop_back();
							new_group.push_back(new_line);
							new_line.clear();
						} else {
							new_group.push_back(new_line);
							new_line.clear();
						}
					} else if ((j > points_index_max) && (j == (points_size - 1))) {
						new_group.push_back(new_line);
						new_line.clear();
					}
				}
			} else {//else push_back the line into new_group
				end_iterate_flag &= true;
			}
		}
	}
	/*loop for erasing the old lines which are unsplit and push_back lines in new_group into groups*/
	ROS_DEBUG("loop for erasing the old lines which are unsplit");
	erased_size = static_cast<int>(groups_erased_index.size());
	if (!groups_erased_index.empty()) {
		for (int i = 0; i < erased_size; i++) {
			std::vector<std::deque<Vector2<double>> >::iterator iter = (*groups).begin() + (groups_erased_index[i] + i);
			(*groups).erase(iter);
			for (int j = 0; j < 2; j++) {
				(*groups).insert((*groups).begin() + (groups_erased_index[i] + j), *(new_group.begin() + j));
			}
		}
	}
	groups_erased_index.clear();
	new_group.clear();

	/*loop for erasing the line which size is less than points_count_lim*/
	ROS_DEBUG("loop for erasing the line which size is less than points_count_lim");
	for (std::vector<std::deque<Vector2<double>> >::iterator iter = (*groups).begin(); iter != (*groups).end();) {
		if (iter->size() < points_count_lim) {
			iter = (*groups).erase(iter);
		} else {
			++iter;
		}
	}

	/*if the lines still can be splited, iterate to split it*/
	if (end_iterate_flag == false) {
		//ROS_INFO("iterate!");
		splitLine2nd(&Lidar_Group, t_lim,10);
	}
//	robot::instance()->pubLineMarker(&Lidar_Group,"splitLine2nd");
	return true;
}

bool Lidar::mergeLine(std::vector<std::deque<Vector2<double>> > *groups, double t_lim , bool is_align)
{
	double a, b, c;
	double x1, y1, x2, y2;
	int points_size, points_size_2nd;
	double t, t_max;
	std::vector<int> merge_index;
	std::deque<Vector2<double>> new_line;
	std::vector<std::deque<Vector2<double>> > new_group;
//	INFO_BLUE("mergeLine");
//	ROS_INFO("(*groups).size(%d)", (*groups).size());
	merge_index.clear();
	new_line.clear();
	new_group.clear();
//	if (!groups->empty()) {
	if (groups->size() > 1) {//it should not merge when the size is 0 or 1
		for (auto iter = groups->begin(); iter != groups->end(); ++iter) {
			t = 0;
			t_max = 0;
			//for merge the last one and the first one
			auto it_next_line = (iter == groups->end()-1) ? groups->begin() : (iter + 1);
/*			if (iter == groups->end()-1) {
				ROS_ERROR("000");
			} else {
				ROS_ERROR("111");
			}*/
			x1 = iter->begin()->x;
			y1 = iter->begin()->y;
			x2 = (it_next_line->end() - 1)->x;
			y2 = (it_next_line->end() - 1)->y;
			if (x1 != x2 && y1 != y2) {
				a = y2 - y1;
				b = x1 - x2;
				c = x2 * y1 - x1 * y2;
			} else {
				continue;
			}
			points_size = static_cast<int>(iter->size());
			points_size_2nd = static_cast<int>(it_next_line->size());
//			ROS_INFO("points_size(1:%d, 2:%d)",points_size, points_size_2nd);

			/*loop for checking the first line*/
			for (int j = 0; j < points_size; j++) {
				t = std::abs((a * (iter->begin() + j)->x + b * (iter->begin() +j)->y + c) / sqrt(a * a + b * b));
				if (t >= t_max) {
					t_max = t;
				}
//				ROS_INFO("1st t = %lf, t_max = %lf", t, t_max);
			}

			/*loop for checking the second line*/
			for (int j = 0; j < points_size_2nd; j++) {
				t = std::abs((a * (it_next_line->begin() + j)->x + b * (it_next_line->begin() +j)->y + c) / sqrt(a * a + b * b));
				if (t >= t_max) {
					t_max = t;
				}
//				ROS_INFO("2st t = %lf, t_max = %lf", t, t_max);
			}

			if (t_max < t_lim) {
				int index = static_cast<int>(std::distance((*groups).begin(), iter));
				merge_index.push_back(index);
//				ROS_ERROR("push_back(index) = %d", index);
			}
		}
	}
	if (groups->size() == 2 && merge_index.size() != 0) {
		merge_index.pop_back();//for the groups size is 2, it should not merge twice
//		ROS_INFO("merge_index.pop_back()");
	}

	/*do merge*/
//	ROS_WARN("before do merge! (*groups).size() = %d", (*groups).size());
	if (!merge_index.empty()) {
		int loop_count = 0;
		auto iter = (*groups).begin();
		for (std::vector<int>::iterator m_iter = merge_index.begin(); m_iter != merge_index.end(); ++m_iter) {
//			std::vector<std::deque<Vector2<double>> >::iterator iter = (*groups).begin() + (*m_iter);
//			ROS_WARN("merging! (*groups).size() = %d", (*groups).size());
			if (groups->size() < 2) {
//				ROS_WARN("groups size < 2, break merge!");
				break;
			}
			if (m_iter != merge_index.begin()) {
				iter += (*m_iter - *(m_iter -1) - 1);
//				ROS_INFO("iter += %d", (*m_iter - *(m_iter -1) - 1));
			} else {
				iter += (*m_iter);
//				ROS_INFO("iter += %d", (*m_iter));
			}
//			ROS_INFO("(*m_iter) = %d", (*m_iter));
			if (iter == (*groups).end()-1) {//to merge the last one and the first one
				points_size = static_cast<int>(iter->size());
				points_size_2nd = static_cast<int>(((*groups).begin())->size());
//				ROS_INFO("points_size_2nd = %d", points_size_2nd);
				for (int j = 0; j < points_size; j++) {
					new_line.push_back(*(iter->begin() + j));
				}
				for (int j = 0; j < points_size_2nd; j++) {
//					ROS_INFO("8");
					new_line.push_back(*(((*groups).begin()->begin()) + j));
				}
				iter = (*groups).erase(iter);
				iter = (*groups).erase((*groups).begin());
//				ROS_INFO("(*groups).erase((*groups).begin())");
				iter = (*groups).insert(iter, new_line);
/*				(*groups).erase(iter - loop_count);
				(*groups).erase(iter + 1 - loop_count);
				(*groups).insert(iter - loop_count, new_line);*/
				new_line.clear();
				loop_count++;
				break;
			} else {
//				ROS_INFO("9");
				points_size = static_cast<int>(iter->size());
				points_size_2nd = static_cast<int>((iter + 1)->size());
				for (int j = 0; j < points_size; j++) {
					new_line.push_back(*(iter->begin() + j));
				}
				for (int j = 0; j < points_size_2nd; j++) {
					new_line.push_back(*((iter + 1)->begin() + j));
				}
/*				(*groups).erase(iter - loop_count);
				(*groups).erase(iter + 1 - loop_count);
				(*groups).insert(iter - loop_count, new_line);*/

				iter = (*groups).erase(iter);
				iter = (*groups).erase(iter);
				iter = (*groups).insert(iter, new_line);

				new_line.clear();
				loop_count++;

			}
		}
	}
//	ROS_WARN("after do merge! (*groups).size() = %d", (*groups).size());

	if(is_align){
		//sort from long to short
		std::sort((*groups).begin(),(*groups).end(),[](std::deque<Vector2<double>> a,std::deque<Vector2<double>> b){
//			ROS_INFO("0");
			auto a_dis = pow((a.begin()->x - (a.end()-1)->x),2) + pow((a.begin()->y - (a.end()-1)->y),2);
			auto b_dis = pow((b.begin()->x - (b.end()-1)->x),2) + pow((b.begin()->y - (b.end()-1)->y),2);
			return a_dis > b_dis;
		});
		//filter line which is shorter than 0.3m
		auto loc = std::find_if((*groups).begin(),(*groups).end(),[](std::deque<Vector2<double>> ite){
			auto dis = sqrtf(powf(static_cast<float>(ite.begin()->x - (ite.end() - 1)->x), 2) + powf(
							static_cast<float>(ite.begin()->y - (ite.end() - 1)->y), 2));
			return dis < 0.3;
		});
		auto dis = std::distance((*groups).begin(),loc);
		(*groups).resize(dis);
	}
	//for erase the line which line is shorter than 10 cm.
	if (!is_align) {
		groups->erase(std::remove_if(groups->begin(),groups->end(),[](const std::deque<Vector2<double>>& a){
			return sqrt(pow(a.front().x - a.back().x,2) + pow(a.front().y - a.back().y,2)) < 0.10;
		}),groups->end());
	}
#if 0
	for(auto &ite:(*groups)) {
//		ROS_INFO("6");
		ACleanMode::pubPointMarkers(&ite,"base_link","merge");
	}
#endif
	return true;
}

bool Lidar::fitLineGroup(std::vector<std::deque<Vector2<double>> > *groups, double dis_lim, bool is_align)
{
	double 	a, b, c;
	double	x_0;
	int	loop_count = 0;
	LineABC	new_fit_line;
//	INFO_BLUE("fitLineGroup");
//	ROS_INFO("(*groups).size(%d)", (*groups).size());
	fit_line.clear();
	if (!(*groups).empty()) {
		for (std::vector<std::deque<Vector2<double>> >::iterator iter = (*groups).begin(); iter != (*groups).end(); ++iter) {

			lineFit((*iter), a, b, c);

			new_fit_line.A = a;
			new_fit_line.B = b;
			new_fit_line.C = c;
			new_fit_line.x1 = iter->begin()->x;
			new_fit_line.x2 = (iter->end()-1)->x;
			new_fit_line.y1 = iter->begin()->y;
			new_fit_line.y2 = (iter->end()-1)->y;
			new_fit_line.len = static_cast<int>(iter->size());

			x_0 = 0 - c / a;
			ROS_INFO("a = %lf, b = %lf, c = %lf", a, b, c);
			ROS_INFO("x_0 = %lf", x_0);
			/*erase the lines which are far away from the robot*/
			double line_to_robot_dis = std::abs(c / (sqrt(a * a + b * b)));
			const auto DIS_MIN = ROBOT_RADIUS - 0.02;
			new_fit_line.dis = line_to_robot_dis;
			if (line_to_robot_dis > dis_lim || line_to_robot_dis < DIS_MIN || (is_align ? 0 : x_0 < 0)) {
//				ROS_ERROR("the line is too far away from robot. line_to_robot_dis = %lf,x_0:%lf,dis_lim:(%lf, %lf)", line_to_robot_dis,x_0,dis_lim, DIS_MIN);
				continue;
			}
			fit_line.push_back(new_fit_line);
//			ROS_INFO("%s %d: line_angle%d = %lf", __FUNCTION__, __LINE__, loop_count, line_angle);
			loop_count++;
		}
		ACleanMode::pubLineMarker(&fit_line);
	} else {
		fit_line_marker.points.clear();
//		robot::instance()->pubFitLineMarker(fit_line_marker);
	}
	fit_line_marker.points.clear();

	return true;
}

void Lidar::pubFitLineMarker(double a, double b, double c, double y1, double y2)
{
	double x1, x2;
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
	lidar_points_.x = 0.0;
	lidar_points_.y = 0.0;
	lidar_points_.z = 0.0;

	x1 = (0 - b * y1 - c) / a;
	x2 = (0 - b * y2 - c) / a;

	lidar_points_.x = x1;
	lidar_points_.y = y1;
	fit_line_marker.points.push_back(lidar_points_);
	lidar_points_.x = x2;
	lidar_points_.y = y2;
	fit_line_marker.points.push_back(lidar_points_);
//	robot::instance()->pubFitLineMarker(fit_line_marker);
}

/*----set lidar marker according to direction-----*/
static uint8_t setLidarMarkerAcr2Dir(double X_MIN,double X_MAX,int angle_from,int angle_to,int dx,int dy,const sensor_msgs::LaserScan *scan_range,uint8_t *lidar_status,uint8_t obs_status)
{
	double x,y,th;
	const	double Y_MIN = 0.140;//ROBOT_RADIUS
	const	double Y_MAX = 0.237;//0.279
	int count = 0;
	uint8_t ret = 0;
	int i =angle_from;
	for (int j = angle_from; j < angle_to; j++) {
		i = j>359? j-360:j;
		if (scan_range->ranges[i] < 4) {
			th = i*1.0 + 180.0;
			if(j >= 314 && j < 404){//314~44
				x = -cos(degree_to_radian(th)) * scan_range->ranges[i];
				//y = -sin(degree_to_radian(th)) * scan_range->ranges[i];
				if (x > X_MIN && x < X_MAX ) {
					count++;
				}
			}
			else if( j >= 44 && j < 134){
				//x = cos(degree_to_radian(th)) * scan_range->ranges[i];
				y = -sin(degree_to_radian(th)) * scan_range->ranges[i];
				if (y > Y_MIN && y < Y_MAX ) {
					count++;
				}
			}
			else if( j >= 134 && j < 224){
				x = cos(degree_to_radian(th)) * scan_range->ranges[i];
				//y = sin(degree_to_radian(th)) * scan_range->ranges[i];
				if (x > X_MIN && x < X_MAX ) {
					count++;
				}
			}
			else if( j >= 224 && j < 314){
				//x = cos(degree_to_radian(th)) * scan_range->ranges[i];
				y = sin(degree_to_radian(th)) * scan_range->ranges[i];
				if (y > Y_MIN && y < Y_MAX ) {
					count++;
				}
			}
		}
	}
	if (count > 10) {
		ret = 1;
		*lidar_status |= obs_status;
	}
	return ret;

}

uint8_t Lidar::lidarMarker(std::vector<Vector2<int>> &markers, int movement_i, int action_i, double X_MAX)
{
//	markers.clear();

	if(!lidarCheckFresh(0.6,4))
		return 0;

	lidarXYPoint_mutex_.lock();
	auto tmp_lidarXY_points = lidarXY_points;
	lidarXYPoint_mutex_.unlock();
	double x, y;
	int dx{}, dy{};
	const	double Y_MAX = 0.20;//0.279
	int	count_array[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
	Vector2<int> marker = {dx,dy};

	for(const auto& point:lidarXY_points){
		x = point.x;
		y = point.y;
		auto dis_to_robot = sqrt(pow(x, 2) + pow(y, 2));
		if (dis_to_robot <= X_MAX) {
			//front
			if (x > 0 && x < X_MAX) {
				//middle
				if (y > -0.056 && y < 0.056) {
					count_array[0]++;
				}
			}
			if (x > 0.056 && x < X_MAX) {
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
			if (x < 0 && x > (-X_MAX)) {
				//middle
				if (y > -0.056 && y < 0.056) {
					count_array[3]++;
				}
			}
			if (x < -0.056 && x > (-X_MAX)) {
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
			if (y > 0 && y < Y_MAX) {
				//middle
				if (x > -0.056 && x < 0.056) {
					count_array[6]++;
				}
				//front
				if (x > 0.056 && x < 0.168) {
					count_array[7]++;
				}
				//back
				if (x > -0.168 && x < -0.056) {
					count_array[8]++;
				}
			}
			//right
			if (y < 0 && y >  (0 - Y_MAX)) {
				//middle
				if (x > -0.056 && x < 0.056) {
					count_array[9]++;
				}
				//front
				if (x > 0.056 && x < 0.168) {
					count_array[10]++;
				}
				//back
				if (x > -0.168 && x < -0.056) {
					count_array[11]++;
				}
			}
		}
	}

	std::string msg = "";
	std::string direction_msg = "";
	uint8_t block_status = 0;
	for (int i = 0; i < 12; i++) {
		if (count_array[i] > 10) {
			switch(i) {
				case 0 : {
					dx = 2;
					dy = 0;
					direction_msg = "front middle";
					block_status |= BLOCK_FRONT;
					break;
				}
				case 1 : {
					dx = 2;
					dy = 1;
					direction_msg = "front left";
					block_status |= BLOCK_LEFT;
					break;
				}
				case 2 : {
					dx = 2;
					dy = -1;
					direction_msg = "front right";
					block_status |= BLOCK_RIGHT;
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
		}
/*		if (!direction_msg.empty()) {
			ROS_INFO("%s %d: \033[36mlidar marker: %s.\033[0m", __FUNCTION__, __LINE__, direction_msg.c_str());
			direction_msg.clear();
		}*/
		marker.x = dx;
		marker.y = dy;
		auto is_left_front = (dx == 1 && dy == 2);
		auto is_right_front = (dx == 1 && dy == -2);
		auto is_left_back = (dx == -1 && dy == 2);
		auto is_right_back = (dx == -1 && dy == -2);
		if (!(dx == 0 && dy == 0)){
			if(!(action_i == 7 && (is_left_back || is_left_front))
						&& !(action_i == 8 && (is_right_back || is_right_front))
//						&& !(movement_i == 2 && (is_left_back || is_right_back))
						&& !(movement_i == 2)) {
				markers.push_back(marker);
//				ROS_WARN("movement_i = %d, action_i = %d", movement_i, action_i);
			} else {
//				ROS_ERROR("movement_i = %d, action_i = %d", movement_i, action_i);
			}
		}

		dx = 0;
		dy = 0;
	}

	return block_status;
}

void Lidar::checkRobotSlip()
{
	if (!slip_enable_ || !lidar.isScanOriginalReady())
	{
		slip_status_ = false;
		return;
	}

	checkSlipInit(acur1_,acur2_,acur3_,acur4_);
	if ((std::abs(wheel.getLeftWheelActualSpeed()) >= 0.10 || std::abs(wheel.getRightWheelActualSpeed()) >= 0.10))
	{
		auto tmp_scan_data = getLidarScanDataOriginal();
		uint16_t same_count = 0;
		uint16_t tol_count = 0;

//		ROS_INFO("start to check slip,leftSpeed:%f,rightSpeed:%f,lidarPoint:%lf",wheel.getLeftWheelActualSpeed(),wheel.getRightWheelActualSpeed(),tmp_scan_data.ranges[155]);
		if(last_slip_scan_frame_.d.size() < 3){
			last_slip_scan_frame_.d.push_back(tmp_scan_data);
			return;
		}

		for(int i = 0; i <= 359; i++){
			if(tmp_scan_data.ranges[i] < dist1_){
				tol_count++;
				if(tmp_scan_data.ranges[i] >dist2_ && tmp_scan_data.ranges[i] < dist1_){//
					if(std::abs(tmp_scan_data.ranges[i] - last_slip_scan_frame_.d.at(0).ranges[i]) <= acur1_ ){
						same_count++;
					}
				}
				else if(tmp_scan_data.ranges[i] >dist3_ && tmp_scan_data.ranges[i] < dist2_){//
					if(std::abs(tmp_scan_data.ranges[i] - last_slip_scan_frame_.d.at(0).ranges[i]) <= acur2_ ){
						same_count++;
					}
				}
				else if(tmp_scan_data.ranges[i] >dist4_ && tmp_scan_data.ranges[i] < dist3_){//
					if(std::abs(tmp_scan_data.ranges[i] - last_slip_scan_frame_.d.at(0).ranges[i]) <= acur3_ ){
						same_count++;
					}
				}
				else if(tmp_scan_data.ranges[i] <= dist4_){
					if(std::abs( tmp_scan_data.ranges[i] - last_slip_scan_frame_.d.at(0).ranges[i]) <= acur4_ ){
						same_count++;
					}
				}
			}
		}

//		ROS_WARN("%s %d: same_count: %d, total_count: %d.", __FUNCTION__, __LINE__, same_count, tol_count);
		if((same_count*1.0)/(tol_count*1.0) >= slip_ranges_percent_ &&
			(slip_ranges_percent_ < 0.8 || (tmp_scan_data.ranges[155] < 4 &&
								(tmp_scan_data.ranges[155] - last_slip_scan_frame_.d.at(0).ranges[155] < 0.03)))){
				if (++slip_frame_cnt_ >= slip_cnt_limit_) {
					ROS_INFO("\033[35m""%s,%d,robot slip!!""\033[0m", __FUNCTION__, __LINE__);
					slip_status_ = true;
					slip_cnt_limit_ = 5;
					beeper.beepForCommand(VALID);
				} else {
					slip_status_ = false;
				}
		}
		else
		{
			slip_frame_cnt_ = 0;
			slip_status_ = false;
		}
		last_slip_scan_frame_.push_back(tmp_scan_data);
	}else{
		slip_frame_cnt_ = 0;
		last_slip_scan_frame_.d.clear();
	}
}

bool Lidar::isRobotSlip()
{
	return slip_status_;
}


int Lidar::compLaneDistance()
{
	int ret = 0;
	static  uint32_t seq = 0;
	double x,y,th,x1,y1;
	int angle_from, angle_to;
	double x_front_min = 4;
	double x_back_min = 4;

	scanLinear_mutex_.lock();
	auto tmp_scan_data = lidarScanData_linear_;
	scanLinear_mutex_.unlock();

	if (tmp_scan_data.header.seq == seq) {
		//ROS_WARN("lidar seq_ still same, quit!seq_ = %d", tmp_scan_data.header.seq_);
		return 0;
	}
	ROS_INFO("compLaneDistance");
	seq = tmp_scan_data.header.seq;
//	int cur_radian = robot::instance()->getWorldPoseRadian() / 10;
	auto cur_radian = robot::instance()->getWorldPoseRadian();
#if 0
	angle_from = 149 - cur_radian;
	angle_to = 210 - cur_radian;
	ROS_INFO("cur_radian = %d", cur_radian);
	for (int j = angle_from; j < angle_to; j++) {
		int i = j>359? j-360:j;
		if (tmp_scan_data.ranges[i] < 4) {
			th = i*1.0 + 180.0;
			x = cos(th * PI / 180.0) * tmp_scan_data.ranges[i];
			y = sin(th * PI / 180.0) * tmp_scan_data.ranges[i];
			x1 = x * cos(0 - cur_radian * PI / 180.0) + y * sin(0 - cur_radian * PI / 180.0);
			//ROS_INFO("x = %lf, y = %lf, x1 = %lf, y1 = %lf", x, y, x1, y1);
			if (std::abs(y1) < ROBOT_RADIUS) {
				if (std::abs(x1) <= x_front_min) {
					x_front_min = std::abs(x1);
					//ROS_WARN("x_front_min = %lf", x_front_min);
				}
			}
		}
	}

	angle_from = 329 - cur_radian;
	angle_to = 400 - cur_radian;
	for (int j = angle_from; j < angle_to; j++) {
		int i = j>359? j-360:j;
		if (tmp_scan_data.ranges[i] < 4) {
			th = i*1.0 + 180.0;
			x = cos(th * PI / 180.0) * tmp_scan_data.ranges[i];
			y = sin(th * PI / 180.0) * tmp_scan_data.ranges[i];
			x1 = x * cos(0 - cur_radian * PI / 180.0) + y * sin(0 - cur_radian * PI / 180.0);
			y1 = y * cos(0 - cur_radian * PI / 180.0) - x * sin(0 - cur_radian * PI / 180.0);
			//ROS_INFO("x = %lf, y = %lf, x1 = %lf, y1 = %lf", x, y, x1, y1);
			if (std::abs(y1) < ROBOT_RADIUS) {
				if (std::abs(x1) <= x_back_min) {
					x_back_min = std::abs(x1);
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
			x = cos(degree_to_radian(th)) * tmp_scan_data.ranges[i];
			y = sin(degree_to_radian(th)) * tmp_scan_data.ranges[i];
			coordinate_transform(&x, &y, LIDAR_THETA, LIDAR_OFFSET_X, LIDAR_OFFSET_Y);
			coordinate_transform(&x, &y, cur_radian, 0, 0);
			//ROS_INFO("x = %lf, y = %lf", x, y);
			if (std::abs(y) < ROBOT_RADIUS) {
				if (x >= 0){
					if (std::abs(x) <= x_front_min) {
						x_front_min = std::abs(x);
						ROS_WARN("x_front_min = %lf",x_front_min);
					}
				} else {
					if (std::abs(x) <= x_back_min) {
						x_back_min = std::abs(x);
						ROS_ERROR("x_back_min = %lf", x_back_min);
					}
				}
			}
		}
	}
	if(x_front_min < x_back_min)
		ret = 0;
	if(x_front_min > x_back_min)
		ret = 1;
	if(x_front_min == x_back_min)
		ret = -1;
	return ret;
}

/*
 * param1 angle range(179~-179)
 * return distance
 */
double Lidar::getLidarDistance(int16_t angle,float  range_max,float range_min)
{
	double distance = 0.0f;
	if(angle <= -180 || angle >= 180){
		return distance;
	}
	lidarXYPoint_mutex_.lock();
	std::vector<geometry_msgs::Point>  lidar_points = lidarXY_points;
	lidarXYPoint_mutex_.unlock();
	int16_t point_angle;
	const int offset = 5;
	int count = 0;
	for(auto& point:lidar_points){
		/*---to calculate the angle base on robot position , from current lidar points*/
		if(point.x >= 0){
			if(point.y < 0){
				point_angle = (int16_t)radian_to_degree(atan(point.x/point.y));
				point_angle = -90 - point_angle;
			}
			else if(point.y > 0){
				point_angle = (int16_t)radian_to_degree(atan(point.x/point.y));
				point_angle = 90 - point_angle;
			}
			else if(point.y == 0)
				point_angle = 0;

		}
		else if(point.x < 0)
		{
			if(point.y < 0){
				point_angle = (int16_t)radian_to_degree(atan(point.x/point.y));
				point_angle = -90 - point_angle;
			}
			else if(point.y > 0){
				point_angle = (int16_t)radian_to_degree(atan(point.x/point.y));
				point_angle = 90 - point_angle;
			}
			else if(point.y == 0)
				point_angle = -180;
		}
		/*---- end ---*/

		int diff_angle = abs(point_angle - angle);
		if(diff_angle > 180)//range angle
			diff_angle = 360 - diff_angle;
		if(diff_angle <= offset){
			float tmp_dist = sqrt( pow(point.x, 2.0) + pow(point.y, 2.0) );
			if(tmp_dist > range_min && tmp_dist < range_max){
				distance += tmp_dist; 
				count++;
			}
		}
		if(count >= offset*2)
			break;
	}
	if(count >0)
		distance = distance /(count*1.0);
	ROS_INFO("\033[1;40;32m%s,%d,last_point_angle = %d, input angle %d,distance = %f\033[0m",__FUNCTION__,__LINE__,point_angle,angle,distance);
	return 	distance;
}

double Lidar::getObstacleDistance(uint8_t dir, double range)
{
	if(!lidarCheckFresh(0.6,4))
		return DBL_MAX;

	lidarXYPoint_mutex_.lock();
	auto tmp_lidarXY_points = lidarXY_points;
	lidarXYPoint_mutex_.unlock();
	double x,y;
	double x_to_robot,y_to_robot;
	double min_dis = DBL_MAX;

	if(range < 0.056)
	{
		ROS_ERROR("range should be higher than 0.056");
		return DBL_MAX;
	}

	for(const auto& point:tmp_lidarXY_points){
		x = point.x;
		y = point.y;
		x_to_robot = std::abs(x) - ROBOT_RADIUS * sin(acos(std::abs(y) / ROBOT_RADIUS));
		y_to_robot = std::abs(y) - ROBOT_RADIUS * sin(acos(std::abs(x) / ROBOT_RADIUS));
//		ROS_INFO("x = %lf, y = %lf", x, y);
		if (dir == 0) {
			if(std::abs(y) < range){
				if(x > 0){
					if (x_to_robot < min_dis) {
						min_dis = x_to_robot;
//						ROS_WARN("back = %lf", min_dis);
						}
					}
			}
		} else if (dir == 1) {
				if (std::abs(y) < range) {
					if (x < 0){
						if (x_to_robot < min_dis) {
							min_dis = x_to_robot;
							//ROS_WARN("back = %lf", back);
						}
					}
				}
		} else if (dir == 2) {
				if (std::abs(x) < range) {
					if (y >= 0 && x > 0){
						if (y_to_robot < min_dis) {
							min_dis = y_to_robot;
							//ROS_WARN("left = %lf",left);
						}
					}
				}
		} else if (dir == 3) {
				if (std::abs(x) < range) {
					if (y < 0 && x > 0){
						if (y_to_robot < min_dis) {
							min_dis = y_to_robot;
							//ROS_WARN("right = %lf",right);
						}
					}
				}
		}
	}
	return min_dis;
}

bool Lidar::lidarCheckFresh(float duration, uint8_t type)
{
	double time_gap;
	if (type == 1)
		time_gap = ros::Time::now().toSec() - scanLinear_update_time_;
	if (type == 2)
		time_gap = ros::Time::now().toSec() - scanOriginal_update_time_;
	if (type == 3)
		time_gap = ros::Time::now().toSec() - scanCompensate_update_time_;
	if (type == 4)
		time_gap = ros::Time::now().toSec() - scanXYPoint_update_time_;

	if (time_gap < duration)
	{
		//ROS_INFO("%s %d: type:%d, time_gap(%lf) < duration(%f).", __FUNCTION__, __LINE__, type, time_gap, duration);
		return true;
	}

	//ROS_INFO("%s %d: type:%d, time_gap(%lf), duration(%f).", __FUNCTION__, __LINE__, type, time_gap, duration);
	return false;
}

//
//void Lidar::pubPointMarker(std::vector<Vector2<double>> *point)
//{
//	int points_size;
//	visualization_msgs::Marker point_marker;
//	point_marker.ns = "point_marker";
//	point_marker.id = 0;
//	point_marker.type = visualization_msgs::Marker::SPHERE_LIST;
//	point_marker.action= 0;//add
//	point_marker.lifetime=ros::Duration(0);
//	point_marker.scale.x = 0.05;
//	point_marker.scale.y = 0.05;
//	point_marker.scale.z = 0.05;
//	point_marker.color.r = 0.0;
//	point_marker.color.g = 1.0;
//	point_marker.color.b = 0.0;
//	point_marker.color.a = 1.0;
//	point_marker.header.frame_id = "/base_link";
//	point_marker.header.stamp = ros::Time::now();
//	laser_points_.x = 0.0;
//	laser_points_.y = 0.0;
//	laser_points_.z = 0.0;
//
//	if (!(*point).empty()) {
//		for (std::vector<Vector2<double>>::iterator iter = (*point).begin(); iter != (*point).end(); ++iter) {
//			laser_points_.x = iter->x;
//			laser_points_.y = iter->y;
//			point_marker.points.push_back(laser_points_);
//		}
//		point_marker_pub.publish(point_marker);
//		point_marker.points.clear();
//	} else {
//		point_marker.points.clear();
//		point_marker_pub.publish(point_marker);
//	}
//}

void Lidar::setLidarScanDataOriginal(const sensor_msgs::LaserScan::ConstPtr &scan)
{
	boost::mutex::scoped_lock lock(scanOriginal_mutex_);
	lidarScanData_original_ = *scan;
}

sensor_msgs::LaserScan Lidar::getLidarScanDataOriginal()
{
	boost::mutex::scoped_lock lock(scanOriginal_mutex_);
	return lidarScanData_original_;
}

void Lidar::init()
{
	PP_INFO();
	switch_ = OFF;

	angle_n_ = {};
	is_scanLinear_ready_ = {};
	is_scanOriginal_ready_ = {};
	is_scanCompensate_ready_ = {};

	lidarScanData_linear_ = {};
	lidarScanData_original_ = {};
	lidarScanData_compensate_ = {};
	lidarXY_points = {};
	scanLinear_update_time_ = {};
	scanOriginal_update_time_ = {};
	scanCompensate_update_time_ = {};
	scanXYPoint_update_time_ = {};

	Lidar_Point = {};
	Lidar_Group = {};
	Lidar_Group_2nd = {};
	fit_line = {};

	fit_line_marker = {};

	lidar_points_ = {};

	// For aligning.
	align_finish_ = {};
	align_radian_ = {};
	laser_points_ = {};

	// For slip checking
	slip_status_ = {false};
	slip_frame_cnt_ = {0};
	slip_scan_deque last_frame_{};
	wheel_cliff_trigger_time_ = 0;
	gyro_tilt_trigger_time_ = 0;

	setScanLinearReady(0);
	setScanOriginalReady(0);
	setScanCompensateReady(0);
	return;
}

bool Lidar::lidar_is_stuck()
{
	if (lidar.isScanOriginalReady() && !lidar.lidarCheckFresh(4, 2))
		return true;
	return false;
}

uint8_t Lidar::lidar_get_status(int movement_i, int action_i)
{
	std::vector<Vector2<int>> markers;

	if (isScanCompensateReady())
		return lidarMarker(markers, movement_i, action_i);

	return 0;
}

void Lidar::checkSlipInit(float &acur1, float &acur2, float &acur3, float &acur4) {
	//For cliff trigger
	if(wheel.getLeftWheelCliffStatus() || wheel.getRightWheelCliffStatus()){
		ROS_INFO("%s,%d,robot wheel cliff detect",__FUNCTION__,__LINE__);
		wheel_cliff_trigger_time_ = ros::Time::now().toSec();
	}
	//For tilt trigger
	if(/*robot::instance()->checkTiltToSlip()*/0){
		ROS_INFO("%s,%d,robot tilt detect",__FUNCTION__,__LINE__);
		gyro_tilt_trigger_time_ = ros::Time::now().toSec();
	}

	if(ros::Time::now().toSec() - gyro_tilt_trigger_time_ < 2){
		slip_cnt_limit_ = 5;
		slip_ranges_percent_ = 0.78;
		acur1 = 0.085;
		acur2 = 0.065;
		acur3 = 0.045;
		acur4 = 0.01;
	}	else if(ros::Time::now().toSec() - wheel_cliff_trigger_time_ < 2){
		slip_cnt_limit_ = 3;
		slip_ranges_percent_ = 0.75;
		acur1 = 0.090;
		acur2 = 0.080;
		acur3 = 0.060;
		acur4 = 0.020;
	} else{
		slip_cnt_limit_ = 5;
		slip_ranges_percent_ = 0.8;
		acur1 = 0.085;
		acur2 = 0.065;
		acur3 = 0.045;
		acur4 = 0.01;
	}
}

double Lidar::checkIsRightAngle(bool is_left) {
	if(isScanOriginalReady() == 0){
//		INFO_BLUE("ScanOriginal NOT Ready! Break!");
		return false;
	}

	scanOriginal_mutex_.lock();
	auto tmp_scan_data = lidarScanData_original_;
	scanOriginal_mutex_.unlock();

	int count{};
	double wall_length{};

	for (int i = 359; i >= 0; i--) {
//		ROS_INFO("laser point(%d, %lf)", i, scan->ranges[i]);
		if (tmp_scan_data.ranges[i] < 4) {
			auto point = polarToCartesian(tmp_scan_data.ranges[i], i);
//			ROS_ERROR("point(%d, %lf, %lf)", i, point.x, point.y);
			if (is_left) {
				if (point.y > 0.160 && point.y < 0.20) {
					if (point.x > 0 && point.x < 0.167) {
//						ROS_INFO("point(%d, %lf, %lf)", i, point.x, point.y);
						count++;
						wall_length = point.x > wall_length ? point.x : wall_length;
					}
					if (point.x > -0.05 && point.x < 0) {
//						ROS_INFO("point(%d, %lf, %lf)", i, point.x, point.y);
						count += 10;
						wall_length = point.x > wall_length ? point.x : wall_length;
					}
				}
			} else {
				if (point.y < -0.160 && point.y > -0.20) {
					if (point.x > 0 && point.x < 0.167) {
//						ROS_INFO("point(%d, %lf, %lf)", i, point.x, point.y);
						count++;
						wall_length = point.x > wall_length ? point.x : wall_length;
					}
					if (point.x > -0.05 && point.x < 0) {
//						ROS_INFO("point(%d, %lf, %lf)", i, point.x, point.y);
						count += 10;
						wall_length = point.x > wall_length ? point.x : wall_length;
					}
				}
			}
		}
	}
//	ROS_WARN("count = %d, is_left = %d", count, is_left);
	if (count > 10) {
		return wall_length;
	} else {
		return wall_length;
	}
}

void Lidar::saveLidarDataToFile(uint32_t seq, sensor_msgs::LaserScan scan)
{
	std::string file = "/var/volatile/tmp/lidar_data_";
	file += std::to_string(seq);
	if (access(file.c_str(), F_OK) != -1)
		// If file exist, no need to generate a new one.
		return;

	FILE *f_write = fopen(file.c_str(), "w");
	if (f_write == nullptr)
		ROS_ERROR("%s %d: Open %s error.", __FUNCTION__, __LINE__, file.c_str());
	else
	{
		ROS_INFO("%s %d: Start writing data to %s.", __FUNCTION__, __LINE__, file.c_str());
		fprintf(f_write, "Seq: %02d\n", scan.header.seq);
		for (int index = 0; index < 360; index++)
		{
			fprintf(f_write, "%f,", scan.ranges[index]);

			if ((100 * index / 360) % 10 == 0)
				printf("\r %d0%%.", 100 * index / 360 / 10);
		}
		printf("\n");
		fprintf(f_write, "\n");
		fclose(f_write);
		ROS_INFO("%s %d: Write data succeeded.", __FUNCTION__, __LINE__);
	}

}

//void Lidar::readLidarDataFromFile(uint32_t seq, float (&scan_data)[360])
void Lidar::readLidarDataFromFile(bool check_front, float (&scan_data)[360])
{
//	std::string file = "/var/volatile/tmp/lidar_data_";
//	file += std::to_string(seq);
	std::string file;
	if (check_front)
		file = "/opt/ros/indigo/share/pp/lidar_checking/front";
	else
		file = "/opt/ros/indigo/share/pp/lidar_checking/back";

	FILE *f_read = fopen(file.c_str(), "r");
	if (f_read == nullptr)
		ROS_ERROR("%s %d: Open %s error.", __FUNCTION__, __LINE__, file.c_str());
	else
	{
		ROS_INFO("%s %d: Start reading data from %s.", __FUNCTION__, __LINE__, file.c_str());
		uint8_t header_len = 8;
		char header[header_len];
		fgets(header, header_len, f_read);
		ROS_INFO("%s %d: First line of file: %s.", __FUNCTION__, __LINE__, header);
		for (int index = 0; index < 360; index++)
		{
			fscanf(f_read, "%f,", &scan_data[index]);
			if ((100 * index / 360) % 10 == 0)
				printf("\r %d0%%.", 100 * index / 360 / 10);
			printf("scan_buf[%03d]=%f\n", index, scan_data[index]);
		}
		printf("\n");
		fclose(f_read);
		ROS_INFO("%s %d: Read data succeeded.", __FUNCTION__, __LINE__);
	}
}

bool Lidar::scanDataChecking(bool check_front, sensor_msgs::LaserScan scan, float (&ref_scan_data)[360])
{
	uint16_t _valid_count{0};
	float _accuracy{0.07};
	if (check_front)
	{
		for (uint16_t data_index = 0; data_index < 360; data_index++)
		{
			printf("scan.ranges[%03d] = %f, ref_scan_data[%03d] = %f, fabs = %f.",
				   data_index, scan.ranges[data_index], data_index, ref_scan_data[data_index],
				   fabs(scan.ranges[data_index] - ref_scan_data[data_index]));
			if (scan.ranges[data_index] != std::numeric_limits<float>::infinity() &&
				ref_scan_data[data_index] != std::numeric_limits<float>::infinity())
//				ref_scan_data[data_index] != 0)
			{
				printf(" Pass inf checking. Accuracy:%f.", ref_scan_data[data_index] * _accuracy);
//				if (fabs(scan.ranges[data_index] - ref_scan_data[data_index]) < ref_scan_data[data_index] * _accuracy)
				if (fabs(scan.ranges[data_index] - ref_scan_data[data_index]) < 0.2)
				{
					_valid_count++;
					printf(" Pass accuracy checking.\n");
				}
				else
					printf("\n");
			}
			else if (scan.ranges[data_index] == std::numeric_limits<float>::infinity() &&
				ref_scan_data[data_index] == std::numeric_limits<float>::infinity())
//				ref_scan_data[data_index] < 0.01)
			{
				printf(" Pass accuracy checking.\n");
				_valid_count++;
			}
			else
				printf("\n");

			if (data_index == 64)
				data_index = 244;
		}
	}
	else
	{
		for (uint16_t data_index = 66; data_index < 244; data_index++)
		{
			printf("scan.ranges[%03d] = %f, ref_scan_data[%03d] = %f, fabs = %f.",
				   data_index, scan.ranges[data_index], data_index, ref_scan_data[data_index],
				   fabs(scan.ranges[data_index] - ref_scan_data[data_index]));
			if (scan.ranges[data_index] != std::numeric_limits<float>::infinity() &&
				ref_scan_data[data_index] != std::numeric_limits<float>::infinity())
//				ref_scan_data[data_index] != 0)
			{
				printf(" Pass inf checking. Accuracy:%f.", ref_scan_data[data_index] * _accuracy);
//				if (fabs(scan.ranges[data_index] - ref_scan_data[data_index]) < ref_scan_data[data_index] * _accuracy)
				if (fabs(scan.ranges[data_index] - ref_scan_data[data_index]) < 0.2)
				{
					_valid_count++;
					printf(" Pass accuracy checking.\n");
				}
				else
					printf("\n");
			}
			else if (scan.ranges[data_index] == std::numeric_limits<float>::infinity() &&
				ref_scan_data[data_index] == std::numeric_limits<float>::infinity())
//				ref_scan_data[data_index] < 0.01)
			{
				printf(" Pass accuracy checking.\n");
				_valid_count++;
			}
			else
				printf("\n");
		}
	}

	ROS_INFO("%s %d: Valid count is %d.", __FUNCTION__, __LINE__, _valid_count);
	return _valid_count > 130;
}
