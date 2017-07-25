#ifndef __LASER_H__
#define __LASER_H__

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <sensor_msgs/LaserScan.h>
#include "mathematics.h"
class Laser
{
public:
	Laser();
	~Laser();

	bool laserObstcalDetected(double distance, int angle, double range);

	bool getLaserDistance(int begin, int end, double range, double dis_lim, double *hine_angle, double *distance);

	void setScanReady(uint8_t val);
	int8_t isScanReady();
	bool isNewDataReady();

	double getLaserDistance(uint16_t angle);

	bool lineFit(const std::vector<Double_Point> &points, double &a, double &b, double &c);

	bool splitLine(const std::vector<Double_Point> &points, double consecutive_lim, int points_count_lim);

	//bool splitLine2nd(const std::vector<std::vector<Double_Point> >	&groups, double t_max, int points_count_lim);
	bool splitLine2nd(std::vector<std::vector<Double_Point> > *groups, double t_max, int points_count_lim);

	bool mergeLine(std::vector<std::vector<Double_Point> > *groups, double t_lim);

	void pubLineMarker(std::vector<std::vector<Double_Point> > *groups);

	bool fitLineGroup(std::vector<std::vector<Double_Point> > *groups, double t_lim, double dis_lim);

	void pubFitLineMarker(double a, double b, double c, double y1, double y2);

	//void startShield(void);
	//void stopShield(void);
	void lidarShieldDetect(bool sd);
	void lidarMotorCtrl(bool orf);
private:

	//void stop(void);
	//void start(void);
	void scanCb(const sensor_msgs::LaserScan::ConstPtr &msg);

	int8_t is_ready_;
	bool is_scanDataReady_;

	ros::NodeHandle	nh_;
	ros::Subscriber	scan_sub_;

	sensor_msgs::LaserScan laser_scan_data_;

	//ros::ServiceClient start_motor_cli_;
	//ros::ServiceClient stop_motor_cli_;
	//ros::ServiceClient start_laser_shield_cli_;
	//ros::ServiceClient stop_laser_shield_cli_;
	ros::ServiceClient lidar_motor_cli_;
	ros::ServiceClient lidar_shield_detect_;

	std::vector<Double_Point>	Laser_Point;
	std::vector<std::vector<Double_Point> >	Laser_Group;
	std::vector<std::vector<Double_Point> >	Laser_Group_2nd;
	std::vector<LineABC>	fit_line;

	ros::Publisher line_marker_pub = nh_.advertise<visualization_msgs::Marker>("line_marker", 1);
	ros::Publisher fit_line_marker_pub = nh_.advertise<visualization_msgs::Marker>("fit_line_marker", 1);
	visualization_msgs::Marker fit_line_marker;

	geometry_msgs::Point laser_points_;
};

#endif
