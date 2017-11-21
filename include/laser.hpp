#ifndef __LASER_H__
#define __LASER_H__

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <movement.h>
#include <sensor_msgs/LaserScan.h>
#include "mathematics.h"

#define ON true
#define OFF false

class Laser
{
public:
	Laser();
	~Laser();

	bool laserObstcalDetected(double distance, int angle, double range);

	bool laserGetFitLine(int begin, int end, double range, double dis_lim, double *hine_angle, double *distance);

	int compLaneDistance();
	double getObstacleDistance(uint8_t dir, double range);
	void setScanReady(uint8_t val);
	void setScan2Ready(uint8_t val);
	void setScan3Ready(uint8_t val);
	int8_t isScanReady();
	int8_t isScan2Ready();
	int8_t isScan3Ready();

	double getLaserDistance(uint16_t angle);

	bool lineFit(const std::vector<Double_Point> &points, double &a, double &b, double &c);

	bool splitLine(const std::vector<Double_Point> &points, double consecutive_lim, int points_count_lim);

	//bool splitLine2nd(const std::vector<std::vector<Double_Point> >	&groups, double t_max, int points_count_lim);
	bool splitLine2nd(std::vector<std::vector<Double_Point> > *groups, double t_max, int points_count_lim);

	bool mergeLine(std::vector<std::vector<Double_Point> > *groups, double t_lim);

	void pubLineMarker(std::vector<std::vector<Double_Point> > *groups);

	bool fitLineGroup(std::vector<std::vector<Double_Point> > *groups, double t_lim, double dis_lim);

	void pubFitLineMarker(double a, double b, double c, double y1, double y2);

	void pubPointMarkers(const std::vector<Point_d_t> *point, std::string frame_id);

	//void startShield(void);
	//void stopShield(void);
	void lidarShieldDetect(bool sd);
	void lidarMotorCtrl(bool orf);

	uint8_t laserMarker(double X_MAX = 0.237);
	uint8_t isRobotSlip();

	bool laserCheckFresh(float duration, uint8_t type = 1);
	bool findLines(std::vector<LineABC> *lines);
	void pubLineMarker(const std::vector<LineABC> *lines);
	bool getAlignAngle(const std::vector<LineABC> *lines,float *align_angle);

private:

	//void stop(void);
	//void start(void);
	void scanCb(const sensor_msgs::LaserScan::ConstPtr &msg);
	void scanCb2(const sensor_msgs::LaserScan::ConstPtr &msg);
	void scanCb3(const sensor_msgs::LaserScan::ConstPtr &msg);
	void laserPointCb(const visualization_msgs::Marker &point_marker);

	int angle_n_;
	uint8_t is_scan_ready_;
	uint8_t is_scan2_ready_;
	uint8_t is_scan3_ready_;
	std::vector<geometry_msgs::Point> laserXY_points;

	ros::NodeHandle	nh_;
	ros::Subscriber	scan_sub_;
	ros::Subscriber	scan_sub2_;
	ros::Subscriber	scan_sub3_;
	ros::Subscriber laserPoint_sub_;

	sensor_msgs::LaserScan laserScanData_;
	sensor_msgs::LaserScan laserScanData_2_;
	sensor_msgs::LaserScan laserScanData_3_;
	double scan_update_time;
	double scan2_update_time;

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
	//static float *last_ranges_;
	ros::Publisher line_marker_pub = nh_.advertise<visualization_msgs::Marker>("line_marker", 1);
	ros::Publisher line_marker_pub2 = nh_.advertise<visualization_msgs::Marker>("line_marker2", 1);
	ros::Publisher point_marker_pub = nh_.advertise<visualization_msgs::Marker>("point_marker", 1);
	ros::Publisher fit_line_marker_pub = nh_.advertise<visualization_msgs::Marker>("fit_line_marker", 1);
//	ros::Publisher laser_filter_pub = nh_.advertise<sensor_msgs::LaserScan>("laser_filter",1);
	visualization_msgs::Marker fit_line_marker;

	geometry_msgs::Point laser_points_;
};

#endif
