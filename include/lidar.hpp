#ifndef __LIDAR_H__
#define __LIDAR_H__

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/LaserScan.h>
#include "mathematics.h"

#define ON true
#define OFF false

class Lidar
{
public:
	Lidar();
	~Lidar();

	bool lidarObstcalDetected(double distance, int angle, double range);

	bool lidarGetFitLine(int begin, int end, double range, double dis_lim, double *hine_angle, double *distance, bool is_left);

	int compLaneDistance();
	double getObstacleDistance(uint8_t dir, double range);
	void setScanLinearReady(uint8_t val);
	void setScanOriginalReady(uint8_t val);
	void setScanCompensateReady(uint8_t val);
	int8_t isScanLinearReady();
	int8_t isScanOriginalReady();
	int8_t isScanCompensateReady();

	double getLidarDistance(uint16_t angle);

	bool lineFit(const std::vector<Double_Point> &points, double &a, double &b, double &c);

	bool splitLine(const std::vector<Double_Point> &points, double consecutive_lim, int points_count_lim);

	//bool splitLine2nd(const std::vector<std::vector<Double_Point> >	&groups, double t_max, int points_count_lim);
	bool splitLine2nd(std::vector<std::vector<Double_Point> > *groups, double t_max, int points_count_lim);

	bool mergeLine(std::vector<std::vector<Double_Point> > *groups, double t_lim);

	bool fitLineGroup(std::vector<std::vector<Double_Point> > *groups, double t_lim, double dis_lim);

	void pubFitLineMarker(double a, double b, double c, double y1, double y2);

	void motorCtrl(bool switch_);
	bool openTimeOut();
	void startAlign();
	bool alignTimeOut();
	bool alignFinish();
	float alignAngle(void)
	{
		return align_angle_;
	}
	void alignAngle(float angle)
	{
		align_angle_ = angle;
	}

	uint8_t lidarMarker(double X_MAX = 0.237);
	uint8_t isRobotSlip();

	bool lidarCheckFresh(float duration, uint8_t type = 1);
	bool findLines(std::vector<LineABC> *lines,bool combine);
	bool getAlignAngle(const std::vector<LineABC> *lines,float *align_angle);

	//void stop(void);
	//void start(void);
	void scanLinearCb(const sensor_msgs::LaserScan::ConstPtr &msg);
	void scanOriginalCb(const sensor_msgs::LaserScan::ConstPtr &msg);
	void scanCompensateCb(const sensor_msgs::LaserScan::ConstPtr &msg);
	void lidarPointCb(const visualization_msgs::Marker &point_marker);
private:
	int angle_n_;
	uint8_t is_scanLinear_ready_;
	uint8_t is_scanOriginal_ready_;
	uint8_t is_scanCompensate_ready_;
	std::vector<geometry_msgs::Point> lidarXY_points;

	sensor_msgs::LaserScan lidarScanData_linear_;
	sensor_msgs::LaserScan lidarScanData_original_;
	sensor_msgs::LaserScan lidarScanData_compensate_;
	double scanLinear_update_time;
	double scanOriginal_update_time;

	std::vector<Double_Point>	Lidar_Point;
	std::vector<std::vector<Double_Point> >	Lidar_Group;
	std::vector<std::vector<Double_Point> >	Lidar_Group_2nd;
	std::vector<LineABC>	fit_line;
	//static float *last_ranges_;

//	ros::Publisher lidar_filter_pub = nh_.advertise<sensor_msgs::LaserScan>("lidar_filter",1);
	visualization_msgs::Marker fit_line_marker;

	geometry_msgs::Point lidar_points_;

	// For opening lidar.
	time_t open_command_time_stamp_;

	// For aligning.
	time_t start_align_time_stamp_;
	bool align_finish_;
	float align_angle_;
};

bool lidar_is_stuck();
uint8_t lidar_get_status();
uint8_t lidar_is_robot_slip();

extern Lidar lidar;
#endif
