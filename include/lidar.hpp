#ifndef __LIDAR_H__
#define __LIDAR_H__


#include <nav_msgs/Odometry.h>
#include "boost/thread.hpp"
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

	bool lidarGetFitLine(int begin, int end, double range, double dis_lim, double *hine_angle, double *distance, bool is_left, bool is_align = false);

	/*
	 * @author Alvin Xie
	 * @brief make use of lidar to judge x+ or x- is more closer to the wall
	 * @return the closer direction of x to the wall
	 * if x+ is closer, return 1, else return 0
	 * */
	int compLaneDistance();

	/*
	 * @author Alvin Xie
	 * @brief make use of lidar to get the obstacle distance
	 * @param dir:0-front 1-back 2-left 3-right
	 *        range: detect range for one side in meters, it will detect both side.
	 * @return the distance to the obstacle
	 * */
	double getObstacleDistance(uint8_t dir, double range);
	void setScanLinearReady(uint8_t val);
	void setScanOriginalReady(uint8_t val);
	void setScanCompensateReady(uint8_t val);
	int8_t isScanLinearReady();
	int8_t isScanOriginalReady();
	int8_t isScanCompensateReady();

	double getLidarDistance(uint16_t angle);

	bool lineFit(const std::vector<Vector2<double>> &points, double &a, double &b, double &c);

	bool splitLine(const std::vector<Vector2<double>> &points, double consecutive_lim, int points_count_lim);

	//bool splitLine2nd(const std::vector<std::vector<Vector2<double>> >	&groups, double t_max, int points_count_lim);
	bool splitLine2nd(std::vector<std::vector<Vector2<double>> > *groups, double t_max, int points_count_lim);

	bool mergeLine(std::vector<std::vector<Vector2<double>> > *groups, double t_lim , bool is_align);

	bool fitLineGroup(std::vector<std::vector<Vector2<double>> > *groups, double dis_lim , bool is_align);

	void pubFitLineMarker(double a, double b, double c, double y1, double y2);

	void motorCtrl(bool new_switch_);
	void startAlign();
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
	void checkRobotSlip();
	bool isRobotSlip();

	// type 1 for linear scan data, type 2 for origin scan data.
	bool lidarCheckFresh(float duration, uint8_t type = 1);
	bool findLines(std::vector<LineABC> *lines,bool combine);
	bool getAlignAngle(const std::vector<LineABC> *lines,float *align_angle);

	//void stop(void);
	//void start(void);
	void scanLinearCb(const sensor_msgs::LaserScan::ConstPtr &msg);
	void scanOriginalCb(const sensor_msgs::LaserScan::ConstPtr &msg);
	void scanCompensateCb(const sensor_msgs::LaserScan::ConstPtr &msg);
	void lidarXYPointCb(const visualization_msgs::Marker &point_marker);
	static void setLidarScanDataOriginal(const sensor_msgs::LaserScan::ConstPtr &scan);
	static sensor_msgs::LaserScan getLidarScanDataOriginal(void);
private:

	// switch_ is the target status of lidar.
	bool switch_{OFF};

	int angle_n_;
	uint8_t is_scanLinear_ready_;
	uint8_t is_scanOriginal_ready_;
	uint8_t is_scanCompensate_ready_;

	sensor_msgs::LaserScan lidarScanData_linear_;
	static sensor_msgs::LaserScan lidarScanData_original_;
	sensor_msgs::LaserScan lidarScanData_compensate_;
	std::vector<geometry_msgs::Point> lidarXY_points;
	double scanLinear_update_time_;
	double scanOriginal_update_time_;
	double scanCompensate_update_time_;
	double scanXYPoint_update_time_;

	std::vector<Vector2<double>>	Lidar_Point;
	std::vector<std::vector<Vector2<double>> >	Lidar_Group;
	std::vector<std::vector<Vector2<double>> >	Lidar_Group_2nd;
	std::vector<LineABC>	fit_line;
	//static float *last_ranges_;

//	ros::Publisher lidar_filter_pub = nh_.advertise<sensor_msgs::LaserScan>("lidar_filter",1);
	visualization_msgs::Marker fit_line_marker;

	geometry_msgs::Point lidar_points_;

	// For aligning.
	bool align_finish_;
	float align_angle_;
	geometry_msgs::Point laser_points_;

	// For slip checking
	bool slip_status_{false};
	uint8_t slip_frame_cnt_{0};
	std::vector<float> last_frame_ranges_{};
	double last_frame_time_stamp_{};
	double current_frame_time_stamp_{};
	float slip_ranges_percent_{0.85};//85%
	uint8_t slip_cnt_limit_{5};

};

bool lidar_is_stuck();

extern Lidar lidar;
#endif
