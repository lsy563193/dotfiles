#ifndef __LIDAR_H__
#define __LIDAR_H__


#include <nav_msgs/Odometry.h>
#include "boost/thread.hpp"
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/LaserScan.h>
#include "mathematics.h"
#include "robot.hpp"

class Lidar
{
public:
	Lidar();
	~Lidar();

	void init();
	bool getFitLine(double r_begin, double r_end, double range, double dis_lim, double *line_radian, double *distance,
									bool is_left, bool is_align = false);

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

	bool lineFit(const std::deque<Vector2<double>> &points, double &a, double &b, double &c);
	bool splitLine(const std::vector<Vector2<double>> &points, double consecutive_lim, int points_count_lim);
	//bool splitLine2nd(const std::vector<std::vector<Vector2<double>> >	&groups, double t_max, int points_count_lim);
	bool splitLine2nd(std::vector<std::deque<Vector2<double>> > *groups, double t_max, int points_count_lim);
	bool mergeLine(std::vector<std::deque<Vector2<double>> > *groups, double t_lim , bool is_align);
	bool fitLineGroup(std::vector<std::deque<Vector2<double>> > *groups, double dis_lim , bool is_align);

	void pubFitLineMarker(double a, double b, double c, double y1, double y2);

	bool motorCtrl(bool new_switch_);
	void startAlign();
	void setAlignFinish();
	bool isAlignFinish();
	float alignRadian(void)
	{
		return align_radian_;
	}
	void alignRadian(float radian)
	{
		align_radian_ = radian;
	}

	uint8_t lidarMarker(std::vector<Vector2<int>> &markers, int movement_i, int action_i, double X_MAX = 0.21);
	void checkRobotSlip();
	void checkSlipInit(float &acur1, float &acur2, float &acur3, float &acur4);
	bool isRobotSlip();

	// type 1 for linear scan data, type 2 for origin scan data.
	bool lidarCheckFresh(float duration, uint8_t type = 1);
	bool findLines(std::vector<LineABC> *lines,bool combine);

	//void stop(void);
	//void start(void);
	void scanLinearCb(const sensor_msgs::LaserScan::ConstPtr &msg);
	void scanOriginalCb(const sensor_msgs::LaserScan::ConstPtr &msg);
	void scanCompensateCb(const sensor_msgs::LaserScan::ConstPtr &msg);
	void scantestCb(const sensor_msgs::LaserScan::ConstPtr &msg);
	void lidarXYPointCb(const visualization_msgs::Marker &point_marker);
	static void setLidarScanDataOriginal(const sensor_msgs::LaserScan::ConstPtr &scan);
	static sensor_msgs::LaserScan getLidarScanDataOriginal(void);
	double getLidarDistance(int16_t angle,float range_max,float range_min);
	uint8_t lidar_get_status(int movement_i, int action_i);
	bool lidar_is_stuck();

	void slipCheckingCtrl(bool enable)
	{
		slip_enable_ = enable;
	}

	double checkIsRightAngle(bool is_left);

	void saveLidarDataToFile(uint32_t seq, sensor_msgs::LaserScan scan);

//	void readLidarDataFromFile(uint32_t seq, float (&scan_data)[360]);
	void readLidarDataFromFile(bool check_front, float (&scan_data)[360]);

	bool scanDataChecking(bool check_front, sensor_msgs::LaserScan scan, float (&ref_scan_data)[360]);

	void setLidarStuckCheckingEnable(bool status);
	bool getLidarStuckCheckingEnable();
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
	std::vector<std::deque<Vector2<double>> >	Lidar_Group;
	std::vector<std::deque<Vector2<double>> >	Lidar_Group_2nd;
	std::vector<LineABC>	fit_line;
	//static float *last_ranges_;

//	ros::Publisher lidar_filter_pub = nh_.advertise<sensor_msgs::LaserScan>("lidar_filter",1);
	visualization_msgs::Marker fit_line_marker;

	geometry_msgs::Point lidar_points_;

	// For aligning.
	bool align_finish_;
	float align_radian_;
	geometry_msgs::Point laser_points_;

	// For slip checking
	bool slip_enable_{false};
	bool slip_status_{false};
	uint8_t slip_frame_cnt_{0};
	DequeArray<sensor_msgs::LaserScan> last_slip_scan_frame_{};
	float slip_ranges_percent_{0.8};//80%
	uint8_t slip_cnt_limit_{5};
	static double wheel_cliff_trigger_time_;
	static double gyro_tilt_trigger_time_;
	float acur1_,acur2_,acur3_,acur4_;//accuracy  ,in meters
	const float dist1_ = 3.5;//range distance 1 ,in meters
	const float dist2_ = 2.5;//range distance 2
	const float dist3_ = 1.5;//range distance 3
	const float dist4_ = 0.5;//range distance 4

	bool lidar_stuck_checking_enable{false};
};


extern Lidar lidar;
#endif
