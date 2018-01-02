#ifndef __ROBOT_H__
#define __ROBOT_H__

#include <ros/ros.h>
#include <nav_msgs/MapMetaData.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Point.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/LaserScan.h>
#include <pp/x900sensor.h>
#include <pp/scan_ctrl.h>
#include <vector>
#include "config.h"
#include "map.h"
#include "pose.h"
#include "mode.hpp"

extern volatile int16_t g_left_wall_baseline;
extern volatile int16_t g_right_wall_baseline;
extern pp::x900sensor   sensor;

typedef enum {
	ODOM_POSITION_ODOM_ANGLE = 0,
	SLAM_POSITION_SLAM_ANGLE,
	SLAM_POSITION_ODOM_ANGLE,
} Baselink_Frame_Type;

class robot
{
public:
	robot(std::string serial_port, int baudrate, std::string lidar_bumper_dev);
	~robot();

	static robot *instance();

	tf::TransformListener		*robot_tf_;

	// Publisher functions.
	void visualizeMarkerInit();
	void pubLineMarker(const std::vector<LineABC> *lines);
	void pubLineMarker(std::vector<std::vector<Vector2<double>> > *groups);
	void pubFitLineMarker(visualization_msgs::Marker fit_line_marker);
	void pubPointMarkers(const std::deque<Vector2<double>> *point, std::string frame_id);
	void setCleanMapMarkers(int16_t x, int16_t y, CellState type);
	void pubCleanMapMarkers(GridMap& map, const std::deque<Cell_t>& path);

	void core_thread_cb();
	// Service caller functions.
	bool lidarMotorCtrl(bool switch_);
	bool slamStart(void);
	bool slamStop(void);

	void initOdomPosition();

	// The scale should be between 0 to 1.
	void updateRobotPose(const float& odom_x, const float& odom_y, const double& odom_yaw,
						const float& slam_correction_x, const float& slam_correction_y, const double& slam_correction_yaw,
						float& robot_correction_x, float& robot_correction_y, double& robot_correction_yaw,
						float& robot_x, float& robot_y, double& robot_yaw);

	void resetCorrection();

	void obsAdjustCount(int count);

	//get and set function
	bool isSensorReady() const
	{
		return is_sensor_ready_;
	}

	bool isTfReady() const
	{
		return is_tf_ready_;
	}

	int16_t getWorldPoseAngle()
	{
		// It is 10x degrees.
		return static_cast<int16_t>(world_pose_.getAngle());
	}

	float getWorldPoseX()
	{
		return world_pose_.getX();
	}

	float getWorldPoseY()
	{
		return world_pose_.getY();
	}

	float getWorldPoseZ()
	{
		return world_pose_.getZ();
	}

	float getRobotCorrectionX() const
	{
		return robot_correction_x_;
	}

	float getRobotCorrectionY() const
	{
		return robot_correction_y_;
	}

	double getRobotCorrectionYaw() const
	{
		return robot_correction_yaw_;
	}

	void setTfReady(bool is_ready)
	{
		is_tf_ready_ = is_ready;
	}

	void setTempSpot(void)
	{
		temp_spot_set_ = true;
	}
	void resetTempSpot(void)
	{
		temp_spot_set_ = false;
	}
	bool isTempSpot(void)
	{
		return temp_spot_set_;
	}

	Baselink_Frame_Type getBaselinkFrameType(void)
	{
		boost::mutex::scoped_lock(baselink_frame_type_mutex_);
		return baselink_frame_type_;
	}

	void setBaselinkFrameType(Baselink_Frame_Type frame)
	{
		boost::mutex::scoped_lock(baselink_frame_type_mutex_);
		baselink_frame_type_ = frame;
		ROS_DEBUG("%s %d: Base link frame type has been reset to %d.", __FUNCTION__, __LINE__, getBaselinkFrameType());
	}

	bool isScanAllow()
	{
		return scan_ctrl_.allow_publishing?true:false;
	}

	Points getTempTarget()const;
	void setTempTarget(std::deque<Vector2<double>>& points);
private:
	Points tmp_plan_path_{};
	Baselink_Frame_Type baselink_frame_type_;
	boost::mutex baselink_frame_type_mutex_;

	bool	is_sensor_ready_;

	bool	is_tf_ready_;

	bool temp_spot_set_;
/*
	// TODO: Delete these offset variables.
	boost::mutex offset_angle_metux_;
	float offset_angle_;
	float start_angle_;
	float saved_offset_angle_;
*/

	bool	is_align_active_;

	Pose world_pose_;

	// This is for the slam correction variables.
	float	robot_correction_x_;
	float	robot_correction_y_;
	double	robot_correction_yaw_;
	float	slam_correction_x_;
	float	slam_correction_y_;
	double	slam_correction_yaw_;
	float	robot_x_;
	float	robot_y_;
	double	robot_yaw_;

	ros::NodeHandle robot_nh_;

	ros::Subscriber sensor_sub_;
	ros::Subscriber map_sub_;
	ros::Subscriber odom_sub_;
	ros::Subscriber	scanLinear_sub_;
	ros::Subscriber	scanOriginal_sub_;
	ros::Subscriber	scanCompensate_sub_;
	ros::Subscriber lidarPoint_sub_;

	ros::Publisher odom_pub_;
	ros::Publisher send_clean_marker_pub_;
	ros::Publisher send_clean_map_marker_pub_;
	ros::Publisher scan_ctrl_pub_;
	ros::Publisher line_marker_pub_;
	ros::Publisher line_marker_pub2_;
	ros::Publisher point_marker_pub_;
	ros::Publisher fit_line_marker_pub_;

	ros::ServiceClient lidar_motor_cli_;
	ros::ServiceClient start_slam_cli_;
	ros::ServiceClient end_slam_cli_;

	visualization_msgs::Marker clean_markers_,bumper_markers_, clean_map_markers_;
	geometry_msgs::Point m_points_;
	std_msgs::ColorRGBA color_;

	tf::TransformListener		*robot_wf_tf_;
	tf::Stamped<tf::Transform>	map_pose;
	tf::Stamped<tf::Transform>	wf_map_pose;

	pp::scan_ctrl scan_ctrl_;

	boost::mutex temp_target_mutex_;
	class Paras;
	Vector2<double> get_middle_point(const Vector2<double>& p1,const Vector2<double>& p2,const Paras& para);
	bool check_is_valid(const Vector2<double>& point, Paras& para, const sensor_msgs::LaserScan::ConstPtr & scan);
	bool check_corner(const sensor_msgs::LaserScan::ConstPtr & scan, const Paras para);

	Vector2<double> polar_to_cartesian(double polar,int i);
	//callback function
	void sensorCb(const pp::x900sensor::ConstPtr &msg);
	void robotOdomCb(const nav_msgs::Odometry::ConstPtr &msg);
//	void robot_map_metadata_cb(const nav_msgs::MapMetaData::ConstPtr& msg);
	void mapCb(const nav_msgs::OccupancyGrid::ConstPtr &msg);
	void scanLinearCb(const sensor_msgs::LaserScan::ConstPtr &msg);
	class Paras{
public:
	explicit Paras(bool is_left):is_left_(is_left)
	{
		narrow = is_left ? 0.187 : 0.197;

		y_min = 0.0;
		y_max = is_left ? 0.3 : 0.25;

		x_min_forward = LIDAR_OFFSET_X;
		x_max_forward = is_left ? 0.3 : 0.25;
		auto y_start_forward = is_left ? 0.06: -0.06;
		auto y_end_forward = is_left ? -ROBOT_RADIUS: ROBOT_RADIUS;
		y_min_forward = std::min(y_start_forward, y_end_forward);
		y_max_forward = std::max(y_start_forward, y_end_forward);

		auto x_side_start = 0.0;
		auto x_side_end = ROBOT_RADIUS;
		x_min_side = std::min(x_side_start, x_side_end);
		x_max_side = std::max(x_side_start, x_side_end);

		auto y_side_start = 0.0;
		auto y_side_end = is_left ? narrow: -narrow;
		y_min_side = std::min(y_side_start, y_side_end);
		y_max_side = std::max(y_side_start, y_side_end);

		auto y_point1_start_corner = is_left ? 0.3 : -0.3;
		auto y_point1_end_corner = is_left ? -4.0 : 4.0;
		y_min_point1_corner = std::min(y_point1_start_corner, y_point1_end_corner);
		y_max_point1_corner = std::max(y_point1_start_corner, y_point1_end_corner);

	 	auto y_point1_start = 0.0;
		auto y_point1_end = is_left ? 4.0 : -4.0;
		y_min_point1 = std::min(y_point1_start, y_point1_end);
		y_max_point1 = std::max(y_point1_start, y_point1_end);

		auto y_target_start = is_left ? ROBOT_RADIUS : -ROBOT_RADIUS;
		auto y_target_end = is_left ? 0.4 : -4.0;
		y_min_target = std::min(y_point1_start, y_point1_end);
		y_max_target = std::max(y_point1_start, y_point1_end);
	};

	bool inPoint1Range(const Vector2<double> &point, bool is_corner) const {
		if(is_corner)
			return (point.x > 0 && point.x < 4 && point.y > y_min_point1_corner && point.y < y_max_point1_corner);
		else
			return (point.x > -4 && point.x < 0.3 && point.y > y_min_point1 && point.y < y_max_point1);
	}

	bool inTargetRange(const Vector2<double> &target) {
		return target.x < 4
					 && ((target.x > CHASE_X && fabs(target.y) < ROBOT_RADIUS)
							 || (target.x > 0  && target.y >y_min_target && target.y < y_max_target ));
	}

	bool inForwardRange(const Vector2<double> &point) const {
		return point.x > x_min_forward && point.x < x_max_forward && point.y > y_min_forward && point.y < y_max_forward;
	}

	bool inSidedRange(const Vector2<double> &point) const {
		return point.x > x_min_side && point.x < x_max_side && point.y > y_min_side && point.y < y_max_side;
	}

	double narrow;
	bool is_left_;
	double x_min_forward;
	double x_max_forward;
	double x_min_side;
	double x_max_side;

	double y_min;
	double y_max;

	double y_min_forward;
	double y_max_forward;

	double y_min_side;
	double y_max_side;

	double y_min_point1_corner;
	double y_max_point1_corner;
	double y_min_point1;
	double y_max_point1;

	double y_min_target;
	double y_max_target;

	const double CHASE_X = 0.107;
};
	bool calcLidarPath(const sensor_msgs::LaserScan::ConstPtr & scan,bool is_left ,std::deque<Vector2<double>>& points);
	void scanOriginalCb(const sensor_msgs::LaserScan::ConstPtr& scan);
	void scanCompensateCb(const sensor_msgs::LaserScan::ConstPtr &msg);
	void lidarPointCb(const visualization_msgs::Marker &point_marker);

	boost::shared_ptr<Mode> p_mode{};
};

int32_t cellToCount(int16_t distance);

int16_t countToCell(int32_t count);

Point32_t getPosition(void);

bool isPos(MapDirection dir);

bool isXAxis(MapDirection dir);

bool isYAxis(MapDirection dir);

Point32_t updatePosition();

void setPosition(int32_t x, int32_t y);
#endif
