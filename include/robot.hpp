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
	Odom_Position_Odom_Angle = 0,
	Map_Position_Map_Angle,
	Map_Position_Odom_Angle,
} Baselink_Frame_Type;

extern bool	g_is_low_bat_pause;
extern bool g_is_manual_pause;
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
	void pubLineMarker(std::vector<std::vector<Double_Point> > *groups);
	void pubFitLineMarker(visualization_msgs::Marker fit_line_marker);
	void pubPointMarkers(const std::vector<Point_d_t> *point, std::string frame_id);
	void setCleanMapMarkers(int8_t x, int8_t y, CellState type);
	void pubCleanMapMarkers(GridMap& map, const std::deque<Cell_t>& path, Cell_t* cell_p = nullptr);

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
/*

	void offsetAngle(float angle)
	{
		boost::mutex::scoped_lock(offset_angle_metux_);
		offset_angle_ = angle;
	};

	float offsetAngle(void) const
	{
		boost::mutex::scoped_lock(offset_angle_metux_);
		return offset_angle_;
	};

	void savedOffsetAngle(float angle)
	{
		if (angle > 180)
			angle -= 360;
		saved_offset_angle_ = -angle;
	};

	float savedOffsetAngle(void) const
	{
		return saved_offset_angle_;
	};
*/

	int16_t getPoseAngle()
	{
		// It is 10x degrees.
		return static_cast<int16_t>(pose.getAngle());
	}

	float getPoseX()
	{
		return pose.getX();
	}

	float getPoseY()
	{
		return pose.getY();
	}

	float getPoseZ()
	{
		return pose.getZ();
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
		baselink_frame_type_mutex_.lock();
		baselink_frame_type_ = frame;
		baselink_frame_type_mutex_.unlock();
		ROS_INFO("%s %d: Base link frame type has been reset to %d.", __FUNCTION__, __LINE__, getBaselinkFrameType());
	}

	bool isScanAllow()
	{
		return scan_ctrl_.allow_publishing?true:false;
	}
private:

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

	Pose pose;

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

	//callback function
	void sensorCb(const pp::x900sensor::ConstPtr &msg);
	void robotOdomCb(const nav_msgs::Odometry::ConstPtr &msg);
//	void robot_map_metadata_cb(const nav_msgs::MapMetaData::ConstPtr& msg);
	void mapCb(const nav_msgs::OccupancyGrid::ConstPtr &msg);
	void scanLinearCb(const sensor_msgs::LaserScan::ConstPtr &msg);
	void scanOriginalCb(const sensor_msgs::LaserScan::ConstPtr &msg);
	void scanCompensateCb(const sensor_msgs::LaserScan::ConstPtr &msg);
	void lidarPointCb(const visualization_msgs::Marker &point_marker);
};

#endif
