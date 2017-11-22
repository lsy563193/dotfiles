#ifndef __ROBOT_H__
#define __ROBOT_H__

#include <ros/ros.h>
#include <nav_msgs/MapMetaData.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <obstacle_detector/Obstacles.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Point.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <pp/x900sensor.h>
#include <pp/scan_ctrl.h>
#include <vector>
#include "config.h"
#include "map.h"
#include "movement.h"

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
	robot();
	~robot();

	static robot *instance();

	tf::TransformListener		*robot_tf_;

	// core function
	void init();
	void displayPositions();
	void pubCleanMarkers(void);
	void setCleanMapMarkers(int8_t x, int8_t y, CellState type);
	void pubCleanMapMarkers(void);
	void visualizeMarkerInit();
	void initOdomPosition();

	//get and set function
	bool isSensorReady() const
	{
		return is_sensor_ready_;
	}

	bool isTfReady() const
	{
		return is_tf_ready_;
	}

	float getAngle() const
	{
		return angle_;
	}

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

	float getAngleV() const
	{
		return angle_v_;
	}


	float getLinearX() const
	{
		return linear_x_;
	}

	float getLinearY() const
	{
		return linear_y_;
	}

	float getLinearZ() const
	{
		return linear_z_;
	}

	int16_t getYaw() const
	{
		return ((int16_t)(position_yaw_ * 1800 / M_PI));
	}

	float getPositionX() const
	{
		return position_x_;
	}

	float getPositionY() const
	{
		return position_y_;
	}

	float getPositionZ() const
	{
		return position_z_;
	}

	float getWfPositionX() const
	{
		return wf_position_x_;
	}

	float getWfPositionY() const
	{
		return wf_position_y_;
	}

	float getOdomPositionX() const
	{
		return odom_pose_x_;
	}

	float getOdomPositionY() const
	{
		return odom_pose_y_;
	}

	double getOdomPositionYaw() const
	{
		return odom_pose_yaw_;
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

	uint32_t mapGetWidth()
	{
		return width_;
	}

	uint32_t mapGetHeight()
	{
		return height_;
	}

	float mapGetResolution()
	{
		return resolution_;
	}

	double mapGetOriginX()
	{
		return origin_x_;
	}

	double mapGetOriginY()
	{
		return origin_y_;
	}

	std::vector<int8_t> *mapGetMapData()
	{
		return map_ptr_;
	}

	// The scale should be between 0 to 1.
	void updateRobotPose(const float& odom_x, const float& odom_y, const double& odom_yaw,
						const float& slam_correction_x, const float& slam_correction_y, const double& slam_correction_yaw,
						float& robot_correction_x, float& robot_correction_y, double& robot_correction_yaw,
						float& robot_x, float& robot_y, double& robot_yaw);

	void resetCorrection();

	void obsAdjustCount(int count);

	//callback function
private:
	void sensorCb(const pp::x900sensor::ConstPtr &msg);
	void robotOdomCb(const nav_msgs::Odometry::ConstPtr &msg);
//	void robot_map_metadata_cb(const nav_msgs::MapMetaData::ConstPtr& msg);
	void mapCb(const nav_msgs::OccupancyGrid::ConstPtr &msg);

	Baselink_Frame_Type baselink_frame_type_;
	boost::mutex baselink_frame_type_mutex_;

	bool	is_sensor_ready_;

	bool	is_tf_ready_;

  bool temp_spot_set_;

	boost::mutex offset_angle_metux_;
	float offset_angle_;
	float start_angle_;
	float saved_offset_angle_;

	/* 1 byte */
	float	angle_;

	bool	is_align_active_;
	/* 1 byte */
	float	angle_v_;

	float	linear_x_;
	float	linear_y_;
	float	linear_z_;

	float	position_x_;
	float	position_y_;
	float	position_z_;
	double	position_yaw_;
	float	wf_position_x_;
	float	wf_position_y_;
	float	odom_pose_x_;
	float	odom_pose_y_;
	double	odom_pose_yaw_;

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

	/*for ros map*/
	uint32_t width_;
	uint32_t height_;
	float resolution_;
	double origin_x_;
	double origin_y_;
	std::vector<int8_t> map_data_;
	std::vector<int8_t> *map_ptr_;

	ros::NodeHandle robot_nh_;
	ros::Subscriber sensor_sub_;
	ros::Subscriber map_sub_;
	ros::Subscriber odom_sub_;
	ros::Publisher odom_pub_;
	ros::Publisher send_clean_marker_pub_;
	ros::Publisher send_clean_map_marker_pub_;
	ros::Publisher scan_ctrl_pub_;

	visualization_msgs::Marker clean_markers_,bumper_markers_, clean_map_markers_;
	geometry_msgs::Point m_points_;
	std_msgs::ColorRGBA color_;

	tf::TransformListener		*robot_wf_tf_;
	tf::Stamped<tf::Transform>	map_pose;
	tf::Stamped<tf::Transform>	wf_map_pose;

	//tf::TransformBroadcaster	robot_broad;
	//geometry_msgs::TransformStamped robot_trans;
	nav_msgs::Odometry odom;
	pp::scan_ctrl scan_ctrl_;
};

#endif
