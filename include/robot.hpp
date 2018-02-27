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
#include <rplidar_ros/SetLidar.h>
#include <vector>
#include "config.h"
#include "map.h"
#include "pose.h"
//#include "mode.hpp"
#include <string.h>

#define  _RATE 50

extern pthread_mutex_t recev_lock;
extern pthread_cond_t  recev_cond;

extern pthread_mutex_t serial_data_ready_mtx;
extern pthread_cond_t serial_data_ready_cond;

extern bool g_pp_shutdown;

extern bool robotbase_thread_stop;
extern bool recei_thread_stop;
extern bool send_thread_stop;
extern bool event_manager_thread_stop;
extern bool event_handle_thread_stop;
extern bool core_thread_stop;

class Mode;

typedef enum {
	ODOM_POSITION_ODOM_ANGLE = 0,
	SLAM_POSITION_SLAM_ANGLE,
	SLAM_POSITION_ODOM_ANGLE,
} Baselink_Frame_Type;

class robot
{
public:
	robot();
	~robot();

	static robot *instance();

	tf::TransformListener		*robot_tf_;

	// Publisher functions.
//	void pubTmpTarget(const Points &points,bool is_virtual=false);
	void robotbase_routine_cb();
	void core_thread_cb();
	// Service caller functions.
	bool lidarMotorCtrl(bool switch_);
	bool slamStart(void);
	bool slamStop(void);

	void initOdomPosition();

	// The scale should be between 0 to 1.
	void updateRobotPose(tf::Vector3 &odom, double odom_yaw);

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

	double getWorldPoseRadian()
	{
		return robot_radian_;
	}

	float getWorldPoseX()
	{
		return static_cast<float>(robot_pos.x());
	}

	float getWorldPoseY()
	{
		return static_cast<float>(robot_pos.y());
	}

	float getWorldPoseZ()
	{
		return static_cast<float>(robot_pos.z());
	}
/*

	float getRobotCorrectionX() const
	{
		return robot_correction_x_;
	}

	float getRobotCorrectionY() const
	{
		return robot_correction_y_;
	}
*/
	bool isBatteryLow() const
	{
		return battery_low_;
	}

	void setBatterLow(bool val)
	{
		battery_low_ = val;
	}

	double getRobotCorrectionRadian() const
	{
		return robot_correction_yaw_;
	}

	void setTfReady(bool is_ready)
	{
		is_tf_ready_ = is_ready;
	}

	double getGyroDynamicRunTime(){
		return gyro_dynamic_run_time_;
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
		boost::mutex::scoped_lock lock(baselink_frame_type_mutex_);
		return baselink_frame_type_;
	}

	void setBaselinkFrameType(Baselink_Frame_Type frame)
	{
		boost::mutex::scoped_lock lock(baselink_frame_type_mutex_);
		baselink_frame_type_ = frame;
		ROS_DEBUG("%s %d: Base link frame type has been reset to %d.", __FUNCTION__, __LINE__, getBaselinkFrameType());
	}

	bool isScanAllow()
	{
		return scan_ctrl_.allow_publishing?true:false;
	}

	void setTempTarget(std::deque<Vector2<double>>& points, uint32_t  seq);

	void debugReceivedStream(const uint8_t *buf);

	void debugSendStream(const uint8_t *buf);

	bool pubScanCtrl(bool is_pub, bool is_force_pub = false);

	void lockScanCtrl(void);

	void unlockScanCtrl(void);
private:

	Baselink_Frame_Type baselink_frame_type_;
	boost::mutex baselink_frame_type_mutex_;
// Lock for odom coordinate
	boost::mutex odom_mutex_;


	bool is_sensor_ready_{};
	bool is_tf_ready_{};

	bool temp_spot_set_{};
	bool battery_low_{};

	tf::Vector3	robot_pos;
	double	robot_radian_;

	// This is for the slam correction variables.
	tf::Vector3	robot_correction_pos;
	double	robot_correction_yaw_;
	tf::Vector3	slam_correction_pos;
	double	slam_correction_yaw_;
	double gyro_dynamic_run_time_;

	ros::NodeHandle robot_nh_;

	ros::Subscriber odom_sub_;

	ros::Publisher odom_pub_;
	ros::Publisher scan_ctrl_pub_;

	ros::ServiceClient lidar_motor_cli_;
	ros::ServiceClient start_slam_cli_;
	ros::ServiceClient end_slam_cli_;

	visualization_msgs::Marker clean_markers_,bumper_markers_, clean_map_markers_;

	tf::TransformListener		*robot_wf_tf_;
	tf::Stamped<tf::Transform>	map_pose;
	tf::Stamped<tf::Transform>	wf_map_pose;

	pp::scan_ctrl scan_ctrl_;


	//callback function
	void robotOdomCb(const nav_msgs::Odometry::ConstPtr &msg);
	void odomPublish();
//	void robot_map_metadata_cb(const nav_msgs::MapMetaData::ConstPtr& msg);

	boost::shared_ptr<Mode> p_mode{};

	bool is_locked_scan_ctrl_{false};
};

float cellToCount(int16_t distance);

int16_t countToCell(int32_t count);

Point_t getPosition(void);

bool isAny(Dir_t dir);

bool isPos(Dir_t dir);

bool isXAxis(Dir_t dir);

void updatePosition();

Mode *getNextMode(int next_mode_i_);

void setPosition(float x, float y);
void resetPosition();
#endif
