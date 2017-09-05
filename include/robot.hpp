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
#include <vector>
#include "config.h"
#include "map.h"

extern volatile int16_t g_left_wall_baseline;
extern volatile int16_t g_right_wall_baseline;
extern pp::x900sensor   sensor;

typedef enum {
	Odom_Position_Odom_Angle = 0,
	Map_Position_Map_Angle,
	Map_Position_Odom_Angle,
} Baselink_Frame_Type;

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

	float getLeftWheelSpeed() const
	{
		return lw_vel_;
	}
	float getRightWheelSpeed() const
	{
		return rw_vel_;
	}
	float getAngleV() const
	{
		return angle_v_;
	}

	int16_t getCliffRight() const
	{
		return sensor.rcliff;
	}

	int16_t getCliffLeft() const
	{
		return sensor.lcliff;
	}

	int16_t getCliffFront() const
	{
		return sensor.fcliff;
	}

	int16_t getLeftWall() const
	{
		return sensor.left_wall - g_left_wall_baseline;
	}

	int16_t getRightWall() const
	{
#if __ROBOT_X900
		return sensor.right_wall - g_right_wall_baseline;
#elif __ROBOT_X400
		return 0;
#endif
	}

	int16_t getOmniWheel() const
	{
#if __ROBOT_X9000
		//return sensor.omni_wheel;
		return omni_wheel_;
#elif __ROBOT_X400
		return 0;
#endif
	}

	void resetOmniWheel()
	{
#if __ROBOT_X9000
		omni_wheel_ = 0;
		reset_mobility_step();
#endif
	}

	int16_t getVisualWall() const
	{
#if __ROBOT_X900
		return sensor.visual_wall;
#elif __ROBOT_X400
		return 0;
#endif
	}

	uint8_t getVacuumSelfCheckStatus() const
	{
		return vacuum_selfcheck_status_;
	}

	bool getLbrushOc() const
	{
		return lbrush_oc_;
	}

	bool getRbrushOc() const
	{
		return rbrush_oc_;
	}

	bool getMbrushOc() const
	{
		return mbrush_oc_;
	}

	bool getVacuumOc() const
	{
		return vacuum_oc_;
	}

	uint8_t getKey() const
	{
		return key;
	}

	int getChargeStatus() const
	{
		return charge_status_;
	}

	uint8_t getIrCtrl() const
	{
		return  ir_ctrl_;
	}

	float getLwheelCurrent() const
	{
		return lw_crt_;
	}

	float getRwheelCurrent() const
	{
		return rw_crt_;
	}

	uint32_t getRcon() const
	{
		return charge_stub_;
	}

	bool getBumperRight() const
	{
		return sensor.rbumper;
	}

	bool getBumperLeft() const
	{
		return sensor.lbumper;
	}

	int16_t getObsLeft() const
	{
		return sensor.l_obs;
	}

	int16_t getObsRight() const
	{
		return sensor.r_obs;
	}

	int16_t getObsFront() const
	{
		return sensor.f_obs;
	}

	bool getWaterTank() const
	{
		return w_tank_;
	}

	uint16_t getBatteryVoltage() const
	{
		return battery_voltage_*10;
	}

	bool isMoving() const
	{
		return is_moving_;
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

	int16_t getXAcc() const
	{
		return x_acc_;
	}

	int16_t getYAcc() const
	{
		return y_acc_;
	}

	int16_t getZAcc() const
	{
		return z_acc_;
	}

	int16_t getInitXAcc() const
	{
		return init_x_acc_;
	}

	int16_t getInitYAcc() const
	{
		return init_y_acc_;
	}

	int16_t getInitZAcc() const
	{
		return init_z_acc_;
	}

	void setInitXAcc(int16_t val)
	{
		init_x_acc_ = val;
	}

	void setInitYAcc(int16_t val)
	{
		init_y_acc_ = val;
	}

	void setInitZAcc(int16_t val)
	{
		init_z_acc_ = val;
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

//#if CONTINUE_CLEANING_AFTER_CHARGE
// These 3 functions are for continue cleaning after charge.
	bool isLowBatPaused(void) const
	{
#if CONTINUE_CLEANING_AFTER_CHARGE
		return low_bat_pause_cleaning_;
#else
		return false;
#endif
	}

	void setLowBatPause(void)
	{
#if CONTINUE_CLEANING_AFTER_CHARGE
		ROS_WARN("%s %d.", __FUNCTION__, __LINE__);
		low_bat_pause_cleaning_ = true;
#endif
	}

	void resetLowBatPause(void)
	{
#if CONTINUE_CLEANING_AFTER_CHARGE
		ROS_WARN("%s %d.", __FUNCTION__, __LINE__);
		low_bat_pause_cleaning_ = false;
#endif
	}

// These 3 functions are for manual pause cleaning.
	bool isManualPaused(void) const
	{
#if MANUAL_PAUSE_CLEANING
		return manual_pause_cleaning_;
#else
		return false;
#endif
	}

	void setManualPause(void)
	{
#if MANUAL_PAUSE_CLEANING
		ROS_WARN("%s %d.", __FUNCTION__, __LINE__);
		manual_pause_cleaning_ = true;
#endif
	}

	void resetManualPause(void)
	{
#if MANUAL_PAUSE_CLEANING
		ROS_WARN("%s %d.", __FUNCTION__, __LINE__);
		manual_pause_cleaning_ = false;
#endif
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

	void setAccInitData();

	//callback function
private:
	void sensorCb(const pp::x900sensor::ConstPtr &msg);
	void robotOdomCb(const nav_msgs::Odometry::ConstPtr &msg);
//	void robot_map_metadata_cb(const nav_msgs::MapMetaData::ConstPtr& msg);
	void mapCb(const nav_msgs::OccupancyGrid::ConstPtr &msg);

	Baselink_Frame_Type baselink_frame_type_;
	boost::mutex baselink_frame_type_mutex_;

// These variable is indicating robot detects battery low, it is going back to home cells.
	bool	low_bat_pause_cleaning_;

// These variable is indicating robot is during pause of navigation mode.
	bool	manual_pause_cleaning_;


	bool	is_sensor_ready_;

	bool	is_tf_ready_;

	float offset_angle_;
    bool temp_spot_set_;

	boost::mutex offset_angle_metux_;

	float saved_offset_angle_;

	/*2 byte*/
	float lw_vel_;
	float rw_vel_;
	/* 1 byte */
	float	angle_;

	bool	is_align_active_;
	/* 1 byte */
	float	angle_v_;

	/* 1 byte */
	int32_t brush_left_;

	/* 1 byte */
	int32_t brush_right_;

	/* 1 byte */
	int32_t brush_main_;

	/* 2 bytes */
	int16_t cliff_right_;

	/* 2 bytes */
	int16_t cliff_left_;

	/* 2 bytes */
	int16_t cliff_front_;

	/*1 byte */
	uint8_t key;

	/*1 byte */
	uint8_t charge_status_;

	/*1 byte*/
	bool w_tank_; //water tank

	/* 1 byte */
	uint16_t battery_voltage_;

	/*1 byte*/
	uint8_t vacuum_selfcheck_status_;
	bool lbrush_oc_; //oc: over current
	bool rbrush_oc_;
	bool mbrush_oc_;
	bool vacuum_oc_;

	/*2 bytes*/
	float lw_crt_;//left wheel current
	
	/*2 bytes*/
	float rw_crt_; // right wheel current

	/*1 byte*/
	uint16_t left_wall_; // left wall sensor
	
	/*1 byte*/
	uint16_t right_wall_; // left wall sensor
	
	/*? byte*/
	int16_t x_acc_; // accelaration of x
	
	/*? byte*/
	int16_t y_acc_; // accelaration of y
	
	/*? byte*/
	int16_t z_acc_; // accelaration of z
	
	/*1 byte*/
	uint8_t gyro_dymc_; // ??
	
	/*1 byte*/
	uint16_t ir_ctrl_;
	
	/*3 bytes*/
	uint32_t charge_stub_;

	/* 1 byte */
	uint32_t rcon_front_left_;

	/* 1 byte */
	uint32_t rcon_front_right_;

	/* 1 byte */
	uint32_t rcon_back_left_;

	/* 1 byte */
	uint32_t rcon_back_right_;

	/* 1 byte */
	uint32_t rcon_left_;

	/* 1 byte */
	uint32_t rcon_right_;

	/* 1 byte */
	bool bumper_right_;

	/* 1 byte */
	bool bumper_left_;
	/* 1 byte */
	int16_t obs_left_;

	/* 1 byte */
	int16_t obs_right_;

	/* 1 byte */
	int16_t obs_front_;

	#if __ROBOT_X900
	//new variable visual wall
	int16_t visual_wall;
	//new variable in x900 robot
	int16_t omni_wheel_;
	//new variable plan
	int8_t plan;
	#endif

	int16_t init_x_acc_;
	int16_t init_y_acc_;
	int16_t init_z_acc_;

	bool	is_moving_;

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
	ros::Subscriber map_metadata_sub_;
//	ros::Subscriber obstacles_sub;
	ros::Publisher robot_odom_pub_;
	ros::Publisher send_clean_marker_pub_;
	ros::Publisher send_clean_map_marker_pub_;
	ros::Publisher send_bumper_marker_pub_;
//	ros::Publisher obstacles_pub_;

	visualization_msgs::Marker clean_markers_,bumper_markers_, clean_map_markers_;
	geometry_msgs::Point m_points_;
	std_msgs::ColorRGBA color_;

	tf::TransformListener		*robot_wf_tf_;
	tf::Stamped<tf::Transform>	map_pose;
	tf::Stamped<tf::Transform>	wf_map_pose;

	tf::TransformBroadcaster	robot_broad;
	geometry_msgs::TransformStamped robot_trans;
	nav_msgs::Odometry robot_odom;
};

#endif
