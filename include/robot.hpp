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
#include <visualization_msgs/Marker.h>
#include <pp/x900sensor.h>
#include <vector>
#include "config.h"

extern volatile int16_t Left_Wall_BaseLine;
extern volatile int16_t Right_Wall_BaseLine;
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
	void pubBumperMarkers(void);
	void visualizeMarkerInit();
	void initOdomPosition();

	//callback function
private:
	void sensorCb(const pp::x900sensor::ConstPtr &msg);
	void robotOdomCb(const nav_msgs::Odometry::ConstPtr &msg);
//	void robot_map_metadata_cb(const nav_msgs::MapMetaData::ConstPtr& msg);
	void mapCb(const nav_msgs::OccupancyGrid::ConstPtr &msg);

	//get and set function
public:
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
		return sensor.left_wall - Left_Wall_BaseLine;
	}

	int16_t getRightWall() const
	{
#if __ROBOT_X900
		return sensor.right_wall - Right_Wall_BaseLine;
#elif __ROBOT_X400
		return 0;
#endif
	}

	int16_t getOmniWheel() const
	{
#if __ROBOT_X9000
		return sensor.omni_wheel;
#elif __ROBOT_X400
		return 0;
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
		return ((int16_t)(yaw_ * 1800 / M_PI));
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

	double getMapYaw() const
	{
		return yaw_;
	}

	double getCorrectionX() const
	{
		return correction_x_;
	}

	double getCorrectionY() const
	{
		return correction_y_;
	}

	double getCorrectionYaw() const
	{
		return correction_yaw_;
	}

	void setTfReady(bool is_ready)
	{
		is_tf_ready_ = is_ready;
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
		low_bat_pause_cleaning_ = true;
#endif
	}

	void resetLowBatPause(void)
	{
#if CONTINUE_CLEANING_AFTER_CHARGE
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
		manual_pause_cleaning_ = true;
#endif
	}

	void resetManualPause(void)
	{
#if MANUAL_PAUSE_CLEANING
		manual_pause_cleaning_ = false;
#endif
	}

	Baselink_Frame_Type getBaselinkFrameType(void)
	{
		return baselink_frame_type_;
	}

	void setBaselinkFrameType(Baselink_Frame_Type frame)
	{
		baselink_frame_type_ = frame;
	}

private:

	Baselink_Frame_Type baselink_frame_type_;

// These variable is for continue cleaning after charge.
	bool	low_bat_pause_cleaning_;

// These variable is for continue cleaning after charge.
	bool	manual_pause_cleaning_;


	bool	is_sensor_ready_;

	bool	is_tf_ready_;

	float offset_angle_;

	boost::mutex offset_angle_metux_;

	float saved_offset_angle_;

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
	float x_acc_; // accelaration of x
	
	/*? byte*/
	float y_acc_; // accelaration of y
	
	/*? byte*/
	float z_acc_; // accelaration of z
	
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
	int16_t omni_wheel;
	//new variable plan
	bool plan;
	#endif
	bool	is_moving_;

	float	linear_x_;
	float	linear_y_;
	float	linear_z_;

	float	position_x_;
	float	position_y_;
	float	position_z_;
	float	wf_position_x_;
	float	wf_position_y_;
	float	odom_pose_x_;
	float	odom_pose_y_;

	double	yaw_;

	float	correction_x_;
	float	correction_y_;
	float	correction_yaw_;

	ros::NodeHandle robot_nh_;
	ros::Subscriber sensor_sub_;
	ros::Subscriber map_sub_;
	ros::Subscriber odom_sub_;
	ros::Subscriber map_metadata_sub_;
//	ros::Subscriber obstacles_sub;
	ros::Publisher send_cmd_pub_;
	ros::Publisher send_clean_marker_pub_;
	ros::Publisher send_bumper_marker_pub_;
//	ros::Publisher obstacles_pub_;

	visualization_msgs::Marker clean_markers_,bumper_markers_;
	geometry_msgs::Point m_points_;

	tf::TransformListener		*robot_wf_tf_;
	tf::Stamped<tf::Transform>	map_pose;
	tf::Stamped<tf::Transform>	wf_map_pose;
};

#endif
