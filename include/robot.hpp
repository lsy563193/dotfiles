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

class robot
{
public:
	robot();
	~robot();

	static robot *instance();
	void init();
//	void Subscriber(void);
	bool isAllReady();
	float getAngle();
	void setAngle(float angle_);

	void offsetAngle(float angle){
		boost::mutex::scoped_lock(offset_angle_metux_);
		offset_angle_ = angle;
	};
	float offsetAngle(void){
		boost::mutex::scoped_lock(offset_angle_metux_);
		return offset_angle_;
	};

	float getAngleV();
	int16_t getCliffRight();
	int16_t getCliffLeft();
	int16_t getCliffFront();
	int16_t getLeftWall();
	int16_t getRightWall();
	int16_t getOmniWheel();
	int16_t getVisualWall();
	uint8_t getVacuumSelfCheckStatus();
	bool getLbrushOc();
	bool getRbrushOc();
	bool getMbrushOc();
	bool getVacuumOc();
	uint8_t getKey();
	int getChargeStatus();
	uint8_t getIrCtrl();
	float getLwheelCurrent();
	float getRwheelCurrent();
	uint32_t getRcon();
	//uint32_t robot_get_rcon_front_left();
	//uint32_t robot_get_rcon_front_right();
	//uint32_t robot_get_rcon_back_left();
	//uint32_t robot_get_rcon_back_right();
	//uint32_t robot_get_rcon_left();
	//uint32_t robot_get_rcon_right();
	bool getBumperRight();
	bool getBumperLeft();
	int16_t getObsLeft();
	int16_t getObsRight();
	int16_t getObsFront();
	bool getWaterTank();
	uint16_t getBatteryVoltage();
	std::vector<int8_t> *getMapData();

	bool isMoving();
	float getLinearX();
	float getLinearY();
	float getLinearZ();

	int16_t getYaw();

	float getPositionX();
	float getPositionY();
	float getPositionZ();
	float getWfPositionX();
	float getWfPositionY();
	float getOdomPositionX();
	float getOdomPositionY();

	double getMapYaw();
	void displayPositions();
	void pubCtrlCommand(void);
	void pubCleanMarkers(void);
	void pubBumperMarkers(void);
	void visualizeMarkerInit();
	void isOdomReady(bool is_ready){is_odom_ready_ = is_ready;};
	void initMumber();

#if CONTINUE_CLEANING_AFTER_CHARGE
// These 3 functions are for continue cleaning after charge.
	bool isCleaningLowBatPaused_(void);
	void cleaningLowBatPause_(void);
	void resetCleaningLowBatPause_(void);
#endif

#if MANUAL_PAUSE_CLEANING
// These 3 functions are for manual pause cleaning.
	bool isCleaningManualPaused(void);
	void setCleaningManualPause(void);
	void resetCleaningManualPause(void);
#endif

private:

#if CONTINUE_CLEANING_AFTER_CHARGE
// These variable is for continue cleaning after charge.
	bool	low_bat_pause_cleaning_;
#endif
#if MANUAL_PAUSE_CLEANING
// These variable is for continue cleaning after charge.
	bool	manual_pause_cleaning_;
#endif
	bool	is_sensor_ready_;
	bool	is_odom_ready_;

	float offset_angle_;
	boost::mutex offset_angle_metux_;

	/* 1 byte */
	float	angle;

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
	float	WF_position_y_;
	float	odom_pose_x_;
	float	odom_pose_y_;

	float	odom_yaw_;
	float	base_link_yaw_;
	float	map_yaw_;

	double	yaw_;

	ros::NodeHandle robot_nh_;
	ros::Subscriber sensor_sub_;
	ros::Subscriber odom_sub_;
	ros::Subscriber map_metadata_sub_;
//	ros::Subscriber obstacles_sub;
	ros::Publisher send_cmd_pub_;
	ros::Publisher send_clean_marker_pub_;
	ros::Publisher send_bumper_marker_pub_;
//	ros::Publisher obstacles_pub_;

	visualization_msgs::Marker clean_markers_,bumper_markers_;
	geometry_msgs::Point m_points_;

	tf::TransformListener		*robot_tf_;
	tf::TransformListener		*robot_wf_tf_;
	tf::Stamped<tf::Transform>	map_pose;
	tf::Stamped<tf::Transform>	wf_map_pose;

	void sensorCb(const pp::x900sensor::ConstPtr &msg);
	void robotOdomCb(const nav_msgs::Odometry::ConstPtr &msg);
//	void robot_map_metadata_cb(const nav_msgs::MapMetaData::ConstPtr& msg);
	void mapCb(const nav_msgs::OccupancyGrid::ConstPtr &msg);
};

#endif
