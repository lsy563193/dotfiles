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
	void Subscriber(void);
	void UnSubscriber(void);
	bool robot_is_all_ready();
	uint8_t robot_get_workmode();
	float robot_get_angle();
	void set_angle(float angle_);
	float robot_get_angle_v();
	int16_t robot_get_cliff_right();
	int16_t robot_get_cliff_left();
	int16_t robot_get_cliff_front();
	int16_t robot_get_left_wall();
	int16_t robot_get_right_wall();
	int16_t robot_get_omni_wheel();
	int16_t robot_get_visual_wall();
	bool robot_get_lbrush_oc();
	bool robot_get_rbrush_oc();
	bool robot_get_mbrush_oc();
	bool robot_get_vacuum_oc();
	uint8_t robot_get_key();
	int robot_get_charge_status();
	uint8_t robot_get_ir_ctrl();
	float robot_get_lwheel_current();
	float robot_get_rwheel_current();
	uint32_t robot_get_rcon();

	bool robot_get_bumper_right();
	bool robot_get_bumper_left();
	int16_t robot_get_obs_left();
	int16_t robot_get_obs_right();
	int16_t robot_get_obs_front();
	bool robot_get_water_tank();
	uint16_t robot_get_battery_voltage();

#if EXPLORE_SCOPE_ENABLE
        std::vector<int8_t> *robot_get_map_data();
#endif

        uint32_t robot_get_width();
        uint32_t robot_get_height();
        float robot_get_resolution();
        double robot_get_origin_x();
        double robot_get_origin_y();

	bool robot_is_moving();
	float robot_get_linear_x();
	float robot_get_linear_y();
	float robot_get_linear_z();

	int16_t robot_get_yaw();

	int16_t robot_get_home_angle();
	float robot_get_position_x();
	float robot_get_position_y();
	float robot_get_WF_position_x();
	float robot_get_WF_position_y();
	float robot_get_odom_position_x();
	float robot_get_odom_position_y();
	float robot_get_position_z();

	float robot_get_map_position_x();
	float robot_get_map_position_y();
	double robot_get_map_yaw();

	void robot_display_positions();
	void pub_ctrl_command(void);
	void pub_clean_markers(void);
	void pub_bumper_markers(void);
	void visualize_marker_init();
	void set_ctrl_data(uint8_t type,uint8_t val);
	void align(void);
	void align_exit(void);
	void align_active(bool);

	void start_lidar(void);
	void stop_lidar(void);
	void start_slam(void);
	void stop_slam(void);

	void init_mumber();
	void slam_type(int type);
	void map_ready(bool);
	bool map_ready(void);
	bool align_active(void){
		return is_align_active_;
	}

#if CONTINUE_CLEANING_AFTER_CHARGE
	// These 3 functions are for continue cleaning after charge.
	bool Is_Cleaning_Paused(void);
	void Set_Cleaning_Pause(void);
	void Reset_Cleaning_Pause(void);
#endif

private:

#if CONTINUE_CLEANING_AFTER_CHARGE
	// These variable is for continue cleaning after charge.
	bool	pause_cleaning;
#endif
	bool	is_sensor_ready;
	bool	is_odom_ready;
	std::vector<int8_t> map_data;
	std::vector<int8_t> *ptr;
	uint32_t seq;
        uint32_t width;
        uint32_t height;
	float resolution;
	double origin_x;
	double origin_y;
	bool	is_map_ready;

	enum align_state{
		detecting = 0,
		rotating = 1,
		finish = 2,
	} line_align_;

	/* 1 byte */
	float	angle;

	bool	is_align_active_;
	int16_t line_angle;
	/* 1 byte */
	float	angle_v;

	/* 1 byte */
	int32_t brush_left;

	/* 1 byte */
	int32_t brush_right;

	/* 1 byte */
	int32_t brush_main;

	/* 2 bytes */
	int16_t cliff_right;

	/* 2 bytes */
	int16_t cliff_left;

	/* 2 bytes */
	int16_t cliff_front;

	/*1 byte */
	uint8_t key;

	/*1 byte */
	uint8_t charge_status;

	/*1 byte*/
	bool w_tank; //water tank 

	/* 1 byte */
	uint16_t battery_voltage;

	/*1 byte*/
	bool lbrush_oc; //oc: over current
	bool rbrush_oc;
	bool mbrush_oc;
	bool vacuum_oc;

	/*2 bytes*/
	float lw_crt;//left wheel current
	
	/*2 bytes*/
	float rw_crt; // right wheel current

	/*1 byte*/
	uint16_t left_wall; // left wall sensor
	
	/*1 byte*/
	uint16_t right_wall; // left wall sensor
	
	/*? byte*/
	float x_acc; // accelaration of x
	
	/*? byte*/
	float y_acc; // accelaration of y
	
	/*? byte*/
	float z_acc; // accelaration of z
	
	/*1 byte*/
	uint8_t gyro_dymc; // ??
	
	/*1 byte*/
	uint16_t ir_ctrl;
	
	/*3 bytes*/
	uint32_t charge_stub;

	/* 1 byte */
	uint32_t rcon_front_left;

	/* 1 byte */
	uint32_t rcon_front_right;

	/* 1 byte */
	uint32_t rcon_back_left;

	/* 1 byte */
	uint32_t rcon_back_right;

	/* 1 byte */
	uint32_t rcon_left;

	/* 1 byte */
	uint32_t rcon_right;

	/* 1 byte */
	bool bumper_right;

	/* 1 byte */
	bool bumper_left;
	/* 1 byte */
	int16_t obs_left;

	/* 1 byte */
	int16_t obs_right;

	/* 1 byte */
	int16_t obs_front;

	#if __ROBOT_X900
	//new variable visual wall
	int16_t visual_wall;
	//new variable in x900 robot
	int16_t omni_wheel;
	#endif
	bool	is_moving;

	float	linear_x;
	float	linear_y;
	float	linear_z;

	float	position_x;
	float	position_y;
	float	position_z;
	float	WF_position_x;
	float	WF_position_y;
	float	odom_pose_x;
	float	odom_pose_y;
	float	position_map_x;
	float	position_map_y;
	float	position_x_off;
	float	position_y_off;
	float	position_z_off;

	float	odom_yaw;
	float	base_link_yaw;
	float	map_yaw;

	double	yaw;

	int slam_type_;
	ros::NodeHandle robot_node_handler;
	ros::Subscriber robot_sensor_sub;
	ros::Subscriber odom_sub;
	ros::Subscriber map_metadata_sub;
	ros::Subscriber map_sub;
	ros::Subscriber obstacles_sub;
	ros::Publisher send_cmd_pub;
	ros::Publisher send_clean_marker_pub;
	ros::Publisher send_bumper_marker_pub;

	ros::ServiceClient start_mator_cli_;
	ros::ServiceClient stop_mator_cli_;

	visualization_msgs::Marker clean_markers,bumper_markers;
	geometry_msgs::Point m_points;

	tf::TransformListener		*robot_tf;
	tf::TransformListener		*robot_WF_tf;
	tf::Stamped<tf::Transform>	map_pose;
	tf::Stamped<tf::Transform>	WF_map_pose;

	void robot_robot_sensor_cb(const pp::x900sensor::ConstPtr& msg);
	void robot_odom_cb(const nav_msgs::Odometry::ConstPtr& msg);
	void robot_obstacles_cb(const obstacle_detector::Obstacles::ConstPtr& obstacles);

#if EXPLORE_SCOPE_ENABLE
	void robot_map_cb(const nav_msgs::OccupancyGrid::ConstPtr& msg);
#endif
};

#endif
