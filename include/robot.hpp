#ifndef __ROBOT_H__
#define __ROBOT_H__

#include <ros/ros.h>
#include <nav_msgs/MapMetaData.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Point.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <pp/x900sensor.h>

class robot
{
public:
	robot();
	~robot();

	static robot *instance();
	void init();

	bool robot_is_all_ready();
	uint8_t robot_get_workmode();
	float robot_get_angle();
	float robot_get_angle_v();
	int16_t robot_get_cliff_right();
	int16_t robot_get_cliff_left();
	int16_t robot_get_cliff_front();
	int16_t robot_get_wall();
	bool robot_get_lbrush_oc();
	bool robot_get_rbrush_oc();
	bool robot_get_mbrush_oc();
	bool robot_get_vacuum_oc();
	uint8_t robot_get_key();
	bool robot_get_charge_status();
	uint8_t robot_get_ir_ctrl();
	float robot_get_lwheel_current();
	float robot_get_rwheel_current();
	uint32_t robot_get_rcon_front_left();
	uint32_t robot_get_rcon_front_right();
	uint32_t robot_get_rcon_back_left();
	uint32_t robot_get_rcon_back_right();
	uint32_t robot_get_rcon_left();
	uint32_t robot_get_rcon_right();
	bool robot_get_bumper_right();
	bool robot_get_bumper_left();
	int16_t robot_get_obs_left();
	int16_t robot_get_obs_right();
	int16_t robot_get_obs_front();
	bool robot_get_water_tank();
	uint16_t robot_get_battery_voltage();

	bool robot_is_moving();
	float robot_get_linear_x();
	float robot_get_linear_y();
	float robot_get_linear_z();

	int16_t robot_get_yaw();

	float robot_get_position_x();
	float robot_get_position_y();
	float robot_get_position_z();

	void robot_display_positions();
	void pub_ctrl_command(void);
	void pub_clean_markers(void);
	void pub_bumper_markers(void);
	void visualize_marker_init();
	void set_ctrl_data(uint8_t type,uint8_t val);
private:
	bool	is_sensor_ready;
	bool	is_scan_ready;
	bool	is_map_ready;

	/* 1 byte */
	float	angle;

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
	bool charge_status;  

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

	bool	is_moving;

	float	linear_x;
	float	linear_y;
	float	linear_z;

	float	position_x;
	float	position_y;
	float	position_z;
	float 	odom_pose_x;
	float   odom_pose_y;
	float	position_x_off;
	float	position_y_off;
	float	position_z_off;

	float	odom_yaw;
	float	base_link_yaw;
	float	map_yaw;

	float	yaw;

	ros::NodeHandle robot_node_handler;
	ros::Subscriber robot_sensor_sub;
	ros::Subscriber odom_sub;
	ros::Subscriber map_metadata_sub;

	ros::Publisher send_cmd_pub;
	ros::Publisher send_clean_marker_pub;
	ros::Publisher send_bumper_marker_pub;

	visualization_msgs::Marker clean_markers,bumper_markers;
	geometry_msgs::Point m_points;

	tf::TransformListener		*robot_tf;
	tf::Stamped<tf::Transform>	map_pose;

	void robot_robot_sensor_cb(const pp::x900sensor::ConstPtr& msg);
	void robot_odom_cb(const nav_msgs::Odometry::ConstPtr& msg);
	void robot_map_metadata_cb(const nav_msgs::MapMetaData::ConstPtr& msg);

};

#endif
