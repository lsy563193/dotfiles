#ifndef __ROBOT_H__
#define __ROBOT_H__

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <pp/sensor.h>

class robot
{
public:
	robot();
	~robot();

	static robot *instance();
	void init();

	bool robot_is_ready();

	float robot_get_angle();
	float robot_get_angle_v();
	int32_t robot_get_vaccum();
	int32_t robot_get_brush_left();
	int32_t robot_get_brush_right();
	int32_t robot_get_brush_main();
	int32_t robot_get_cliff_right();
	int32_t robot_get_cliff_left();
	int32_t robot_get_cliff_front();
	int32_t robot_get_wall();
	int32_t robot_get_rcon_front_left();
	int32_t robot_get_rcon_front_right();
	int32_t robot_get_rcon_back_left();
	int32_t robot_get_rcon_back_right();
	int32_t robot_get_rcon_left();
	int32_t robot_get_rcon_right();
	int32_t robot_get_bumper_right();
	int32_t robot_get_bumper_left();
	int32_t robot_get_obs_left();
	int32_t robot_get_obs_right();
	int32_t robot_get_obs_front();
	int32_t robot_get_box();
	int32_t robot_get_battery_voltage();
	int32_t robot_get_crg();

	bool robot_is_moving();
	float robot_get_linear_x();
	float robot_get_linear_y();
	float robot_get_linear_z();

	float robot_get_position_x();
	float robot_get_position_y();
	float robot_get_position_z();

private:
	bool	is_ready;

	/* 1 byte */
	float	angle;

	/* 1 byte */
	float	angle_v;

	/* 1 byte */
	int32_t vaccum;

	/* 1 byte */
	int32_t brush_left;

	/* 1 byte */
	int32_t brush_right;

	/* 1 byte */
	int32_t brush_main;

	/* 2 bytes */
	int32_t cliff_right;

	/* 2 bytes */
	int32_t cliff_left;

	/* 2 bytes */
	int32_t cliff_front;

	/* 2 bytes */
	int32_t wall;

	/* 1 byte */
	int32_t rcon_front_left;

	/* 1 byte */
	int32_t rcon_front_right;

	/* 1 byte */
	int32_t rcon_back_left;

	/* 1 byte */
	int32_t rcon_back_right;

	/* 1 byte */
	int32_t rcon_left;

	/* 1 byte */
	int32_t rcon_right;

	/* 1 byte */
	int32_t bumper_right;

	/* 1 byte */
	int32_t bumper_left;

	/* 1 byte */
	int32_t obs_left;

	/* 1 byte */
	int32_t obs_right;

	/* 1 byte */
	int32_t obs_front;

	/* 1 byte */
	int32_t box;

	/* 1 byte */
	int32_t battery_voltage;

	/* 1byte */
	int32_t crg;

	bool	is_moving;

	float	linear_x;
	float	linear_y;
	float	linear_z;

	float	position_x;
	float	position_y;
	float	position_z;

	ros::NodeHandle robot_node_handler;
	ros::Subscriber robot_sensor_sub;
	ros::Subscriber odom_sub;
	ros::Subscriber amcl_pose_sub;

	void robot_robot_sensor_cb(const pp::sensor::ConstPtr& msg);
	void robot_odom_cb(const nav_msgs::Odometry::ConstPtr& msg);
	void robot_amcl_pose_cb(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);

};

#endif
