#include <stdio.h>
#include <time.h>

#include "gyro.h"

#include "robot.hpp"

static	robot *robot_obj = NULL;

time_t	start_time;

robot::robot()
{
	this->init();
	this->robot_sensor_sub = this->robot_node_handler.subscribe("/robot_sensor", 1, &robot::robot_robot_sensor_cb, this);
	this->odom_sub = this->robot_node_handler.subscribe("/odom", 1, &robot::robot_odom_cb, this);

	this->robot_tf = new tf::TransformListener(this->robot_node_handler, ros::Duration(10), true);

	this->is_moving = false;
	this->is_sensor_ready = false;
	this->is_scan_ready = false;

	this->bumper_left = 0;
	this->bumper_right = 0;

	this->linear_x = 0.0;
	this->linear_y = 0.0;
	this->linear_z = 0.0;
	printf("%s %d: robot init done!\n", __FUNCTION__, __LINE__);
	start_time = time(NULL);
}

robot::~robot()
{
	delete this->robot_tf;

	robot_obj = NULL;
}

robot *robot::instance()
{
	return robot_obj;
}

void robot::init()
{
	if (robot_obj == NULL) {
		robot_obj = this;
	}
}

bool robot::robot_is_all_ready()
{
	return (this->is_sensor_ready && this->is_scan_ready) ? true : false;
}

void robot::robot_robot_sensor_cb(const pp::sensor::ConstPtr& msg)
{
	this->angle = msg->angle;

	this->angle_v = msg->angle_v;

	this->cliff_right = msg->rcliff;

	this->cliff_left = msg->lcliff;

	this->cliff_front = msg->fcliff;

	this->wall = msg->wall;

	this->bumper_right = msg->rbumper;

	this->bumper_left = msg->lbumper;

	this->obs_left = msg->lobs;

	this->obs_right = msg->robs;

	this->obs_front = msg->fobs;

	this->battery_voltage = msg->batv;

	if (this->is_sensor_ready == false) {
		if (time(NULL) - start_time > 2) {
			//printf("%s %d: Gyro starting angle: %d\n", __FUNCTION__, __LINE__, (int16_t)((this->angle * 10 + 3600)) % 3600);

			Gyro_SetImuOffset(((int16_t)(this->angle * 10 + 3600)) % 3600);
			Gyro_SetImuAngle(((int16_t)(this->angle * 10 + 3600)) % 3600, this->angle_v);
			this->is_sensor_ready = true;
		}
	} else {
		Gyro_SetImuAngle(((int16_t)(this->angle * 10 + 3600)) % 3600, this->angle_v);
	}

#if 0
	printf("%s %d:\n\t\tangle: %f\tangle_v: %f\n", __FUNCTION__, __LINE__, angle, angle_v);
	printf("\t\tvaccum: %d\tbox: %d\tbattery_voltage: %d, brush left: %d\t brush right: %d\tbrush main: %d\n", vaccum, box, battery_voltage, brush_left, brush_right, brush_main);
	printf("\t\tbumper_right: %d\tbumper_left: %d\tobs_left: %d\tobs_right: %d\tobs_front: %d\n", bumper_right, bumper_left, obs_left, obs_right , obs_front);
	printf("\t\tcliff right: %d\tcliff left: %d\t cliff front: %d\t wall: %d\n", cliff_right, cliff_left, cliff_front, wall);
	printf("\t\trcon left: %d\trcon right: %d\trcon fl: %d\trcon fr: %d\trcon bl: %d\trcon br: %d\n", rcon_left, rcon_right, rcon_front_left, rcon_front_right, rcon_back_left, rcon_back_right);
#endif
}

void robot::robot_odom_cb(const nav_msgs::Odometry::ConstPtr& msg)
{
	double odom_yaw, base_link_yaw, map_yaw, pitch, roll;

	tf::Matrix3x3				mat;
	tf::Stamped<tf::Pose>		ident;
	tf::StampedTransform		transform;
	tf::Stamped<tf::Transform>	odom_pose, base_link_pose, map_pose;

	this->linear_x = msg->twist.twist.linear.x;
	this->linear_y = msg->twist.twist.linear.y;
	this->linear_z = msg->twist.twist.linear.z;

	if (this->linear_x == 0.0 && this->linear_y == 0.0 && this->linear_z == 0.0) {
		this->is_moving = false;
	} else {
		this->is_moving = true;
	}

	ident.setIdentity();
	ident.frame_id_ = "base_link";
	ident.stamp_ = msg->header.stamp;

	try {
		this->robot_tf->transformPose("odom", ident, odom_pose);
		mat = odom_pose.getBasis();
		mat.getEulerYPR(odom_yaw, pitch, roll);
		this->odom_yaw = odom_yaw;
		this->odom_pose = odom_pose;
	} catch(tf::TransformException e) {
		ROS_WARN("Failed to compute odom pose, skipping scan (%s)", e.what());
		//return;
	}

	ident.frame_id_ = "/odom";
	ident.stamp_ = msg->header.stamp;

	try {
		this->robot_tf->transformPose("/base_link", ident, base_link_pose);
		mat = odom_pose.getBasis();
		mat.getEulerYPR(base_link_yaw, pitch, roll);
		this->base_link_yaw = base_link_yaw;
		this->base_link_pose = base_link_pose;
	} catch(tf::TransformException e) {
		ROS_WARN("Failed to compute odom pose, skipping scan (%s)", e.what());
		//return;
	}

	ident.frame_id_ = "base_link";
	ident.stamp_ = msg->header.stamp;

	try {
		this->robot_tf->lookupTransform("/map", "base_link", ros::Time(0), transform);
		this->yaw = tf::getYaw(transform.getRotation());

		Gyro_SetAngle(((int16_t)(this->yaw * 1800 / M_PI + 3600)) % 3600, this->angle_v);
		//printf("%s %d: offset: %d\n", __FUNCTION__, __LINE__, ((int16_t)(this->yaw * 1800 / M_PI + 3600)) % 3600 - Gyro_GetAngle(0));
	} catch(tf::TransformException e) {
		ROS_WARN("Failed to compute odom pose, skipping scan (%s)", e.what());
		return;
	}

	try {
		this->robot_tf->transformPose("/map", ident, map_pose);
		mat = odom_pose.getBasis();
		mat.getEulerYPR(map_yaw, pitch, roll);
		this->map_yaw = map_yaw;
		this->map_pose = map_pose;
	} catch(tf::TransformException e) {
		ROS_WARN("Failed to compute odom pose, skipping scan (%s)", e.what());
		return;
	}

	if (!this->is_scan_ready) {
		this->position_x_off = map_pose.getOrigin().x();
		this->position_y_off = map_pose.getOrigin().y();
		this->position_z_off = map_pose.getOrigin().z();
	}

	this->position_y = map_pose.getOrigin().y() - position_y_off;
	this->position_x = map_pose.getOrigin().x() - position_x_off;

	if (this->is_scan_ready == false) {
		this->is_scan_ready = true;
	}
}

float robot::robot_get_angle()
{
	return this->angle;
}

float robot::robot_get_angle_v()
{
	return this->angle_v;
}

int32_t robot::robot_get_vaccum()
{
	return this->vaccum;
}

int32_t robot::robot_get_brush_left()
{
	return this->brush_left;
}

int32_t robot::robot_get_brush_right()
{
	return this->brush_right;
}

int32_t robot::robot_get_brush_main()
{
	return this->brush_main;
}

int32_t robot::robot_get_cliff_right()
{
	return this->cliff_right;
}

int32_t robot::robot_get_cliff_left()
{
	return this->cliff_left;
}

int32_t robot::robot_get_cliff_front()
{
	return this->cliff_front;
}

int32_t robot::robot_get_wall()
{
	return this->wall;
}

int32_t robot::robot_get_rcon_front_left()
{
	return this->rcon_front_left;
}

int32_t robot::robot_get_rcon_front_right()
{
	return this->rcon_front_right;
}

int32_t robot::robot_get_rcon_back_left()
{
	return this->rcon_back_left;
}

int32_t robot::robot_get_rcon_back_right()
{
	return this->rcon_back_right;
}

int32_t robot::robot_get_rcon_left()
{
	return this->rcon_left;
}

int32_t robot::robot_get_rcon_right()
{
	return this->rcon_right;
}

int32_t robot::robot_get_bumper_right()
{
	return this->bumper_right;
}

int32_t robot::robot_get_bumper_left()
{
	return this->bumper_left;
}

int32_t robot::robot_get_obs_left()
{
	return this->obs_left;
}

int32_t robot::robot_get_obs_right()
{
	return this->obs_right;
}

int32_t robot::robot_get_obs_front()
{
	return this->obs_front;
}

int32_t robot::robot_get_box()
{
	return this->box;
}

int32_t robot::robot_get_battery_voltage()
{
	return this->battery_voltage;
}

int32_t robot::robot_get_crg()
{
	return this->crg;
}

bool robot::robot_is_moving()
{
	return this->is_moving;
}

float robot::robot_get_linear_x()
{
	return this->linear_x;
}

float robot::robot_get_linear_y()
{
	return this->linear_y;
}

float robot::robot_get_linear_z()
{
	return this->linear_z;
}

float robot::robot_get_position_x()
{
	return this->position_x;
}

float robot::robot_get_position_y()
{
	return this->position_y;
}

float robot::robot_get_position_z()
{
	return this->position_z;
}

void robot::robot_display_positions()
{
	printf("base_link->odom: (%f, %f) %f(%f)\todom->base_link: (%f, %f) %f(%f)\tbase_link->map: (%f, %f) %f(%f) Gyro: %d\tyaw: %f(%f)\n",
			this->odom_pose.getOrigin().x(), this->odom_pose.getOrigin().y(), this->odom_yaw, this->odom_yaw * 1800 / M_PI,
			this->base_link_pose.getOrigin().x(), this->base_link_pose.getOrigin().y(), this->base_link_yaw, this->base_link_yaw * 1800 / M_PI,
			this->map_pose.getOrigin().x(), this->map_pose.getOrigin().y(), this->map_yaw, this->map_yaw * 1800 / M_PI, Gyro_GetAngle(0), this->yaw, this->yaw * 1800 / M_PI);
}
