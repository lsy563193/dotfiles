#include <stdio.h>

#include "gyro.h"

#include "robot.hpp"

static	robot *robot_obj = NULL;

robot::robot()
{
	printf("%s %d\n", __FUNCTION__, __LINE__);
	this->init();
	this->robot_sensor_sub = this->robot_node_handler.subscribe("/robot_sensor", 1, &robot::robot_robot_sensor_cb, this);
	this->odom_sub = this->robot_node_handler.subscribe("/odom", 1, &robot::robot_odom_cb, this);

	this->robot_tf = new tf::TransformListener(this->robot_node_handler, ros::Duration(10), true);
	this->scan_sub = this->robot_node_handler.subscribe("scan", 1, &robot::robot_scan_cb, this);

	is_sensor_ready = is_scan_ready = false;

	bumper_left = 0;
	bumper_right = 0;

	is_moving = false;
	linear_x = 0.0;
	linear_y = 0.0;
	linear_z = 0.0;
}

robot::~robot()
{
	delete this->robot_tf;
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
	return (is_sensor_ready && is_scan_ready) ? true : false;
}

void robot::robot_robot_sensor_cb(const pp::sensor::ConstPtr& msg)
{
	angle = msg->angle;

	angle_v = msg->angle_v;

	vaccum = msg->vaccum;

	brush_left = msg->lbrush;

	brush_right = msg->rbrush;

	brush_main = msg->mbrush;

	cliff_right = msg->rcliff;

	cliff_left = msg->lcliff;

	cliff_front = msg->fcliff;

	wall  = msg->wall;

	rcon_front_left = msg->flrcon;

	rcon_front_right = msg->frrcon;

	rcon_back_left = msg->blrcon;

	rcon_back_right = msg->brrcon;

	rcon_left = msg->lrcon;

	rcon_right = msg->rrcon;

	bumper_right = msg->rbumper;

	bumper_left = msg->lbumper;

	obs_left = msg->lobs;

	obs_right = msg->robs;

	obs_front = msg->fobs;

	box = msg->box;

	battery_voltage = msg->batv;

	if (is_sensor_ready == false) {
		printf("%s %d: Gyro starting angle: %d\n", __FUNCTION__, __LINE__, (int16_t)((angle * 10 + 3600)) % 3600);

		Gyro_SetAngle(((int16_t)(angle * 10 + 3600)) % 3600, angle_v);
		is_sensor_ready = true;
	} else {
		Gyro_SetAngle(((int16_t)(angle * 10 + 3600)) % 3600, angle_v);
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
	linear_x = msg->twist.twist.linear.x;
	linear_y = msg->twist.twist.linear.y;
	linear_z = msg->twist.twist.linear.z;

	if (linear_x == 0.0 && linear_y == 0.0 && linear_z == 0.0) {
		is_moving = false;
	} else {
		is_moving = true;
	}
}

void robot::robot_scan_cb(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	tf::Stamped<tf::Pose> ident;
	tf::Stamped<tf::Transform> odom_pose;
	ident.setIdentity();
	ident.frame_id_ = "base_link";
	ident.stamp_ = msg->header.stamp;

	try {
		this->robot_tf->transformPose("odom", ident, odom_pose);
	} catch(tf::TransformException e) {
		ROS_WARN("Failed to compute odom pose, skipping scan (%s)", e.what());
		return;
	}

	double yaw,pitch,roll;
	tf::Matrix3x3 mat =  odom_pose.getBasis();
	mat.getEulerYPR(yaw, pitch, roll);

	if (!is_scan_ready) {
		position_x_off = odom_pose.getOrigin().x();
		position_y_off = odom_pose.getOrigin().y();
		position_z_off = odom_pose.getOrigin().z();
	}

	position_y = odom_pose.getOrigin().y() - position_y_off;
	position_x = odom_pose.getOrigin().x() - position_x_off;

	if (is_scan_ready) {
		//Gyro_SetAngle(((int16_t)(rad2deg(yaw, 10) + 3600)) % 3600, angle_v);
	} else {
		//Gyro_SetOffset(((int16_t)(rad2deg(yaw, 10) + 3600)) % 3600);
		//Gyro_SetAngle(((int16_t)(rad2deg(yaw, 10) + 3600)) % 3600, angle_v);
		is_scan_ready = true;
	}

#if 0
	printf("%s %d: time %f: pos.: (%.6f, %.6f) yaw: %.6f(%d)\troll: %.6f\tpitch: %.6f\tpos: (%.6f, %.6f)\n", __FUNCTION__, __LINE__,
		msg->header.stamp.toSec(), odom_pose.getOrigin().x(), odom_pose.getOrigin().y(), yaw, Gyro_GetAngle(0), roll, pitch, position_x, position_y);
#endif

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
